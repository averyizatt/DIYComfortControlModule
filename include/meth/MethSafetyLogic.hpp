#pragma once

#include <cstdint>

#include "state/StateHelpers.hpp"

namespace meth {

enum class ManualTestRejectReason : uint8_t {
  NONE = 0,
  OFFLINE = 1,
  FAULT = 2,
  COOLDOWN = 3,
  DUTY_ZERO = 4,
  DUTY_OVER_MAX = 5,
  CONFIRMATION_REQUIRED = 6,
};

struct ManualTestDecision {
  bool allowed = false;
  ManualTestRejectReason reason = ManualTestRejectReason::NONE;

  ManualTestDecision() = default;
  ManualTestDecision(bool isAllowed, ManualTestRejectReason rejectReason)
      : allowed(isAllowed), reason(rejectReason) {}
};

inline bool canArm(const state::VehicleState& s) {
  return s.meth_online && state::methSafetyInputsValid(s) && !state::hasCriticalMethFault(s) &&
         s.meth_tank_level > 10U;
}

inline uint8_t progressivePumpDuty(bool armed, bool faulted, bool lowTank, bool sensorsValid,
                                   float boostKpa, float startKpa, float fullKpa, uint8_t maxDuty) {
  if (!armed || faulted || lowTank || !sensorsValid || maxDuty == 0U || fullKpa <= startKpa) return 0U;
  if (boostKpa <= startKpa) return 0U;
  if (boostKpa >= fullKpa) return maxDuty;
  const float ratio = (boostKpa - startKpa) / (fullKpa - startKpa);
  const float duty = ratio * static_cast<float>(maxDuty);
  return static_cast<uint8_t>(duty < 0.0f ? 0.0f : (duty > static_cast<float>(maxDuty) ? static_cast<float>(maxDuty) : duty + 0.5f));
}

inline ManualTestDecision evaluateManualTestRequest(const state::VehicleState& s, uint8_t duty,
                                                    bool confirmed, uint32_t nowMs, uint32_t lastStopMs,
                                                    uint32_t cooldownMs = 3000U) {
  if (!s.meth_online) return {false, ManualTestRejectReason::OFFLINE};
  if (state::hasCriticalMethFault(s)) return {false, ManualTestRejectReason::FAULT};
  if ((nowMs - lastStopMs) < cooldownMs) return {false, ManualTestRejectReason::COOLDOWN};
  if (duty == 0U) return {false, ManualTestRejectReason::DUTY_ZERO};
  if (duty > 100U) return {false, ManualTestRejectReason::DUTY_OVER_MAX};
  if (!confirmed) return {false, ManualTestRejectReason::CONFIRMATION_REQUIRED};
  return {true, ManualTestRejectReason::NONE};
}

inline bool manualTestTimedOut(bool running, uint32_t nowMs, uint32_t startedMs, uint32_t timeoutMs = 5000U) {
  return running && (nowMs - startedMs) > timeoutMs;
}

struct FaultLatchState {
  bool critical_latched = false;
  bool fault_active = false;
};

inline FaultLatchState updateFaultLatch(FaultLatchState current, bool lowTank, bool noFlow,
                                        bool overpressure, bool underpressure,
                                        bool clearRequested, bool safeToClear) {
  current.fault_active = lowTank || noFlow || overpressure || underpressure || current.critical_latched;
  if (noFlow || overpressure || underpressure) current.critical_latched = true;
  if (clearRequested && safeToClear) {
    current.critical_latched = false;
    current.fault_active = lowTank || noFlow || overpressure || underpressure;
  }
  return current;
}

}  // namespace meth
