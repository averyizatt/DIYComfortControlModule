#pragma once

#include <cstdint>

#include "state/VehicleStateData.hpp"

namespace state {

inline bool nodeTimedOut(uint32_t nowMs, uint32_t lastSeenMs, uint32_t timeoutMs) {
  return lastSeenMs == 0 || (nowMs - lastSeenMs) > timeoutMs;
}

inline bool methSafetyInputsValid(const VehicleState& s) {
  return s.intake_temp_valid && s.meth_pressure_valid;
}

inline bool hasCriticalMethFault(const VehicleState& s) {
  return s.meth_fault_flags != 0U || s.meth_state == MethState::FAULT;
}

inline bool methCanLossDisarms(const VehicleState& s, uint32_t nowMs, uint32_t timeoutMs) {
  return nodeTimedOut(nowMs, s.last_meth_ms, timeoutMs);
}

inline bool safeToClearMethFault(const VehicleState& s) {
  const bool tankOkay = s.meth_tank_level > 10U;
  const bool flowOkay = s.meth_flow_status != static_cast<uint8_t>(can_protocol::FlowStatus::NO_FLOW);
  const bool pressureOkay = s.meth_pressure_valid && s.meth_pressure_psi >= 5.0f && s.meth_pressure_psi <= 250.0f;
  return tankOkay && flowOkay && pressureOkay;
}

inline bool anyFaultActive(const VehicleState& s) {
  return s.fault_flags != 0U || s.knock_sensor_fault || hasCriticalMethFault(s);
}

}  // namespace state
