#pragma once

#include <cstdint>

namespace web {

inline const char* manualTestRejectReasonText(uint8_t code) {
  switch (code) {
    case 1: return "offline";
    case 2: return "fault";
    case 3: return "cooldown";
    case 4: return "duty_zero";
    case 5: return "duty_over_max";
    case 6: return "confirmation_required";
    default: return "none";
  }
}

inline bool ledChannelInRange(int channel) {
  return channel >= 1 && channel <= 3;
}

inline bool knockSimulationAllowed(bool demoBuildEnabled, bool runtimeDemoEnabled) {
  return demoBuildEnabled || runtimeDemoEnabled;
}

inline bool unsafeKnockResponseRequiresConfirmation(uint8_t responseMode) {
  return responseMode >= 2U;
}

inline bool manualTestRequiresConfirmation(uint8_t duty, bool confirmed) {
  return duty > 0U && !confirmed;
}

}  // namespace web
