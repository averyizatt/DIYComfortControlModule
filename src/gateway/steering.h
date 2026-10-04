#pragma once
#include "config.h"
#include "logic.h"
#include <can_contract/gateway_protocol.h>

namespace gateway {
// Out-of-window readings (including idle ADC saturation) mean no ladder button.
inline uint8_t ladderButton(uint32_t mv) {
  // Windows from gateway_config, centred on the installed wheel's measured levels.
  for (const auto& window : gateway_config::steeringWindows)
    if (mv >= window.lowMv && mv <= window.highMv) return window.mask;
  return 0;
}
// ON from its measured voltage, with hysteresis between the two thresholds.
inline bool onPressed(uint32_t mv, bool previous) {
  return previous ? mv > gateway_config::steeringOnReleaseMv : mv >= gateway_config::steeringOnPressMv;
}
struct SteeringButtons {
  DebouncedButton on;
  uint8_t ladderStable = 0, ladderCandidate = 0;
  uint32_t ladderSince = 0;
  uint8_t update(bool onHigh, uint32_t ladderMv, uint32_t now) {
    on.update(onHigh, now);
    const uint8_t raw = ladderButton(ladderMv);
    if (raw != ladderCandidate) { ladderCandidate = raw; ladderSince = now; }
    // Debounce one selection atomically, never four independently held bits.
    if (uint32_t(now - ladderSince) >= 25) ladderStable = ladderCandidate;
    return ladderStable | (on.stable ? can_protocol::gateway::BUTTON_ON : 0);
  }
};
} // namespace gateway
