#pragma once
#include "config.h"
#include "logic.h"
#include <can_contract/gateway_protocol.h>

namespace gateway {
// Out-of-window readings (including idle ADC saturation) mean no ladder button.
inline uint8_t ladderButton(uint32_t mv) {
  namespace wire = can_protocol::gateway;
  constexpr uint32_t levels[] = {0, 550, 1450, 2120};
  constexpr uint8_t masks[] = {wire::BUTTON_OFF, wire::BUTTON_COAST,
      wire::BUTTON_SET_ACCEL, wire::BUTTON_RESUME};
  constexpr uint32_t tolerance = gateway_config::steeringToleranceMv;
  static_assert(tolerance < 275, "Steering voltage windows must not overlap");
  for (unsigned i = 0; i < 4; ++i) {
    const uint32_t difference = mv > levels[i] ? mv - levels[i] : levels[i] - mv;
    if (difference <= tolerance) return masks[i];
  }
  return 0;
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
