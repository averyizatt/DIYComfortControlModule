#pragma once

#include <cstdint>
#include <vector>

namespace ccm::tach::math {

inline uint16_t clampRpm(uint32_t rpm, uint16_t maxRpm = 9000U) {
  return static_cast<uint16_t>(rpm > maxRpm ? maxRpm : rpm);
}

inline uint16_t divisorForMode(uint8_t mode) {
  return mode == 0U ? 15U : 30U;
}

inline uint32_t rpmToFrequencyHz(uint32_t rpm, uint8_t mode, bool enabled = true) {
  if (!enabled) return 0U;
  const uint16_t divisor = divisorForMode(mode);
  const uint16_t clampedRpm = clampRpm(rpm);
  const uint32_t hz = clampedRpm / divisor;
  return hz > 0U ? hz : 1U;
}

inline uint16_t frequencyHzToRpm(uint32_t hz, uint8_t mode) {
  return clampRpm(hz * divisorForMode(mode));
}

inline float applySmoothing(float previous, uint16_t rpm, float alpha) {
  if (alpha < 0.0f) alpha = 0.0f;
  if (alpha > 1.0f) alpha = 1.0f;
  return previous + (static_cast<float>(rpm) - previous) * alpha;
}

inline std::vector<uint16_t> generateSweep(uint16_t maxRpm, uint16_t step) {
  std::vector<uint16_t> result;
  if (step == 0U) return result;
  for (uint16_t rpm = 0; rpm <= maxRpm; rpm = static_cast<uint16_t>(rpm + step)) {
    result.push_back(rpm);
    if (rpm + step < rpm) break;
  }
  for (uint16_t rpm = maxRpm; rpm > 0U; rpm = rpm > step ? static_cast<uint16_t>(rpm - step) : 0U) {
    result.push_back(rpm);
    if (rpm <= step) {
      result.push_back(0U);
      break;
    }
  }
  return result;
}

}  // namespace ccm::tach::math
