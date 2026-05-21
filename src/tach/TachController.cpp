#include "tach/TachController.hpp"

#include <Arduino.h>

#include "tach/TachMath.hpp"

namespace ccm::tach {

void TachController::begin(uint8_t pin, uint8_t channel, uint8_t duty) {
  duty_ = duty;
  hal_.begin(pin, channel);
}

void TachController::updateRpm(uint16_t rpm) {
  filteredRpm_ = math::applySmoothing(filteredRpm_, rpm, alpha_);
  const auto hz = rpmToFrequency(static_cast<uint16_t>(filteredRpm_));
  hal_.setFrequencyHz(hz, duty_);
}

void TachController::startupSweep(uint16_t maxRpm, uint16_t step, uint32_t delayMs) {
  for (uint16_t rpm = 0; rpm <= maxRpm; rpm += step) {
    updateRpm(rpm);
    delay(delayMs);
  }
  for (uint16_t rpm = maxRpm;; rpm = (rpm > step) ? static_cast<uint16_t>(rpm - step) : 0) {
    updateRpm(rpm);
    delay(delayMs);
    if (rpm == 0) break;
  }
}

uint32_t TachController::rpmToFrequency(uint16_t rpm) const {
  return math::rpmToFrequencyHz(rpm, static_cast<uint8_t>(scaleMode_));
}

}  // namespace ccm::tach
