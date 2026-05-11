#include "tach/TachController.hpp"

#include <Arduino.h>

namespace ccm::tach {

void TachController::begin(uint8_t pin, uint8_t channel, uint8_t duty) {
  duty_ = duty;
  hal_.begin(pin, channel);
}

void TachController::updateRpm(uint16_t rpm) {
  filteredRpm_ = alpha_ * rpm + (1.0f - alpha_) * filteredRpm_;
  const auto hz = rpmToFrequency(static_cast<uint16_t>(filteredRpm_));
  hal_.setFrequencyHz(hz, duty_);
}

void TachController::startupSweep(uint16_t maxRpm, uint16_t step, uint32_t delayMs) {
  for (uint16_t rpm = 0; rpm <= maxRpm; rpm += step) {
    updateRpm(rpm);
    delay(delayMs);
  }
  for (int rpm = maxRpm; rpm >= 0; rpm -= step) {
    updateRpm(static_cast<uint16_t>(rpm));
    delay(delayMs);
  }
}

uint32_t TachController::rpmToFrequency(uint16_t rpm) const {
  const uint16_t divisor = (scaleMode_ == core::TachScaleMode::RpmDiv15) ? 15 : 30;
  const uint32_t hz = rpm / divisor;
  return hz > 0 ? hz : 1;
}

}  // namespace ccm::tach
