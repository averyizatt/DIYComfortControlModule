#pragma once

#include "core/SharedTypes.hpp"
#include "hal/TachHal.hpp"

namespace ccm::tach {

class TachController {
 public:
  explicit TachController(hal::TachHal& hal) : hal_(hal) {}

  void begin(uint8_t pin, uint8_t channel, uint8_t duty);
  void setSource(core::RpmSource source) { source_ = source; }
  void setScaleMode(core::TachScaleMode mode) { scaleMode_ = mode; }
  void setSmoothing(float alpha) { alpha_ = alpha; }
  void updateRpm(uint16_t rpm);
  void startupSweep(uint16_t maxRpm, uint16_t step, uint32_t delayMs);

 private:
  uint32_t rpmToFrequency(uint16_t rpm) const;

  hal::TachHal& hal_;
  core::RpmSource source_ = core::RpmSource::CanBus;
  core::TachScaleMode scaleMode_ = core::TachScaleMode::RpmDiv15;
  uint8_t duty_ = 128;
  float alpha_ = 0.3f;
  float filteredRpm_ = 0.0f;
};

}  // namespace ccm::tach
