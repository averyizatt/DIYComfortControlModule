#pragma once

#include "hal/SensorHal.hpp"

namespace ccm::sensors {

class EnvironmentService {
 public:
  explicit EnvironmentService(hal::SensorHal& hal) : hal_(hal) {}

  bool begin();
  core::EnvironmentData readEnvironment();
  core::PowerData readPower();

 private:
  hal::SensorHal& hal_;
};

}  // namespace ccm::sensors
