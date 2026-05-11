#pragma once

#include "core/SharedTypes.hpp"

namespace ccm::hal {

class SensorHal {
 public:
  virtual ~SensorHal() = default;
  virtual bool begin() = 0;
  virtual core::EnvironmentData readEnvironment() = 0;
  virtual core::PowerData readPower() = 0;
};

}  // namespace ccm::hal
