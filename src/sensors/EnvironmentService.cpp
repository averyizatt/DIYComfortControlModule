#include "sensors/EnvironmentService.hpp"

namespace ccm::sensors {

bool EnvironmentService::begin() {
  return hal_.begin();
}

core::EnvironmentData EnvironmentService::readEnvironment() {
  return hal_.readEnvironment();
}

core::PowerData EnvironmentService::readPower() {
  return hal_.readPower();
}

}  // namespace ccm::sensors
