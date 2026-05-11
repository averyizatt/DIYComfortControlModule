#include "safety/SafetyManager.hpp"

namespace ccm::safety {

void SafetyManager::begin(float undervoltageThreshold) {
  undervoltageThreshold_ = undervoltageThreshold;
}

core::SystemFault SafetyManager::evaluate(const core::PowerData& power, bool canOnline, bool gpsOnline) const {
  core::SystemFault faults = core::SystemFault::None;
  if (power.batteryV < undervoltageThreshold_ || power.undervoltage) {
    faults = faults | core::SystemFault::Undervoltage;
  }
  if (!canOnline) {
    faults = faults | core::SystemFault::CanOffline;
  }
  if (!gpsOnline) {
    faults = faults | core::SystemFault::GpsOffline;
  }
  return faults;
}

}  // namespace ccm::safety
