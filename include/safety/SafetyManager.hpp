#pragma once

#include "core/SharedTypes.hpp"

namespace ccm::safety {

class SafetyManager {
 public:
  void begin(float undervoltageThreshold);
  core::SystemFault evaluate(const core::PowerData& power, bool canOnline, bool gpsOnline) const;

 private:
  float undervoltageThreshold_ = 11.6f;
};

}  // namespace ccm::safety
