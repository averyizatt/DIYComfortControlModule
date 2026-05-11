#pragma once

#include "core/SharedTypes.hpp"

namespace ccm::hal {

class GpsHal {
 public:
  virtual ~GpsHal() = default;
  virtual bool begin(uint32_t baud) = 0;
  virtual void poll() = 0;
  virtual core::GpsData latest() const = 0;
};

}  // namespace ccm::hal
