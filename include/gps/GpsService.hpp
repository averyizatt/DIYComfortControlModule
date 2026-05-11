#pragma once

#include "hal/GpsHal.hpp"

namespace ccm::gps {

class GpsService {
 public:
  explicit GpsService(hal::GpsHal& hal) : hal_(hal) {}

  bool begin(uint32_t baud);
  void poll();
  core::GpsData data() const;
  bool online(uint32_t nowMs, uint32_t timeoutMs) const;

 private:
  hal::GpsHal& hal_;
  core::GpsData latest_{};
};

}  // namespace ccm::gps
