#include "gps/GpsService.hpp"

namespace ccm::gps {

bool GpsService::begin(uint32_t baud) {
  return hal_.begin(baud);
}

void GpsService::poll() {
  hal_.poll();
  latest_ = hal_.latest();
}

core::GpsData GpsService::data() const {
  return latest_;
}

bool GpsService::online(uint32_t nowMs, uint32_t timeoutMs) const {
  if (!latest_.validFix) return false;
  return (nowMs - latest_.lastFixMs) <= timeoutMs;
}

}  // namespace ccm::gps
