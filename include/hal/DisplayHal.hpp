#pragma once

#include "core/SharedTypes.hpp"

namespace ccm::hal {

class DisplayHal {
 public:
  virtual ~DisplayHal() = default;
  virtual bool begin() = 0;
  virtual void setBrightness(uint8_t value) = 0;
  virtual void renderDashboard(const core::DashboardData& dashboard) = 0;
  virtual void renderPopup(const char* message) = 0;
};

}  // namespace ccm::hal
