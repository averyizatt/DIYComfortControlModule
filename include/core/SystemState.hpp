#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "core/SharedTypes.hpp"

namespace ccm::core {

class SystemState {
 public:
  SystemState();
  ~SystemState();

  void updateDashboard(const DashboardData& data);
  DashboardData readDashboard() const;

  void setFaults(SystemFault faults);
  SystemFault getFaults() const;

 private:
  mutable SemaphoreHandle_t mutex_;
  DashboardData dashboard_{};
  SystemFault faults_ = SystemFault::None;
};

}  // namespace ccm::core
