#include "core/SystemState.hpp"

namespace ccm::core {

SystemState::SystemState() {
  mutex_ = xSemaphoreCreateMutex();
}

SystemState::~SystemState() {
  if (mutex_) {
    vSemaphoreDelete(mutex_);
  }
}

void SystemState::updateDashboard(const DashboardData& data) {
  if (!mutex_) return;
  if (xSemaphoreTake(mutex_, portMAX_DELAY) == pdTRUE) {
    dashboard_ = data;
    xSemaphoreGive(mutex_);
  }
}

DashboardData SystemState::readDashboard() const {
  DashboardData out{};
  if (!mutex_) return out;
  if (xSemaphoreTake(mutex_, portMAX_DELAY) == pdTRUE) {
    out = dashboard_;
    xSemaphoreGive(mutex_);
  }
  return out;
}

void SystemState::setFaults(SystemFault faults) {
  if (!mutex_) return;
  if (xSemaphoreTake(mutex_, portMAX_DELAY) == pdTRUE) {
    faults_ = faults;
    xSemaphoreGive(mutex_);
  }
}

SystemFault SystemState::getFaults() const {
  SystemFault out = SystemFault::None;
  if (!mutex_) return out;
  if (xSemaphoreTake(mutex_, portMAX_DELAY) == pdTRUE) {
    out = faults_;
    xSemaphoreGive(mutex_);
  }
  return out;
}

}  // namespace ccm::core
