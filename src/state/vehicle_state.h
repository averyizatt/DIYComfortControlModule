#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "state/VehicleStateData.hpp"

namespace state {

class VehicleStateStore {
 public:
  ~VehicleStateStore() {
    if (mutex_ != nullptr) {
      vSemaphoreDelete(mutex_);
      mutex_ = nullptr;
    }
  }

  void begin() {
    mutex_ = xSemaphoreCreateMutex();
  }

  VehicleState read() const {
    VehicleState copy{};
    if (!mutex_) return copy;
    if (xSemaphoreTake(mutex_, portMAX_DELAY) == pdTRUE) {
      copy = state_;
      xSemaphoreGive(mutex_);
    }
    return copy;
  }

  void write(const VehicleState& next) {
    if (!mutex_) return;
    if (xSemaphoreTake(mutex_, portMAX_DELAY) == pdTRUE) {
      state_ = next;
      xSemaphoreGive(mutex_);
    }
  }

  template <typename Fn>
  void mutate(Fn fn) {
    if (!mutex_) return;
    if (xSemaphoreTake(mutex_, portMAX_DELAY) == pdTRUE) {
      fn(state_);
      xSemaphoreGive(mutex_);
    }
  }

 private:
  mutable SemaphoreHandle_t mutex_ = nullptr;
  VehicleState state_{};
};

extern VehicleStateStore g_vehicle_state;

}  // namespace state
