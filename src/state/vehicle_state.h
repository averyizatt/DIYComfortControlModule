#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

namespace state {

enum class MethState : uint8_t {
  OFF = 0,
  ARMED = 1,
  SPRAYING = 2,
  FAULT = 3,
  TEST = 4,
};

struct VehicleState {
  // Core dash values
  uint16_t rpm = 0;
  float speed = 0.0f;                   // km/h
  float battery_voltage = 12.5f;        // volts
  float boost_kpa = 0.0f;
  float afr = 14.7f;
  float coolant_temp = 0.0f;

  // Environment temps
  float cabin_temp = 22.0f;
  float outside_temp = 20.0f;
  float engine_bay_temp = 35.0f;
  float intake_temp = 25.0f;
  float intercooler_temp = 24.0f;

  // Water meth
  MethState meth_state = MethState::OFF;
  uint8_t meth_pump_duty = 0;
  uint8_t meth_tank_level = 100;
  uint8_t meth_flow_status = 0;

  // GPS / CAN health
  bool gps_fix = false;
  uint8_t gps_satellites = 0;
  bool can_online = false;

  // Taillight status
  uint8_t taillight_left_state = 0;
  uint8_t taillight_right_state = 0;
  uint8_t taillight_input_flags = 0;
  uint8_t taillight_brightness = 0;
  int8_t taillight_die_temp_c = 0;
  uint8_t taillight_thermal_derate = 0;

  // Fault + diagnostics
  uint16_t fault_flags = 0;
  uint8_t master_state = 1;             // RUN
  uint8_t ui_page = 0;                  // DASH
  uint8_t input_flags = 0;
  uint16_t generated_tach_hz10 = 0;
  uint16_t raw_tach_hz10 = 0;
  uint8_t tach_source = 0;
  uint8_t tach_status_flags = 0;
  uint8_t pulses_per_rev10 = 20;
  int16_t gps_altitude_m = 0;
  uint8_t gps_fix_type = 0;
  uint8_t gps_status_flags = 0;
  uint32_t uptime_ms = 0;

  // Timeout/online tracking
  bool taillight_online = false;
  bool meth_online = false;
  bool gps_stale = true;
  uint32_t last_taillight_ms = 0;
  uint32_t last_meth_ms = 0;
  uint32_t last_gps_ms = 0;

  // Reset counters (placeholder, filled by platform-specific reset reason logic)
  uint16_t brownout_reset_count = 0;
  uint16_t watchdog_reset_count = 0;
  bool manual_test_running = false;
};

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
