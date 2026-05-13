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

enum class LedMode : uint8_t {
  OFF = 0,
  STATIC_COLOR = 1,
  BREATHING = 2,
  RAINBOW = 3,
  RPM_REACTIVE = 4,
  WARNING_FLASH = 5,
  METH_ACTIVE = 6,
  CAN_FAULT = 7,
  STARTUP_SWEEP = 8,
};

enum class MethCanLossBehavior : uint8_t {
  DISARM = 0,
  HOLD_LAST_VALID = 1,
};

enum class RaceMode : uint8_t {
  OFF = 0,
  ACCEL = 1,
  LAP = 2,
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
  uint8_t meth_selected_ratio_percent = 50;
  uint8_t meth_config_version = 0;
  bool meth_desired_armed = false;
  uint8_t meth_boost_trigger_kpa = 120;
  int8_t meth_iat_safety_threshold = 55;
  uint8_t meth_max_pump_duty = 200;
  MethCanLossBehavior meth_can_loss_behavior = MethCanLossBehavior::DISARM;
  bool meth_manual_test_confirmation_required = true;

  // GPS / CAN health
  bool gps_fix = false;
  uint8_t gps_satellites = 0;
  bool can_online = false;
  uint32_t can_rx_count = 0;
  uint32_t can_tx_count = 0;
  uint32_t can_bad_checksum_count = 0;
  uint16_t can_last_rx_id = 0;
  uint16_t can_last_tx_id = 0;
  uint32_t can_last_rx_ms = 0;
  uint32_t can_last_tx_ms = 0;

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
  uint8_t meth_manual_test_reject_reason = 0;
  uint16_t meth_manual_test_cooldown_ms_remaining = 0;

  // LED config/state
  bool led_channel_1_enabled = true;
  bool led_channel_2_enabled = true;
  bool led_channel_3_enabled = true;
  uint32_t led_channel_1_color = 0x00FF80;
  uint32_t led_channel_2_color = 0x0080FF;
  uint32_t led_channel_3_color = 0xFF8000;
  LedMode led_channel_1_mode = LedMode::STATIC_COLOR;
  LedMode led_channel_2_mode = LedMode::STATIC_COLOR;
  LedMode led_channel_3_mode = LedMode::STATIC_COLOR;
  uint8_t led_channel_1_brightness = 180;
  uint8_t led_channel_2_brightness = 180;
  uint8_t led_channel_3_brightness = 180;
  uint8_t led_global_brightness = 180;
  uint8_t led_theme = 0;
  bool led_startup_preview = false;

  // UI/system settings snapshot
  uint8_t display_brightness = 180;
  bool night_mode_enabled = false;
  uint8_t tach_scaling_mode = 0;

  // Web and diagnostics
  uint16_t web_connected_clients = 0;
  bool wifi_connected = false;
  bool wifi_ap_mode = true;
  bool sd_mounted = false;
  uint64_t sd_size_bytes = 0;
  uint64_t sd_used_bytes = 0;
  uint32_t sd_write_error_count = 0;
  bool touch_online = false;
  float ui_fps = 0.0f;
  float tach_input_frequency_hz = 0.0f;
  float tach_generated_frequency_hz = 0.0f;
  int8_t esp_die_temp_c = 0;
  uint32_t heap_free_bytes = 0;
  uint8_t reset_reason = 0;
  char current_log_file[64]{};
  char last_sd_write_status[32]{};

  // GPS coordinates
  double gps_latitude = 0.0;
  double gps_longitude = 0.0;

  // Race performance + timing
  RaceMode race_mode = RaceMode::OFF;
  bool race_enabled = false;
  bool race_running = false;
  bool race_use_metric_targets = false;
  bool race_auto_start = true;
  bool race_start_point_set = false;
  bool race_data_valid = false;
  uint8_t race_quality_percent = 0;
  uint8_t race_validation_flags = 0;
  uint8_t race_min_satellites = 6;
  uint16_t race_sample_min_ms = 40;
  uint16_t race_sample_max_ms = 450;
  float race_start_finish_radius_m = 20.0f;
  float race_start_latitude = 0.0f;
  float race_start_longitude = 0.0f;

  uint32_t race_run_start_ms = 0;
  uint32_t race_elapsed_ms = 0;
  float race_distance_m = 0.0f;
  float race_0_30_s = -1.0f;
  float race_0_60_s = -1.0f;
  float race_60_130_s = -1.0f;
  float race_100_150_kph_s = -1.0f;
  float race_eighth_mile_et_s = -1.0f;
  float race_quarter_mile_et_s = -1.0f;
  float race_eighth_mile_trap_mph = 0.0f;
  float race_quarter_mile_trap_mph = 0.0f;

  uint16_t race_lap_count = 0;
  float race_last_lap_s = -1.0f;
  float race_best_lap_s = -1.0f;
  float race_lap_delta_s = 0.0f;
  uint32_t race_last_lap_ms = 0;
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
