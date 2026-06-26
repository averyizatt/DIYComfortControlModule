#pragma once

#include <cstdint>

#include "can/can_protocol.h"

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
  RPM_GAUGE = 9,
};

enum class RaceMode : uint8_t {
  OFF = 0,
  ACCEL = 1,
  LAP = 2,
};

struct VehicleState {
  uint16_t rpm = 0;
  float speed = 0.0f;
  float battery_voltage = 12.5f;
  float boost_kpa = 0.0f;
  float afr = 14.7f;
  float coolant_temp = 0.0f;

  float cabin_temp = 22.0f;
  float outside_temp = 20.0f;
  float engine_bay_temp = 35.0f;
  float intake_temp = 25.0f;
  float intercooler_temp = 24.0f;
  float oil_pressure_psi = 0.0f;
  float fuel_pressure_psi = 0.0f;
  float meth_pressure_psi = 0.0f;
  float boost_ref_pressure_psi = 0.0f;
  float spare_pressure_1_psi = 0.0f;
  float spare_pressure_2_psi = 0.0f;
  bool cabin_temp_valid = false;
  bool outside_temp_valid = false;
  bool engine_bay_temp_valid = false;
  bool intake_temp_valid = false;
  bool oil_pressure_valid = false;
  bool fuel_pressure_valid = false;
  bool meth_pressure_valid = false;
  bool boost_ref_pressure_valid = false;
  bool spare_pressure_1_valid = false;
  bool spare_pressure_2_valid = false;
  uint16_t analog_sensor_fault_flags = 0;
  uint32_t last_analog_sensor_ms = 0;

  MethState meth_state = MethState::OFF;
  uint8_t meth_pump_duty = 0;
  uint8_t meth_tank_level = 100;
  uint8_t meth_flow_status = 0;
  uint8_t meth_selected_ratio_percent = 50;
  uint8_t meth_config_version = 0;
  bool meth_desired_armed = false;

  bool knock_enabled = true;
  uint8_t knock_adc_pin = 48;
  float knock_energy = 0.0f;
  float knock_baseline = 0.0f;
  float knock_threshold = 0.0f;
  uint8_t knock_event_count = 0;
  uint16_t knock_last_event_rpm = 0;
  uint8_t knock_last_event_boost_kpa = 0;
  bool knock_signal_valid = true;
  bool knock_warning_active = false;
  bool knock_critical_active = false;
  bool knock_baseline_learned = false;
  bool knock_sensor_fault = false;
  bool knock_clipping_detected = false;
  uint16_t knock_signal_clip_high_count = 0;
  uint16_t knock_signal_clip_low_count = 0;

  float knock_boost_enable_kpa = 120.0f;
  uint16_t knock_rpm_enable_min = 2500;
  float knock_threshold_multiplier = 2.5f;
  float knock_threshold_offset = 8.0f;
  uint16_t knock_event_cooldown_ms = 250;
  uint8_t knock_warning_threshold_count = 2;
  uint8_t knock_critical_threshold_count = 4;
  bool knock_baseline_learning_enabled = true;
  bool knock_demo_mode_enabled = false;
  uint8_t knock_response_mode = 1;

  int8_t knock_last_event_iat_c = 0;
  uint32_t knock_last_event_time_ms = 0;
  bool knock_logging_active = false;
  bool knock_online = true;
  uint32_t last_knock_ms = 0;

  bool knock_reset_baseline_request = false;
  bool knock_clear_event_count_request = false;
  bool knock_simulate_event_request = false;
  bool knock_fault_pending = false;
  uint8_t knock_fault_code_pending = 0;
  uint8_t knock_fault_severity_pending = 0;
  uint8_t knock_fault_data0_pending = 0;
  uint8_t knock_fault_data1_pending = 0;

  bool gps_fix = false;
  uint8_t gps_satellites = 0;
  uint8_t gps_satellites_in_view = 0;
  uint8_t gps_fix_quality = 0;
  uint8_t gps_fix_mode = 0;
  uint16_t gps_hdop_x10 = 0;
  bool can_online = false;
  uint32_t can_rx_count = 0;
  uint32_t can_tx_count = 0;
  uint32_t can_bad_checksum_count = 0;
  uint16_t can_last_rx_id = 0;
  uint16_t can_last_tx_id = 0;
  uint32_t can_last_rx_ms = 0;
  uint32_t can_last_tx_ms = 0;

  uint8_t taillight_left_state = 0;
  uint8_t taillight_right_state = 0;
  uint8_t taillight_input_flags = 0;
  uint8_t taillight_brightness = 0;
  int8_t taillight_die_temp_c = 0;
  uint8_t taillight_thermal_derate = 0;
  uint8_t taillight_mode_commanded = 0;

  uint16_t fault_flags = 0;
  uint8_t master_state = static_cast<uint8_t>(can_protocol::MasterState::RUN);
  uint8_t ui_page = 0;
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

  bool taillight_online = false;
  bool meth_online = false;
  bool gps_stale = true;
  uint32_t last_taillight_ms = 0;
  uint32_t last_meth_ms = 0;
  uint32_t last_gps_ms = 0;

  uint16_t brownout_reset_count = 0;
  uint16_t watchdog_reset_count = 0;
  bool manual_test_running = false;
  uint8_t meth_manual_test_reject_reason = 0;
  bool bench_test_mode = false;
  uint16_t meth_manual_test_cooldown_ms_remaining = 0;

  bool led_channel_1_enabled = true;
  bool led_channel_2_enabled = true;
  bool led_channel_3_enabled = true;
  uint32_t led_channel_1_color = 0x00FF80;
  uint32_t led_channel_2_color = 0x0080FF;
  uint32_t led_channel_3_color = 0xFF8000;
  LedMode led_channel_1_mode = LedMode::STATIC_COLOR;
  LedMode led_channel_2_mode = LedMode::STATIC_COLOR;
  LedMode led_channel_3_mode = LedMode::RPM_GAUGE;
  uint8_t led_channel_1_brightness = 180;
  uint8_t led_channel_2_brightness = 180;
  uint8_t led_channel_3_brightness = 180;
  uint8_t led_global_brightness = 180;
  uint8_t led_theme = 0;
  bool led_startup_preview = false;

  uint8_t display_brightness = 180;
  bool night_mode_enabled = false;
  uint8_t tach_scaling_mode = 0;
  bool analog_sensors_enabled = true;
  uint16_t analog_sensor_sample_ms = 50;
  float thermistor_pullup_ohms = 10000.0f;
  uint8_t iat_adc_pin = 255;
  uint8_t engine_bay_adc_pin = 255;
  uint8_t cabin_temp_adc_pin = 255;
  uint8_t ambient_temp_adc_pin = 255;
  uint8_t oil_pressure_adc_pin = 255;
  uint8_t fuel_pressure_adc_pin = 255;
  uint8_t meth_pressure_adc_pin = 255;
  uint8_t boost_ref_pressure_adc_pin = 255;
  uint8_t spare_pressure_1_adc_pin = 255;
  uint8_t spare_pressure_2_adc_pin = 255;
  bool iat_sensor_enabled = false;
  bool engine_bay_sensor_enabled = false;
  bool cabin_temp_sensor_enabled = false;
  bool ambient_temp_sensor_enabled = false;
  bool oil_pressure_sensor_enabled = false;
  bool fuel_pressure_sensor_enabled = false;
  bool meth_pressure_sensor_enabled = false;
  bool boost_ref_pressure_sensor_enabled = false;
  bool spare_pressure_1_sensor_enabled = false;
  bool spare_pressure_2_sensor_enabled = false;
  float pressure_sensor_min_v = 0.5f;
  float pressure_sensor_max_v = 4.5f;
  float pressure_sensor_max_psi = 100.0f;

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
  uint32_t heap_min_free_bytes = 0xFFFFFFFFUL;  // watermark: lowest heap ever observed
  uint8_t reset_reason = 0;
  char current_log_file[64]{};
  char last_sd_write_status[32]{};

  double gps_latitude = 0.0;
  double gps_longitude = 0.0;

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

  // IMU / G-force (MPU-6050)
  float imu_g_lateral      = 0.0f;  // left/right G  (+ = right)
  float imu_g_longitudinal = 0.0f;  // fore/aft G     (+ = acceleration)
  float imu_g_total        = 0.0f;  // sqrt(lat² + lon²) horizontal vector
  float imu_g_peak         = 0.0f;  // max imu_g_total since boot
  bool  imu_online         = false;
};

}  // namespace state
