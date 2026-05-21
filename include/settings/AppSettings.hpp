#pragma once

#include <cstdint>

namespace settings {

struct AppSettings {
  uint8_t display_brightness = 180;
  bool night_mode_enabled = false;
  uint8_t tach_pulses_per_rev10 = 20;
  uint8_t tach_scaling_mode = 0;

  bool led_ch_enabled[3] = {true, true, true};
  uint32_t led_ch_color[3] = {0x00FF80, 0x0080FF, 0xFF8000};
  uint8_t led_ch_mode[3] = {1, 1, 1};
  uint8_t led_ch_brightness[3] = {180, 180, 180};
  uint8_t led_global_brightness = 180;
  uint8_t led_theme = 0;

  uint8_t meth_selected_ratio_percent = 50;
  bool knock_enabled = true;
  uint8_t knock_adc_pin = 48;
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

  bool race_use_metric_targets = false;
  bool race_auto_start = true;
  uint8_t race_min_satellites = 6;
  uint16_t race_sample_min_ms = 40;
  uint16_t race_sample_max_ms = 450;
  float race_start_finish_radius_m = 20.0f;
  float race_start_latitude = 0.0f;
  float race_start_longitude = 0.0f;
  bool race_start_point_set = false;

  bool wifi_ap_mode = true;
  char wifi_ssid[33]{};
  char wifi_password[65]{};
  char web_password[33]{};

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
};

}  // namespace settings
