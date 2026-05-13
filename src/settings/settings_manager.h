#pragma once

#include <Arduino.h>
#include <Preferences.h>

#include "state/vehicle_state.h"

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
  uint8_t meth_boost_trigger_kpa = 120;
  int8_t meth_iat_safety_threshold = 55;
  uint8_t meth_max_pump_duty = 200;
  uint8_t meth_can_loss_behavior = 0;

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
};

class SettingsManager {
 public:
  bool begin();
  const AppSettings& data() const { return settings_; }

  void loadIntoState(state::VehicleState& s) const;
  void updateFromState(const state::VehicleState& s);
  bool save();

  bool setWifiCredentials(const char* ssid, const char* password, bool apMode);
  bool setWebPassword(const char* password);

 private:
  void load();

  Preferences prefs_;
  AppSettings settings_{};
  bool started_ = false;
};

}  // namespace settings
