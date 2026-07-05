#pragma once

#include "settings/AppSettings.hpp"
#include "web/WebApiLogic.hpp"

namespace settings {

inline AppSettings normalizeSettings(AppSettings settings) {
  if (settings.display_brightness < 10) settings.display_brightness = 10;
  if (settings.meth_selected_ratio_percent > 100) settings.meth_selected_ratio_percent = 100;
  if (settings.led_global_brightness < 10) settings.led_global_brightness = 180;
  for (uint8_t i = 0; i < 3; ++i) {
    if (settings.led_ch_mode[i] == 1) settings.led_ch_mode[i] = 10;
    else if (settings.led_ch_mode[i] != 0 &&
             settings.led_ch_mode[i] != 2 &&
             settings.led_ch_mode[i] != 3 &&
             settings.led_ch_mode[i] != 10 &&
             settings.led_ch_mode[i] != 11) {
      settings.led_ch_mode[i] = 0;
    }
    if (settings.led_ch_mode[i] == 0) {
      settings.led_ch_enabled[i] = false;
      settings.led_ch_brightness[i] = 0;
    } else {
      settings.led_ch_enabled[i] = true;
      if (settings.led_ch_brightness[i] < 10) settings.led_ch_brightness[i] = 180;
    }
  }
  settings.led_ch_enabled[0] = true;
  settings.led_ch_mode[0] = 9;
  settings.led_ch_brightness[0] = 180;
  settings.led_ch_color[0] = 0xFFFFFF;
  for (uint8_t i = 0; i < 4; ++i) {
    if (settings.led_zone_mode[i] != 0 &&
             settings.led_zone_mode[i] != 1 &&
             settings.led_zone_mode[i] != 2 &&
             settings.led_zone_mode[i] != 3 &&
             settings.led_zone_mode[i] != 4 &&
             settings.led_zone_mode[i] != 5 &&
             settings.led_zone_mode[i] != 10 &&
             settings.led_zone_mode[i] != 11) {
      settings.led_zone_mode[i] = 0;
    }
    if (settings.led_zone_mode[i] == 0) {
      settings.led_zone_enabled[i] = false;
      settings.led_zone_brightness[i] = 0;
    } else {
      settings.led_zone_enabled[i] = true;
      if (settings.led_zone_brightness[i] < 10) settings.led_zone_brightness[i] = 140;
    }
  }

  if (settings.knock_warning_threshold_count < 1) settings.knock_warning_threshold_count = 1;
  if (settings.knock_critical_threshold_count < settings.knock_warning_threshold_count) {
    settings.knock_critical_threshold_count = settings.knock_warning_threshold_count;
  }
  if (settings.knock_gain < 0.25f) settings.knock_gain = 0.25f;
  if (settings.knock_gain > 8.0f) settings.knock_gain = 8.0f;
  if (settings.knock_threshold_multiplier < 1.0f) settings.knock_threshold_multiplier = 1.0f;
  if (settings.knock_event_cooldown_ms < 50) settings.knock_event_cooldown_ms = 50;
  if (settings.knock_response_mode > 3) settings.knock_response_mode = 1;

  if (settings.race_min_satellites < 1) settings.race_min_satellites = 1;
  if (settings.race_min_satellites > 20) settings.race_min_satellites = 20;
  if (settings.race_sample_min_ms < 10) settings.race_sample_min_ms = 10;
  if (settings.race_sample_max_ms <= settings.race_sample_min_ms) {
    settings.race_sample_max_ms = settings.race_sample_min_ms + 50;
  }
  if (settings.race_start_finish_radius_m < 5.0f) settings.race_start_finish_radius_m = 5.0f;

  if (settings.analog_sensor_sample_ms < 10) settings.analog_sensor_sample_ms = 10;
  if (settings.analog_sensor_sample_ms > 1000) settings.analog_sensor_sample_ms = 1000;
  if (settings.thermistor_pullup_ohms < 1000.0f) settings.thermistor_pullup_ohms = 1000.0f;
  if (settings.thermistor_pullup_ohms > 100000.0f) settings.thermistor_pullup_ohms = 100000.0f;
  if (settings.pressure_sensor_min_v < 0.1f) settings.pressure_sensor_min_v = 0.1f;
  if (settings.pressure_sensor_max_v <= settings.pressure_sensor_min_v + 0.1f) {
    settings.pressure_sensor_max_v = settings.pressure_sensor_min_v + 0.1f;
  }
  if (settings.pressure_sensor_max_v > 5.0f) settings.pressure_sensor_max_v = 5.0f;
  if (settings.pressure_sensor_max_psi < 5.0f) settings.pressure_sensor_max_psi = 5.0f;

  return settings;
}

inline bool unsafeSettingsRequireConfirmation(const AppSettings& current, const AppSettings& proposed) {
  return proposed.knock_response_mode != current.knock_response_mode &&
         web::unsafeKnockResponseRequiresConfirmation(proposed.knock_response_mode);
}

}  // namespace settings
