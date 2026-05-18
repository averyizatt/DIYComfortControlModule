#include "settings/settings_manager.h"

#include <cstring>

namespace settings {

namespace {
constexpr const char* kPrefsNs = "ccm_cfg";

template <size_t N>
void safeCopy(char (&dst)[N], const char* src) {
  if (!src) {
    dst[0] = '\0';
    return;
  }
  strncpy(dst, src, N - 1);
  dst[N - 1] = '\0';
}
}  // namespace

bool SettingsManager::begin() {
  if (!prefs_.begin(kPrefsNs, false)) return false;
  started_ = true;
  load();
  return true;
}

void SettingsManager::load() {
  if (!started_) return;

  settings_.display_brightness = prefs_.getUChar("disp_br", settings_.display_brightness);
  settings_.night_mode_enabled = prefs_.getBool("night", settings_.night_mode_enabled);
  settings_.tach_pulses_per_rev10 = prefs_.getUChar("tach_ppr10", settings_.tach_pulses_per_rev10);
  settings_.tach_scaling_mode = prefs_.getUChar("tach_scale", settings_.tach_scaling_mode);

  settings_.led_global_brightness = prefs_.getUChar("led_g_br", settings_.led_global_brightness);
  settings_.led_theme = prefs_.getUChar("led_theme", settings_.led_theme);

  for (int i = 0; i < 3; ++i) {
    const String enK = String("led") + i + "_en";
    const String modeK = String("led") + i + "_mode";
    const String brK = String("led") + i + "_br";
    const String colK = String("led") + i + "_col";
    settings_.led_ch_enabled[i] = prefs_.getBool(enK.c_str(), settings_.led_ch_enabled[i]);
    settings_.led_ch_mode[i] = prefs_.getUChar(modeK.c_str(), settings_.led_ch_mode[i]);
    settings_.led_ch_brightness[i] = prefs_.getUChar(brK.c_str(), settings_.led_ch_brightness[i]);
    settings_.led_ch_color[i] = prefs_.getULong(colK.c_str(), settings_.led_ch_color[i]);
  }

  settings_.meth_selected_ratio_percent = prefs_.getUChar("meth_ratio", settings_.meth_selected_ratio_percent);
  if (settings_.meth_selected_ratio_percent > 100) settings_.meth_selected_ratio_percent = 100;
  settings_.knock_enabled = prefs_.getBool("knock_en", settings_.knock_enabled);
  settings_.knock_adc_pin = prefs_.getUChar("knock_pin", settings_.knock_adc_pin);
  settings_.knock_boost_enable_kpa = prefs_.getFloat("knock_boost", settings_.knock_boost_enable_kpa);
  settings_.knock_rpm_enable_min = prefs_.getUShort("knock_rpm", settings_.knock_rpm_enable_min);
  settings_.knock_threshold_multiplier = prefs_.getFloat("knock_mult", settings_.knock_threshold_multiplier);
  settings_.knock_threshold_offset = prefs_.getFloat("knock_offs", settings_.knock_threshold_offset);
  settings_.knock_event_cooldown_ms = prefs_.getUShort("knock_cd", settings_.knock_event_cooldown_ms);
  settings_.knock_warning_threshold_count = prefs_.getUChar("knock_warn", settings_.knock_warning_threshold_count);
  settings_.knock_critical_threshold_count = prefs_.getUChar("knock_crit", settings_.knock_critical_threshold_count);
  settings_.knock_baseline_learning_enabled = prefs_.getBool("knock_bl", settings_.knock_baseline_learning_enabled);
  settings_.knock_demo_mode_enabled = prefs_.getBool("knock_demo", settings_.knock_demo_mode_enabled);
  settings_.knock_response_mode = prefs_.getUChar("knock_resp", settings_.knock_response_mode);
  // Keep thresholds ordered by enforcing warning >= 1 and critical >= warning.
  if (settings_.knock_warning_threshold_count < 1) settings_.knock_warning_threshold_count = 1;
  if (settings_.knock_critical_threshold_count < settings_.knock_warning_threshold_count) {
    settings_.knock_critical_threshold_count = settings_.knock_warning_threshold_count;
  }
  if (settings_.knock_threshold_multiplier < 1.0f) settings_.knock_threshold_multiplier = 1.0f;
  if (settings_.knock_event_cooldown_ms < 50) settings_.knock_event_cooldown_ms = 50;
  if (settings_.knock_response_mode > 3) settings_.knock_response_mode = 1;

  settings_.race_use_metric_targets = prefs_.getBool("race_metric", settings_.race_use_metric_targets);
  settings_.race_auto_start = prefs_.getBool("race_auto", settings_.race_auto_start);
  settings_.race_min_satellites = prefs_.getUChar("race_sat", settings_.race_min_satellites);
  if (settings_.race_min_satellites < 1) settings_.race_min_satellites = 1;
  if (settings_.race_min_satellites > 20) settings_.race_min_satellites = 20;
  settings_.race_sample_min_ms = prefs_.getUShort("race_smin", settings_.race_sample_min_ms);
  if (settings_.race_sample_min_ms < 10) settings_.race_sample_min_ms = 10;
  settings_.race_sample_max_ms = prefs_.getUShort("race_smax", settings_.race_sample_max_ms);
  if (settings_.race_sample_max_ms <= settings_.race_sample_min_ms)
    settings_.race_sample_max_ms = settings_.race_sample_min_ms + 50;
  settings_.race_start_finish_radius_m = prefs_.getFloat("race_rad", settings_.race_start_finish_radius_m);
  if (settings_.race_start_finish_radius_m < 5.0f) settings_.race_start_finish_radius_m = 5.0f;
  settings_.race_start_latitude = prefs_.getFloat("race_lat", settings_.race_start_latitude);
  settings_.race_start_longitude = prefs_.getFloat("race_lon", settings_.race_start_longitude);
  settings_.race_start_point_set = prefs_.getBool("race_sf_set", settings_.race_start_point_set);

  if (settings_.display_brightness < 10) settings_.display_brightness = 10;

  settings_.wifi_ap_mode = prefs_.getBool("wifi_ap", settings_.wifi_ap_mode);
  const String ssid = prefs_.getString("wifi_ssid", "");
  const String pass = prefs_.getString("wifi_pass", "");
  const String webpass = prefs_.getString("web_pass", "");
  safeCopy(settings_.wifi_ssid, ssid.c_str());
  safeCopy(settings_.wifi_password, pass.c_str());
  safeCopy(settings_.web_password, webpass.c_str());
}

void SettingsManager::loadIntoState(state::VehicleState& s) const {
  s.display_brightness = settings_.display_brightness;
  s.night_mode_enabled = settings_.night_mode_enabled;
  s.pulses_per_rev10 = settings_.tach_pulses_per_rev10;
  s.tach_scaling_mode = settings_.tach_scaling_mode;

  s.led_channel_1_enabled = settings_.led_ch_enabled[0];
  s.led_channel_2_enabled = settings_.led_ch_enabled[1];
  s.led_channel_3_enabled = settings_.led_ch_enabled[2];
  s.led_channel_1_color = settings_.led_ch_color[0];
  s.led_channel_2_color = settings_.led_ch_color[1];
  s.led_channel_3_color = settings_.led_ch_color[2];
  s.led_channel_1_mode = static_cast<state::LedMode>(settings_.led_ch_mode[0]);
  s.led_channel_2_mode = static_cast<state::LedMode>(settings_.led_ch_mode[1]);
  s.led_channel_3_mode = static_cast<state::LedMode>(settings_.led_ch_mode[2]);
  s.led_channel_1_brightness = settings_.led_ch_brightness[0];
  s.led_channel_2_brightness = settings_.led_ch_brightness[1];
  s.led_channel_3_brightness = settings_.led_ch_brightness[2];
  s.led_global_brightness = settings_.led_global_brightness;
  s.led_theme = settings_.led_theme;

  s.meth_selected_ratio_percent = settings_.meth_selected_ratio_percent;
  s.knock_enabled = settings_.knock_enabled;
  s.knock_adc_pin = settings_.knock_adc_pin;
  s.knock_boost_enable_kpa = settings_.knock_boost_enable_kpa;
  s.knock_rpm_enable_min = settings_.knock_rpm_enable_min;
  s.knock_threshold_multiplier = settings_.knock_threshold_multiplier;
  s.knock_threshold_offset = settings_.knock_threshold_offset;
  s.knock_event_cooldown_ms = settings_.knock_event_cooldown_ms;
  s.knock_warning_threshold_count = settings_.knock_warning_threshold_count;
  s.knock_critical_threshold_count = settings_.knock_critical_threshold_count;
  s.knock_baseline_learning_enabled = settings_.knock_baseline_learning_enabled;
  s.knock_demo_mode_enabled = settings_.knock_demo_mode_enabled;
  s.knock_response_mode = settings_.knock_response_mode;
  s.race_use_metric_targets = settings_.race_use_metric_targets;
  s.race_auto_start = settings_.race_auto_start;
  s.race_min_satellites = settings_.race_min_satellites;
  s.race_sample_min_ms = settings_.race_sample_min_ms;
  s.race_sample_max_ms = settings_.race_sample_max_ms;
  s.race_start_finish_radius_m = settings_.race_start_finish_radius_m;
  s.race_start_latitude = settings_.race_start_latitude;
  s.race_start_longitude = settings_.race_start_longitude;
  s.race_start_point_set = settings_.race_start_point_set;
  s.wifi_ap_mode = settings_.wifi_ap_mode;
}

void SettingsManager::updateFromState(const state::VehicleState& s) {
  settings_.display_brightness = s.display_brightness;
  settings_.night_mode_enabled = s.night_mode_enabled;
  settings_.tach_pulses_per_rev10 = s.pulses_per_rev10;
  settings_.tach_scaling_mode = s.tach_scaling_mode;

  settings_.led_ch_enabled[0] = s.led_channel_1_enabled;
  settings_.led_ch_enabled[1] = s.led_channel_2_enabled;
  settings_.led_ch_enabled[2] = s.led_channel_3_enabled;
  settings_.led_ch_color[0] = s.led_channel_1_color;
  settings_.led_ch_color[1] = s.led_channel_2_color;
  settings_.led_ch_color[2] = s.led_channel_3_color;
  settings_.led_ch_mode[0] = static_cast<uint8_t>(s.led_channel_1_mode);
  settings_.led_ch_mode[1] = static_cast<uint8_t>(s.led_channel_2_mode);
  settings_.led_ch_mode[2] = static_cast<uint8_t>(s.led_channel_3_mode);
  settings_.led_ch_brightness[0] = s.led_channel_1_brightness;
  settings_.led_ch_brightness[1] = s.led_channel_2_brightness;
  settings_.led_ch_brightness[2] = s.led_channel_3_brightness;
  settings_.led_global_brightness = s.led_global_brightness;
  settings_.led_theme = s.led_theme;

  settings_.meth_selected_ratio_percent = s.meth_selected_ratio_percent;
  settings_.knock_enabled = s.knock_enabled;
  settings_.knock_adc_pin = s.knock_adc_pin;
  settings_.knock_boost_enable_kpa = s.knock_boost_enable_kpa;
  settings_.knock_rpm_enable_min = s.knock_rpm_enable_min;
  settings_.knock_threshold_multiplier = s.knock_threshold_multiplier;
  settings_.knock_threshold_offset = s.knock_threshold_offset;
  settings_.knock_event_cooldown_ms = s.knock_event_cooldown_ms;
  settings_.knock_warning_threshold_count = s.knock_warning_threshold_count;
  settings_.knock_critical_threshold_count = s.knock_critical_threshold_count;
  settings_.knock_baseline_learning_enabled = s.knock_baseline_learning_enabled;
  settings_.knock_demo_mode_enabled = s.knock_demo_mode_enabled;
  settings_.knock_response_mode = s.knock_response_mode;
  settings_.race_use_metric_targets = s.race_use_metric_targets;
  settings_.race_auto_start = s.race_auto_start;
  settings_.race_min_satellites = s.race_min_satellites;
  settings_.race_sample_min_ms = s.race_sample_min_ms;
  settings_.race_sample_max_ms = s.race_sample_max_ms;
  settings_.race_start_finish_radius_m = s.race_start_finish_radius_m;
  settings_.race_start_latitude = s.race_start_latitude;
  settings_.race_start_longitude = s.race_start_longitude;
  settings_.race_start_point_set = s.race_start_point_set;
  settings_.wifi_ap_mode = s.wifi_ap_mode;
}

bool SettingsManager::save() {
  if (!started_) return false;

  prefs_.putUChar("disp_br", settings_.display_brightness);
  prefs_.putBool("night", settings_.night_mode_enabled);
  prefs_.putUChar("tach_ppr10", settings_.tach_pulses_per_rev10);
  prefs_.putUChar("tach_scale", settings_.tach_scaling_mode);

  prefs_.putUChar("led_g_br", settings_.led_global_brightness);
  prefs_.putUChar("led_theme", settings_.led_theme);
  for (int i = 0; i < 3; ++i) {
    const String enK = String("led") + i + "_en";
    const String modeK = String("led") + i + "_mode";
    const String brK = String("led") + i + "_br";
    const String colK = String("led") + i + "_col";
    prefs_.putBool(enK.c_str(), settings_.led_ch_enabled[i]);
    prefs_.putUChar(modeK.c_str(), settings_.led_ch_mode[i]);
    prefs_.putUChar(brK.c_str(), settings_.led_ch_brightness[i]);
    prefs_.putULong(colK.c_str(), settings_.led_ch_color[i]);
  }

  prefs_.putUChar("meth_ratio", settings_.meth_selected_ratio_percent);
  prefs_.putBool("knock_en", settings_.knock_enabled);
  prefs_.putUChar("knock_pin", settings_.knock_adc_pin);
  prefs_.putFloat("knock_boost", settings_.knock_boost_enable_kpa);
  prefs_.putUShort("knock_rpm", settings_.knock_rpm_enable_min);
  prefs_.putFloat("knock_mult", settings_.knock_threshold_multiplier);
  prefs_.putFloat("knock_offs", settings_.knock_threshold_offset);
  prefs_.putUShort("knock_cd", settings_.knock_event_cooldown_ms);
  prefs_.putUChar("knock_warn", settings_.knock_warning_threshold_count);
  prefs_.putUChar("knock_crit", settings_.knock_critical_threshold_count);
  prefs_.putBool("knock_bl", settings_.knock_baseline_learning_enabled);
  prefs_.putBool("knock_demo", settings_.knock_demo_mode_enabled);
  prefs_.putUChar("knock_resp", settings_.knock_response_mode);
  prefs_.putBool("race_metric", settings_.race_use_metric_targets);
  prefs_.putBool("race_auto", settings_.race_auto_start);
  prefs_.putUChar("race_sat", settings_.race_min_satellites);
  prefs_.putUShort("race_smin", settings_.race_sample_min_ms);
  prefs_.putUShort("race_smax", settings_.race_sample_max_ms);
  prefs_.putFloat("race_rad", settings_.race_start_finish_radius_m);
  prefs_.putFloat("race_lat", settings_.race_start_latitude);
  prefs_.putFloat("race_lon", settings_.race_start_longitude);
  prefs_.putBool("race_sf_set", settings_.race_start_point_set);

  prefs_.putBool("wifi_ap", settings_.wifi_ap_mode);
  prefs_.putString("wifi_ssid", settings_.wifi_ssid);
  prefs_.putString("wifi_pass", settings_.wifi_password);
  prefs_.putString("web_pass", settings_.web_password);
  return true;
}

bool SettingsManager::setWifiCredentials(const char* ssid, const char* password, bool apMode) {
  safeCopy(settings_.wifi_ssid, ssid);
  safeCopy(settings_.wifi_password, password);
  settings_.wifi_ap_mode = apMode;
  return save();
}

bool SettingsManager::setWebPassword(const char* password) {
  safeCopy(settings_.web_password, password);
  return save();
}

}  // namespace settings
