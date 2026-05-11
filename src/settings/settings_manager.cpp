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
  settings_.meth_boost_trigger_kpa = prefs_.getUChar("meth_boost", settings_.meth_boost_trigger_kpa);
  settings_.meth_iat_safety_threshold = static_cast<int8_t>(prefs_.getChar("meth_iat", settings_.meth_iat_safety_threshold));
  settings_.meth_max_pump_duty = prefs_.getUChar("meth_duty", settings_.meth_max_pump_duty);
  settings_.meth_can_loss_behavior = prefs_.getUChar("meth_closs", settings_.meth_can_loss_behavior);

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
  s.meth_boost_trigger_kpa = settings_.meth_boost_trigger_kpa;
  s.meth_iat_safety_threshold = settings_.meth_iat_safety_threshold;
  s.meth_max_pump_duty = settings_.meth_max_pump_duty;
  s.meth_can_loss_behavior = static_cast<state::MethCanLossBehavior>(settings_.meth_can_loss_behavior);
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
  settings_.meth_boost_trigger_kpa = s.meth_boost_trigger_kpa;
  settings_.meth_iat_safety_threshold = s.meth_iat_safety_threshold;
  settings_.meth_max_pump_duty = s.meth_max_pump_duty;
  settings_.meth_can_loss_behavior = static_cast<uint8_t>(s.meth_can_loss_behavior);
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
  prefs_.putUChar("meth_boost", settings_.meth_boost_trigger_kpa);
  prefs_.putChar("meth_iat", settings_.meth_iat_safety_threshold);
  prefs_.putUChar("meth_duty", settings_.meth_max_pump_duty);
  prefs_.putUChar("meth_closs", settings_.meth_can_loss_behavior);

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
