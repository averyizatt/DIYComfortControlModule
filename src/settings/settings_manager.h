#pragma once

#include <Arduino.h>
#include <Preferences.h>

#include "settings/AppSettings.hpp"
#include "state/vehicle_state.h"

namespace settings {

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
