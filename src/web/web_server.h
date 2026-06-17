#pragma once

#include <Arduino.h>
#ifndef CCM_WEB_ENABLED
#define CCM_WEB_ENABLED 0
#endif

#if CCM_WEB_ENABLED
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#endif

#include "can/can_manager.h"
#include "race/race_manager.h"
#include "settings/settings_manager.h"
#include "state/vehicle_state.h"

namespace web {

#if CCM_WEB_ENABLED
class WebServerManager {
 public:
  bool begin(state::VehicleStateStore* stateStore, settings::SettingsManager* settingsMgr, canbus::CanManager* canMgr, race::RacePerformanceManager* raceMgr);
  void tick();

 private:
  bool checkAuth(AsyncWebServerRequest* request) const;
  void sendState(AsyncWebServerRequest* request) const;
  void sendSettings(AsyncWebServerRequest* request) const;
  void sendDiagnostics(AsyncWebServerRequest* request) const;
  void sendCanStatus(AsyncWebServerRequest* request) const;
  String stateJson() const;
  String canStatusJson() const;

  AsyncWebServer server_{80};
  AsyncWebSocket ws_{"/ws"};

  state::VehicleStateStore* stateStore_ = nullptr;
  settings::SettingsManager* settingsMgr_ = nullptr;
  canbus::CanManager* canMgr_ = nullptr;
  race::RacePerformanceManager* raceMgr_ = nullptr;
  uint32_t lastWsPushMs_ = 0;
};
#else
class WebServerManager {
 public:
  bool begin(state::VehicleStateStore*, settings::SettingsManager*, canbus::CanManager*, race::RacePerformanceManager*) { return false; }
  void tick() {}
};
#endif

}  // namespace web
