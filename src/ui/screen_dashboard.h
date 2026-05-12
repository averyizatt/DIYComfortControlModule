#pragma once

#include <Arduino.h>

#include "can/can_manager.h"
#include "race/race_manager.h"
#include "settings/settings_manager.h"
#include "state/vehicle_state.h"
#include "touch/touch_manager.h"

namespace ui {

class ScreenDashboard {
 public:
  bool begin(uint8_t lcdCs, uint8_t lcdRst, uint8_t lcdDc, uint8_t spiSck, uint8_t spiMosi, uint8_t spiMiso);
  void tick(const state::VehicleState& s, uint32_t nowMs);
  void handleTouch(const touch::TouchSample& sample, uint32_t nowMs);

  void attach(canbus::CanManager* canMgr, race::RacePerformanceManager* raceMgr, settings::SettingsManager* settingsMgr);
  bool online() const { return online_; }

 private:
  struct Rect {
    int16_t x = 0;
    int16_t y = 0;
    int16_t w = 0;
    int16_t h = 0;
    bool contains(uint16_t px, uint16_t py) const {
      return px >= static_cast<uint16_t>(x) && py >= static_cast<uint16_t>(y) && px < static_cast<uint16_t>(x + w) &&
             py < static_cast<uint16_t>(y + h);
    }
  };

  void render(const state::VehicleState& s);
  void drawHeader(const state::VehicleState& s);
  void drawLiveCard(const state::VehicleState& s);
  void drawStatusCard(const state::VehicleState& s);
  void drawControlCard(const state::VehicleState& s);
  void drawRaceCard(const state::VehicleState& s);
  void drawButton(const Rect& r, const char* text, bool active);
  uint8_t nextMethRatio(uint8_t current) const;
  touch::TouchSample normalizeTouch(const touch::TouchSample& sample) const;

  canbus::CanManager* canMgr_ = nullptr;
  race::RacePerformanceManager* raceMgr_ = nullptr;
  settings::SettingsManager* settingsMgr_ = nullptr;

  bool online_ = false;
  bool touchActive_ = false;
  uint32_t lastTouchMs_ = 0;
  uint32_t lastRenderMs_ = 0;

  static constexpr uint16_t kWidth = 320;
  static constexpr uint16_t kHeight = 480;
  static constexpr uint32_t kRenderIntervalMs = 100;
  static constexpr uint32_t kTouchDebounceMs = 180;

  Rect methArmBtn_{12, 258, 144, 42};
  Rect methRatioBtn_{164, 258, 144, 42};
  Rect raceStartAccelBtn_{12, 372, 72, 42};
  Rect raceStartLapBtn_{90, 372, 72, 42};
  Rect raceStopBtn_{168, 372, 72, 42};
  Rect raceResetBtn_{246, 372, 62, 42};
};

}  // namespace ui
