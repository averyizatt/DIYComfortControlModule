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
  enum class Page : uint8_t { DASH = 0, METH = 1, TAIL = 2, RACE = 3, DIAG = 4 };

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
  void drawTaillightCard(const state::VehicleState& s);
  void drawRaceCard(const state::VehicleState& s);
  void drawDiagnosticsCard(const state::VehicleState& s);
  void drawTabs();
  void drawButton(const Rect& r, const char* text, bool active);
  void setPage(Page page);
  void setActionFeedback(const char* text, uint32_t nowMs);
  uint8_t uiPageFor(Page page) const;
  Page pageFromUi(uint8_t uiPage) const;
  uint8_t nextMethRatio(uint8_t current) const;
  touch::TouchSample normalizeTouch(const touch::TouchSample& sample) const;

  canbus::CanManager* canMgr_ = nullptr;
  race::RacePerformanceManager* raceMgr_ = nullptr;
  settings::SettingsManager* settingsMgr_ = nullptr;

  bool online_ = false;
  bool touchActive_ = false;
  uint32_t lastTouchMs_ = 0;
  uint32_t lastRenderMs_ = 0;
  uint32_t actionFeedbackUntilMs_ = 0;
  char actionFeedback_[32]{};
  Page page_ = Page::DASH;

  static constexpr uint16_t kWidth = 320;
  static constexpr uint16_t kHeight = 480;
  static constexpr uint32_t kRenderIntervalMs = 100;
  static constexpr uint32_t kTouchDebounceMs = 180;
  static constexpr uint32_t kActionFeedbackMs = 900;

  Rect tabDashBtn_{8, 48, 56, 28};
  Rect tabMethBtn_{70, 48, 56, 28};
  Rect tabTailBtn_{132, 48, 56, 28};
  Rect tabRaceBtn_{194, 48, 56, 28};
  Rect tabDiagBtn_{256, 48, 56, 28};

  Rect methArmBtn_{12, 258, 144, 50};
  Rect methRatioBtn_{164, 258, 144, 50};
  Rect tailStockBtn_{12, 318, 144, 50};
  Rect tailSequentialBtn_{164, 318, 144, 50};
  Rect tailShowBtn_{12, 376, 144, 50};
  Rect tailDemoBtn_{164, 376, 144, 50};
  Rect raceStartAccelBtn_{12, 372, 144, 42};
  Rect raceStartLapBtn_{164, 372, 144, 42};
  Rect raceStopBtn_{12, 420, 144, 42};
  Rect raceResetBtn_{164, 420, 144, 42};
};

}  // namespace ui
