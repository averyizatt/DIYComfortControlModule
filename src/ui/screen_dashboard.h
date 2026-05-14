#pragma once

#include <Arduino.h>
#include <lvgl.h>

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
  /// Feeds a raw touch sample into the LVGL input device. Called from the touch task.
  void handleTouch(const touch::TouchSample& sample, uint32_t nowMs);

  void attach(canbus::CanManager* canMgr, race::RacePerformanceManager* raceMgr, settings::SettingsManager* settingsMgr);
  bool online() const { return online_; }

 private:
  // ---- UI construction ------------------------------------------------
  void buildUi();
  void buildDashTab(lv_obj_t* parent);
  void buildMethTab(lv_obj_t* parent);
  void buildTailTab(lv_obj_t* parent);
  void buildRaceTab(lv_obj_t* parent);
  void buildDiagTab(lv_obj_t* parent);

  // ---- Per-tick update ------------------------------------------------
  void updateHeader(const state::VehicleState& s, uint32_t nowMs);
  void updateDashTab(const state::VehicleState& s);
  void updateMethTab(const state::VehicleState& s, uint32_t nowMs);
  void updateTailTab(const state::VehicleState& s);
  void updateRaceTab(const state::VehicleState& s);
  void updateDiagTab(const state::VehicleState& s);

  // ---- Helpers --------------------------------------------------------
  void    setActionFeedback(const char* text, uint32_t nowMs);
  uint8_t nextMethRatio(uint8_t current) const;
  touch::TouchSample normalizeRaw(const touch::TouchSample& raw) const;

  // ---- LVGL callbacks -------------------------------------------------
  static void lvglFlushCb(lv_disp_drv_t* drv, const lv_area_t* area, lv_color_t* colors);
  static void lvglTouchReadCb(lv_indev_drv_t* drv, lv_indev_data_t* data);

  // ---- Button event handlers -----------------------------------------
  static void onMethArmClicked(lv_event_t* e);
  static void onMethRatioClicked(lv_event_t* e);
  static void onTailStockClicked(lv_event_t* e);
  static void onTailSeqClicked(lv_event_t* e);
  static void onTailShowMenuClicked(lv_event_t* e);
  static void onTailDemoClicked(lv_event_t* e);
  static void onTailShowPrevClicked(lv_event_t* e);
  static void onTailShowNextClicked(lv_event_t* e);
  static void onTailShowBackClicked(lv_event_t* e);
  static void onTailShowOptClicked(lv_event_t* e);
  static void onRaceAccelClicked(lv_event_t* e);
  static void onRaceLapClicked(lv_event_t* e);
  static void onRaceStopClicked(lv_event_t* e);
  static void onRaceResetClicked(lv_event_t* e);

  // ---- Dependencies ---------------------------------------------------
  canbus::CanManager*           canMgr_      = nullptr;
  race::RacePerformanceManager* raceMgr_     = nullptr;
  settings::SettingsManager*    settingsMgr_ = nullptr;

  bool     online_                = false;
  uint32_t actionFeedbackUntilMs_ = 0;
  char     actionFeedback_[48]    = {};

  /// Latest normalized touch state; written by handleTouch(), read by lvglTouchReadCb().
  touch::TouchSample lastTouch_ = {};

  // ---- LVGL widget pointers (created once in buildUi) -----------------

  // Header
  lv_obj_t* titleLabel_    = nullptr;
  lv_obj_t* feedbackLabel_ = nullptr;

  // Tab view
  lv_obj_t* tabview_ = nullptr;

  // DASH tab
  lv_obj_t* dashLiveLabel_   = nullptr;  // RPM / speed / boost / IAT
  lv_obj_t* dashEnvLabel_    = nullptr;  // battery / cabin / outside / GPS
  lv_obj_t* dashStatusLabel_ = nullptr;  // CAN / METH / GPS online, faults
  lv_obj_t* dashRaceLabel_   = nullptr;  // quick race stats

  // METH tab
  lv_obj_t* methStateLabel_    = nullptr;
  lv_obj_t* methSensorLabel_   = nullptr;
  lv_obj_t* methParamLabel_    = nullptr;
  lv_obj_t* methArmBtn_        = nullptr;
  lv_obj_t* methArmBtnLabel_   = nullptr;
  lv_obj_t* methRatioBtn_      = nullptr;
  lv_obj_t* methRatioBtnLabel_ = nullptr;

  // TAIL tab
  lv_obj_t* tailStatusLabel_   = nullptr;
  lv_obj_t* tailModePanel_     = nullptr;  // holds mode buttons; hidden when show submenu active
  lv_obj_t* tailStockBtn_      = nullptr;
  lv_obj_t* tailSeqBtn_        = nullptr;
  lv_obj_t* tailShowMenuBtn_   = nullptr;
  lv_obj_t* tailDemoBtn_       = nullptr;
  lv_obj_t* tailShowPanel_     = nullptr;  // show submenu; hidden by default
  lv_obj_t* tailShowPageLabel_ = nullptr;
  lv_obj_t* tailShowPrevBtn_   = nullptr;
  lv_obj_t* tailShowNextBtn_   = nullptr;
  lv_obj_t* tailShowBackBtn_   = nullptr;
  lv_obj_t* tailShowOptBtns_[6] = {};
  uint8_t   tailShowPage_      = 0;

  // RACE tab
  lv_obj_t* raceLiveLabel_     = nullptr;
  lv_obj_t* raceStatsLabel_    = nullptr;
  lv_obj_t* raceAccelBtn_      = nullptr;
  lv_obj_t* raceLapBtn_        = nullptr;
  lv_obj_t* raceStopBtn_       = nullptr;
  lv_obj_t* raceResetBtn_      = nullptr;

  // DIAG tab
  lv_obj_t* diagLabel_ = nullptr;

  // ---- Constants ------------------------------------------------------
  static constexpr uint16_t kWidth  = 320;
  static constexpr uint16_t kHeight = 480;
  static constexpr uint16_t kTabBarH = 36;   // LVGL tabview tab bar height
  static constexpr uint16_t kHdrH   = 38;    // custom header height
  static constexpr uint32_t kActionFeedbackMs          = 1200;
  static constexpr uint8_t  kTaillightShowOptionsPerPage = 6;
  static constexpr uint8_t  kTaillightShowOptionCount    = 24;
  static constexpr uint8_t  kTaillightShowPageCount      =
      static_cast<uint8_t>((kTaillightShowOptionCount + kTaillightShowOptionsPerPage - 1U) / kTaillightShowOptionsPerPage);
};

}  // namespace ui
