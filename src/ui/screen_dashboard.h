#pragma once

#include <Arduino.h>
#include <lvgl.h>

#include "can/can_manager.h"
#include "race/race_manager.h"
#include "settings/settings_manager.h"
#include "state/vehicle_state.h"
#include "storage/sd_manager.h"
#include "touch/touch_manager.h"

namespace ui {

class ScreenDashboard {
 public:
  bool begin(uint8_t lcdCs, uint8_t lcdRst, uint8_t lcdDc,
             uint8_t spiSck, uint8_t spiMosi, uint8_t spiMiso);
  void tick(const state::VehicleState& s, uint32_t nowMs);
  void handleTouch(const touch::TouchSample& sample, uint32_t nowMs);
  void attach(canbus::CanManager* canMgr, race::RacePerformanceManager* raceMgr,
              settings::SettingsManager* settingsMgr, storage::SdManager* sdMgr);
  bool online() const { return online_; }

 private:
  // ---- Layout constants -----------------------------------------------
  static constexpr uint16_t kWidth    = 480;
  static constexpr uint16_t kHeight   = 320;
  static constexpr uint16_t kHdrH     = 36;
  static constexpr uint16_t kNavH     = 52;
  static constexpr uint16_t kContentH = kHeight - kHdrH - kNavH;  // 232
  static constexpr uint16_t kArcSize  = 180;
  static constexpr uint8_t  kPageCount = 8;
  static constexpr uint32_t kActionFeedbackMs             = 1200;
  static constexpr uint8_t  kTaillightShowOptionsPerPage  = 6;
  static constexpr uint8_t  kTaillightShowOptionCount     = 33;
  static constexpr uint8_t  kTaillightShowPageCount       =
      static_cast<uint8_t>((kTaillightShowOptionCount + kTaillightShowOptionsPerPage - 1U)
                           / kTaillightShowOptionsPerPage);

  // ---- UI construction ------------------------------------------------
  void buildUi();
  void buildHeader(lv_obj_t* scr);
  void buildContentArea(lv_obj_t* scr);
  void buildNavBar(lv_obj_t* scr);
  void buildDashPage(lv_obj_t* parent);
  void buildMethPage(lv_obj_t* parent);
  void buildTailPage(lv_obj_t* parent);
  void buildLedsPage(lv_obj_t* parent);
  void buildGpsPage(lv_obj_t* parent);
  void buildTempsPage(lv_obj_t* parent);
  void buildDiagPage(lv_obj_t* parent);
  void buildKnockPage(lv_obj_t* parent);
  void buildSdBrowser(lv_obj_t* parent);
  void showPage(uint8_t idx);

  // ---- Per-tick updates -----------------------------------------------
  void updateHeader(const state::VehicleState& s, uint32_t nowMs);
  void updateDashPage(const state::VehicleState& s);
  void updateMethPage(const state::VehicleState& s, uint32_t nowMs);
  void updateTailPage(const state::VehicleState& s);
  void updateLedsPage(const state::VehicleState& s);
  void updateGpsPage(const state::VehicleState& s);
  void updateTempsPage(const state::VehicleState& s);
  void updateDiagPage(const state::VehicleState& s);
  void updateKnockPage(const state::VehicleState& s, uint32_t nowMs);
  void updateActivePage(const state::VehicleState& s, uint32_t nowMs);
  void forceContentRepaint();

  // ---- Helpers --------------------------------------------------------
  void    setActionFeedback(const char* text, uint32_t nowMs);
  uint8_t nextMethRatio(uint8_t current) const;
  touch::TouchSample normalizeRaw(const touch::TouchSample& raw) const;
  void refreshSdBrowser(uint32_t nowMs, bool force);
  bool enterSdDirectory(const char* name, uint32_t nowMs);
  void setSdPathRoot();
  void setSdPathParent();

  // ---- LVGL callbacks -------------------------------------------------
  static void lvglFlushCb(lv_disp_drv_t* drv, const lv_area_t* area, lv_color_t* colors);
  static void lvglRounderCb(lv_disp_drv_t* drv, lv_area_t* area);
  static void lvglTouchReadCb(lv_indev_drv_t* drv, lv_indev_data_t* data);

  // ---- Event handlers -------------------------------------------------
  static void onNavClicked(lv_event_t* e);
  static void onLedModeClicked(lv_event_t* e);
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
  static void onKnockEnableClicked(lv_event_t* e);
  static void onKnockResetBaselineClicked(lv_event_t* e);
  static void onKnockClearEventsClicked(lv_event_t* e);
  static void onKnockSimulateClicked(lv_event_t* e);
  static void onLedMasterSwitchChanged(lv_event_t* e);
  static void onBenchTestClicked(lv_event_t* e);
  static void onSdFileRowClicked(lv_event_t* e);
  static void onSdUpClicked(lv_event_t* e);
  static void onSdPrevClicked(lv_event_t* e);
  static void onSdNextClicked(lv_event_t* e);
  static void onSdTestClicked(lv_event_t* e);

  // ---- Dependencies ---------------------------------------------------
  canbus::CanManager*           canMgr_      = nullptr;
  race::RacePerformanceManager* raceMgr_     = nullptr;
  settings::SettingsManager*    settingsMgr_ = nullptr;
  storage::SdManager*           sdMgr_       = nullptr;

  bool     online_                = false;
  uint32_t actionFeedbackUntilMs_ = 0;
  char     actionFeedback_[48]    = {};
  uint8_t  activePage_            = 0;
  bool     pageSwitchPending_      = false;
  uint32_t lastStressPageSwitchMs_ = 0;
  uint32_t lastHeaderUpdateMs_    = 0;
  uint32_t lastPageUpdateMs_      = 0;
#if !LV_TICK_CUSTOM
  uint32_t lastLvTickMs_          = 0;
#endif
  char     hdrBatText_[12]        = {};
  char     rpmText_[12]           = {};
  char     spdText_[12]           = {};
  char     boostText_[24]         = {};
  char     dashEnvText_[40]       = {};
  char     dashStatusText_[64]    = {};
  char     dashRaceText_[40]      = {};
  char     gLiveText_[16]         = {};
  char     gPeakText_[16]         = {};
  int32_t  dashRpmArcLast_        = -999999;
  int32_t  dashSpdArcLast_        = -999999;
  int32_t  dashBoostBarLast_      = -999999;
  int32_t  dashLatBarLast_        = -999999;
  int32_t  methDutyMeterLast_     = -999999;
  int32_t  gpsSatMeterLast_       = -999999;

  touch::TouchSample lastTouch_ = {};
  portMUX_TYPE        touchMux_  = portMUX_INITIALIZER_UNLOCKED;  // guards lastTouch_ (touchTask writes, screenTask reads)

  // ---- LVGL widget pointers -------------------------------------------

  // Header
  lv_obj_t* hdrBatLabel_      = nullptr;  // left:   "12.6V"
  lv_obj_t* hdrTitleLabel_    = nullptr;  // center: page name (cyan)
  lv_obj_t* hdrFaultDot_      = nullptr;  // right:  status indicator
  lv_obj_t* hdrFeedbackLabel_ = nullptr;  // far-right: action feedback

  // Bottom nav bar (0=DASH 1=METH 2=TAIL 3=LEDS 4=GPS 5=TEMPS 6=DIAG 7=KNOCK)
  lv_obj_t* navBtns_[8]     = {};
  lv_obj_t* navBtnIcons_[8] = {};
  lv_obj_t* bottomEdgeGuard_ = nullptr;

  // Page content panels (one visible at a time)
  lv_obj_t* contentArea_ = nullptr;
  lv_obj_t* pages_[8] = {};

  // -- DASH page --
  lv_obj_t* rpmArc_          = nullptr;  // unused on the simplified DASH page
  lv_obj_t* spdArc_          = nullptr;  // main MPH gauge
  lv_obj_t* rpmValLabel_     = nullptr;  // large RPM number
  lv_obj_t* spdValLabel_     = nullptr;  // large speed number
  lv_obj_t* boostBar_        = nullptr;
  lv_obj_t* boostValLabel_   = nullptr;
  lv_obj_t* dashEnvLabel_    = nullptr;  // meth duty / tank
  lv_obj_t* dashStatusLabel_ = nullptr;  // meth activation state
  lv_obj_t* dashRaceLabel_   = nullptr;  // compact voltage / GPS / IAT row
  lv_obj_t* raceAccelBtn_    = nullptr;
  lv_obj_t* raceLapBtn_      = nullptr;
  lv_obj_t* raceStopBtn_     = nullptr;
  lv_obj_t* raceResetBtn_    = nullptr;
  lv_obj_t* gLiveLabel_      = nullptr;  // "X.XX G" current G
  lv_obj_t* gPeakLabel_      = nullptr;  // "PK X.XX" peak G
  lv_obj_t* gLatBar_         = nullptr;  // lateral G bar (-100..+100)

  // -- METH page --
  lv_obj_t* methBadgeLabel_    = nullptr;
  lv_obj_t* methStateLabel_    = nullptr;
  lv_obj_t* methSensorLabel_   = nullptr;
  lv_obj_t* methParamLabel_    = nullptr;
  lv_obj_t* methOnlineLed_     = nullptr;
  lv_obj_t* methOfflineSpinner_ = nullptr;
  lv_obj_t* methDutyMeter_     = nullptr;
  lv_meter_indicator_t* methDutyNeedle_ = nullptr;
  lv_obj_t* methTankMeter_     = nullptr;
  lv_meter_indicator_t* methTankArc_ = nullptr;
  lv_obj_t* methFlowDropdown_  = nullptr;
  lv_obj_t* methArmBtn_        = nullptr;
  lv_obj_t* methArmBtnLabel_   = nullptr;
  lv_obj_t* methRatioBtn_      = nullptr;
  lv_obj_t* methRatioBtnLabel_ = nullptr;

  // -- TAIL page --
  lv_obj_t* tailStatusLabel_   = nullptr;
  lv_obj_t* tailOnlineLed_     = nullptr;
  lv_obj_t* tailModeDropdown_  = nullptr;
  lv_obj_t* tailModePanel_     = nullptr;
  lv_obj_t* tailStockBtn_      = nullptr;
  lv_obj_t* tailSeqBtn_        = nullptr;
  lv_obj_t* tailShowMenuBtn_   = nullptr;
  lv_obj_t* tailDemoBtn_       = nullptr;
  lv_obj_t* tailShowPanel_     = nullptr;
  lv_obj_t* tailShowPageLabel_ = nullptr;
  lv_obj_t* tailShowPrevBtn_   = nullptr;
  lv_obj_t* tailShowNextBtn_   = nullptr;
  lv_obj_t* tailShowBackBtn_   = nullptr;
  lv_obj_t* tailShowOptBtns_[6] = {};
  uint8_t   tailShowPage_      = 0;

  // -- LEDS page --
  lv_obj_t* ledStatusLabel_  = nullptr;
  lv_obj_t* ledMasterSwitch_ = nullptr;
  lv_obj_t* ledMasterLabel_  = nullptr;
  lv_obj_t* ledModeBtns_[5]  = {};  // OFF, STATIC, BREATHE, RAINBOW, RPM

  // -- GPS page --
  lv_obj_t* gpsSpdLabel_  = nullptr;
  lv_obj_t* gpsInfoLabel_ = nullptr;
  lv_obj_t* gpsFixLed_    = nullptr;
  lv_obj_t* gpsSatMeter_  = nullptr;
  lv_meter_indicator_t* gpsSatNeedle_ = nullptr;
  float     gpsSpeedFilteredMph_      = 0.0f;
  bool      gpsSpeedFilterInitialized_ = false;

  // -- TEMPS page --
  lv_obj_t* tempsLabel_ = nullptr;
  lv_obj_t* tempsTable_ = nullptr;

  // -- DIAG page --
  lv_obj_t* diagLabel_     = nullptr;
  lv_obj_t* benchTestBtn_  = nullptr;
  static constexpr uint8_t kSdFileRowCount = 5;
  lv_obj_t* sdPathLabel_ = nullptr;
  lv_obj_t* sdFileRows_[kSdFileRowCount] = {};
  lv_obj_t* sdFileRowLabels_[kSdFileRowCount] = {};
  lv_obj_t* sdUpBtn_ = nullptr;
  lv_obj_t* sdPrevBtn_ = nullptr;
  lv_obj_t* sdNextBtn_ = nullptr;
  lv_obj_t* sdTestBtn_ = nullptr;
  storage::SdFileEntry sdEntries_[kSdFileRowCount] = {};
  uint8_t sdEntryCount_ = 0;
  uint16_t sdTotalEntries_ = 0;
  uint16_t sdListOffset_ = 0;
  uint32_t sdBrowserLastRefreshMs_ = 0;
  char sdCurrentPath_[64] = "/";

  // -- KNOCK page --
  lv_obj_t* knockStateLabel_       = nullptr;
  lv_obj_t* knockSensorLabel_      = nullptr;
  lv_obj_t* knockEnergyLabel_      = nullptr;
  lv_obj_t* knockEnergyBar_        = nullptr;
  lv_obj_t* knockBaselineLabel_    = nullptr;
  lv_obj_t* knockBaselineBar_      = nullptr;
  lv_obj_t* knockThresholdLabel_   = nullptr;
  lv_obj_t* knockThresholdBar_     = nullptr;
  lv_obj_t* knockGraphLabel_       = nullptr;
  lv_obj_t* knockGraphChart_       = nullptr;
  lv_chart_series_t* knockGraphEnergySeries_ = nullptr;
  lv_chart_series_t* knockGraphBaselineSeries_ = nullptr;
  lv_chart_series_t* knockGraphThresholdSeries_ = nullptr;
  lv_obj_t* knockWarnLed_          = nullptr;
  lv_obj_t* knockCritLed_          = nullptr;
  lv_obj_t* knockLearningSpinner_  = nullptr;
  lv_obj_t* knockEventLabel_       = nullptr;
  lv_obj_t* knockLastLabel_        = nullptr;
  lv_obj_t* knockEnableBtn_        = nullptr;
  lv_obj_t* knockEnableBtnLabel_   = nullptr;
  lv_obj_t* knockResetBlBtn_       = nullptr;
  lv_obj_t* knockClearEvtBtn_      = nullptr;
  lv_obj_t* knockSimulateBtn_      = nullptr;
  lv_obj_t* knockLogLabel_         = nullptr;
};

}  // namespace ui
