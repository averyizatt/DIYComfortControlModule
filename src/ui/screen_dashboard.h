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
  void attach(canbus::CanManager* canMgr, race::RacePerformanceManager* raceMgr,
              settings::SettingsManager* settingsMgr, storage::SdManager* sdMgr,
              touch::TouchManager* touchMgr);
  bool online() const { return online_; }

 private:
  // ---- Layout constants -----------------------------------------------
  static constexpr uint16_t kWidth    = 480;
  static constexpr uint16_t kHeight   = 320;
  static constexpr uint16_t kHdrH     = 44;
  static constexpr uint16_t kNavH     = 52;
  static constexpr uint16_t kContentH = kHeight - kHdrH - kNavH;
  static constexpr uint16_t kArcSize  = 180;
  static constexpr uint8_t  kPageCount = 8;
  static constexpr uint32_t kActionFeedbackMs             = 1200;
  static constexpr uint8_t  kTaillightShowOptionsPerPage  = 6;
  static constexpr uint8_t  kTaillightShowOptionCount     = 33;
  static constexpr uint8_t  kTaillightShowPageCount       =
      static_cast<uint8_t>((kTaillightShowOptionCount + kTaillightShowOptionsPerPage - 1U)
                           / kTaillightShowOptionsPerPage);
  static constexpr uint8_t  kUiActionQueueSize = 16;
  static constexpr uint8_t  kTouchCalPointCount = 9;

  enum class UiActionType : uint8_t {
    None,
    Nav,
    MethArm,
    MethRatio,
    TailMode,
    TailShowMenu,
    TailShowPrev,
    TailShowNext,
    TailShowBack,
    TailShowOption,
    LedShowMenu,
    LedShowBack,
    LedShowMode,
    LedShowOn,
    LedShowOff,
    LedShowClear,
    LedColor,
    LedMode,
    LedMaster,
    KnockEnable,
    KnockResetBaseline,
    KnockGainDown,
    KnockGainUp,
    KnockMultiplierDown,
    KnockMultiplierUp,
    BenchTest,
    DiagInfo,
    DiagTools,
    DiagStorage,
    DiagTrends,
    ThemeProfile,
    FaultPage,
    FaultClose,
    LedOutputTest,
    CanPing,
    Restart,
    RaceStartAccel,
    RaceStartLap,
    RaceStop,
    RaceReset,
    RaceSetStartFinish,
    RaceMarkLap,
    SdFileRow,
    SdUp,
    SdPrev,
    SdNext,
    SdTest,
    TouchCalStart,
    TouchCalClose,
    TouchCalSample,
  };

  struct UiAction {
    UiActionType type = UiActionType::None;
    uint8_t arg0 = 0;
    uint8_t arg1 = 0;
    uint32_t value = 0;
  };

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
  void buildStatusOverlays(lv_obj_t* scr);
  void updateStatusOverlays(const state::VehicleState& s, uint32_t nowMs);
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
  void queueSettingsSave(uint32_t nowMs);
  void serviceQueuedSettingsSave(uint32_t nowMs);
  bool enqueueAction(const UiAction& action, uint32_t nowMs);
  void serviceUiActions(uint32_t nowMs);
  void performUiAction(const UiAction& action, uint32_t nowMs);
  bool shouldAcceptUiTap(lv_obj_t* target, uint32_t nowMs);
  bool confirmOrEnqueue(const UiAction& action, const char* prompt, uint32_t nowMs);
  void serviceTouchGestures(uint32_t nowMs);
  void buildTouchCalibrationOverlay(lv_obj_t* scr);
  void buildFaultOverlay(lv_obj_t* scr);
  void showTouchCalibration(bool show);
  void showFaultOverlay(bool show);
  void updateTouchCalibrationPrompt();
  void recordTouchCalibrationSample(uint32_t nowMs);
  void runLedOutputTest(uint32_t nowMs);

  // ---- LVGL callbacks -------------------------------------------------
  static void lvglFlushCb(lv_disp_drv_t* drv, const lv_area_t* area, lv_color_t* colors);
  static void lvglRounderCb(lv_disp_drv_t* drv, lv_area_t* area);
  static void lvglTouchReadCb(lv_indev_drv_t* drv, lv_indev_data_t* data);

  // ---- Event handlers -------------------------------------------------
  static void onNavClicked(lv_event_t* e);
  static void onLedModeClicked(lv_event_t* e);
  static void onLedShowMenuClicked(lv_event_t* e);
  static void onLedShowBackClicked(lv_event_t* e);
  static void onLedShowModeClicked(lv_event_t* e);
  static void onLedShowOnClicked(lv_event_t* e);
  static void onLedShowOffClicked(lv_event_t* e);
  static void onLedShowClearClicked(lv_event_t* e);
  static void onLedColorClicked(lv_event_t* e);
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
  static void onKnockEnableClicked(lv_event_t* e);
  static void onKnockResetBaselineClicked(lv_event_t* e);
  static void onKnockGainDownClicked(lv_event_t* e);
  static void onKnockGainUpClicked(lv_event_t* e);
  static void onKnockMultiplierDownClicked(lv_event_t* e);
  static void onKnockMultiplierUpClicked(lv_event_t* e);
  static void onLedMasterSwitchChanged(lv_event_t* e);
  static void onBenchTestClicked(lv_event_t* e);
  static void onDiagInfoClicked(lv_event_t* e);
  static void onDiagToolsClicked(lv_event_t* e);
  static void onDiagStorageClicked(lv_event_t* e);
  static void onDiagTrendsClicked(lv_event_t* e);
  static void onThemeProfileClicked(lv_event_t* e);
  static void onFaultPageClicked(lv_event_t* e);
  static void onFaultCloseClicked(lv_event_t* e);
  static void onLedOutputTestClicked(lv_event_t* e);
  static void onCanPingClicked(lv_event_t* e);
  static void onRestartClicked(lv_event_t* e);
  static void onRaceAccelClicked(lv_event_t* e);
  static void onRaceLapClicked(lv_event_t* e);
  static void onRaceStopClicked(lv_event_t* e);
  static void onRaceResetClicked(lv_event_t* e);
  static void onRaceSetStartClicked(lv_event_t* e);
  static void onRaceMarkLapClicked(lv_event_t* e);
  static void onSdFileRowClicked(lv_event_t* e);
  static void onSdUpClicked(lv_event_t* e);
  static void onSdPrevClicked(lv_event_t* e);
  static void onSdNextClicked(lv_event_t* e);
  static void onSdTestClicked(lv_event_t* e);
  static void onTouchCalStartClicked(lv_event_t* e);
  static void onTouchCalCloseClicked(lv_event_t* e);

  // ---- Dependencies ---------------------------------------------------
  canbus::CanManager*           canMgr_      = nullptr;
  race::RacePerformanceManager* raceMgr_     = nullptr;
  settings::SettingsManager*    settingsMgr_ = nullptr;
  storage::SdManager*           sdMgr_       = nullptr;
  touch::TouchManager*          touchMgr_    = nullptr;

  bool     online_                = false;
  uint32_t actionFeedbackUntilMs_ = 0;
  char     actionFeedback_[48]    = {};
  uint8_t  activePage_            = 0;
  bool     pageSwitchPending_      = false;
  bool     settingsSaveQueued_     = false;
  bool     suppressLedMasterEvent_ = false;
  bool     filteredTouchDown_      = false;
  bool     gestureTouchWasDown_    = false;
  bool     touchDebugWasDown_      = false;
  int8_t   pendingDirectNav_       = -1;
  uint8_t  uiActionHead_           = 0;
  uint8_t  uiActionTail_           = 0;
  uint8_t  uiActionCount_          = 0;
  uint8_t  touchCalIndex_          = 0;
  uint8_t  ledOutputTestStep_      = 0;
  uint8_t  themeProfileMode_       = 0;  // 0 auto, 1 day, 2 night
  bool     nightProfileApplied_    = false;
  bool     startupComplete_        = false;
  UiActionType pendingConfirmType_ = UiActionType::None;
  uint32_t lastStressPageSwitchMs_ = 0;
  uint32_t lastHeaderUpdateMs_    = 0;
  uint32_t lastPageUpdateMs_      = 0;
  uint32_t settingsSaveDueMs_     = 0;
  uint32_t filteredTouchSampleMs_ = 0;
  uint32_t filteredTouchPressStartMs_ = 0;
  uint32_t lastAcceptedTapMs_ = 0;
  uint32_t pendingConfirmUntilMs_ = 0;
  uint32_t gestureStartMs_ = 0;
  uint32_t lastHeaderTapMs_ = 0;
  uint32_t startupBeginMs_ = 0;
  int16_t  touchCalOffsetX_ = 0;
  int16_t  touchCalOffsetY_ = 0;
  int16_t  touchCalAccumX_ = 0;
  int16_t  touchCalAccumY_ = 0;
  uint16_t gestureStartX_ = 0;
  uint16_t gestureStartY_ = 0;
#if !LV_TICK_CUSTOM
  uint32_t lastLvTickMs_          = 0;
#endif
  char     hdrTimeText_[12]       = {};
  char     rpmText_[12]           = {};
  char     spdText_[12]           = {};
  char     boostText_[24]         = {};
  char     dashEnvText_[40]       = {};
  char     dashStatusText_[64]    = {};
  char     dashRaceText_[64]      = {};
  char     gLiveText_[16]         = {};
  char     gPeakText_[16]         = {};

  UiAction uiActions_[kUiActionQueueSize] = {};
  UiAction pendingConfirmAction_ = {};
  lv_obj_t* lastAcceptedTapTarget_ = nullptr;
  touch::TouchSample rawTouch_ = {};
  touch::TouchSample filteredTouch_ = {};
  portMUX_TYPE        touchMux_  = portMUX_INITIALIZER_UNLOCKED;  // guards touch samples (touchTask writes, screenTask reads)

  // ---- LVGL widget pointers -------------------------------------------

  // Header
  lv_obj_t* hdrBatLabel_      = nullptr;  // left:   GPS UTC clock
  lv_obj_t* headerBar_        = nullptr;
  lv_obj_t* hdrTitleLabel_    = nullptr;  // center: page name (cyan)
  lv_obj_t* hdrFaultDot_      = nullptr;  // right:  status indicator
  lv_obj_t* hdrFeedbackLabel_ = nullptr;  // far-right: action feedback
  lv_obj_t* alertStrip_       = nullptr;
  lv_obj_t* alertStripLabel_  = nullptr;
  lv_obj_t* themeTint_        = nullptr;
  lv_obj_t* startupOverlay_   = nullptr;
  lv_obj_t* startupStatus_    = nullptr;
  lv_obj_t* startupProgress_  = nullptr;

  // Bottom nav bar (0=DASH 1=METH 2=TAIL 3=LEDS 4=GPS 5=TEMPS 6=DIAG 7=KNOCK)
  lv_obj_t* navBtns_[8]     = {};
  lv_obj_t* navBtnIcons_[8] = {};
  lv_obj_t* navBar_         = nullptr;
  lv_obj_t* navDivider_     = nullptr;
  lv_obj_t* templateImgs_[8] = {};
  lv_obj_t* topContentEdgeGuard_ = nullptr;
  lv_obj_t* bottomEdgeGuard_ = nullptr;

  // Page content panels (one visible at a time)
  lv_obj_t* contentArea_ = nullptr;
  lv_obj_t* pages_[8] = {};

  // -- DASH page --
  lv_obj_t* rpmValLabel_     = nullptr;  // large RPM number
  lv_obj_t* spdValLabel_     = nullptr;  // large speed number
  lv_obj_t* boostValLabel_   = nullptr;
  lv_obj_t* dashEnvLabel_    = nullptr;  // meth duty / tank
  lv_obj_t* dashStatusLabel_ = nullptr;  // meth activation state
  lv_obj_t* dashRaceLabel_   = nullptr;  // compact voltage / GPS / IAT row
  lv_obj_t* raceAccelBtn_    = nullptr;
  lv_obj_t* gLiveLabel_      = nullptr;  // "X.XX G" current G
  lv_obj_t* gPeakLabel_      = nullptr;  // "PK X.XX" peak G

  // -- METH page --
  lv_obj_t* methBadgeLabel_    = nullptr;
  lv_obj_t* methStateLabel_    = nullptr;
  lv_obj_t* methSensorLabel_   = nullptr;
  lv_obj_t* methParamLabel_    = nullptr;
  lv_obj_t* methDutyLabel_     = nullptr;
  lv_obj_t* methTankLabel_     = nullptr;
  lv_obj_t* methMapLabel_      = nullptr;
  lv_obj_t* methPressureLabel_ = nullptr;
  lv_obj_t* methIatLabel_      = nullptr;
  lv_obj_t* methBayLabel_      = nullptr;
  lv_obj_t* methArmBtn_        = nullptr;
  lv_obj_t* methArmBtnLabel_   = nullptr;
  lv_obj_t* methRatioBtn_      = nullptr;
  lv_obj_t* methRatioBtnLabel_ = nullptr;

  // -- TAIL page --
  lv_obj_t* tailStatusLabel_   = nullptr;
  lv_obj_t* tailOnlineLed_     = nullptr;
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
  static constexpr uint8_t kInteriorLedUiCount = 5;
  static constexpr uint8_t kLedModeButtonCount = 2;
  static constexpr uint8_t kLedShowModeButtonCount = 5;
  static constexpr uint8_t kLedColorButtonCount = 6;
  lv_obj_t* ledMasterSwitch_ = nullptr;
  lv_obj_t* ledMasterLabel_  = nullptr;
  lv_obj_t* ledMainPanel_ = nullptr;
  lv_obj_t* ledShowPanel_ = nullptr;
  lv_obj_t* ledShowBtn_ = nullptr;
  lv_obj_t* ledShowBackBtn_ = nullptr;
  lv_obj_t* ledShowOnBtn_ = nullptr;
  lv_obj_t* ledShowOffBtn_ = nullptr;
  lv_obj_t* ledShowClearBtn_ = nullptr;
  lv_obj_t* ledShowModeBtns_[kLedShowModeButtonCount] = {};
  lv_obj_t* ledColorBtns_[kLedColorButtonCount] = {};
  lv_obj_t* ledStripLabels_[kInteriorLedUiCount] = {};
  lv_obj_t* ledModeBtns_[kInteriorLedUiCount][kLedModeButtonCount] = {};

  // -- GPS page --
  lv_obj_t* gpsSpdLabel_  = nullptr;
  lv_obj_t* gpsInfoLabel_ = nullptr;
  lv_obj_t* gpsReceiverLabel_ = nullptr;
  lv_obj_t* gpsSatLabel_ = nullptr;
  lv_obj_t* gpsHdopLabel_ = nullptr;

  // -- TEMPS page --
  lv_obj_t* tempsLabel_ = nullptr;
  lv_obj_t* tempsTable_ = nullptr;
  lv_obj_t* tempsValueLabels_[6] = {};
  lv_obj_t* tempsStatusLabels_[6] = {};

  // -- DIAG page --
  lv_obj_t* diagInfoBtn_  = nullptr;
  lv_obj_t* diagToolsBtn_ = nullptr;
  lv_obj_t* diagStorageBtn_ = nullptr;
  lv_obj_t* diagTrendsBtn_ = nullptr;
  lv_obj_t* diagInfoPanel_ = nullptr;
  lv_obj_t* diagToolsPanel_ = nullptr;
  lv_obj_t* diagStoragePanel_ = nullptr;
  lv_obj_t* diagTrendsPanel_ = nullptr;
  lv_obj_t* diagLabel_     = nullptr;
  lv_obj_t* diagStatusCards_[6] = {};
  lv_obj_t* trendCharts_[4] = {};
  lv_chart_series_t* trendSeriesA_[4] = {};
  lv_chart_series_t* trendSeriesB_[4] = {};
  lv_obj_t* faultOverlay_  = nullptr;
  lv_obj_t* faultLabel_    = nullptr;
  lv_obj_t* faultCloseBtn_ = nullptr;
  lv_obj_t* faultPageBtn_  = nullptr;
  lv_obj_t* ledOutputTestBtn_ = nullptr;
  lv_obj_t* canPingBtn_    = nullptr;
  lv_obj_t* restartBtn_    = nullptr;
  lv_obj_t* diagRaceStatusLabel_ = nullptr;
  lv_obj_t* diagRaceAccelBtn_  = nullptr;
  lv_obj_t* diagRaceLapBtn_    = nullptr;
  lv_obj_t* diagRaceMarkBtn_   = nullptr;
  lv_obj_t* diagRaceStopBtn_   = nullptr;
  lv_obj_t* diagRaceResetBtn_  = nullptr;
  lv_obj_t* diagRaceSetStartBtn_ = nullptr;
  lv_obj_t* benchTestBtn_  = nullptr;
  lv_obj_t* themeProfileBtn_ = nullptr;
  lv_obj_t* touchCalBtn_   = nullptr;
  lv_obj_t* touchCalOverlay_ = nullptr;
  lv_obj_t* touchCalPromptLabel_ = nullptr;
  lv_obj_t* touchCalTargetLabel_ = nullptr;
  lv_obj_t* touchCalCloseBtn_ = nullptr;
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
  lv_obj_t* knockLearningSpinner_  = nullptr;
  lv_obj_t* knockEventLabel_       = nullptr;
  lv_obj_t* knockLastLabel_        = nullptr;
  lv_obj_t* knockEnableBtn_        = nullptr;
  lv_obj_t* knockEnableBtnLabel_   = nullptr;
  lv_obj_t* knockResetBlBtn_       = nullptr;
  lv_obj_t* knockGainDownBtn_      = nullptr;
  lv_obj_t* knockGainUpBtn_        = nullptr;
  lv_obj_t* knockMultDownBtn_      = nullptr;
  lv_obj_t* knockMultUpBtn_        = nullptr;
};

}  // namespace ui
