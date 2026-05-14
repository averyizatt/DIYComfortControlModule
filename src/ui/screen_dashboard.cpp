#include "ui/screen_dashboard.h"

#include <cstdio>
#include <cstring>

#if __has_include(<Arduino_GFX_Library.h>)
#include <Arduino_GFX_Library.h>
#define CCM_HAS_ARDUINO_GFX 1
#else
#define CCM_HAS_ARDUINO_GFX 0
#endif

#include "can/can_protocol.h"
#include "state/vehicle_state.h"

namespace ui {

// ---------------------------------------------------------------------------
// Static module-level storage
// ---------------------------------------------------------------------------

#if CCM_HAS_ARDUINO_GFX
static Arduino_DataBus* s_bus = nullptr;
static Arduino_GFX*     s_gfx = nullptr;
#endif

// LVGL draw buffers – two 320x40 slices (~25 KB each, double-buffered).
static lv_color_t s_buf1[320 * 40];
static lv_color_t s_buf2[320 * 40];

// ---------------------------------------------------------------------------
// LVGL driver callbacks (static)
// ---------------------------------------------------------------------------

void ScreenDashboard::lvglFlushCb(lv_disp_drv_t* drv, const lv_area_t* area, lv_color_t* colors) {
#if CCM_HAS_ARDUINO_GFX
  if (s_gfx) {
    const int32_t w = area->x2 - area->x1 + 1;
    const int32_t h = area->y2 - area->y1 + 1;
    s_gfx->draw16bitRGBBitmap(area->x1, area->y1, reinterpret_cast<uint16_t*>(colors), w, h);
  }
#else
  (void)area;
  (void)colors;
#endif
  lv_disp_flush_ready(drv);
}

void ScreenDashboard::lvglTouchReadCb(lv_indev_drv_t* drv, lv_indev_data_t* data) {
  auto* self = static_cast<ScreenDashboard*>(drv->user_data);
  if (self->lastTouch_.touched) {
    data->point.x = static_cast<lv_coord_t>(self->lastTouch_.x);
    data->point.y = static_cast<lv_coord_t>(self->lastTouch_.y);
    data->state   = LV_INDEV_STATE_PRESSED;
  } else {
    data->state = LV_INDEV_STATE_RELEASED;
  }
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void ScreenDashboard::attach(canbus::CanManager* canMgr, race::RacePerformanceManager* raceMgr,
                              settings::SettingsManager* settingsMgr) {
  canMgr_      = canMgr;
  raceMgr_     = raceMgr;
  settingsMgr_ = settingsMgr;
}

bool ScreenDashboard::begin(uint8_t lcdCs, uint8_t lcdRst, uint8_t lcdDc,
                             uint8_t spiSck, uint8_t spiMosi, uint8_t spiMiso) {
#if CCM_HAS_ARDUINO_GFX
  s_bus = new Arduino_ESP32SPI(lcdDc, lcdCs, spiSck, spiMosi, spiMiso, FSPI);
  s_gfx = new Arduino_ST7796(s_bus, lcdRst, 0 /*rotation*/, true /*ips*/, kWidth, kHeight);
  if (!s_gfx || !s_gfx->begin()) return false;
#else
  (void)lcdCs; (void)lcdRst; (void)lcdDc;
  (void)spiSck; (void)spiMosi; (void)spiMiso;
#endif

  // ---- LVGL init ----
  lv_init();

  // Display driver
  static lv_disp_draw_buf_t drawBuf;
  lv_disp_draw_buf_init(&drawBuf, s_buf1, s_buf2, 320 * 40);

  static lv_disp_drv_t dispDrv;
  lv_disp_drv_init(&dispDrv);
  dispDrv.hor_res     = static_cast<lv_coord_t>(kWidth);
  dispDrv.ver_res     = static_cast<lv_coord_t>(kHeight);
  dispDrv.flush_cb    = lvglFlushCb;
  dispDrv.draw_buf    = &drawBuf;
  lv_disp_drv_register(&dispDrv);

  // Touch input device
  static lv_indev_drv_t indevDrv;
  lv_indev_drv_init(&indevDrv);
  indevDrv.type      = LV_INDEV_TYPE_POINTER;
  indevDrv.read_cb   = lvglTouchReadCb;
  indevDrv.user_data = this;
  lv_indev_drv_register(&indevDrv);

  // Dark theme
  lv_theme_t* theme = lv_theme_default_init(
      lv_disp_get_default(),
      lv_palette_main(LV_PALETTE_BLUE),
      lv_palette_main(LV_PALETTE_CYAN),
      true  /* dark mode */,
      &lv_font_montserrat_14);
  lv_disp_set_theme(lv_disp_get_default(), theme);

  buildUi();
  online_ = true;
  return true;
}

void ScreenDashboard::tick(const state::VehicleState& s, uint32_t nowMs) {
  if (!online_) return;
  updateHeader(s, nowMs);
  updateDashTab(s);
  updateMethTab(s, nowMs);
  updateTailTab(s);
  updateRaceTab(s);
  updateDiagTab(s);
  lv_task_handler();
}

void ScreenDashboard::handleTouch(const touch::TouchSample& sample, uint32_t /*nowMs*/) {
  // Normalize raw coordinates and buffer for the LVGL indev driver.
  lastTouch_ = normalizeRaw(sample);

  // Mark touch event in shared vehicle state so CAN telemetry can observe it.
  if (lastTouch_.touched) {
    state::g_vehicle_state.mutate([](state::VehicleState& vs) {
      vs.input_flags |= can_protocol::input_flag::TOUCH;
    });
  }
}

// ---------------------------------------------------------------------------
// UI construction
// ---------------------------------------------------------------------------

static lv_obj_t* makeLabel(lv_obj_t* parent, lv_coord_t x, lv_coord_t y,
                            lv_coord_t w, const char* text,
                            const lv_font_t* font = nullptr) {
  lv_obj_t* lbl = lv_label_create(parent);
  lv_label_set_text(lbl, text);
  lv_label_set_long_mode(lbl, LV_LABEL_LONG_CLIP);
  lv_obj_set_pos(lbl, x, y);
  lv_obj_set_width(lbl, w);
  if (font) lv_obj_set_style_text_font(lbl, font, LV_PART_MAIN);
  return lbl;
}

static lv_obj_t* makeBtn(lv_obj_t* parent, const char* text,
                          lv_coord_t x, lv_coord_t y, lv_coord_t w, lv_coord_t h,
                          lv_event_cb_t cb, void* userData) {
  lv_obj_t* btn = lv_btn_create(parent);
  lv_obj_set_pos(btn, x, y);
  lv_obj_set_size(btn, w, h);
  lv_obj_t* lbl = lv_label_create(btn);
  lv_label_set_text(lbl, text);
  lv_obj_center(lbl);
  if (cb) lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, userData);
  return btn;
}

// Returns the label child of a button (first child).
static lv_obj_t* btnLabel(lv_obj_t* btn) {
  return lv_obj_get_child(btn, 0);
}

void ScreenDashboard::buildUi() {
  lv_obj_t* scr = lv_scr_act();
  lv_obj_set_style_bg_color(scr, lv_color_black(), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);

  // ---- Fixed header bar (above tabview) ----
  lv_obj_t* hdr = lv_obj_create(scr);
  lv_obj_set_pos(hdr, 0, 0);
  lv_obj_set_size(hdr, kWidth, kHdrH);
  lv_obj_set_style_radius(hdr, 0, LV_PART_MAIN);
  lv_obj_set_style_border_width(hdr, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_all(hdr, 0, LV_PART_MAIN);
  lv_obj_clear_flag(hdr, LV_OBJ_FLAG_SCROLLABLE);

  titleLabel_ = lv_label_create(hdr);
  lv_label_set_text(titleLabel_, "Cabin Master");
  lv_obj_set_style_text_font(titleLabel_, &lv_font_montserrat_16, LV_PART_MAIN);
  lv_obj_align(titleLabel_, LV_ALIGN_LEFT_MID, 6, 0);

  feedbackLabel_ = lv_label_create(hdr);
  lv_label_set_text(feedbackLabel_, "");
  lv_obj_set_style_text_font(feedbackLabel_, &lv_font_montserrat_12, LV_PART_MAIN);
  lv_obj_set_style_text_color(feedbackLabel_, lv_palette_main(LV_PALETTE_GREEN), LV_PART_MAIN);
  lv_obj_align(feedbackLabel_, LV_ALIGN_RIGHT_MID, -6, 0);

  // ---- Tabview (covers rest of screen) ----
  tabview_ = lv_tabview_create(scr, LV_DIR_TOP, kTabBarH);
  lv_obj_set_pos(tabview_, 0, kHdrH);
  lv_obj_set_size(tabview_, kWidth, static_cast<lv_coord_t>(kHeight - kHdrH));

  lv_obj_t* tabDash = lv_tabview_add_tab(tabview_, "DASH");
  lv_obj_t* tabMeth = lv_tabview_add_tab(tabview_, "METH");
  lv_obj_t* tabTail = lv_tabview_add_tab(tabview_, "TAIL");
  lv_obj_t* tabRace = lv_tabview_add_tab(tabview_, "RACE");
  lv_obj_t* tabDiag = lv_tabview_add_tab(tabview_, "DIAG");

  buildDashTab(tabDash);
  buildMethTab(tabMeth);
  buildTailTab(tabTail);
  buildRaceTab(tabRace);
  buildDiagTab(tabDiag);
}

void ScreenDashboard::buildDashTab(lv_obj_t* parent) {
  lv_obj_set_style_pad_all(parent, 4, LV_PART_MAIN);

  dashLiveLabel_ = makeLabel(parent, 0, 0, 310,
      "RPM: 0\nBOOST: 0 kPa  IAT: 0.0 C  SPD: 0.0",
      &lv_font_montserrat_20);

  dashEnvLabel_  = makeLabel(parent, 0, 60, 310,
      "BAT: 0.0V  CAB: 0.0 C  OUT: 0.0 C  GPS: 0 sats");

  dashStatusLabel_ = makeLabel(parent, 0, 92, 310,
      "CAN: --  METH: --  GPS: --\nFaults: 0x0000  Touch: --  FPS: 0");

  dashRaceLabel_ = makeLabel(parent, 0, 145, 310,
      "Race: STOPPED\n0-60: 0.000s   1/4: 0.000s\nLap: 0.000s  Laps: 0");
}

void ScreenDashboard::buildMethTab(lv_obj_t* parent) {
  lv_obj_set_style_pad_all(parent, 4, LV_PART_MAIN);

  methStateLabel_  = makeLabel(parent, 0,  0, 310, "METH | State: 0 | Duty: 0% | Tank: 0%");
  methSensorLabel_ = makeLabel(parent, 0, 24, 310, "MAP: 0 kPa  IAT: 0.0 C  Bay: 0.0 C");
  methParamLabel_  = makeLabel(parent, 0, 48, 310, "Ratio: 0%  Flow: 0  Armed: NO");

  methArmBtn_      = makeBtn(parent, "ARM",    4,  82, 148, 48, onMethArmClicked,   this);
  methArmBtnLabel_ = btnLabel(methArmBtn_);

  methRatioBtn_      = makeBtn(parent, "RATIO 50%", 160, 82, 148, 48, onMethRatioClicked, this);
  methRatioBtnLabel_ = btnLabel(methRatioBtn_);
}

void ScreenDashboard::buildTailTab(lv_obj_t* parent) {
  lv_obj_set_style_pad_all(parent, 4, LV_PART_MAIN);

  tailStatusLabel_ = makeLabel(parent, 0, 0, 310,
      "Taillights: OFFLINE  Bright: 0\nL: 0  R: 0  Thermal derate: 0%");

  // Mode button panel (2x2 grid)
  tailModePanel_ = lv_obj_create(parent);
  lv_obj_set_pos(tailModePanel_, 0, 52);
  lv_obj_set_size(tailModePanel_, 312, 200);
  lv_obj_set_style_pad_all(tailModePanel_, 2, LV_PART_MAIN);
  lv_obj_set_style_border_width(tailModePanel_, 0, LV_PART_MAIN);
  lv_obj_set_style_bg_opa(tailModePanel_, LV_OPA_TRANSP, LV_PART_MAIN);
  lv_obj_clear_flag(tailModePanel_, LV_OBJ_FLAG_SCROLLABLE);

  tailStockBtn_    = makeBtn(tailModePanel_, "STOCK",       2,  0, 148, 48, onTailStockClicked,    this);
  tailSeqBtn_      = makeBtn(tailModePanel_, "SEQUENTIAL", 158,  0, 148, 48, onTailSeqClicked,      this);
  tailShowMenuBtn_ = makeBtn(tailModePanel_, "SHOW MENU",   2, 54, 148, 48, onTailShowMenuClicked, this);
  tailDemoBtn_     = makeBtn(tailModePanel_, "DEMO",       158, 54, 148, 48, onTailDemoClicked,     this);

  // Show-option submenu panel (hidden by default)
  tailShowPanel_ = lv_obj_create(parent);
  lv_obj_set_pos(tailShowPanel_, 0, 52);
  lv_obj_set_size(tailShowPanel_, 312, 310);
  lv_obj_set_style_pad_all(tailShowPanel_, 2, LV_PART_MAIN);
  lv_obj_set_style_border_width(tailShowPanel_, 0, LV_PART_MAIN);
  lv_obj_set_style_bg_opa(tailShowPanel_, LV_OPA_TRANSP, LV_PART_MAIN);
  lv_obj_clear_flag(tailShowPanel_, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_flag(tailShowPanel_, LV_OBJ_FLAG_HIDDEN);

  tailShowPageLabel_ = makeLabel(tailShowPanel_, 0, 0, 312, "Page 1/4");

  tailShowPrevBtn_ = makeBtn(tailShowPanel_, "< PREV",  2,  22,  80, 38, onTailShowPrevClicked, this);
  tailShowBackBtn_ = makeBtn(tailShowPanel_, "BACK",   90,  22, 130, 38, onTailShowBackClicked, this);
  tailShowNextBtn_ = makeBtn(tailShowPanel_, "NEXT >", 228,  22,  80, 38, onTailShowNextClicked, this);

  // 6 option buttons in a 3x2 grid
  const lv_coord_t optW = 100, optH = 40, optGap = 4;
  for (uint8_t i = 0; i < kTaillightShowOptionsPerPage; ++i) {
    const lv_coord_t col = static_cast<lv_coord_t>(i % 3);
    const lv_coord_t row = static_cast<lv_coord_t>(i / 3);
    const lv_coord_t bx  = col * (optW + optGap);
    const lv_coord_t by  = static_cast<lv_coord_t>(66) + row * (optH + optGap);
    tailShowOptBtns_[i] = makeBtn(tailShowPanel_, "--", bx, by, optW, optH, onTailShowOptClicked, this);
  }
}

void ScreenDashboard::buildRaceTab(lv_obj_t* parent) {
  lv_obj_set_style_pad_all(parent, 4, LV_PART_MAIN);

  raceLiveLabel_  = makeLabel(parent, 0, 0, 310,
      "RPM: 0   SPD: 0.0 km/h", &lv_font_montserrat_20);

  raceStatsLabel_ = makeLabel(parent, 0, 36, 310,
      "0-60: 0.000s   1/4mi: 0.000s\nLap best: 0.000s  Laps: 0  Quality: 0%");

  raceAccelBtn_ = makeBtn(parent, "ACCEL",   4,  90, 148, 46, onRaceAccelClicked, this);
  raceLapBtn_   = makeBtn(parent, "LAP",   160,  90, 148, 46, onRaceLapClicked,   this);
  raceStopBtn_  = makeBtn(parent, "STOP",    4, 142, 148, 46, onRaceStopClicked,  this);
  raceResetBtn_ = makeBtn(parent, "RESET", 160, 142, 148, 46, onRaceResetClicked, this);
}

void ScreenDashboard::buildDiagTab(lv_obj_t* parent) {
  lv_obj_set_style_pad_all(parent, 4, LV_PART_MAIN);
  // Parent tab is already scrollable by default in LVGL 8
  diagLabel_ = makeLabel(parent, 0, 0, 310,
      "Diagnostics loading...", &lv_font_montserrat_12);
  lv_label_set_long_mode(diagLabel_, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(diagLabel_, 310);
}

// ---------------------------------------------------------------------------
// Per-tick update helpers
// ---------------------------------------------------------------------------

void ScreenDashboard::updateHeader(const state::VehicleState& s, uint32_t nowMs) {
  char hdr[48];
  snprintf(hdr, sizeof(hdr), "Cabin Master  |  %s", s.touch_online ? "TOUCH OK" : "TOUCH ?");
  lv_label_set_text(titleLabel_, hdr);

  if (actionFeedback_[0] != '\0' &&
      nowMs < actionFeedbackUntilMs_ &&
      (actionFeedbackUntilMs_ - nowMs) <= kActionFeedbackMs) {
    lv_label_set_text(feedbackLabel_, actionFeedback_);
  } else {
    lv_label_set_text(feedbackLabel_, "");
  }
}

void ScreenDashboard::updateDashTab(const state::VehicleState& s) {
  char buf[128];

  snprintf(buf, sizeof(buf), "RPM: %u   SPD: %.1f km/h\nBOOST: %.0f kPa  IAT: %.1f C",
           static_cast<unsigned>(s.rpm),
           static_cast<double>(s.speed),
           static_cast<double>(s.boost_kpa),
           static_cast<double>(s.intake_temp));
  lv_label_set_text(dashLiveLabel_, buf);

  snprintf(buf, sizeof(buf), "BAT: %.1fV  CAB: %.1f C  OUT: %.1f C  GPS: %u sats",
           static_cast<double>(s.battery_voltage),
           static_cast<double>(s.cabin_temp),
           static_cast<double>(s.outside_temp),
           static_cast<unsigned>(s.gps_satellites));
  lv_label_set_text(dashEnvLabel_, buf);

  snprintf(buf, sizeof(buf),
           "CAN: %s  METH: %s  GPS: %s\nFaults: 0x%04X  Touch: %s  FPS: %.0f",
           s.can_online  ? "ONLINE"  : "OFFLINE",
           s.meth_online ? "ONLINE"  : "OFFLINE",
           s.gps_fix     ? "FIX"     : "NO FIX",
           static_cast<unsigned>(s.fault_flags),
           s.touch_online ? "OK" : "?",
           static_cast<double>(s.ui_fps));
  lv_label_set_text(dashStatusLabel_, buf);

  snprintf(buf, sizeof(buf),
           "Race: %s\n0-60: %.3fs   1/4: %.3fs\nLap: %.3fs  Laps: %u",
           s.race_running ? "RUNNING" : "STOPPED",
           static_cast<double>(s.race_0_60_s),
           static_cast<double>(s.race_quarter_mile_et_s),
           static_cast<double>(s.race_best_lap_s),
           static_cast<unsigned>(s.race_lap_count));
  lv_label_set_text(dashRaceLabel_, buf);
}

void ScreenDashboard::updateMethTab(const state::VehicleState& s, uint32_t /*nowMs*/) {
  char buf[96];

  snprintf(buf, sizeof(buf), "METH | State: %u | Duty: %u%% | Tank: %s",
           static_cast<unsigned>(s.meth_state),
           static_cast<unsigned>(s.meth_pump_duty),
           (s.meth_tank_level == 0) ? "EMPTY" : "OK");
  lv_label_set_text(methStateLabel_, buf);

  snprintf(buf, sizeof(buf), "MAP: %.0f kPa  IAT: %.1f C  Bay: %.1f C",
           static_cast<double>(s.boost_kpa),
           static_cast<double>(s.intake_temp),
           static_cast<double>(s.engine_bay_temp));
  lv_label_set_text(methSensorLabel_, buf);

  snprintf(buf, sizeof(buf), "Ratio: %u%%  Flow: %u  Armed: %s",
           static_cast<unsigned>(s.meth_selected_ratio_percent),
           static_cast<unsigned>(s.meth_flow_status),
           s.meth_desired_armed ? "YES" : "NO");
  lv_label_set_text(methParamLabel_, buf);

  lv_label_set_text(methArmBtnLabel_,
                    s.meth_desired_armed ? "DISARM" : "ARM");

  snprintf(buf, sizeof(buf), "RATIO  %u%%",
           static_cast<unsigned>(s.meth_selected_ratio_percent));
  lv_label_set_text(methRatioBtnLabel_, buf);
}

void ScreenDashboard::updateTailTab(const state::VehicleState& s) {
  char buf[96];

  snprintf(buf, sizeof(buf),
           "Taillights: %s  Bright: %u\nL: %u  R: %u  Thermal derate: %u%%",
           s.taillight_online ? "ONLINE" : "OFFLINE",
           static_cast<unsigned>(s.taillight_brightness),
           static_cast<unsigned>(s.taillight_left_state),
           static_cast<unsigned>(s.taillight_right_state),
           static_cast<unsigned>(s.taillight_thermal_derate));
  lv_label_set_text(tailStatusLabel_, buf);

  // Update show-submenu option button labels based on current page
  for (uint8_t i = 0; i < kTaillightShowOptionsPerPage; ++i) {
    const uint16_t optVal = static_cast<uint16_t>(tailShowPage_) * kTaillightShowOptionsPerPage + i;
    char label[12];
    if (optVal < kTaillightShowOptionCount) {
      snprintf(label, sizeof(label), "SHOW %u", static_cast<unsigned>(optVal + 1U));
    } else {
      snprintf(label, sizeof(label), "--");
    }
    lv_label_set_text(btnLabel(tailShowOptBtns_[i]), label);
  }

  char page[16];
  snprintf(page, sizeof(page), "Page %u/%u",
           static_cast<unsigned>(tailShowPage_ + 1U),
           static_cast<unsigned>(kTaillightShowPageCount));
  lv_label_set_text(tailShowPageLabel_, page);
}

void ScreenDashboard::updateRaceTab(const state::VehicleState& s) {
  char buf[80];

  snprintf(buf, sizeof(buf), "RPM: %u   SPD: %.1f km/h",
           static_cast<unsigned>(s.rpm),
           static_cast<double>(s.speed));
  lv_label_set_text(raceLiveLabel_, buf);

  snprintf(buf, sizeof(buf),
           "0-60: %.3fs   1/4mi: %.3fs\nLap best: %.3fs  Laps: %u  Quality: %u%%",
           static_cast<double>(s.race_0_60_s),
           static_cast<double>(s.race_quarter_mile_et_s),
           static_cast<double>(s.race_best_lap_s),
           static_cast<unsigned>(s.race_lap_count),
           static_cast<unsigned>(s.race_quality_percent));
  lv_label_set_text(raceStatsLabel_, buf);
}

void ScreenDashboard::updateDiagTab(const state::VehicleState& s) {
  char buf[480];
  snprintf(buf, sizeof(buf),
           "CAN RX: %lu  TX: %lu\n"
           "Last RX ID: 0x%03X  TX ID: 0x%03X\n"
           "Fault flags: 0x%04X\n"
           "Heap free: %lu B\n"
           "ESP die temp: %d C\n"
           "SD mounted: %s  Errors: %lu\n"
           "Uptime: %lu s\n"
           "GPS fix: %s  Sats: %u\n"
           "Touch: %s\n"
           "UI FPS: %.1f\n"
           "Log: %s\n"
           "SD: %s",
           static_cast<unsigned long>(s.can_rx_count),
           static_cast<unsigned long>(s.can_tx_count),
           static_cast<unsigned>(s.can_last_rx_id),
           static_cast<unsigned>(s.can_last_tx_id),
           static_cast<unsigned>(s.fault_flags),
           static_cast<unsigned long>(s.heap_free_bytes),
           static_cast<int>(s.esp_die_temp_c),
           s.sd_mounted ? "YES" : "NO",
           static_cast<unsigned long>(s.sd_write_error_count),
           static_cast<unsigned long>(s.uptime_ms / 1000UL),
           s.gps_fix ? "YES" : "NO",
           static_cast<unsigned>(s.gps_satellites),
           s.touch_online ? "OK" : "OFFLINE",
           static_cast<double>(s.ui_fps),
           s.current_log_file,
           s.last_sd_write_status);
  lv_label_set_text(diagLabel_, buf);
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

void ScreenDashboard::setActionFeedback(const char* text, uint32_t nowMs) {
  if (!text) return;
  strncpy(actionFeedback_, text, sizeof(actionFeedback_) - 1);
  actionFeedback_[sizeof(actionFeedback_) - 1] = '\0';
  actionFeedbackUntilMs_ = nowMs + kActionFeedbackMs;
}

uint8_t ScreenDashboard::nextMethRatio(uint8_t current) const {
  if (current < 25U)  return 25U;
  if (current < 50U)  return 50U;
  if (current < 75U)  return 75U;
  if (current < 100U) return 100U;
  return 25U;
}

touch::TouchSample ScreenDashboard::normalizeRaw(const touch::TouchSample& raw) const {
  if (!raw.touched) return raw;
  touch::TouchSample t = raw;

  // Some FT62xx controllers report in a 480x320 landscape frame while the
  // dashboard renders in 320x480 portrait. Rotate into portrait space.
  if (t.x <= kHeight && t.y <= kWidth) {
    const uint16_t x = t.x;
    t.x = t.y;
    t.y = static_cast<uint16_t>(kHeight > x ? (kHeight - x) : 0U);
  }

  // Fallback: scale raw 12-bit (0..4095) ranges down to pixel coordinates.
  if (t.x > kWidth)  t.x = static_cast<uint16_t>((static_cast<uint32_t>(t.x) * kWidth)  / 4095U);
  if (t.y > kHeight) t.y = static_cast<uint16_t>((static_cast<uint32_t>(t.y) * kHeight) / 4095U);
  return t;
}

// ---------------------------------------------------------------------------
// Button event handlers
// ---------------------------------------------------------------------------

void ScreenDashboard::onMethArmClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  if (!self->canMgr_) return;
  const bool arm = !state::g_vehicle_state.read().meth_desired_armed;
  if (self->canMgr_->sendMethArm(arm)) {
    state::g_vehicle_state.mutate([arm](state::VehicleState& vs) { vs.meth_desired_armed = arm; });
    self->setActionFeedback(arm ? "METH ARMED" : "METH DISARMED", millis());
  } else {
    self->setActionFeedback("METH CMD REJECTED", millis());
  }
}

void ScreenDashboard::onMethRatioClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  const uint8_t ratio = self->nextMethRatio(state::g_vehicle_state.read().meth_selected_ratio_percent);
  state::g_vehicle_state.mutate([ratio](state::VehicleState& vs) { vs.meth_selected_ratio_percent = ratio; });
  if (self->settingsMgr_) {
    self->settingsMgr_->updateFromState(state::g_vehicle_state.read());
    self->settingsMgr_->save();
  }
  if (self->canMgr_) self->canMgr_->sendMethConfigBroadcast();
  self->setActionFeedback("METH RATIO UPDATED", millis());
}

void ScreenDashboard::onTailStockClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  if (!self->canMgr_) return;
  self->canMgr_->sendTaillightMode(can_protocol::taillight_mode::STOCK)
      ? self->setActionFeedback("TAIL STOCK", millis())
      : self->setActionFeedback("TAIL CMD REJECTED", millis());
}

void ScreenDashboard::onTailSeqClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  if (!self->canMgr_) return;
  self->canMgr_->sendTaillightMode(can_protocol::taillight_mode::SEQUENTIAL)
      ? self->setActionFeedback("TAIL SEQUENTIAL", millis())
      : self->setActionFeedback("TAIL CMD REJECTED", millis());
}

void ScreenDashboard::onTailDemoClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  if (!self->canMgr_) return;
  self->canMgr_->sendTaillightMode(can_protocol::taillight_mode::DEMO)
      ? self->setActionFeedback("TAIL DEMO", millis())
      : self->setActionFeedback("TAIL CMD REJECTED", millis());
}

void ScreenDashboard::onTailShowMenuClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  self->tailShowPage_ = 0;
  lv_obj_add_flag(self->tailModePanel_,   LV_OBJ_FLAG_HIDDEN);
  lv_obj_clear_flag(self->tailShowPanel_, LV_OBJ_FLAG_HIDDEN);
  self->setActionFeedback("SHOW MENU", millis());
}

void ScreenDashboard::onTailShowPrevClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  self->tailShowPage_ = (self->tailShowPage_ == 0)
      ? static_cast<uint8_t>(kTaillightShowPageCount - 1U)
      : static_cast<uint8_t>(self->tailShowPage_ - 1U);
  self->setActionFeedback("SHOW PAGE", millis());
}

void ScreenDashboard::onTailShowNextClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  self->tailShowPage_ = static_cast<uint8_t>((self->tailShowPage_ + 1U) % kTaillightShowPageCount);
  self->setActionFeedback("SHOW PAGE", millis());
}

void ScreenDashboard::onTailShowBackClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  lv_obj_clear_flag(self->tailModePanel_, LV_OBJ_FLAG_HIDDEN);
  lv_obj_add_flag(self->tailShowPanel_,   LV_OBJ_FLAG_HIDDEN);
  self->setActionFeedback("SHOW MENU EXIT", millis());
}

void ScreenDashboard::onTailShowOptClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  lv_obj_t* btn = lv_event_get_target(e);

  uint8_t idx = kTaillightShowOptionsPerPage;
  for (uint8_t i = 0; i < kTaillightShowOptionsPerPage; ++i) {
    if (self->tailShowOptBtns_[i] == btn) { idx = i; break; }
  }
  if (idx >= kTaillightShowOptionsPerPage) return;

  const uint16_t optVal = static_cast<uint16_t>(self->tailShowPage_) * kTaillightShowOptionsPerPage + idx;
  if (optVal >= kTaillightShowOptionCount) {
    self->setActionFeedback("SHOW SLOT EMPTY", millis());
    return;
  }
  if (!self->canMgr_) return;
  const uint8_t option = static_cast<uint8_t>(optVal);
  if (self->canMgr_->sendTaillightShowOption(option)) {
    char fb[20];
    snprintf(fb, sizeof(fb), "SHOW %u", static_cast<unsigned>(option + 1U));
    self->setActionFeedback(fb, millis());
  } else {
    self->setActionFeedback("SHOW CMD REJECTED", millis());
  }
}

void ScreenDashboard::onRaceAccelClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  if (self->raceMgr_) {
    self->raceMgr_->startRun(state::RaceMode::ACCEL);
    self->setActionFeedback("RACE ACCEL START", millis());
  }
}

void ScreenDashboard::onRaceLapClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  if (self->raceMgr_) {
    self->raceMgr_->startRun(state::RaceMode::LAP);
    self->setActionFeedback("RACE LAP START", millis());
  }
}

void ScreenDashboard::onRaceStopClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  if (self->raceMgr_) {
    self->raceMgr_->stopRun();
    self->setActionFeedback("RACE STOPPED", millis());
  }
}

void ScreenDashboard::onRaceResetClicked(lv_event_t* e) {
  auto* self = static_cast<ScreenDashboard*>(lv_event_get_user_data(e));
  if (self->raceMgr_) {
    self->raceMgr_->resetSession();
    self->setActionFeedback("RACE RESET", millis());
  }
}

}  // namespace ui
