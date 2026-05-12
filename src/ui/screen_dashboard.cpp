#include "ui/screen_dashboard.h"

#include <cstring>

#if __has_include(<Arduino_GFX_Library.h>)
#include <Arduino_GFX_Library.h>
#define CCM_HAS_ARDUINO_GFX 1
#else
#define CCM_HAS_ARDUINO_GFX 0
#endif

namespace ui {

namespace {
constexpr uint16_t kBg = 0x1082;
constexpr uint16_t kPanel = 0x18E3;
constexpr uint16_t kBorder = 0x3A0A;
constexpr uint16_t kText = 0xFFFF;
constexpr uint16_t kSubtle = 0xBDF7;
constexpr uint16_t kWarn = 0xFD20;
constexpr uint16_t kOk = 0x4FE9;
constexpr uint16_t kBtn = 0x222F;
constexpr uint16_t kBtnActive = 0x33D4;
constexpr uint16_t kTab = 0x29A8;
constexpr uint16_t kTabActive = 0x33D4;

#if CCM_HAS_ARDUINO_GFX
Arduino_DataBus* g_bus = nullptr;
Arduino_GFX* g_gfx = nullptr;
#endif
}  // namespace

void ScreenDashboard::attach(canbus::CanManager* canMgr, race::RacePerformanceManager* raceMgr, settings::SettingsManager* settingsMgr) {
  canMgr_ = canMgr;
  raceMgr_ = raceMgr;
  settingsMgr_ = settingsMgr;
}

bool ScreenDashboard::begin(uint8_t lcdCs, uint8_t lcdRst, uint8_t lcdDc, uint8_t spiSck, uint8_t spiMosi, uint8_t spiMiso) {
#if CCM_HAS_ARDUINO_GFX
  g_bus = new Arduino_ESP32SPI(lcdDc, lcdCs, spiSck, spiMosi, spiMiso, VSPI);
  g_gfx = new Arduino_ST7796(g_bus, lcdRst, 0, true, kWidth, kHeight);
  online_ = (g_gfx != nullptr) && g_gfx->begin();
  if (online_) {
    g_gfx->fillScreen(kBg);
    g_gfx->setTextWrap(false);
  }
#else
  (void)lcdCs;
  (void)lcdRst;
  (void)lcdDc;
  (void)spiSck;
  (void)spiMosi;
  (void)spiMiso;
  online_ = false;
#endif
  return online_;
}

void ScreenDashboard::tick(const state::VehicleState& s, uint32_t nowMs) {
  if (!online_) return;
  if ((nowMs - lastRenderMs_) < kRenderIntervalMs) return;
  lastRenderMs_ = nowMs;
  page_ = pageFromUi(s.ui_page);
  render(s);
}

void ScreenDashboard::render(const state::VehicleState& s) {
#if CCM_HAS_ARDUINO_GFX
  if (!g_gfx) return;
  g_gfx->fillScreen(kBg);
  drawHeader(s);
  drawTabs();
  switch (page_) {
    case Page::DASH:
      drawLiveCard(s);
      drawStatusCard(s);
      drawRaceCard(s);
      break;
    case Page::METH:
      drawStatusCard(s);
      drawControlCard(s);
      break;
    case Page::RACE:
      drawLiveCard(s);
      drawRaceCard(s);
      break;
    case Page::DIAG:
      drawDiagnosticsCard(s);
      break;
  }
#else
  (void)s;
#endif
}

void ScreenDashboard::drawHeader(const state::VehicleState& s) {
#if CCM_HAS_ARDUINO_GFX
  g_gfx->fillRect(0, 0, kWidth, 42, kPanel);
  g_gfx->drawRect(0, 0, kWidth, 42, kBorder);
  g_gfx->setTextColor(kText, kPanel);
  g_gfx->setTextSize(2);
  g_gfx->setCursor(8, 8);
  g_gfx->print("Cabin Master");

  g_gfx->setTextSize(1);
  g_gfx->setCursor(8, 30);
  g_gfx->setTextColor(s.touch_online ? kOk : kWarn, kPanel);
  g_gfx->print(s.touch_online ? "TOUCH ONLINE" : "TOUCH OFFLINE");
  if (millis() < actionFeedbackUntilMs_ && actionFeedback_[0] != '\0') {
    g_gfx->setTextColor(kOk, kPanel);
    g_gfx->setCursor(130, 30);
    g_gfx->print(actionFeedback_);
  }
#else
  (void)s;
#endif
}

void ScreenDashboard::drawTabs() {
#if CCM_HAS_ARDUINO_GFX
  drawButton(tabDashBtn_, "DASH", page_ == Page::DASH);
  drawButton(tabMethBtn_, "METH", page_ == Page::METH);
  drawButton(tabRaceBtn_, "RACE", page_ == Page::RACE);
  drawButton(tabDiagBtn_, "DIAG", page_ == Page::DIAG);
#endif
}

void ScreenDashboard::drawLiveCard(const state::VehicleState& s) {
#if CCM_HAS_ARDUINO_GFX
  g_gfx->fillRect(8, 84, 304, 90, kPanel);
  g_gfx->drawRect(8, 84, 304, 90, kBorder);
  g_gfx->setTextColor(kText, kPanel);
  g_gfx->setTextSize(2);
  g_gfx->setCursor(14, 92);
  g_gfx->printf("RPM %u", static_cast<unsigned>(s.rpm));
  g_gfx->setCursor(170, 92);
  g_gfx->printf("SPD %.1f", static_cast<double>(s.speed));

  g_gfx->setTextSize(1);
  g_gfx->setTextColor(kSubtle, kPanel);
  g_gfx->setCursor(14, 122);
  g_gfx->printf("BAT %.1fV", static_cast<double>(s.battery_voltage));
  g_gfx->setCursor(110, 122);
  g_gfx->printf("CAB %.1fC", static_cast<double>(s.cabin_temp));
  g_gfx->setCursor(210, 122);
  g_gfx->printf("OUT %.1fC", static_cast<double>(s.outside_temp));

  g_gfx->setCursor(14, 142);
  g_gfx->printf("BOOST %.0fkPa", static_cast<double>(s.boost_kpa));
  g_gfx->setCursor(138, 142);
  g_gfx->printf("IAT %.1fC", static_cast<double>(s.intake_temp));
  g_gfx->setCursor(220, 142);
  g_gfx->printf("GPS %u", static_cast<unsigned>(s.gps_satellites));
#else
  (void)s;
#endif
}

void ScreenDashboard::drawStatusCard(const state::VehicleState& s) {
#if CCM_HAS_ARDUINO_GFX
  g_gfx->fillRect(8, 182, 304, 60, kPanel);
  g_gfx->drawRect(8, 182, 304, 60, kBorder);
  g_gfx->setTextColor(kText, kPanel);
  g_gfx->setTextSize(1);
  g_gfx->setCursor(14, 192);
  g_gfx->print("CAN:");
  g_gfx->setTextColor(s.can_online ? kOk : kWarn, kPanel);
  g_gfx->print(s.can_online ? "ONLINE" : "OFFLINE");

  g_gfx->setTextColor(kText, kPanel);
  g_gfx->setCursor(120, 192);
  g_gfx->print("METH:");
  g_gfx->setTextColor(s.meth_online ? kOk : kWarn, kPanel);
  g_gfx->print(s.meth_online ? "ONLINE" : "OFFLINE");

  g_gfx->setTextColor(kText, kPanel);
  g_gfx->setCursor(228, 192);
  g_gfx->print("GPS:");
  g_gfx->setTextColor(s.gps_fix ? kOk : kWarn, kPanel);
  g_gfx->print(s.gps_fix ? "FIX" : "NOFIX");

  g_gfx->setTextColor(kSubtle, kPanel);
  g_gfx->setCursor(14, 212);
  g_gfx->printf("Faults:0x%04X", static_cast<unsigned>(s.fault_flags));
  g_gfx->setCursor(140, 212);
  g_gfx->printf("Touch:%s", s.touch_online ? "OK" : "BAD");
  g_gfx->setCursor(230, 212);
  g_gfx->printf("UI %.0f", static_cast<double>(s.ui_fps));
#else
  (void)s;
#endif
}

void ScreenDashboard::drawButton(const Rect& r, const char* text, bool active) {
#if CCM_HAS_ARDUINO_GFX
  const uint16_t bg = (r.y == tabDashBtn_.y) ? (active ? kTabActive : kTab) : (active ? kBtnActive : kBtn);
  g_gfx->fillRect(r.x, r.y, r.w, r.h, bg);
  g_gfx->drawRect(r.x, r.y, r.w, r.h, kBorder);
  g_gfx->setTextColor(kText, bg);
  g_gfx->setTextSize(1);
  g_gfx->setCursor(r.x + 8, r.y + 16);
  g_gfx->print(text);
#else
  (void)r;
  (void)text;
  (void)active;
#endif
}

void ScreenDashboard::drawControlCard(const state::VehicleState& s) {
#if CCM_HAS_ARDUINO_GFX
  g_gfx->fillRect(8, 248, 304, 220, kPanel);
  g_gfx->drawRect(8, 248, 304, 220, kBorder);
  g_gfx->setTextColor(kText, kPanel);
  g_gfx->setTextSize(1);
  g_gfx->setCursor(14, 258);
  g_gfx->printf("Water Meth  State:%u Duty:%u", static_cast<unsigned>(s.meth_state), static_cast<unsigned>(s.meth_pump_duty));
  g_gfx->setCursor(14, 276);
  g_gfx->printf("Ratio:%u%%  Trigger:%ukPa  MaxDuty:%u", static_cast<unsigned>(s.meth_selected_ratio_percent),
                static_cast<unsigned>(s.meth_boost_trigger_kpa), static_cast<unsigned>(s.meth_max_pump_duty));
  g_gfx->setCursor(14, 294);
  g_gfx->printf("Tank:%u%%  Flow:%u  DesiredArm:%u", static_cast<unsigned>(s.meth_tank_level), static_cast<unsigned>(s.meth_flow_status),
                s.meth_desired_armed ? 1U : 0U);

  drawButton(methArmBtn_, s.meth_desired_armed ? "DISARM" : "ARM", s.meth_desired_armed);
  char ratioLabel[32];
  snprintf(ratioLabel, sizeof(ratioLabel), "RATIO %u%%", static_cast<unsigned>(s.meth_selected_ratio_percent));
  drawButton(methRatioBtn_, ratioLabel, false);
#else
  (void)s;
#endif
}

void ScreenDashboard::drawRaceCard(const state::VehicleState& s) {
#if CCM_HAS_ARDUINO_GFX
  g_gfx->fillRect(8, 248, 304, 220, kPanel);
  g_gfx->drawRect(8, 248, 304, 220, kBorder);
  g_gfx->setTextColor(kText, kPanel);
  g_gfx->setTextSize(1);
  g_gfx->setCursor(14, 258);
  g_gfx->printf("Race Mode:%u Running:%u Quality:%u%%", static_cast<unsigned>(s.race_mode), s.race_running ? 1U : 0U,
                static_cast<unsigned>(s.race_quality_percent));
  g_gfx->setCursor(14, 276);
  g_gfx->printf("0-60: %.3fs   1/4: %.3fs", static_cast<double>(s.race_0_60_s), static_cast<double>(s.race_quarter_mile_et_s));
  g_gfx->setCursor(14, 294);
  g_gfx->printf("Lap best: %.3fs  Laps: %u", static_cast<double>(s.race_best_lap_s), static_cast<unsigned>(s.race_lap_count));

  drawButton(raceStartAccelBtn_, "ACCEL", s.race_running && s.race_mode == state::RaceMode::ACCEL);
  drawButton(raceStartLapBtn_, "LAP", s.race_running && s.race_mode == state::RaceMode::LAP);
  drawButton(raceStopBtn_, "STOP", false);
  drawButton(raceResetBtn_, "RESET", false);
#else
  (void)s;
#endif
}

void ScreenDashboard::drawDiagnosticsCard(const state::VehicleState& s) {
#if CCM_HAS_ARDUINO_GFX
  g_gfx->fillRect(8, 84, 304, 384, kPanel);
  g_gfx->drawRect(8, 84, 304, 384, kBorder);
  g_gfx->setTextColor(kText, kPanel);
  g_gfx->setTextSize(1);
  g_gfx->setCursor(14, 96);
  g_gfx->print("Diagnostics");
  g_gfx->setCursor(14, 120);
  g_gfx->printf("CAN RX/TX: %lu / %lu", static_cast<unsigned long>(s.can_rx_count), static_cast<unsigned long>(s.can_tx_count));
  g_gfx->setCursor(14, 140);
  g_gfx->printf("Last CAN RX/TX ID: %u / %u", static_cast<unsigned>(s.can_last_rx_id), static_cast<unsigned>(s.can_last_tx_id));
  g_gfx->setCursor(14, 160);
  g_gfx->printf("Fault flags: 0x%04X", static_cast<unsigned>(s.fault_flags));
  g_gfx->setCursor(14, 180);
  g_gfx->printf("Heap: %lu bytes", static_cast<unsigned long>(s.heap_free_bytes));
  g_gfx->setCursor(14, 200);
  g_gfx->printf("ESP die temp: %dC", static_cast<int>(s.esp_die_temp_c));
  g_gfx->setCursor(14, 220);
  g_gfx->printf("SD mounted: %u", s.sd_mounted ? 1U : 0U);
  g_gfx->setCursor(14, 240);
  g_gfx->printf("SD errors: %lu", static_cast<unsigned long>(s.sd_write_error_count));
  g_gfx->setCursor(14, 260);
  g_gfx->printf("Uptime: %lus", static_cast<unsigned long>(s.uptime_ms / 1000UL));
  g_gfx->setCursor(14, 280);
  g_gfx->printf("GPS fix/sat: %u / %u", s.gps_fix ? 1U : 0U, static_cast<unsigned>(s.gps_satellites));
  g_gfx->setCursor(14, 300);
  g_gfx->printf("Touch online: %u", s.touch_online ? 1U : 0U);
  g_gfx->setCursor(14, 320);
  g_gfx->printf("UI FPS: %.1f", static_cast<double>(s.ui_fps));
  g_gfx->setCursor(14, 340);
  g_gfx->printf("Log file: %.26s", s.current_log_file);
  g_gfx->setCursor(14, 360);
  g_gfx->printf("SD status: %.26s", s.last_sd_write_status);
#else
  (void)s;
#endif
}

uint8_t ScreenDashboard::nextMethRatio(uint8_t current) const {
  if (current < 25U) return 25U;
  if (current < 50U) return 50U;
  if (current < 75U) return 75U;
  if (current < 100U) return 100U;
  return 25U;
}

void ScreenDashboard::handleTouch(const touch::TouchSample& sample, uint32_t nowMs) {
  const touch::TouchSample normalized = normalizeTouch(sample);
  if (!normalized.touched) {
    touchActive_ = false;
    return;
  }

  if (touchActive_ || ((nowMs - lastTouchMs_) < kTouchDebounceMs)) return;
  touchActive_ = true;
  lastTouchMs_ = nowMs;

  state::g_vehicle_state.mutate([&](state::VehicleState& s) {
    s.input_flags |= can_protocol::input_flag::TOUCH;
  });

  if (tabDashBtn_.contains(normalized.x, normalized.y)) {
    setPage(Page::DASH);
    setActionFeedback("PAGE DASH", nowMs);
    return;
  }
  if (tabMethBtn_.contains(normalized.x, normalized.y)) {
    setPage(Page::METH);
    setActionFeedback("PAGE METH", nowMs);
    return;
  }
  if (tabRaceBtn_.contains(normalized.x, normalized.y)) {
    setPage(Page::RACE);
    setActionFeedback("PAGE RACE", nowMs);
    return;
  }
  if (tabDiagBtn_.contains(normalized.x, normalized.y)) {
    setPage(Page::DIAG);
    setActionFeedback("PAGE DIAG", nowMs);
    return;
  }

  if (methArmBtn_.contains(normalized.x, normalized.y)) {
    const state::VehicleState s = state::g_vehicle_state.read();
    if (canMgr_) {
      const bool arm = !s.meth_desired_armed;
      const bool sent = canMgr_->sendMethArm(arm);
      if (sent) {
        state::g_vehicle_state.mutate([&](state::VehicleState& live) { live.meth_desired_armed = arm; });
        setActionFeedback(arm ? "METH ARMED" : "METH DISARMED", nowMs);
      } else {
        setActionFeedback("METH CMD REJECTED", nowMs);
      }
    }
    return;
  }

  if (methRatioBtn_.contains(normalized.x, normalized.y)) {
    const state::VehicleState s = state::g_vehicle_state.read();
    const uint8_t ratio = nextMethRatio(s.meth_selected_ratio_percent);
    state::g_vehicle_state.mutate([&](state::VehicleState& live) { live.meth_selected_ratio_percent = ratio; });
    if (settingsMgr_) {
      settingsMgr_->updateFromState(state::g_vehicle_state.read());
      settingsMgr_->save();
    }
    if (canMgr_) {
      canMgr_->sendMethConfigBroadcast();
    }
    setActionFeedback("METH RATIO UPDATED", nowMs);
    return;
  }

  if (!raceMgr_) return;
  if (raceStartAccelBtn_.contains(normalized.x, normalized.y)) {
    raceMgr_->startRun(state::RaceMode::ACCEL);
    setActionFeedback("RACE ACCEL START", nowMs);
    return;
  }
  if (raceStartLapBtn_.contains(normalized.x, normalized.y)) {
    raceMgr_->startRun(state::RaceMode::LAP);
    setActionFeedback("RACE LAP START", nowMs);
    return;
  }
  if (raceStopBtn_.contains(normalized.x, normalized.y)) {
    raceMgr_->stopRun();
    setActionFeedback("RACE STOPPED", nowMs);
    return;
  }
  if (raceResetBtn_.contains(normalized.x, normalized.y)) {
    raceMgr_->resetSession();
    setActionFeedback("RACE RESET", nowMs);
  }
}

void ScreenDashboard::setPage(Page page) {
  page_ = page;
  state::g_vehicle_state.mutate([&](state::VehicleState& s) { s.ui_page = uiPageFor(page); });
}

void ScreenDashboard::setActionFeedback(const char* text, uint32_t nowMs) {
  if (!text) return;
  strncpy(actionFeedback_, text, sizeof(actionFeedback_) - 1);
  actionFeedback_[sizeof(actionFeedback_) - 1] = '\0';
  actionFeedbackUntilMs_ = nowMs + kActionFeedbackMs;
}

uint8_t ScreenDashboard::uiPageFor(Page page) const {
  switch (page) {
    case Page::DASH:
      return static_cast<uint8_t>(can_protocol::UiPage::DASH);
    case Page::METH:
      return static_cast<uint8_t>(can_protocol::UiPage::METH);
    case Page::RACE:
      return static_cast<uint8_t>(can_protocol::UiPage::ENVIRONMENT);
    case Page::DIAG:
      return static_cast<uint8_t>(can_protocol::UiPage::DIAGNOSTICS);
  }
  return static_cast<uint8_t>(can_protocol::UiPage::DASH);
}

ScreenDashboard::Page ScreenDashboard::pageFromUi(uint8_t uiPage) const {
  if (uiPage == static_cast<uint8_t>(can_protocol::UiPage::METH)) return Page::METH;
  if (uiPage == static_cast<uint8_t>(can_protocol::UiPage::DIAGNOSTICS)) return Page::DIAG;
  if (uiPage == static_cast<uint8_t>(can_protocol::UiPage::ENVIRONMENT)) return Page::RACE;
  return Page::DASH;
}

touch::TouchSample ScreenDashboard::normalizeTouch(const touch::TouchSample& sample) const {
  if (!sample.touched) return sample;
  touch::TouchSample t = sample;

  // Some FT62xx-style controllers report coordinates in a 480x320 landscape frame while
  // the dashboard is rendered in 320x480 portrait. When values fit that pattern, rotate
  // + remap into portrait space before hit-testing.
  if (t.x <= kHeight && t.y <= kWidth) {
    const uint16_t x = t.x;
    t.x = t.y;
    t.y = static_cast<uint16_t>(kHeight > x ? (kHeight - x) : 0U);
  }

  // Fallback scaling for controllers reporting raw 12-bit (0..4095) coordinate ranges.
  if (t.x > kWidth) {
    t.x = static_cast<uint16_t>((static_cast<uint32_t>(t.x) * kWidth) / 4095U);
  }
  if (t.y > kHeight) {
    t.y = static_cast<uint16_t>((static_cast<uint32_t>(t.y) * kHeight) / 4095U);
  }
  return t;
}

}  // namespace ui
