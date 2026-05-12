#include "ui/screen_dashboard.h"

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
  render(s);
}

void ScreenDashboard::render(const state::VehicleState& s) {
#if CCM_HAS_ARDUINO_GFX
  if (!g_gfx) return;
  g_gfx->fillScreen(kBg);
  drawHeader(s);
  drawLiveCard(s);
  drawStatusCard(s);
  drawControlCard(s);
  drawRaceCard(s);
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
  g_gfx->setCursor(8, 10);
  g_gfx->print("Foxbody Cabin Master");

  g_gfx->setTextSize(1);
  g_gfx->setCursor(8, 30);
  g_gfx->setTextColor(s.touch_online ? kOk : kWarn, kPanel);
  g_gfx->print(s.touch_online ? "TOUCH ONLINE" : "TOUCH OFFLINE");
#else
  (void)s;
#endif
}

void ScreenDashboard::drawLiveCard(const state::VehicleState& s) {
#if CCM_HAS_ARDUINO_GFX
  g_gfx->fillRect(8, 50, 304, 90, kPanel);
  g_gfx->drawRect(8, 50, 304, 90, kBorder);
  g_gfx->setTextColor(kText, kPanel);
  g_gfx->setTextSize(2);
  g_gfx->setCursor(14, 58);
  g_gfx->printf("RPM %u", static_cast<unsigned>(s.rpm));
  g_gfx->setCursor(170, 58);
  g_gfx->printf("SPD %.1f", static_cast<double>(s.speed));

  g_gfx->setTextSize(1);
  g_gfx->setTextColor(kSubtle, kPanel);
  g_gfx->setCursor(14, 88);
  g_gfx->printf("BAT %.1fV", static_cast<double>(s.battery_voltage));
  g_gfx->setCursor(110, 88);
  g_gfx->printf("CAB %.1fC", static_cast<double>(s.cabin_temp));
  g_gfx->setCursor(210, 88);
  g_gfx->printf("OUT %.1fC", static_cast<double>(s.outside_temp));

  g_gfx->setCursor(14, 108);
  g_gfx->printf("BOOST %.0fkPa", static_cast<double>(s.boost_kpa));
  g_gfx->setCursor(138, 108);
  g_gfx->printf("IAT %.1fC", static_cast<double>(s.intake_temp));
  g_gfx->setCursor(220, 108);
  g_gfx->printf("GPS %u", static_cast<unsigned>(s.gps_satellites));
#else
  (void)s;
#endif
}

void ScreenDashboard::drawStatusCard(const state::VehicleState& s) {
#if CCM_HAS_ARDUINO_GFX
  g_gfx->fillRect(8, 146, 304, 60, kPanel);
  g_gfx->drawRect(8, 146, 304, 60, kBorder);
  g_gfx->setTextColor(kText, kPanel);
  g_gfx->setTextSize(1);
  g_gfx->setCursor(14, 156);
  g_gfx->print("CAN:");
  g_gfx->setTextColor(s.can_online ? kOk : kWarn, kPanel);
  g_gfx->print(s.can_online ? "ONLINE" : "OFFLINE");

  g_gfx->setTextColor(kText, kPanel);
  g_gfx->setCursor(120, 156);
  g_gfx->print("METH:");
  g_gfx->setTextColor(s.meth_online ? kOk : kWarn, kPanel);
  g_gfx->print(s.meth_online ? "ONLINE" : "OFFLINE");

  g_gfx->setTextColor(kText, kPanel);
  g_gfx->setCursor(228, 156);
  g_gfx->print("GPS:");
  g_gfx->setTextColor(s.gps_fix ? kOk : kWarn, kPanel);
  g_gfx->print(s.gps_fix ? "FIX" : "NOFIX");

  g_gfx->setTextColor(kSubtle, kPanel);
  g_gfx->setCursor(14, 176);
  g_gfx->printf("Faults:0x%04X", static_cast<unsigned>(s.fault_flags));
  g_gfx->setCursor(140, 176);
  g_gfx->printf("Touch:%s", s.touch_online ? "OK" : "BAD");
  g_gfx->setCursor(230, 176);
  g_gfx->printf("UI %.0f", static_cast<double>(s.ui_fps));
#else
  (void)s;
#endif
}

void ScreenDashboard::drawButton(const Rect& r, const char* text, bool active) {
#if CCM_HAS_ARDUINO_GFX
  g_gfx->fillRect(r.x, r.y, r.w, r.h, active ? kBtnActive : kBtn);
  g_gfx->drawRect(r.x, r.y, r.w, r.h, kBorder);
  g_gfx->setTextColor(kText, active ? kBtnActive : kBtn);
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
  g_gfx->fillRect(8, 212, 304, 96, kPanel);
  g_gfx->drawRect(8, 212, 304, 96, kBorder);
  g_gfx->setTextColor(kText, kPanel);
  g_gfx->setTextSize(1);
  g_gfx->setCursor(14, 220);
  g_gfx->printf("Water Meth  State:%u Duty:%u", static_cast<unsigned>(s.meth_state), static_cast<unsigned>(s.meth_pump_duty));
  g_gfx->setCursor(14, 236);
  g_gfx->printf("Ratio:%u%%  Trigger:%ukPa  MaxDuty:%u", static_cast<unsigned>(s.meth_selected_ratio_percent),
                static_cast<unsigned>(s.meth_boost_trigger_kpa), static_cast<unsigned>(s.meth_max_pump_duty));

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
  g_gfx->fillRect(8, 314, 304, 158, kPanel);
  g_gfx->drawRect(8, 314, 304, 158, kBorder);
  g_gfx->setTextColor(kText, kPanel);
  g_gfx->setTextSize(1);
  g_gfx->setCursor(14, 322);
  g_gfx->printf("Race Mode:%u Running:%u Quality:%u%%", static_cast<unsigned>(s.race_mode), s.race_running ? 1U : 0U,
                static_cast<unsigned>(s.race_quality_percent));
  g_gfx->setCursor(14, 338);
  g_gfx->printf("0-60: %.3fs   1/4: %.3fs", static_cast<double>(s.race_0_60_s), static_cast<double>(s.race_quarter_mile_et_s));
  g_gfx->setCursor(14, 354);
  g_gfx->printf("Lap best: %.3fs  Laps: %u", static_cast<double>(s.race_best_lap_s), static_cast<unsigned>(s.race_lap_count));

  drawButton(raceStartAccelBtn_, "ACCEL", s.race_running && s.race_mode == state::RaceMode::ACCEL);
  drawButton(raceStartLapBtn_, "LAP", s.race_running && s.race_mode == state::RaceMode::LAP);
  drawButton(raceStopBtn_, "STOP", false);
  drawButton(raceResetBtn_, "RESET", false);
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
    s.ui_page = 0;
  });

  if (methArmBtn_.contains(normalized.x, normalized.y)) {
    const state::VehicleState s = state::g_vehicle_state.read();
    if (canMgr_) {
      const bool arm = !s.meth_desired_armed;
      const bool sent = canMgr_->sendMethArm(arm);
      if (sent) {
        state::g_vehicle_state.mutate([&](state::VehicleState& live) { live.meth_desired_armed = arm; });
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
    return;
  }

  if (!raceMgr_) return;
  if (raceStartAccelBtn_.contains(normalized.x, normalized.y)) {
    raceMgr_->startRun(state::RaceMode::ACCEL);
    return;
  }
  if (raceStartLapBtn_.contains(normalized.x, normalized.y)) {
    raceMgr_->startRun(state::RaceMode::LAP);
    return;
  }
  if (raceStopBtn_.contains(normalized.x, normalized.y)) {
    raceMgr_->stopRun();
    return;
  }
  if (raceResetBtn_.contains(normalized.x, normalized.y)) {
    raceMgr_->resetSession();
  }
}

touch::TouchSample ScreenDashboard::normalizeTouch(const touch::TouchSample& sample) const {
  if (!sample.touched) return sample;
  touch::TouchSample t = sample;

  if (t.x <= kHeight && t.y <= kWidth) {
    const uint16_t x = t.x;
    t.x = t.y;
    t.y = static_cast<uint16_t>(kHeight > x ? (kHeight - x) : 0U);
  }

  if (t.x > kWidth) {
    t.x = static_cast<uint16_t>((static_cast<uint32_t>(t.x) * kWidth) / 4095U);
  }
  if (t.y > kHeight) {
    t.y = static_cast<uint16_t>((static_cast<uint32_t>(t.y) * kHeight) / 4095U);
  }
  return t;
}

}  // namespace ui
