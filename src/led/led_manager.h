#pragma once

#include <Arduino.h>
#include <FastLED.h>

#include "state/vehicle_state.h"

namespace led {

enum class LedUiMode : uint8_t {
  Off = 0,
  LowWhite = 1,
  HighWhite = 2,
};

class LedManager {
 public:
  bool begin(uint8_t pin1, uint8_t pin2, uint8_t pin3,
             uint16_t ledsChannel1 = 16, uint16_t ledsChannel2 = 0,
             uint16_t ledsChannel3 = 0,
             uint16_t ledsChannel3Offset = 0);
  void tick(const state::VehicleState& s);
  void setLedUiMode(LedUiMode mode);
  void triggerStartupSweep();

 private:
  struct Channel {
    CRGB* leds = nullptr;
    uint16_t count = 0;
    uint16_t offset = 0;
    bool enabled = false;
    state::LedMode mode = state::LedMode::OFF;
    uint8_t brightness = 0;
  };
  struct ZoneSegment {
    uint8_t channel = 0;
    uint16_t start = 0;
    uint16_t count = 0;
  };

  void clearAll();
  void clearInterior();
  void fillChannel(Channel& ch, const CRGB& color);
  void fillZone(uint8_t zone, const CRGB& color);
  void configureZones();
  void applyUiMode(LedUiMode mode, bool forceLog);
  void renderRpmGauge(const state::VehicleState& s);
  void renderStartupSweep(uint32_t nowMs);
  void renderFallbackModes(const state::VehicleState& s, uint32_t nowMs);
  static bool uiModeFromState(const state::VehicleState& s, LedUiMode& mode);
  static const char* uiModeName(LedUiMode mode);
  static uint8_t uiModeBrightness(LedUiMode mode);
  static CRGB whiteForBrightness(uint8_t brightness);

  Channel channels_[3];
  ZoneSegment zones_[state::kLedZoneCount];
  bool started_ = false;
  bool startupSweepActive_ = false;
  uint32_t startupStartMs_ = 0;
  uint32_t lastFrameMs_ = 0;
  bool modeApplied_ = false;
  LedUiMode currentMode_ = LedUiMode::Off;
};

}  // namespace led
