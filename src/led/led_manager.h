#pragma once

#include <Adafruit_NeoPixel.h>
#include <Arduino.h>

#include "state/vehicle_state.h"

namespace led {

class LedManager {
 public:
  // ledsChannel3: if non-zero, overrides ledsPerChannel for channel 3 only.
  // ledsChannel3Offset: first N pixels on channel 3 are always kept dark.
  bool begin(uint8_t pin1, uint8_t pin2, uint8_t pin3,
             uint16_t ledsPerChannel = 16, uint16_t ledsChannel3 = 0,
             uint16_t ledsChannel3Offset = 0);
  void tick(const state::VehicleState& s);
  void triggerStartupSweep();

 private:
  struct Channel {
    Adafruit_NeoPixel* strip = nullptr;
    bool enabled = true;
    uint32_t color = 0;
    state::LedMode mode = state::LedMode::OFF;
    uint8_t brightness = 0;
    uint16_t ledOffset = 0;  // first N pixels are always dark; pattern runs on [offset, n)
  };

  void renderChannel(Channel& ch, const state::VehicleState& s, uint32_t nowMs, uint8_t idx);
  void renderRpmStartup(Channel& ch, uint32_t elapsed);
  uint16_t effectiveRpm(const state::VehicleState& s) const;
  uint32_t scaleColor(uint32_t rgb, uint8_t brightness) const;
  uint32_t wheel(uint8_t p) const;
  static uint8_t rpmGaugeLitCount(uint16_t rpm, uint16_t numLeds);
  static uint32_t rpmBandColor(uint16_t rpm);
  static uint32_t rpmGaugeColor(uint16_t ledIdx, uint16_t numLeds);

  Adafruit_NeoPixel strip1_{1, 1, NEO_GRB + NEO_KHZ800};
  Adafruit_NeoPixel strip2_{1, 1, NEO_GRB + NEO_KHZ800};
  Adafruit_NeoPixel strip3_{1, 1, NEO_GRB + NEO_KHZ800};
  Channel channels_[3];

  uint16_t ledsPerChannel_ = 16;
  bool started_ = false;
  bool startupSweepActive_ = true;
  uint32_t startupStartMs_ = 0;
  uint32_t lastFrameMs_ = 0;
};

}  // namespace led
