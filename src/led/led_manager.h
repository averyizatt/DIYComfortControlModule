#pragma once

#include <Adafruit_NeoPixel.h>
#include <Arduino.h>

#include "state/vehicle_state.h"

namespace led {

#ifndef CCM_LED_PIXEL_TYPE_CH1
#define CCM_LED_PIXEL_TYPE_CH1 (NEO_GRB + NEO_KHZ800)
#endif

#ifndef CCM_LED_PIXEL_TYPE_CH2
#define CCM_LED_PIXEL_TYPE_CH2 (NEO_GRB + NEO_KHZ800)
#endif

#ifndef CCM_LED_PIXEL_TYPE_CH3
#define CCM_LED_PIXEL_TYPE_CH3 (NEO_GRB + NEO_KHZ800)
#endif

#ifndef CCM_LED_CH1_HAS_WHITE
#define CCM_LED_CH1_HAS_WHITE 0
#endif

#ifndef CCM_LED_CH2_HAS_WHITE
#define CCM_LED_CH2_HAS_WHITE 0
#endif

#ifndef CCM_LED_CH3_HAS_WHITE
#define CCM_LED_CH3_HAS_WHITE 0
#endif

class LedManager {
 public:
  // Channel 1 is the RPM strip. Channels 2/3 can over-send for solid interior
  // lighting so their exact LED counts do not need to be known up front.
  // ledsChannel2/3: if zero, fall back to ledsChannel1.
  // ledsChannel3Offset: first N pixels on channel 3 are always kept dark.
  bool begin(uint8_t pin1, uint8_t pin2, uint8_t pin3,
             uint16_t ledsChannel1 = 16, uint16_t ledsChannel2 = 0,
             uint16_t ledsChannel3 = 0,
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
  void renderStartupAnimation(Channel& ch, uint32_t elapsed, uint8_t idx);
  uint16_t effectiveRpm(const state::VehicleState& s, uint32_t nowMs) const;
  uint32_t scaleColor(uint32_t rgb, uint8_t brightness) const;
  uint32_t whiteColor(uint8_t brightness, uint8_t idx) const;
  uint32_t wheel(uint8_t p) const;
  static uint8_t rpmGaugeLitCount(uint16_t rpm, uint16_t numLeds);
  static uint32_t rpmBandColor(uint16_t rpm);
  static uint32_t rpmGaugeColor(uint16_t ledIdx, uint16_t numLeds);

  Adafruit_NeoPixel strip1_{1, 1, CCM_LED_PIXEL_TYPE_CH1};
  Adafruit_NeoPixel strip2_{1, 1, CCM_LED_PIXEL_TYPE_CH2};
  Adafruit_NeoPixel strip3_{1, 1, CCM_LED_PIXEL_TYPE_CH3};
  Channel channels_[3];

  uint16_t ledsPerChannel_ = 16;
  uint16_t ledsChannel2_ = 16;
  uint16_t ledsChannel3_ = 16;
  bool started_ = false;
  bool startupSweepActive_ = true;
  uint32_t startupStartMs_ = 0;
  uint32_t lastFrameMs_ = 0;
};

}  // namespace led
