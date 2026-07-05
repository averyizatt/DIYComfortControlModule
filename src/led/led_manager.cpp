#include "led/led_manager.h"

#include "pin_map.h"

namespace led {

namespace {
#ifndef CCM_LED_FRAME_INTERVAL_MS
#define CCM_LED_FRAME_INTERVAL_MS 33
#endif

#ifndef CCM_MAIN_LED_COUNT
#define CCM_MAIN_LED_COUNT 18
#endif

#ifndef CCM_INTERIOR_LED_SEND_COUNT
#define CCM_INTERIOR_LED_SEND_COUNT 180
#endif

#ifndef CCM_LED_LOW_WHITE_BRIGHTNESS
#define CCM_LED_LOW_WHITE_BRIGHTNESS 35
#endif

#ifndef CCM_LED_HIGH_WHITE_BRIGHTNESS
#define CCM_LED_HIGH_WHITE_BRIGHTNESS 180
#endif

#ifndef CCM_LED_DRIVER_CHANNEL
#define CCM_LED_DRIVER_CHANNEL 3
#endif
#ifndef CCM_LED_DRIVER_START
#define CCM_LED_DRIVER_START 0
#endif
#ifndef CCM_LED_DRIVER_COUNT
#define CCM_LED_DRIVER_COUNT 0
#endif
#ifndef CCM_LED_PASSENGER_CHANNEL
#define CCM_LED_PASSENGER_CHANNEL 3
#endif
#ifndef CCM_LED_PASSENGER_START
#define CCM_LED_PASSENGER_START 0
#endif
#ifndef CCM_LED_PASSENGER_COUNT
#define CCM_LED_PASSENGER_COUNT 0
#endif
#ifndef CCM_LED_FRONT_CHANNEL
#define CCM_LED_FRONT_CHANNEL 2
#endif
#ifndef CCM_LED_FRONT_START
#define CCM_LED_FRONT_START 0
#endif
#ifndef CCM_LED_FRONT_COUNT
#define CCM_LED_FRONT_COUNT 0
#endif
#ifndef CCM_LED_BACK_CHANNEL
#define CCM_LED_BACK_CHANNEL 2
#endif
#ifndef CCM_LED_BACK_START
#define CCM_LED_BACK_START 0
#endif
#ifndef CCM_LED_BACK_COUNT
#define CCM_LED_BACK_COUNT 0
#endif

constexpr uint32_t kFrameIntervalMs =
    (CCM_LED_FRAME_INTERVAL_MS < 16) ? 16U : static_cast<uint32_t>(CCM_LED_FRAME_INTERVAL_MS);
constexpr uint32_t kStartupDurationMs = 2200;
constexpr uint8_t kLowWhiteBrightness = CCM_LED_LOW_WHITE_BRIGHTNESS;
constexpr uint8_t kHighWhiteBrightness = CCM_LED_HIGH_WHITE_BRIGHTNESS;
constexpr uint16_t kMaxCh1 = CCM_MAIN_LED_COUNT;
constexpr uint16_t kMaxCh2 = CCM_INTERIOR_LED_SEND_COUNT;
constexpr uint16_t kMaxCh3 = CCM_INTERIOR_LED_SEND_COUNT;
constexpr uint16_t kRpmGaugeIdle = 800;
constexpr uint16_t kRpmGaugeMax = 6500;
constexpr uint16_t kRpmGaugeYellow = 4500;
constexpr uint16_t kRpmGaugeRed = 5600;

CRGB g_ch1[kMaxCh1];
CRGB g_ch2[kMaxCh2];
CRGB g_ch3[kMaxCh3];

uint16_t clampCount(uint16_t requested, uint16_t maxCount) {
  if (requested == 0) return maxCount;
  return min<uint16_t>(requested, maxCount);
}

uint8_t clampInteriorChannelIndex(uint8_t oneBased) {
  if (oneBased < 2U) return 1;
  if (oneBased > 3U) return 2;
  return static_cast<uint8_t>(oneBased - 1U);
}

const char* modeName(state::LedMode mode) {
  switch (mode) {
    case state::LedMode::OFF: return "OFF";
    case state::LedMode::STATIC_COLOR: return "COLOR";
    case state::LedMode::LOW_LIGHT: return "LOW";
    case state::LedMode::HIGH_LIGHT: return "HIGH";
    case state::LedMode::RAINBOW: return "RAINBOW";
    case state::LedMode::BREATHING: return "BREATHE";
    case state::LedMode::RPM_REACTIVE: return "CHASE";
    case state::LedMode::WARNING_FLASH: return "SPARKLE";
    case state::LedMode::RPM_GAUGE: return "RPM";
    default: return "OTHER";
  }
}

const char* zoneName(uint8_t zone) {
  switch (zone) {
    case 0: return "DRIVER";
    case 1: return "PASS";
    case 2: return "FRONT";
    case 3: return "BACK";
    default: return "?";
  }
}
}  // namespace

const char* LedManager::uiModeName(LedUiMode mode) {
  switch (mode) {
    case LedUiMode::Off: return "OFF";
    case LedUiMode::LowWhite: return "LOW_WHITE";
    case LedUiMode::HighWhite: return "HIGH_WHITE";
    default: return "UNKNOWN";
  }
}

uint8_t LedManager::uiModeBrightness(LedUiMode mode) {
  switch (mode) {
    case LedUiMode::LowWhite: return kLowWhiteBrightness;
    case LedUiMode::HighWhite: return kHighWhiteBrightness;
    case LedUiMode::Off:
    default: return 0;
  }
}

CRGB LedManager::whiteForBrightness(uint8_t brightness) {
  return CRGB(brightness, brightness, brightness);
}

CRGB colorFromRgb(uint32_t rgb, uint8_t brightness) {
  CRGB color(static_cast<uint8_t>((rgb >> 16) & 0xFFU),
             static_cast<uint8_t>((rgb >> 8) & 0xFFU),
             static_cast<uint8_t>(rgb & 0xFFU));
  color.nscale8_video(brightness);
  return color;
}

void LedManager::configureZones() {
  auto setZone = [&](uint8_t zone, uint8_t channelOneBased, uint16_t start, uint16_t count) {
    if (zone >= state::kLedZoneCount) return;
    const uint8_t channel = clampInteriorChannelIndex(channelOneBased);
    zones_[zone].channel = channel;
    zones_[zone].start = min<uint16_t>(start, channels_[channel].count);
    const uint16_t available = channels_[channel].count - zones_[zone].start;
    zones_[zone].count = (count == 0U) ? available : min<uint16_t>(count, available);
  };

  const uint16_t ch2Half = channels_[1].count / 2U;
  const uint16_t ch3Half = channels_[2].count / 2U;

  setZone(0, CCM_LED_DRIVER_CHANNEL, CCM_LED_DRIVER_START,
          CCM_LED_DRIVER_COUNT ? CCM_LED_DRIVER_COUNT : ch3Half);
  setZone(1, CCM_LED_PASSENGER_CHANNEL,
          CCM_LED_PASSENGER_COUNT ? CCM_LED_PASSENGER_START :
                                    (CCM_LED_PASSENGER_START ? CCM_LED_PASSENGER_START : ch3Half),
          CCM_LED_PASSENGER_COUNT ? CCM_LED_PASSENGER_COUNT : (channels_[2].count - ch3Half));
  setZone(2, CCM_LED_FRONT_CHANNEL, CCM_LED_FRONT_START,
          CCM_LED_FRONT_COUNT ? CCM_LED_FRONT_COUNT : ch2Half);
  setZone(3, CCM_LED_BACK_CHANNEL,
          CCM_LED_BACK_COUNT ? CCM_LED_BACK_START :
                               (CCM_LED_BACK_START ? CCM_LED_BACK_START : ch2Half),
          CCM_LED_BACK_COUNT ? CCM_LED_BACK_COUNT : (channels_[1].count - ch2Half));
}

bool LedManager::uiModeFromState(const state::VehicleState& s, LedUiMode& mode) {
  bool allOff = true;
  bool allLow = true;
  bool allHigh = true;
  for (uint8_t i = 0; i < state::kLedZoneCount; ++i) {
    const bool enabled = s.led_zone_enabled[i] && s.led_zone_mode[i] != state::LedMode::OFF;
    allOff = allOff && !enabled;
    allLow = allLow && enabled && s.led_zone_mode[i] == state::LedMode::LOW_LIGHT;
    allHigh = allHigh && enabled && s.led_zone_mode[i] == state::LedMode::HIGH_LIGHT;
  }

  if (allOff) {
    mode = LedUiMode::Off;
    return true;
  }

  if (allLow) {
    mode = LedUiMode::LowWhite;
    return true;
  }

  if (allHigh) {
    mode = LedUiMode::HighWhite;
    return true;
  }

  return false;
}

bool LedManager::begin(uint8_t pin1, uint8_t pin2, uint8_t pin3,
                       uint16_t ledsChannel1, uint16_t ledsChannel2,
                       uint16_t ledsChannel3, uint16_t ledsChannel3Offset) {
  (void)pin1;
  (void)pin2;
  (void)pin3;

  channels_[0].leds = g_ch1;
  channels_[0].count = clampCount(ledsChannel1, kMaxCh1);
  channels_[0].offset = 0;
  channels_[1].leds = g_ch2;
  channels_[1].count = clampCount(ledsChannel2, kMaxCh2);
  channels_[1].offset = 0;
  channels_[2].leds = g_ch3;
  channels_[2].count = clampCount(ledsChannel3, kMaxCh3);
  channels_[2].offset = min<uint16_t>(ledsChannel3Offset, channels_[2].count);

  pinMode(pins::kLedData1, OUTPUT);
  pinMode(pins::kLedData2, OUTPUT);
  pinMode(pins::kLedData3, OUTPUT);
  digitalWrite(pins::kLedData1, LOW);
  digitalWrite(pins::kLedData2, LOW);
  digitalWrite(pins::kLedData3, LOW);

  FastLED.addLeds<WS2812B, pins::kLedData1, GRB>(g_ch1, channels_[0].count);
  FastLED.addLeds<WS2812B, pins::kLedData2, GRB>(g_ch2, channels_[1].count);
  FastLED.addLeds<WS2812B, pins::kLedData3, GRB>(g_ch3, channels_[2].count);
  FastLED.setBrightness(255);
  FastLED.clear(true);
  configureZones();

  started_ = true;
  currentMode_ = LedUiMode::Off;
  modeApplied_ = false;
  startupSweepActive_ = true;
  startupStartMs_ = millis();
  Serial.printf("[LED] FastLED WS2812B/GRB pins ch1=%u ch2=%u ch3=%u count1=%u count2=%u count3=%u ch3_offset=%u\n",
                static_cast<unsigned>(pins::kLedData1),
                static_cast<unsigned>(pins::kLedData2),
                static_cast<unsigned>(pins::kLedData3),
                static_cast<unsigned>(channels_[0].count),
                static_cast<unsigned>(channels_[1].count),
                static_cast<unsigned>(channels_[2].count),
                static_cast<unsigned>(channels_[2].offset));
  for (uint8_t i = 0; i < state::kLedZoneCount; ++i) {
    Serial.printf("[LED] zone %s channel=%u start=%u count=%u\n",
                  zoneName(i),
                  static_cast<unsigned>(zones_[i].channel + 1U),
                  static_cast<unsigned>(zones_[i].start),
                  static_cast<unsigned>(zones_[i].count));
  }
  Serial.println("[LED] startup animation begin");
  return true;
}

void LedManager::clearAll() {
  for (uint8_t i = 0; i < 3; ++i) {
    if (channels_[i].leds && channels_[i].count > 0) {
      fill_solid(channels_[i].leds, channels_[i].count, CRGB::Black);
    }
    channels_[i].enabled = false;
    channels_[i].mode = state::LedMode::OFF;
    channels_[i].brightness = 0;
  }
}

void LedManager::clearInterior() {
  for (uint8_t i = 1; i < 3; ++i) {
    if (channels_[i].leds && channels_[i].count > 0) {
      fill_solid(channels_[i].leds, channels_[i].count, CRGB::Black);
    }
    channels_[i].enabled = false;
    channels_[i].mode = state::LedMode::OFF;
    channels_[i].brightness = 0;
  }
}

void LedManager::fillChannel(Channel& ch, const CRGB& color) {
  if (!ch.leds || ch.count == 0) return;
  fill_solid(ch.leds, ch.count, CRGB::Black);
  for (uint16_t i = ch.offset; i < ch.count; ++i) {
    ch.leds[i] = color;
  }
}

void LedManager::fillZone(uint8_t zone, const CRGB& color) {
  if (zone >= state::kLedZoneCount) return;
  const ZoneSegment& seg = zones_[zone];
  if (seg.channel >= 3U) return;
  Channel& ch = channels_[seg.channel];
  if (!ch.leds || ch.count == 0 || seg.count == 0) return;
  const uint16_t start = min<uint16_t>(seg.start, ch.count);
  const uint16_t end = min<uint16_t>(static_cast<uint16_t>(start + seg.count), ch.count);
  for (uint16_t px = start; px < end; ++px) {
    ch.leds[px] = color;
  }
}

void LedManager::applyUiMode(LedUiMode mode, bool forceLog) {
  startupSweepActive_ = false;
  const uint8_t brightness = uiModeBrightness(mode);

  if (mode == LedUiMode::Off) {
    clearInterior();
  } else {
    clearInterior();
    const CRGB white = whiteForBrightness(brightness);
    for (uint8_t i = 1; i < 3; ++i) {
      channels_[i].enabled = true;
      channels_[i].mode = (mode == LedUiMode::HighWhite) ? state::LedMode::HIGH_LIGHT
                                                         : state::LedMode::LOW_LIGHT;
      channels_[i].brightness = brightness;
    }
    for (uint8_t zone = 0; zone < state::kLedZoneCount; ++zone) {
      fillZone(zone, white);
    }
  }

  FastLED.setBrightness(255);
  FastLED.show();
  if (mode == LedUiMode::Off) {
    delayMicroseconds(300);
    FastLED.show();
  }

  if (forceLog || !modeApplied_ || currentMode_ != mode) {
    Serial.printf("[LED] apply mode=%s brightness=%u animations=0 show_called=1\n",
                  uiModeName(mode),
                  static_cast<unsigned>(brightness));
  }
  currentMode_ = mode;
  modeApplied_ = true;
  lastFrameMs_ = millis();
}

void LedManager::setLedUiMode(LedUiMode mode) {
  if (!started_) return;
  applyUiMode(mode, true);
}

void LedManager::triggerStartupSweep() {
  startupSweepActive_ = true;
  startupStartMs_ = millis();
  modeApplied_ = false;
}

void LedManager::renderStartupSweep(uint32_t nowMs) {
  const uint32_t elapsed = nowMs - startupStartMs_;
  const uint8_t breath = beatsin8(10, 18, 76);

  for (uint8_t chIdx = 0; chIdx < 3; ++chIdx) {
    Channel& ch = channels_[chIdx];
    if (!ch.leds || ch.count == 0) continue;

    fill_solid(ch.leds, ch.count, CRGB::Black);
    const uint16_t activeCount = (ch.count > ch.offset) ? (ch.count - ch.offset) : 0;
    if (activeCount == 0) continue;

    const uint32_t channelDelay = static_cast<uint32_t>(chIdx) * 120U;
    if (elapsed < channelDelay) continue;
    const uint32_t localElapsed = (elapsed > channelDelay) ? (elapsed - channelDelay) : 0U;
    const uint16_t head = static_cast<uint16_t>(
        min<uint32_t>(activeCount - 1U,
                      (static_cast<uint32_t>(activeCount) * localElapsed) / kStartupDurationMs));

    for (uint16_t i = 0; i <= head; ++i) {
      const uint16_t pos = ch.offset + i;
      const uint8_t fade = static_cast<uint8_t>(10U + ((static_cast<uint32_t>(i) * 42U) / activeCount));
      ch.leds[pos] = CRGB(fade, fade, fade);
    }
    ch.leds[ch.offset + head] = CRGB(breath, breath, breath);
  }

  FastLED.setBrightness(255);
  FastLED.show();

  if (elapsed >= (kStartupDurationMs + 240U)) {
    startupSweepActive_ = false;
    modeApplied_ = false;
    Serial.println("[LED] startup animation complete");
  }
}

void LedManager::renderRpmGauge(const state::VehicleState& s) {
  Channel& ch = channels_[0];
  if (!ch.leds || ch.count == 0) return;

  fill_solid(ch.leds, ch.count, CRGB::Black);
  const uint16_t activeCount = (ch.count > ch.offset) ? (ch.count - ch.offset) : 0;
  if (activeCount == 0) return;

  const uint16_t rpm = min<uint16_t>(max<uint16_t>(s.rpm, kRpmGaugeIdle), kRpmGaugeMax);
  uint16_t lit = 0;
  if (s.rpm >= kRpmGaugeIdle) {
    lit = static_cast<uint16_t>(
        ((static_cast<uint32_t>(rpm - kRpmGaugeIdle) * activeCount) /
         (kRpmGaugeMax - kRpmGaugeIdle)) + 1U);
    lit = min<uint16_t>(lit, activeCount);
  }

  for (uint16_t i = 0; i < lit; ++i) {
    CRGB color = CRGB(0, 90, 24);
    if (rpm >= kRpmGaugeRed) {
      color = CRGB(120, 0, 0);
    } else if (rpm >= kRpmGaugeYellow) {
      color = CRGB(110, 72, 0);
    }
    const uint8_t scale = static_cast<uint8_t>(80U + ((static_cast<uint32_t>(i) * 80U) / activeCount));
    ch.leds[ch.offset + i] = color;
    ch.leds[ch.offset + i].nscale8_video(scale);
  }

  ch.enabled = true;
  ch.mode = state::LedMode::RPM_GAUGE;
  ch.brightness = 180;
}

void LedManager::renderFallbackModes(const state::VehicleState& s, uint32_t nowMs) {
  renderRpmGauge(s);

  for (uint8_t i = 1; i < 3; ++i) {
    if (channels_[i].leds && channels_[i].count > 0) {
      fill_solid(channels_[i].leds, channels_[i].count, CRGB::Black);
    }
    channels_[i].enabled = false;
    channels_[i].mode = state::LedMode::OFF;
    channels_[i].brightness = 0;
  }

  for (uint8_t zone = 0; zone < state::kLedZoneCount; ++zone) {
    if (!s.led_zone_enabled[zone] || s.led_zone_mode[zone] == state::LedMode::OFF) continue;

    const uint8_t chIdx = zones_[zone].channel;
    if (chIdx < 3U) {
      channels_[chIdx].enabled = true;
      channels_[chIdx].mode = s.led_zone_mode[zone];
      channels_[chIdx].brightness = s.led_zone_brightness[zone];
    }

    if (s.led_zone_mode[zone] == state::LedMode::HIGH_LIGHT) {
      fillZone(zone, whiteForBrightness(kHighWhiteBrightness));
    } else if (s.led_zone_mode[zone] == state::LedMode::LOW_LIGHT) {
      fillZone(zone, whiteForBrightness(kLowWhiteBrightness));
    } else if (s.led_zone_mode[zone] == state::LedMode::STATIC_COLOR) {
      fillZone(zone, colorFromRgb(s.led_zone_color[zone],
                                  max<uint8_t>(s.led_zone_brightness[zone],
                                               static_cast<uint8_t>(80))));
    } else if (s.led_zone_mode[zone] == state::LedMode::RAINBOW) {
      const ZoneSegment& seg = zones_[zone];
      if (seg.channel < 3U && channels_[seg.channel].leds) {
        Channel& ch = channels_[seg.channel];
        const uint16_t start = min<uint16_t>(seg.start, ch.count);
        const uint16_t end = min<uint16_t>(static_cast<uint16_t>(start + seg.count), ch.count);
        for (uint16_t px = start; px < end; ++px) {
          ch.leds[px] = CHSV(static_cast<uint8_t>((nowMs / 8U) + px * 5U + zone * 32U), 255, 140);
        }
      }
    } else if (s.led_zone_mode[zone] == state::LedMode::BREATHING) {
      const uint8_t wave = beatsin8(18, 18, 120, 0, static_cast<uint8_t>(zone * 54U));
      fillZone(zone, CRGB(wave, wave, wave));
    } else if (s.led_zone_mode[zone] == state::LedMode::RPM_REACTIVE) {
      const ZoneSegment& seg = zones_[zone];
      if (seg.channel < 3U && channels_[seg.channel].leds) {
        Channel& ch = channels_[seg.channel];
        const uint16_t start = min<uint16_t>(seg.start, ch.count);
        const uint16_t end = min<uint16_t>(static_cast<uint16_t>(start + seg.count), ch.count);
        const uint16_t len = (end > start) ? (end - start) : 0;
        const uint16_t head = len ? static_cast<uint16_t>((nowMs / 35U + zone * 9U) % len) : 0;
        for (uint16_t i = 0; i < len; ++i) {
          const uint16_t dist = (i > head) ? (i - head) : (head - i);
          const uint8_t val = (dist < 3U) ? static_cast<uint8_t>(150U - dist * 38U) : 10U;
          ch.leds[start + i] = colorFromRgb(s.led_zone_color[zone], val);
        }
      }
    } else if (s.led_zone_mode[zone] == state::LedMode::WARNING_FLASH) {
      const ZoneSegment& seg = zones_[zone];
      if (seg.channel < 3U && channels_[seg.channel].leds) {
        Channel& ch = channels_[seg.channel];
        const uint16_t start = min<uint16_t>(seg.start, ch.count);
        const uint16_t end = min<uint16_t>(static_cast<uint16_t>(start + seg.count), ch.count);
        for (uint16_t px = start; px < end; ++px) {
          const uint8_t sparkle = static_cast<uint8_t>((px * 37U + nowMs / 13U + zone * 53U) & 0xFFU);
          ch.leds[px] = (sparkle > 246U) ? CRGB(180, 180, 180)
                                         : colorFromRgb(s.led_zone_color[zone], 18);
        }
      }
    } else {
      const uint8_t pulse = static_cast<uint8_t>((nowMs / 6U) & 0xFFU);
      CRGB color;
      hsv2rgb_rainbow(CHSV(static_cast<uint8_t>(pulse + zone * 24U), 255,
                           max<uint8_t>(s.led_zone_brightness[zone], static_cast<uint8_t>(80))),
                      color);
      fillZone(zone, color);
    }
  }

  FastLED.setBrightness(255);
  FastLED.show();

  static bool loggedOnce = false;
  if (!loggedOnce) {
    Serial.printf("[LED] fallback render zones D=%s P=%s F=%s B=%s show_called=1\n",
                  modeName(s.led_zone_mode[0]), modeName(s.led_zone_mode[1]),
                  modeName(s.led_zone_mode[2]), modeName(s.led_zone_mode[3]));
    loggedOnce = true;
  }
}

void LedManager::tick(const state::VehicleState& s) {
  if (!started_) return;
  const uint32_t nowMs = millis();

  if (startupSweepActive_) {
    renderStartupSweep(nowMs);
    return;
  }

  modeApplied_ = false;
  if ((nowMs - lastFrameMs_) < kFrameIntervalMs) return;
  lastFrameMs_ = nowMs;
  renderFallbackModes(s, nowMs);
}

}  // namespace led
