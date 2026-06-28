#include "led/led_manager.h"

namespace led {

namespace {
#ifndef CCM_LED_FRAME_INTERVAL_MS
#define CCM_LED_FRAME_INTERVAL_MS 33
#endif

constexpr uint32_t kFrameIntervalMs =
    (CCM_LED_FRAME_INTERVAL_MS < 16) ? 16U : static_cast<uint32_t>(CCM_LED_FRAME_INTERVAL_MS);
constexpr uint32_t kBreathingPeriodMs = 2200;
constexpr uint32_t kWarningFlashMs = 120;
constexpr uint32_t kCanFaultFlashMs = 100;
constexpr uint32_t kStartupSweepDurationMs = 1800;
constexpr uint32_t kStartupStepMs = 40;
constexpr uint8_t kStartupBrightnessCap = 110;
constexpr uint16_t kRpmBrightnessScaleDivisor = 30;
constexpr uint16_t kRpmGaugeStart = 1000;
constexpr uint16_t kRpmGaugeGreenEnd = 3000;
constexpr uint16_t kRpmGaugeYellowEnd = 5000;
constexpr uint16_t kRpmGaugeRedEnd = 6000;
constexpr uint32_t kRpmGreen = 0x00FF00;
constexpr uint32_t kRpmYellow = 0xFFFF00;
constexpr uint32_t kRpmRed = 0xFF0000;
constexpr float kPi = 3.14159265f;
// RPM startup animation phases (ms from boot)
constexpr uint32_t kStartupWipeEnd = 850U;
constexpr uint32_t kStartupSettleEnd = 1350U;
constexpr uint32_t kStartupPulseEnd = 1850U;
constexpr uint32_t kStartupFadeEnd = 2400U;
constexpr uint32_t kStartupSoftWhite = 0xFFFFFF;
constexpr uint32_t kStartupDimWhite = 0x303030;
constexpr uint32_t kStartupRed = 0xFF0000;

uint16_t benchPreviewRpm(uint32_t nowMs) {
  const float phase = (nowMs % 4000U) / 4000.0f;
  const float wave = 0.5f + 0.5f * sinf((phase * 2.0f * kPi) - (kPi / 2.0f));
  return static_cast<uint16_t>(900U + (6400.0f * wave));
}

const char* modeName(state::LedMode mode) {
  switch (mode) {
    case state::LedMode::OFF: return "OFF";
    case state::LedMode::STATIC_COLOR: return "STATIC";
    case state::LedMode::BREATHING: return "BREATH";
    case state::LedMode::RAINBOW: return "RAINBOW";
    case state::LedMode::RPM_REACTIVE: return "RPM_REACTIVE";
    case state::LedMode::WARNING_FLASH: return "WARN";
    case state::LedMode::METH_ACTIVE: return "METH";
    case state::LedMode::CAN_FAULT: return "CAN_FAULT";
    case state::LedMode::STARTUP_SWEEP: return "STARTUP";
    case state::LedMode::RPM_GAUGE: return "RPM";
    case state::LedMode::LOW_LIGHT: return "LOW";
    case state::LedMode::HIGH_LIGHT: return "HIGH";
    default: return "UNKNOWN";
  }
}

}  // namespace

uint8_t LedManager::rpmGaugeLitCount(uint16_t rpm, uint16_t numLeds) {
  if (rpm < kRpmGaugeStart || numLeds == 0) {
    return 0;
  }

  const uint16_t greenCount = min<uint16_t>(numLeds, max<uint16_t>(1, (numLeds * 3U + 3U) / 7U));
  const uint16_t remainingAfterGreen = numLeds - greenCount;
  const uint16_t yellowCount = (remainingAfterGreen == 0)
      ? 0
      : min<uint16_t>(remainingAfterGreen, max<uint16_t>(1, (numLeds * 2U + 3U) / 7U));
  const uint16_t redCount = numLeds - greenCount - yellowCount;

  uint8_t lit = 0;
  for (uint16_t i = 0; i < greenCount; ++i) {
    const uint16_t threshold = (greenCount <= 1)
        ? kRpmGaugeStart
        : static_cast<uint16_t>(
              kRpmGaugeStart +
              ((static_cast<uint32_t>(kRpmGaugeGreenEnd - kRpmGaugeStart) * i) / (greenCount - 1U)));
    if (rpm >= threshold) ++lit;
  }

  for (uint16_t i = 0; i < yellowCount; ++i) {
    const uint16_t threshold = static_cast<uint16_t>(
        kRpmGaugeGreenEnd +
        ((static_cast<uint32_t>(kRpmGaugeYellowEnd - kRpmGaugeGreenEnd) * (i + 1U)) / yellowCount));
    if (rpm >= threshold) ++lit;
  }

  for (uint16_t i = 0; i < redCount; ++i) {
    const uint16_t threshold = static_cast<uint16_t>(
        kRpmGaugeYellowEnd +
        ((static_cast<uint32_t>(kRpmGaugeRedEnd - kRpmGaugeYellowEnd) * (i + 1U)) / redCount));
    if (rpm >= threshold) ++lit;
  }

  return static_cast<uint8_t>(min<uint16_t>(lit, numLeds));
}

uint32_t LedManager::rpmBandColor(uint16_t rpm) {
  if (rpm >= kRpmGaugeYellowEnd) return kRpmRed;
  if (rpm >= kRpmGaugeGreenEnd) return kRpmYellow;
  return kRpmGreen;
}

uint32_t LedManager::rpmGaugeColor(uint16_t ledIdx, uint16_t numLeds) {
  // Fixed shift-light zones: low LEDs green, middle LEDs yellow, top LEDs red.
  if (numLeds == 0) return 0;

  const uint16_t greenCount = min<uint16_t>(numLeds, max<uint16_t>(1, (numLeds * 3U + 3U) / 7U));
  const uint16_t remainingAfterGreen = numLeds - greenCount;
  const uint16_t yellowCount = (remainingAfterGreen == 0)
      ? 0
      : min<uint16_t>(remainingAfterGreen, max<uint16_t>(1, (numLeds * 2U + 3U) / 7U));

  if (ledIdx < greenCount) return kRpmGreen;
  if (ledIdx < (greenCount + yellowCount)) return kRpmYellow;
  return kRpmRed;
}

void LedManager::renderStartupAnimation(Channel& ch, uint32_t elapsed, uint8_t idx) {
  if (!ch.strip) return;
  const uint16_t total = ch.strip->numPixels();
  const uint16_t offset = ch.ledOffset;
  const uint16_t n = (total > offset) ? (total - offset) : 0;
  if (n == 0) { ch.strip->show(); return; }

  for (uint16_t i = 0; i < offset; ++i) ch.strip->setPixelColor(i, 0U);
  if (!ch.enabled || ch.mode == state::LedMode::OFF) {
    ch.strip->setBrightness(kStartupBrightnessCap);
    for (uint16_t i = 0; i < n; ++i) ch.strip->setPixelColor(offset + i, 0U);
    ch.strip->show();
    return;
  }

  const uint32_t channelDelay = static_cast<uint32_t>(idx) * 120U;
  if (elapsed < channelDelay) {
    ch.strip->setBrightness(kStartupBrightnessCap);
    for (uint16_t i = 0; i < n; ++i) ch.strip->setPixelColor(offset + i, 0U);
    ch.strip->show();
    return;
  }
  const uint32_t t = (elapsed > channelDelay) ? (elapsed - channelDelay) : 0U;

  if (t < kStartupWipeEnd) {
    ch.strip->setBrightness(kStartupBrightnessCap);
    const uint16_t head = static_cast<uint16_t>(
        min<uint32_t>(n - 1U, (static_cast<uint32_t>(n) * t) / kStartupWipeEnd));
    for (uint16_t i = 0; i < n; ++i) {
      uint32_t color = 0U;
      if (i <= head) {
        const uint16_t dist = head - i;
        if (dist == 0U) {
          color = kStartupSoftWhite;
        } else if (dist <= 2U) {
          color = scaleColor(kStartupSoftWhite, static_cast<uint8_t>(80U / dist));
        } else {
          color = scaleColor(kStartupDimWhite, 24);
        }
      }
      ch.strip->setPixelColor(offset + i, color);
    }
  } else if (t < kStartupSettleEnd) {
    ch.strip->setBrightness(kStartupBrightnessCap);
    const float settle = static_cast<float>(t - kStartupWipeEnd) /
                         static_cast<float>(kStartupSettleEnd - kStartupWipeEnd);
    const uint8_t brightness = static_cast<uint8_t>(28U + (42.0f * (1.0f - settle)));
    for (uint16_t i = 0; i < n; ++i)
      ch.strip->setPixelColor(offset + i, scaleColor(kStartupSoftWhite, brightness));
  } else if (t < kStartupPulseEnd) {
    const float phase = static_cast<float>(t - kStartupSettleEnd) /
                        static_cast<float>(kStartupPulseEnd - kStartupSettleEnd);
    const float wave = 0.5f + 0.5f * sinf(phase * kPi);
    ch.strip->setBrightness(kStartupBrightnessCap);
    for (uint16_t i = 0; i < n; ++i) {
      const uint8_t white = static_cast<uint8_t>(18U + 18.0f * (1.0f - wave));
      const uint8_t red = static_cast<uint8_t>(35U + 70.0f * wave);
      const uint32_t base = scaleColor(kStartupSoftWhite, white);
      const uint32_t accent = scaleColor(kStartupRed, red);
      ch.strip->setPixelColor(offset + i, (i % 3U == idx) ? accent : base);
    }
  } else if (t < kStartupFadeEnd) {
    const float alpha = 1.0f - static_cast<float>(t - kStartupPulseEnd) /
                                static_cast<float>(kStartupFadeEnd - kStartupPulseEnd);
    ch.strip->setBrightness(static_cast<uint8_t>(kStartupBrightnessCap * alpha));
    for (uint16_t i = 0; i < n; ++i)
      ch.strip->setPixelColor(offset + i, scaleColor(kStartupSoftWhite, 22));
  } else {
    ch.strip->setBrightness(kStartupBrightnessCap);
    for (uint16_t i = 0; i < n; ++i) ch.strip->setPixelColor(offset + i, 0U);
  }
  ch.strip->show();
}

bool LedManager::begin(uint8_t pin1, uint8_t pin2, uint8_t pin3,
                       uint16_t ledsChannel1, uint16_t ledsChannel2,
                       uint16_t ledsChannel3,
                       uint16_t ledsChannel3Offset) {
  const uint16_t leds2 = (ledsChannel2 > 0) ? ledsChannel2 : ledsChannel1;
  const uint16_t leds3 = (ledsChannel3 > 0) ? ledsChannel3 : ledsChannel1;
  ledsPerChannel_ = ledsChannel1;
  ledsChannel2_ = leds2;
  ledsChannel3_ = leds3;
  strip1_.updateType(CCM_LED_PIXEL_TYPE_CH1);
  strip2_.updateType(CCM_LED_PIXEL_TYPE_CH2);
  strip3_.updateType(CCM_LED_PIXEL_TYPE_CH3);
  strip1_.updateLength(ledsPerChannel_);
  strip2_.updateLength(ledsChannel2_);
  strip3_.updateLength(ledsChannel3_);
  pinMode(pin1, OUTPUT);
  pinMode(pin2, OUTPUT);
  pinMode(pin3, OUTPUT);
  digitalWrite(pin1, LOW);
  digitalWrite(pin2, LOW);
  digitalWrite(pin3, LOW);
  strip1_.setPin(pin1);
  strip2_.setPin(pin2);
  strip3_.setPin(pin3);
  strip1_.begin();
  strip2_.begin();
  strip3_.begin();
  strip1_.clear();
  strip2_.clear();
  strip3_.clear();
  // ESP-IDF5 NeoPixel backend allocates a frame buffer on the caller stack
  // inside show(). Avoid calling show() from setup()/loopTask, and let the
  // dedicated led task render the first frame on its larger stack.
  channels_[0].strip = &strip1_;
  channels_[1].strip = &strip2_;
  channels_[2].strip = &strip3_;
  channels_[2].ledOffset = min<uint16_t>(ledsChannel3Offset, leds3);
  Serial.printf("[LED] pins ch1=%u ch2=%u ch3=%u count1=%u count2=%u count3=%u ch3_offset=%u\n",
                 static_cast<unsigned>(pin1),
                 static_cast<unsigned>(pin2),
                 static_cast<unsigned>(pin3),
                 static_cast<unsigned>(ledsPerChannel_),
                 static_cast<unsigned>(ledsChannel2_),
                 static_cast<unsigned>(ledsChannel3_),
                 static_cast<unsigned>(channels_[2].ledOffset));
  startupSweepActive_ = true;
  startupStartMs_ = millis();
  started_ = true;
  return true;
}

void LedManager::triggerStartupSweep() {
  startupSweepActive_ = true;
  startupStartMs_ = millis();
}

uint32_t LedManager::scaleColor(uint32_t rgb, uint8_t brightness) const {
  const uint8_t r = static_cast<uint8_t>((((rgb >> 16) & 0xFF) * brightness) / 255);
  const uint8_t g = static_cast<uint8_t>((((rgb >> 8) & 0xFF) * brightness) / 255);
  const uint8_t b = static_cast<uint8_t>(((rgb & 0xFF) * brightness) / 255);
  return (static_cast<uint32_t>(r) << 16) | (static_cast<uint32_t>(g) << 8) | b;
}

uint32_t LedManager::whiteColor(uint8_t brightness, uint8_t idx) const {
  const bool hasWhite =
      ((idx == 0U) && (CCM_LED_CH1_HAS_WHITE != 0)) ||
      ((idx == 1U) && (CCM_LED_CH2_HAS_WHITE != 0)) ||
      ((idx == 2U) && (CCM_LED_CH3_HAS_WHITE != 0));
  return hasWhite ? Adafruit_NeoPixel::Color(0, 0, 0, brightness)
                  : Adafruit_NeoPixel::Color(brightness, brightness, brightness);
}

uint32_t LedManager::wheel(uint8_t p) const {
  p = 255 - p;
  if (p < 85) return ((255 - p * 3) << 16) | (0 << 8) | (p * 3);
  if (p < 170) {
    p -= 85;
    return (0 << 16) | ((p * 3) << 8) | (255 - p * 3);
  }
  p -= 170;
  return ((p * 3) << 16) | ((255 - p * 3) << 8) | 0;
}

uint16_t LedManager::effectiveRpm(const state::VehicleState& s, uint32_t nowMs) const {
  if (s.rpm > 0U) {
    return s.rpm;
  }

  const uint16_t tachHz10 = (s.raw_tach_hz10 > 0U) ? s.raw_tach_hz10 : s.generated_tach_hz10;
  if (tachHz10 == 0U) {
    return s.bench_test_mode ? benchPreviewRpm(nowMs) : 0U;
  }

  const uint8_t pulsesPerRev10 = (s.pulses_per_rev10 > 0U) ? s.pulses_per_rev10 : 20U;
  const uint32_t rpm = (static_cast<uint32_t>(tachHz10) * 60U) / pulsesPerRev10;
  return static_cast<uint16_t>(min<uint32_t>(rpm, 9000U));
}

void LedManager::renderChannel(Channel& ch, const state::VehicleState& s, uint32_t nowMs, uint8_t idx) {
  if (!ch.strip) return;
  ch.strip->setBrightness(s.led_global_brightness);

  const uint16_t total  = ch.strip->numPixels();
  const uint16_t offset = ch.ledOffset;
  const uint16_t n      = (total > offset) ? (total - offset) : 0;

  // Always keep the hidden prefix dark
  for (uint16_t i = 0; i < offset; ++i) ch.strip->setPixelColor(i, 0U);

  uint8_t channelBrightness = ch.brightness;
  if (s.night_mode_enabled && channelBrightness > 90) channelBrightness = 90;

  if (!ch.enabled || ch.mode == state::LedMode::OFF || n == 0) {
    for (uint16_t i = 0; i < n; ++i) ch.strip->setPixelColor(offset + i, 0U);
    ch.strip->show();
    return;
  }

  uint32_t rgb = ch.color;
  switch (ch.mode) {
    case state::LedMode::STATIC_COLOR:
      rgb = scaleColor(ch.color, channelBrightness);
      for (uint16_t i = 0; i < n; ++i) ch.strip->setPixelColor(offset + i, rgb);
      break;
    case state::LedMode::BREATHING: {
      const float phase = (nowMs % kBreathingPeriodMs) / static_cast<float>(kBreathingPeriodMs);
      const float wave = 0.2f + 0.8f * (0.5f + 0.5f * sinf(phase * 6.283185f));
      rgb = scaleColor(ch.color, static_cast<uint8_t>(channelBrightness * wave));
      for (uint16_t i = 0; i < n; ++i) ch.strip->setPixelColor(offset + i, rgb);
      break;
    }
    case state::LedMode::LOW_LIGHT:
      rgb = whiteColor(s.night_mode_enabled ? static_cast<uint8_t>(45) : static_cast<uint8_t>(55), idx);
      for (uint16_t i = 0; i < n; ++i) ch.strip->setPixelColor(offset + i, rgb);
      break;
    case state::LedMode::HIGH_LIGHT:
      rgb = whiteColor(s.night_mode_enabled ? static_cast<uint8_t>(90) : static_cast<uint8_t>(180), idx);
      for (uint16_t i = 0; i < n; ++i) ch.strip->setPixelColor(offset + i, rgb);
      break;
    case state::LedMode::RAINBOW:
      for (uint16_t i = 0; i < n; ++i) {
        const uint8_t p = static_cast<uint8_t>(((i * 256 / n) + (nowMs / 8)) & 0xFF);
        ch.strip->setPixelColor(offset + i, scaleColor(wheel(p), channelBrightness));
      }
      break;
    case state::LedMode::RPM_REACTIVE: {
      const uint16_t rpm = effectiveRpm(s, nowMs);
      const uint8_t react = static_cast<uint8_t>(min<uint16_t>(255, rpm / kRpmBrightnessScaleDivisor));
      rgb = (rpm == 0U) ? 0U : scaleColor(rpmBandColor(rpm), min<uint8_t>(255, max<uint8_t>(20, react)));
      for (uint16_t i = 0; i < n; ++i) ch.strip->setPixelColor(offset + i, rgb);
      break;
    }
    case state::LedMode::WARNING_FLASH:
      rgb = ((nowMs / kWarningFlashMs) % 2 == 0) ? scaleColor(kRpmRed, channelBrightness) : 0;
      for (uint16_t i = 0; i < n; ++i) ch.strip->setPixelColor(offset + i, rgb);
      break;
    case state::LedMode::METH_ACTIVE:
      rgb = (s.meth_state == state::MethState::SPRAYING) ? scaleColor(0x00FFFF, channelBrightness) : scaleColor(0x001010, 20);
      for (uint16_t i = 0; i < n; ++i) ch.strip->setPixelColor(offset + i, rgb);
      break;
    case state::LedMode::CAN_FAULT:
      rgb = ((nowMs / kCanFaultFlashMs) % 2 == 0) ? scaleColor(0xFF0000, channelBrightness) : 0;
      for (uint16_t i = 0; i < n; ++i) ch.strip->setPixelColor(offset + i, rgb);
      break;
    case state::LedMode::STARTUP_SWEEP: {
      const uint16_t pos = static_cast<uint16_t>((nowMs / kStartupStepMs + idx * 3) % max<uint16_t>(1, n));
      for (uint16_t i = 0; i < n; ++i) ch.strip->setPixelColor(offset + i, 0U);
      ch.strip->setPixelColor(offset + pos, scaleColor(ch.color, channelBrightness));
      break;
    }
    case state::LedMode::RPM_GAUGE: {
      const uint16_t rpm = effectiveRpm(s, nowMs);
      const uint8_t lit = rpmGaugeLitCount(rpm, n);
      for (uint16_t i = 0; i < n; ++i) {
        ch.strip->setPixelColor(offset + i,
            (i < lit) ? scaleColor(rpmGaugeColor(i, n), channelBrightness) : 0U);
      }
      break;
    }
    case state::LedMode::OFF:
    default:
      for (uint16_t i = 0; i < n; ++i) ch.strip->setPixelColor(offset + i, 0U);
      break;
  }

  const bool rpmMode = (ch.mode == state::LedMode::RPM_GAUGE || ch.mode == state::LedMode::RPM_REACTIVE);
  if (idx == 0U && s.fault_flags != 0 && ch.mode != state::LedMode::CAN_FAULT && !rpmMode &&
      ((nowMs / 160) % 2 == 0)) {
    for (uint16_t i = 0; i < n; ++i)
      ch.strip->setPixelColor(offset + i, scaleColor(0xFF0000, channelBrightness));
  }

  ch.strip->show();
}

void LedManager::tick(const state::VehicleState& s) {
  if (!started_) return;
  const uint32_t nowMs = millis();
  if ((nowMs - lastFrameMs_) < kFrameIntervalMs) return;
  lastFrameMs_ = nowMs;

  channels_[0].enabled = s.led_channel_1_enabled;
  channels_[1].enabled = s.led_channel_2_enabled;
  channels_[2].enabled = s.led_channel_3_enabled;
  channels_[0].color = s.led_channel_1_color;
  channels_[1].color = s.led_channel_2_color;
  channels_[2].color = s.led_channel_3_color;
  channels_[0].mode = s.led_channel_1_mode;
  channels_[1].mode = s.led_channel_2_mode;
  channels_[2].mode = s.led_channel_3_mode;
  channels_[0].brightness = s.led_channel_1_brightness;
  channels_[1].brightness = s.led_channel_2_brightness;
  channels_[2].brightness = s.led_channel_3_brightness;

  static bool loggedOnce = false;
  static bool lastEnabled[3] = {};
  static state::LedMode lastMode[3] = {};
  static uint8_t lastBrightness[3] = {};
  static uint8_t lastGlobalBrightness = 0;
  for (uint8_t i = 0; i < 3; ++i) {
    const bool changed = !loggedOnce ||
        lastEnabled[i] != channels_[i].enabled ||
        lastMode[i] != channels_[i].mode ||
        lastBrightness[i] != channels_[i].brightness ||
        lastGlobalBrightness != s.led_global_brightness;
    if (changed) {
      const uint16_t pixels = channels_[i].strip ? channels_[i].strip->numPixels() : 0U;
      Serial.printf("[LED:FRAME] ch=%u mode=%s enabled=%u global=%u br=%u pixels=%u startup=%u\n",
                    static_cast<unsigned>(i + 1U),
                    modeName(channels_[i].mode),
                    channels_[i].enabled ? 1U : 0U,
                    static_cast<unsigned>(s.led_global_brightness),
                    static_cast<unsigned>(channels_[i].brightness),
                    static_cast<unsigned>(pixels),
                    startupSweepActive_ ? 1U : 0U);
      lastEnabled[i] = channels_[i].enabled;
      lastMode[i] = channels_[i].mode;
      lastBrightness[i] = channels_[i].brightness;
    }
  }
  lastGlobalBrightness = s.led_global_brightness;
  loggedOnce = true;

  if (s.bench_test_mode) {
    channels_[0].enabled = true;
    channels_[0].mode = state::LedMode::RPM_GAUGE;
    channels_[0].brightness = max<uint8_t>(channels_[0].brightness, static_cast<uint8_t>(180));
    if (!s.led_startup_preview) {
      startupSweepActive_ = false;
    }
  }

  if (startupSweepActive_) {
    const uint32_t elapsed = nowMs - startupStartMs_;
    for (uint8_t i = 0; i < 3; ++i) {
      renderStartupAnimation(channels_[i], elapsed, i);
    }
    if (elapsed > (kStartupFadeEnd + 240U) && !s.led_startup_preview) {
      startupSweepActive_ = false;
    }
    return;
  }

  for (uint8_t i = 0; i < 3; ++i) {
    renderChannel(channels_[i], s, nowMs, i);
  }
}

}  // namespace led
