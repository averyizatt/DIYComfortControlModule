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
constexpr uint8_t kStartupBrightnessCap = 80;
constexpr uint16_t kRpmBrightnessScaleDivisor = 30;
constexpr uint16_t kRpmGaugeStart = 1000;
constexpr uint16_t kRpmGaugeGreenEnd = 3000;
constexpr uint16_t kRpmGaugeYellowEnd = 5000;
constexpr uint16_t kRpmGaugeRedEnd = 6000;
constexpr uint8_t kRpmGaugePreviewBrightnessMin = 6;
constexpr uint8_t kRpmGaugePreviewBrightnessMax = 28;
constexpr uint8_t kRpmGaugePreviewBrightnessDivisor = 14;
constexpr uint32_t kRpmGreen = 0x00D64A;
constexpr uint32_t kRpmYellow = 0xFFD000;
constexpr uint32_t kRpmRed = 0xFF2000;
// RPM startup animation phases (ms from boot)
constexpr uint32_t kRpmStartupWipeEnd   = 700U;   // green wipe 0→6
constexpr uint32_t kRpmStartupColorEnd  = 1400U;  // flip to RPM colors
constexpr uint32_t kRpmStartupHoldEnd   = 1700U;  // hold all on
constexpr uint32_t kRpmStartupFadeEnd   = 2200U;  // fade to black
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

void LedManager::renderRpmStartup(Channel& ch, uint32_t elapsed) {
  if (!ch.strip) return;
  const uint16_t total = ch.strip->numPixels();
  const uint16_t offset = ch.ledOffset;
  const uint16_t n = (total > offset) ? (total - offset) : 0;
  if (n == 0) { ch.strip->show(); return; }

  // Always blank the hidden prefix
  for (uint16_t i = 0; i < offset; ++i) ch.strip->setPixelColor(i, 0U);

  const uint32_t step = (n > 0) ? (kRpmStartupWipeEnd / n) : kRpmStartupWipeEnd;

  if (elapsed < kRpmStartupWipeEnd) {
    // Phase 0: wipe green in from LED offset → offset+n-1
    const uint16_t lit = static_cast<uint16_t>(min<uint32_t>(n, elapsed / step + 1));
    for (uint16_t i = 0; i < n; ++i)
      ch.strip->setPixelColor(offset + i, i < lit ? 0x00C800U : 0U);
  } else if (elapsed < kRpmStartupColorEnd) {
    // Phase 1: flip each LED to its RPM gradient colour, left→right
    const uint32_t t2 = elapsed - kRpmStartupWipeEnd;
    const uint16_t flipped = static_cast<uint16_t>(min<uint32_t>(n, t2 / step + 1));
    for (uint16_t i = 0; i < n; ++i) {
      ch.strip->setPixelColor(offset + i,
          i < flipped ? rpmGaugeColor(i, n) : 0x00C800U);
    }
  } else if (elapsed < kRpmStartupHoldEnd) {
    // Phase 2: all visible LEDs on in final RPM colours
    ch.strip->setBrightness(kStartupBrightnessCap);
    for (uint16_t i = 0; i < n; ++i)
      ch.strip->setPixelColor(offset + i, rpmGaugeColor(i, n));
  } else if (elapsed < kRpmStartupFadeEnd) {
    // Phase 3: fade brightness to black
    const float alpha = 1.0f - static_cast<float>(elapsed - kRpmStartupHoldEnd) /
                                static_cast<float>(kRpmStartupFadeEnd - kRpmStartupHoldEnd);
    ch.strip->setBrightness(static_cast<uint8_t>(kStartupBrightnessCap * alpha));
    for (uint16_t i = 0; i < n; ++i)
      ch.strip->setPixelColor(offset + i, rpmGaugeColor(i, n));
  } else {
    ch.strip->setBrightness(kStartupBrightnessCap);
    for (uint16_t i = 0; i < n; ++i) ch.strip->setPixelColor(offset + i, 0U);
  }
  ch.strip->show();
}

bool LedManager::begin(uint8_t pin1, uint8_t pin2, uint8_t pin3,
                       uint16_t ledsPerChannel, uint16_t ledsChannel3,
                       uint16_t ledsChannel3Offset) {
  const uint16_t leds3 = (ledsChannel3 > 0) ? ledsChannel3 : ledsPerChannel;
  ledsPerChannel_ = ledsPerChannel;
  strip1_.updateLength(ledsPerChannel_);
  strip2_.updateLength(ledsPerChannel_);
  strip3_.updateLength(leds3);
  strip1_.setPin(pin1);
  strip2_.setPin(pin2);
  strip3_.setPin(pin3);
  strip1_.begin();
  strip2_.begin();
  strip3_.begin();
  strip1_.show();
  strip2_.show();
  strip3_.show();
  channels_[0].strip = &strip1_;
  channels_[1].strip = &strip2_;
  channels_[2].strip = &strip3_;
  channels_[2].ledOffset = min<uint16_t>(ledsChannel3Offset, leds3);
  Serial0.printf("[LED] ch1=%u ch2=%u ch3=%u ch3_offset=%u\n",
                 static_cast<unsigned>(ledsPerChannel_),
                 static_cast<unsigned>(ledsPerChannel_),
                 static_cast<unsigned>(leds3),
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

uint16_t LedManager::effectiveRpm(const state::VehicleState& s) const {
  if (s.rpm > 0U) {
    return s.rpm;
  }

  const uint16_t tachHz10 = (s.raw_tach_hz10 > 0U) ? s.raw_tach_hz10 : s.generated_tach_hz10;
  if (tachHz10 == 0U) {
    return 0U;
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
    case state::LedMode::RAINBOW:
      for (uint16_t i = 0; i < n; ++i) {
        const uint8_t p = static_cast<uint8_t>(((i * 256 / n) + (nowMs / 8)) & 0xFF);
        ch.strip->setPixelColor(offset + i, scaleColor(wheel(p), channelBrightness));
      }
      break;
    case state::LedMode::RPM_REACTIVE: {
      const uint16_t rpm = effectiveRpm(s);
      const uint8_t react = static_cast<uint8_t>(min<uint16_t>(255, rpm / kRpmBrightnessScaleDivisor));
      rgb = (rpm == 0U) ? 0U : scaleColor(rpmBandColor(rpm), min<uint8_t>(255, max<uint8_t>(20, react)));
      for (uint16_t i = 0; i < n; ++i) ch.strip->setPixelColor(offset + i, rgb);
      break;
    }
    case state::LedMode::WARNING_FLASH:
      rgb = ((nowMs / kWarningFlashMs) % 2 == 0) ? scaleColor(0xFF4000, channelBrightness) : 0;
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
      const uint16_t rpm = effectiveRpm(s);
      const uint8_t lit = rpmGaugeLitCount(rpm, n);
      const uint8_t previewBrightness = (rpm == 0U)
          ? 0U
          : min<uint8_t>(kRpmGaugePreviewBrightnessMax,
                         max<uint8_t>(kRpmGaugePreviewBrightnessMin,
                                      channelBrightness / kRpmGaugePreviewBrightnessDivisor));
      for (uint16_t i = 0; i < n; ++i) {
        const uint8_t pixelBrightness = (i < lit) ? channelBrightness : previewBrightness;
        ch.strip->setPixelColor(offset + i,
            pixelBrightness > 0U ? scaleColor(rpmGaugeColor(i, n), pixelBrightness) : 0U);
      }
      break;
    }
    case state::LedMode::OFF:
    default:
      for (uint16_t i = 0; i < n; ++i) ch.strip->setPixelColor(offset + i, 0U);
      break;
  }

  const bool rpmMode = (ch.mode == state::LedMode::RPM_GAUGE || ch.mode == state::LedMode::RPM_REACTIVE);
  if (s.fault_flags != 0 && ch.mode != state::LedMode::CAN_FAULT && !rpmMode && ((nowMs / 160) % 2 == 0)) {
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

  if (startupSweepActive_) {
    const uint32_t elapsed = nowMs - startupStartMs_;
    for (uint8_t i = 0; i < 3; ++i) {
      if (channels_[i].mode == state::LedMode::RPM_GAUGE) {
        renderRpmStartup(channels_[i], elapsed);
      } else {
        channels_[i].mode = state::LedMode::STARTUP_SWEEP;
        const uint8_t originalBrightness = channels_[i].brightness;
        channels_[i].brightness = min<uint8_t>(originalBrightness, kStartupBrightnessCap);
        renderChannel(channels_[i], s, nowMs + i * 80U, i);
        channels_[i].brightness = originalBrightness;
      }
    }
    if (elapsed > kRpmStartupFadeEnd && !s.led_startup_preview) {
      startupSweepActive_ = false;
    }
    return;
  }

  for (uint8_t i = 0; i < 3; ++i) {
    renderChannel(channels_[i], s, nowMs, i);
  }
}

}  // namespace led
