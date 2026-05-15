#include "led/led_manager.h"

namespace led {

namespace {
constexpr uint32_t kFrameIntervalMs = 16;
constexpr uint32_t kBreathingPeriodMs = 2200;
constexpr uint32_t kWarningFlashMs = 120;
constexpr uint32_t kCanFaultFlashMs = 100;
constexpr uint32_t kStartupSweepDurationMs = 1800;
constexpr uint32_t kStartupStepMs = 40;
constexpr uint16_t kRpmBrightnessScaleDivisor = 30;
constexpr uint16_t kRpmGaugeMax = 7000;
// RPM startup animation phases (ms from boot)
constexpr uint32_t kRpmStartupWipeEnd   = 700U;   // green wipe 0→6
constexpr uint32_t kRpmStartupColorEnd  = 1400U;  // flip to RPM colors
constexpr uint32_t kRpmStartupHoldEnd   = 1700U;  // hold all on
constexpr uint32_t kRpmStartupFadeEnd   = 2200U;  // fade to black
}  // namespace

uint32_t LedManager::rpmGaugeColor(uint16_t ledIdx, uint16_t numLeds) {
  // Smooth green→yellow→red gradient across all LEDs
  const uint8_t pos = (numLeds > 1)
      ? static_cast<uint8_t>(ledIdx * 255U / (numLeds - 1U))
      : 0U;
  const uint8_t r = pos;
  const uint8_t g = static_cast<uint8_t>(255U - pos);
  return (static_cast<uint32_t>(r) << 16) | (static_cast<uint32_t>(g) << 8);
}

void LedManager::renderRpmStartup(Channel& ch, uint32_t elapsed) {
  if (!ch.strip) return;
  const uint16_t n   = ch.strip->numPixels();
  const uint32_t step = (n > 0) ? (kRpmStartupWipeEnd / n) : kRpmStartupWipeEnd;

  if (elapsed < kRpmStartupWipeEnd) {
    // Phase 0: wipe green in from LED 0 → n-1
    const uint16_t lit = static_cast<uint16_t>(min<uint32_t>(n, elapsed / step + 1));
    ch.strip->clear();
    for (uint16_t i = 0; i < lit; ++i) ch.strip->setPixelColor(i, 0x00C800U);
  } else if (elapsed < kRpmStartupColorEnd) {
    // Phase 1: flip each LED to its RPM gradient colour, left→right
    const uint32_t t2 = elapsed - kRpmStartupWipeEnd;
    const uint16_t flipped = static_cast<uint16_t>(min<uint32_t>(n, t2 / step + 1));
    for (uint16_t i = 0; i < n; ++i) {
      ch.strip->setPixelColor(i, i < flipped ? rpmGaugeColor(i, n) : 0x00C800U);
    }
  } else if (elapsed < kRpmStartupHoldEnd) {
    // Phase 2: all LEDs on in final RPM colours
    ch.strip->setBrightness(200);
    for (uint16_t i = 0; i < n; ++i) ch.strip->setPixelColor(i, rpmGaugeColor(i, n));
  } else if (elapsed < kRpmStartupFadeEnd) {
    // Phase 3: fade brightness to black
    const float alpha = 1.0f - static_cast<float>(elapsed - kRpmStartupHoldEnd) /
                                static_cast<float>(kRpmStartupFadeEnd - kRpmStartupHoldEnd);
    ch.strip->setBrightness(static_cast<uint8_t>(200.0f * alpha));
    for (uint16_t i = 0; i < n; ++i) ch.strip->setPixelColor(i, rpmGaugeColor(i, n));
  } else {
    ch.strip->setBrightness(200);
    ch.strip->clear();
  }
  ch.strip->show();
}

bool LedManager::begin(uint8_t pin1, uint8_t pin2, uint8_t pin3,
                       uint16_t ledsPerChannel, uint16_t ledsChannel3) {
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
  startupSweepActive_ = true;
  startupStartMs_ = millis();
  started_ = true;
  return true;
}

void LedManager::triggerStartupSweep() {
  startupSweepActive_ = true;
  startupStartMs_ = millis();
}

void LedManager::fillStrip(Adafruit_NeoPixel& strip, uint32_t rgb) {
  for (uint16_t i = 0; i < strip.numPixels(); ++i) {
    strip.setPixelColor(i, rgb);
  }
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

void LedManager::renderChannel(Channel& ch, const state::VehicleState& s, uint32_t nowMs, uint8_t idx) {
  if (!ch.strip) return;
  ch.strip->setBrightness(s.led_global_brightness);

  uint8_t channelBrightness = ch.brightness;
  if (s.night_mode_enabled && channelBrightness > 90) channelBrightness = 90;

  if (!ch.enabled || ch.mode == state::LedMode::OFF) {
    fillStrip(*ch.strip, 0);
    ch.strip->show();
    return;
  }

  uint32_t rgb = ch.color;
  switch (ch.mode) {
    case state::LedMode::STATIC_COLOR:
      rgb = scaleColor(ch.color, channelBrightness);
      fillStrip(*ch.strip, rgb);
      break;
    case state::LedMode::BREATHING: {
      const float phase = (nowMs % kBreathingPeriodMs) / static_cast<float>(kBreathingPeriodMs);
      const float wave = 0.2f + 0.8f * (0.5f + 0.5f * sinf(phase * 6.283185f));
      rgb = scaleColor(ch.color, static_cast<uint8_t>(channelBrightness * wave));
      fillStrip(*ch.strip, rgb);
      break;
    }
    case state::LedMode::RAINBOW:
      for (uint16_t i = 0; i < ch.strip->numPixels(); ++i) {
        const uint8_t p = static_cast<uint8_t>(((i * 256 / ch.strip->numPixels()) + (nowMs / 8)) & 0xFF);
        ch.strip->setPixelColor(i, scaleColor(wheel(p), channelBrightness));
      }
      break;
    case state::LedMode::RPM_REACTIVE: {
      const uint8_t react = static_cast<uint8_t>(min<uint16_t>(255, s.rpm / kRpmBrightnessScaleDivisor));
      rgb = scaleColor(ch.color, min<uint8_t>(255, max<uint8_t>(20, react)));
      fillStrip(*ch.strip, rgb);
      break;
    }
    case state::LedMode::WARNING_FLASH:
      rgb = ((nowMs / kWarningFlashMs) % 2 == 0) ? scaleColor(0xFF4000, channelBrightness) : 0;
      fillStrip(*ch.strip, rgb);
      break;
    case state::LedMode::METH_ACTIVE:
      rgb = (s.meth_state == state::MethState::SPRAYING) ? scaleColor(0x00FFFF, channelBrightness) : scaleColor(0x001010, 20);
      fillStrip(*ch.strip, rgb);
      break;
    case state::LedMode::CAN_FAULT:
      rgb = ((nowMs / kCanFaultFlashMs) % 2 == 0) ? scaleColor(0xFF0000, channelBrightness) : 0;
      fillStrip(*ch.strip, rgb);
      break;
    case state::LedMode::STARTUP_SWEEP: {
      const uint16_t pos = static_cast<uint16_t>((nowMs / kStartupStepMs + idx * 3) % max<uint16_t>(1, ch.strip->numPixels()));
      fillStrip(*ch.strip, 0);
      ch.strip->setPixelColor(pos, scaleColor(ch.color, channelBrightness));
      break;
    }
    case state::LedMode::RPM_GAUGE: {
      const uint16_t n = ch.strip->numPixels();
      // Number of lit LEDs proportional to RPM (at least 1 if engine is turning)
      const uint8_t lit = (s.rpm == 0U) ? 0U
          : static_cast<uint8_t>(max<uint32_t>(
                1U, min<uint32_t>(n,
                    (static_cast<uint32_t>(s.rpm) * n + kRpmGaugeMax - 1U) / kRpmGaugeMax)));
      for (uint16_t i = 0; i < n; ++i) {
        ch.strip->setPixelColor(i,
            i < lit ? scaleColor(rpmGaugeColor(i, n), channelBrightness) : 0U);
      }
      break;
    }
    case state::LedMode::OFF:
    default:
      fillStrip(*ch.strip, 0);
      break;
  }

  if (s.fault_flags != 0 && ch.mode != state::LedMode::CAN_FAULT && ((nowMs / 160) % 2 == 0)) {
    fillStrip(*ch.strip, scaleColor(0xFF0000, channelBrightness));
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
        renderChannel(channels_[i], s, nowMs + i * 80U, i);
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
