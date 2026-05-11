#include "led/led_manager.h"

namespace led {

bool LedManager::begin(uint8_t pin1, uint8_t pin2, uint8_t pin3, uint16_t ledsPerChannel) {
  ledsPerChannel_ = ledsPerChannel;
  strip1_.updateLength(ledsPerChannel_);
  strip2_.updateLength(ledsPerChannel_);
  strip3_.updateLength(ledsPerChannel_);
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
      const float phase = (nowMs % 2200U) / 2200.0f;
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
      const uint8_t react = static_cast<uint8_t>(min<uint16_t>(255, s.rpm / 30));
      rgb = scaleColor(ch.color, min<uint8_t>(255, max<uint8_t>(20, react)));
      fillStrip(*ch.strip, rgb);
      break;
    }
    case state::LedMode::WARNING_FLASH:
      rgb = ((nowMs / 120) % 2 == 0) ? scaleColor(0xFF4000, channelBrightness) : 0;
      fillStrip(*ch.strip, rgb);
      break;
    case state::LedMode::METH_ACTIVE:
      rgb = (s.meth_state == state::MethState::SPRAYING) ? scaleColor(0x00FFFF, channelBrightness) : scaleColor(0x001010, 20);
      fillStrip(*ch.strip, rgb);
      break;
    case state::LedMode::CAN_FAULT:
      rgb = ((nowMs / 100) % 2 == 0) ? scaleColor(0xFF0000, channelBrightness) : 0;
      fillStrip(*ch.strip, rgb);
      break;
    case state::LedMode::STARTUP_SWEEP: {
      const uint16_t pos = static_cast<uint16_t>((nowMs / 40 + idx * 3) % max<uint16_t>(1, ch.strip->numPixels()));
      fillStrip(*ch.strip, 0);
      ch.strip->setPixelColor(pos, scaleColor(ch.color, channelBrightness));
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
  if ((nowMs - lastFrameMs_) < 16) return;
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
      channels_[i].mode = state::LedMode::STARTUP_SWEEP;
      renderChannel(channels_[i], s, nowMs + i * 80U, i);
    }
    if (elapsed > 1800U && !s.led_startup_preview) {
      startupSweepActive_ = false;
    }
    return;
  }

  for (uint8_t i = 0; i < 3; ++i) {
    renderChannel(channels_[i], s, nowMs, i);
  }
}

}  // namespace led
