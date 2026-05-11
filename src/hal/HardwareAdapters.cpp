#include "hal/HardwareAdapters.hpp"

#include <esp32-hal-ledc.h>

#include "config/SystemConfig.hpp"

namespace ccm::hal {

bool TwaiCanAdapter::begin(uint32_t bitrate) {
  bitrate_ = bitrate;
  return bitrate_ > 0;
}

bool TwaiCanAdapter::send(const can::CanFrame& frame) {
  return frame.dlc <= 8 && bitrate_ > 0;
}

bool TwaiCanAdapter::receive(can::CanFrame&) {
  return false;
}

bool UartGpsAdapter::begin(uint32_t baud) {
  serial_.begin(baud, SERIAL_8N1, config::kGpsRxPin, config::kGpsTxPin);
  return true;
}

void UartGpsAdapter::poll() {
  while (serial_.available() > 0) {
    parser_.encode(static_cast<char>(serial_.read()));
  }

  latest_.validFix = parser_.location.isValid();
  latest_.speedKph = parser_.speed.kmph();
  latest_.latitude = parser_.location.lat();
  latest_.longitude = parser_.location.lng();
  latest_.altitudeM = parser_.altitude.meters();
  latest_.satellites = parser_.satellites.value();
  if (latest_.validFix) {
    latest_.lastFixMs = millis();
  }
}

bool GpioButtonAdapter::begin() {
  pinMode(upPin_, INPUT_PULLUP);
  pinMode(downPin_, INPUT_PULLUP);
  pinMode(selectPin_, INPUT_PULLUP);
  return true;
}

bool GpioButtonAdapter::debounceRead(uint8_t pin, bool& lastState, uint32_t& lastMs) {
  const bool current = digitalRead(pin);
  if (current != lastState && (millis() - lastMs) >= kDebounceMs) {
    lastMs = millis();
    lastState = current;
    return true;
  }
  return false;
}

bool GpioButtonAdapter::poll(ButtonEvent& event) {
  event = {};
  if (debounceRead(upPin_, upState_, upDebounceMs_) && upState_ == LOW) {
    event.type = ButtonEventType::Up;
  } else if (debounceRead(downPin_, downState_, downDebounceMs_) && downState_ == LOW) {
    event.type = ButtonEventType::Down;
  } else if (debounceRead(selectPin_, selectState_, selectDebounceMs_) && selectState_ == LOW) {
    event.type = ButtonEventType::Select;
  }

  if (event.type != ButtonEventType::None) {
    event.timestampMs = millis();
    return true;
  }

  return false;
}

bool LedcTachAdapter::begin(uint8_t pin, uint8_t channel) {
  pin_ = pin;
  channel_ = channel;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  initialized_ = ledcAttachChannel(pin_, 100, 8, channel_);
#else
  ledcSetup(channel_, 100, 8);
  ledcAttachPin(pin_, channel_);
  initialized_ = true;
#endif
  return initialized_;
}

void LedcTachAdapter::setFrequencyHz(uint32_t hz, uint8_t duty) {
  if (!initialized_ || hz == 0) return;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWriteTone(pin_, hz);
  ledcWrite(pin_, duty);
#else
  ledcWriteTone(channel_, hz);
  ledcWrite(channel_, duty);
#endif
}

core::EnvironmentData StubSensorAdapter::readEnvironment() {
  core::EnvironmentData env;
  const uint32_t t = millis() / 1000;
  env.cabinC = 26.0f + sinf(t * 0.01f);
  env.engineBayC = 55.0f + sinf(t * 0.03f);
  env.outsideC = 22.0f + sinf(t * 0.008f);
  env.intakeC = 33.0f + sinf(t * 0.02f);
  env.humidity = 45.0f;
  return env;
}

core::PowerData StubSensorAdapter::readPower() {
  core::PowerData p;
  p.batteryV = 12.9f;
  p.undervoltage = p.batteryV < config::kUndervoltageThreshold;
  return p;
}

}  // namespace ccm::hal
