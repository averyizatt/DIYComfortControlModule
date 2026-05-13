#include "hal/HardwareAdapters.hpp"

#include <esp32-hal-ledc.h>
#include <Wire.h>

#if __has_include(<driver/twai.h>)
#include <driver/twai.h>
#define CCM_HAS_TWAI 1
#else
#define CCM_HAS_TWAI 0
#endif

#include "config/SystemConfig.hpp"

namespace ccm::hal {

bool TwaiCanAdapter::begin(uint32_t bitrate) {
  bitrate_ = bitrate;
#if CCM_HAS_TWAI
  twai_timing_config_t tConfig{};
  if (bitrate == 125000) {
    tConfig = TWAI_TIMING_CONFIG_125KBITS();
  } else if (bitrate == 250000) {
    tConfig = TWAI_TIMING_CONFIG_250KBITS();
  } else if (bitrate == 500000) {
    tConfig = TWAI_TIMING_CONFIG_500KBITS();
  } else if (bitrate == 1000000) {
    tConfig = TWAI_TIMING_CONFIG_1MBITS();
  } else {
    return false;
  }

  const twai_general_config_t gConfig =
      TWAI_GENERAL_CONFIG_DEFAULT(static_cast<gpio_num_t>(config::kCanTxPin), static_cast<gpio_num_t>(config::kCanRxPin), TWAI_MODE_NORMAL);
  const twai_filter_config_t fConfig = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&gConfig, &tConfig, &fConfig) != ESP_OK) {
    return false;
  }
  if (twai_start() != ESP_OK) {
    twai_driver_uninstall();
    return false;
  }
  started_ = true;
  return true;
#else
  return false;
#endif
}

bool TwaiCanAdapter::send(const can::CanFrame& frame) {
#if CCM_HAS_TWAI
  if (!started_ || frame.dlc > 8) return false;

  twai_message_t tx{};
  tx.identifier = frame.id;
  tx.extd = 0;
  tx.rtr = 0;
  tx.data_length_code = frame.dlc;
  for (uint8_t i = 0; i < frame.dlc; ++i) {
    tx.data[i] = frame.data[i];
  }
  return twai_transmit(&tx, 0) == ESP_OK;
#else
  (void)frame;
  return false;
#endif
}

bool TwaiCanAdapter::receive(can::CanFrame& frame) {
#if CCM_HAS_TWAI
  if (!started_) return false;
  twai_message_t rx{};
  if (twai_receive(&rx, 0) != ESP_OK) return false;
  frame.id = static_cast<uint16_t>(rx.identifier & 0x7FFU);
  frame.dlc = rx.data_length_code;
  for (uint8_t i = 0; i < frame.dlc && i < 8; ++i) {
    frame.data[i] = rx.data[i];
  }
  return true;
#else
  (void)frame;
  return false;
#endif
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

bool BoardSensorAdapter::begin() {
  pinMode(config::kBatterySensePin, INPUT);
  analogReadResolution(12);
#if defined(ADC_11db)
  analogSetPinAttenuation(config::kBatterySensePin, ADC_11db);
#endif

  pinMode(config::kGyroAddrSelPin, OUTPUT);
  digitalWrite(config::kGyroAddrSelPin, LOW);
  pinMode(config::kGyroIntPin, INPUT_PULLUP);
  Wire.begin(config::kGyroSdaPin, config::kGyroSclPin);

  Wire.beginTransmission(config::kGyroI2cAddrPrimary);
  gyroOnline_ = (Wire.endTransmission() == 0);
  if (!gyroOnline_) {
    Wire.beginTransmission(config::kGyroI2cAddrSecondary);
    gyroOnline_ = (Wire.endTransmission() == 0);
  }
  return true;
}

core::EnvironmentData BoardSensorAdapter::readEnvironment() {
  core::EnvironmentData env;
  env.cabinC = NAN;
  env.engineBayC = static_cast<float>(temperatureRead());
  env.outsideC = NAN;
  env.intakeC = NAN;
  env.humidity = NAN;
  (void)gyroOnline_;
  return env;
}

core::PowerData BoardSensorAdapter::readPower() {
  core::PowerData p;
  const uint16_t raw = analogRead(config::kBatterySensePin);
  const float adcVolts = (static_cast<float>(raw) / config::kAdcMaxCount) * config::kAdcRefVoltage;
  const float dividerScale = (config::kBatteryDividerTopOhms + config::kBatteryDividerBottomOhms) / config::kBatteryDividerBottomOhms;
  p.batteryV = adcVolts * dividerScale;
  p.undervoltage = p.batteryV < config::kUndervoltageThreshold;
  return p;
}

}  // namespace ccm::hal
