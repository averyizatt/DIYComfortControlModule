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
  // Step 1: open at factory-default baud (9600) to talk to the module on first boot.
  // Increase the RX buffer to handle 38400 baud bursts without overflow.
  serial_.setRxBufferSize(512);
  serial_.begin(baud, SERIAL_8N1, config::kGpsRxPin, config::kGpsTxPin);
  delay(100);  // let GPS finish its power-up NMEA burst

  // Step 2: Switch GPS UART to 38400 baud.
  // UBX-CFG-PRT: portID=UART1, 8N1, 38400 baud, UBX+NMEA in+out.
  // CK_A=0x93, CK_B=0x90 (Fletcher over class through last payload byte).
  static const uint8_t kUbxSetBaud38400[] = {
    0xB5, 0x62,              // UBX sync header
    0x06, 0x00,              // class=CFG, id=PRT
    0x14, 0x00,              // payload length = 20
    0x01,                    // portID: UART1
    0x00,                    // reserved
    0x00, 0x00,              // txReady: disabled
    0xD0, 0x08, 0x00, 0x00, // mode: 8 data, 1 stop, no parity
    0x00, 0x96, 0x00, 0x00, // baudRate: 38400 (little-endian)
    0x07, 0x00,              // inProtoMask: UBX+NMEA+RTCM
    0x03, 0x00,              // outProtoMask: UBX+NMEA
    0x00, 0x00,              // flags
    0x00, 0x00,              // reserved
    0x93, 0x90               // CK_A, CK_B
  };
  serial_.write(kUbxSetBaud38400, sizeof(kUbxSetBaud38400));
  serial_.flush();
  delay(100);  // allow GPS to switch its UART before we change host baud

  // Step 3: Reinitialize host UART at 38400 to match GPS.
  serial_.begin(38400, SERIAL_8N1, config::kGpsRxPin, config::kGpsTxPin);
  delay(50);

  // Step 4: Set navigation update rate to 5 Hz (200 ms measurement period).
  // UBX-CFG-RATE: measRate=200 ms, navRate=1, timeRef=GPS.
  // CK_A=0xDE, CK_B=0x6A.
  static const uint8_t kUbxSetRate5Hz[] = {
    0xB5, 0x62,  // UBX sync header
    0x06, 0x08,  // class=CFG, id=RATE
    0x06, 0x00,  // payload length = 6
    0xC8, 0x00,  // measRate: 200 ms (5 Hz), little-endian
    0x01, 0x00,  // navRate: 1 solution per measurement epoch
    0x01, 0x00,  // timeRef: GPS time
    0xDE, 0x6A   // CK_A, CK_B
  };
  serial_.write(kUbxSetRate5Hz, sizeof(kUbxSetRate5Hz));

  // Step 5: Enable SBAS (WAAS/EGNOS/MSAS) augmentation for faster first fix.
  // UBX-CFG-SBAS: enable, all usage bits, 3 channels, auto-scan.
  // CK_A=0x2F, CK_B=0xD5.
  static const uint8_t kUbxEnableSbas[] = {
    0xB5, 0x62,              // UBX sync header
    0x06, 0x16,              // class=CFG, id=SBAS
    0x08, 0x00,              // payload length = 8
    0x01,                    // mode: enable SBAS
    0x07,                    // usage: range + differential + integrity
    0x03,                    // max SBAS channels
    0x00,                    // scanmode2
    0x00, 0x00, 0x00, 0x00, // scanmode1: auto-scan all PRNs
    0x2F, 0xD5               // CK_A, CK_B
  };
  serial_.write(kUbxEnableSbas, sizeof(kUbxEnableSbas));

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

  if (config::kGyroAddrSelPin != 255) {
    pinMode(config::kGyroAddrSelPin, OUTPUT);
    digitalWrite(config::kGyroAddrSelPin, LOW);
  }
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
