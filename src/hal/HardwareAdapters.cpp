#include "hal/HardwareAdapters.hpp"

#include <cstdlib>

#include <esp_arduino_version.h>
#include <esp32-hal-ledc.h>
#include <Wire.h>

#if __has_include(<driver/twai.h>)
#include <driver/twai.h>
#define CCM_HAS_TWAI 1
#else
#define CCM_HAS_TWAI 0
#endif

#include "config/SystemConfig.hpp"

#ifndef CCM_IMU_ENABLED
#define CCM_IMU_ENABLED 0
#endif

#ifndef CCM_GPS_SERIAL_LOGS
#define CCM_GPS_SERIAL_LOGS 1
#endif

namespace ccm::hal {

namespace {
constexpr uint32_t kTachPwmInitialHz = 100;
constexpr uint8_t kTachPwmResolutionBits = 8;

constexpr uint32_t kGpsNoRxProbeMs = 15000;
constexpr uint32_t kGpsNoGoodSentenceProbeMs = 15000;
constexpr uint32_t kGpsPreferredBaudHoldMs = 30000;
constexpr uint32_t kGpsProbeBauds[] = {9600, 38400, 57600, 115200, 4800};
constexpr const char* kGpsGgaTalkers[] = {"GPGGA", "GNGGA"};
constexpr const char* kGpsRmcTalkers[] = {"GPRMC", "GNRMC"};
constexpr const char* kGpsGsvTalkers[] = {"GPGSV", "GNGSV", "GLGSV", "GAGSV", "GBGSV", "BDGSV"};
constexpr const char* kGpsGsaTalkers[] = {"GPGSA", "GNGSA", "GLGSA", "GAGSA", "GBGSA", "BDGSA"};

bool parseUnsignedField(const char* value, uint32_t& out) {
  if (!value || value[0] == '\0') {
    return false;
  }

  char* end = nullptr;
  const unsigned long parsed = std::strtoul(value, &end, 10);
  if (end == value) {
    return false;
  }

  out = static_cast<uint32_t>(parsed);
  return true;
}

void updateCustomField(TinyGPSCustom& field, uint32_t& target) {
  if (!field.isUpdated()) {
    return;
  }

  uint32_t parsed = 0;
  if (parseUnsignedField(field.value(), parsed)) {
    target = parsed;
  }
}
}  // namespace

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

void UartGpsAdapter::openSerial(uint32_t baud) {
  const uint8_t rxPin = activePinsSwapped_ ? config::kGpsTxPin : config::kGpsRxPin;
  const uint8_t txPin = activePinsSwapped_ ? config::kGpsRxPin : config::kGpsTxPin;
  serial_.end();
  serial_.setRxBufferSize(2048);
  pinMode(rxPin, INPUT);
  const uint8_t rxIdleBeforeBegin = static_cast<uint8_t>(digitalRead(rxPin));
  serial_.begin(baud, SERIAL_8N1, rxPin, txPin);
  while (serial_.available() > 0) {
    (void)serial_.read();
  }

  parser_ = TinyGPSPlus();
  attachCustomNmeaFields();
  latest_ = {};
  latest_.rxIdleLevel = rxIdleBeforeBegin;
  latest_.baud = baud;
  currentBaud_ = baud;
  lastBaudOpenMs_ = millis();
  lastGoodSentenceMs_ = 0;
  lastPassedChecksum_ = 0;
  ubxTuningSent_ = false;
  seenGgaFixQuality_ = false;
  seenGsaFixMode_ = false;
  seenRmcStatus_ = false;
  rmcStatusValid_ = false;
  if (CCM_GPS_SERIAL_LOGS) {
    Serial.printf("[GPS] UART baud=%lu RX=%u TX=%u rx_idle=%u pinmap=%s\n",
                 static_cast<unsigned long>(baud),
                 static_cast<unsigned>(rxPin),
                 static_cast<unsigned>(txPin),
                 static_cast<unsigned>(rxIdleBeforeBegin),
                 activePinsSwapped_ ? "alternate" : "configured");
  }
}

void UartGpsAdapter::attachCustomNmeaFields() {
  for (uint8_t i = 0; i < 2; ++i) {
    ggaFixQuality_[i].begin(parser_, kGpsGgaTalkers[i], 6);
    rmcStatus_[i].begin(parser_, kGpsRmcTalkers[i], 2);
  }
  for (uint8_t i = 0; i < 6; ++i) {
    gsvSatsInView_[i].begin(parser_, kGpsGsvTalkers[i], 3);
    gsaFixMode_[i].begin(parser_, kGpsGsaTalkers[i], 2);
  }
}

bool UartGpsAdapter::begin(uint32_t baud) {
  preferredBaud_ = baud;
  baudProbeIndex_ = 0;
  noRxProbePhase_ = 0;
  activePinsSwapped_ = false;
  for (uint8_t i = 0; i < sizeof(kGpsProbeBauds) / sizeof(kGpsProbeBauds[0]); ++i) {
    if (kGpsProbeBauds[i] == baud) {
      baudProbeIndex_ = i;
      break;
    }
  }
  openSerial(baud);
  return true;
}

void UartGpsAdapter::sendUbx(uint8_t msgClass, uint8_t msgId,
                             const uint8_t* payload, size_t payloadLen) {
  const uint8_t lenL = static_cast<uint8_t>(payloadLen & 0xFFU);
  const uint8_t lenH = static_cast<uint8_t>((payloadLen >> 8) & 0xFFU);
  uint8_t ckA = 0;
  uint8_t ckB = 0;

  auto addChecksum = [&](uint8_t b) {
    ckA = static_cast<uint8_t>(ckA + b);
    ckB = static_cast<uint8_t>(ckB + ckA);
  };

  const uint8_t header[] = {0xB5, 0x62, msgClass, msgId, lenL, lenH};
  serial_.write(header, sizeof(header));
  addChecksum(msgClass);
  addChecksum(msgId);
  addChecksum(lenL);
  addChecksum(lenH);

  for (size_t i = 0; i < payloadLen; ++i) {
    const uint8_t b = payload ? payload[i] : 0;
    serial_.write(b);
    addChecksum(b);
  }

  serial_.write(ckA);
  serial_.write(ckB);
}

void UartGpsAdapter::sendOptionalUbxTuning() {
  if (ubxTuningSent_) {
    return;
  }

  // UBX-CFG-NAV5: automotive dynamic model, automatic 2D/3D fix mode.
  // This helps receivers stay locked while moving once a solution is forming.
  static const uint8_t kUbxCfgNav5Automotive[] = {
    0x05, 0x00,  // mask: dynamic model + fix mode
    0x04,        // dynModel: automotive
    0x03,        // fixMode: auto 2D/3D
    0x00, 0x00, 0x00, 0x00,  // fixedAlt
    0x00, 0x00, 0x00, 0x00,  // fixedAltVar
    0x00,        // minElev
    0x00,        // drLimit
    0x00, 0x00,  // pDop
    0x00, 0x00,  // tDop
    0x00, 0x00,  // pAcc
    0x00, 0x00,  // tAcc
    0x00,        // staticHoldThresh
    0x00,        // dgpsTimeOut
    0x00,        // cnoThreshNumSVs
    0x00,        // cnoThresh
    0x00, 0x00,  // reserved1
    0x00, 0x00,  // staticHoldMaxDist
    0x00,        // utcStandard
    0x00, 0x00, 0x00, 0x00, 0x00  // reserved2
  };
  sendUbx(0x06, 0x24, kUbxCfgNav5Automotive, sizeof(kUbxCfgNav5Automotive));

  if (currentBaud_ >= 38400U) {
    // Set navigation update rate to 5 Hz (200 ms measurement period).
    // UBX-CFG-RATE: measRate=200 ms, navRate=1, timeRef=GPS.
    static const uint8_t kUbxSetRate5HzPayload[] = {
      0xC8, 0x00,  // measRate: 200 ms (5 Hz), little-endian
      0x01, 0x00,  // navRate: 1 solution per measurement epoch
      0x01, 0x00   // timeRef: GPS time
    };
    sendUbx(0x06, 0x08, kUbxSetRate5HzPayload, sizeof(kUbxSetRate5HzPayload));
  }

  // Enable SBAS (WAAS/EGNOS/MSAS) augmentation where available.
  // UBX-CFG-SBAS: enable, all usage bits, 3 channels, auto-scan.
  static const uint8_t kUbxEnableSbasPayload[] = {
    0x01,                    // mode: enable SBAS
    0x07,                    // usage: range + differential + integrity
    0x03,                    // max SBAS channels
    0x00,                    // scanmode2
    0x00, 0x00, 0x00, 0x00  // scanmode1: auto-scan all PRNs
  };
  sendUbx(0x06, 0x16, kUbxEnableSbasPayload, sizeof(kUbxEnableSbasPayload));
  serial_.flush();

  ubxTuningSent_ = true;
  if (CCM_GPS_SERIAL_LOGS) {
    Serial.printf("[GPS] NMEA detected at %lu baud; sent UBX automotive+SBAS tuning%s\n",
                 static_cast<unsigned long>(currentBaud_),
                 currentBaud_ >= 38400U ? " + 5Hz rate" : "");
  }
}

void UartGpsAdapter::maybeAutoBaud(uint32_t nowMs) {
  if (lastGoodSentenceMs_ != 0) {
    return;
  }
  if (preferredBaud_ != 0 && currentBaud_ == preferredBaud_ &&
      (nowMs - lastBaudOpenMs_) < kGpsPreferredBaudHoldMs) {
    return;
  }

  const bool noRx = latest_.lastRxMs == 0 && (nowMs - lastBaudOpenMs_) >= kGpsNoRxProbeMs;
  const bool noGoodSentence = latest_.lastRxMs != 0 && lastGoodSentenceMs_ == 0 &&
                              (nowMs - lastBaudOpenMs_) >= kGpsNoGoodSentenceProbeMs;
  if (!noRx && !noGoodSentence) {
    return;
  }

  if (noRx && noRxProbePhase_ == 0U) {
    noRxProbePhase_ = 1U;
    activePinsSwapped_ = !activePinsSwapped_;
    if (CCM_GPS_SERIAL_LOGS) {
      Serial.printf("[GPS] no UART bytes at %lu baud; trying alternate GPS RX/TX pin orientation\n",
                   static_cast<unsigned long>(currentBaud_));
    }
    openSerial(currentBaud_);
    return;
  }

  const uint8_t probeCount = sizeof(kGpsProbeBauds) / sizeof(kGpsProbeBauds[0]);
  for (uint8_t tries = 0; tries < probeCount; ++tries) {
    baudProbeIndex_ = static_cast<uint8_t>((baudProbeIndex_ + 1U) % probeCount);
    const uint32_t nextBaud = kGpsProbeBauds[baudProbeIndex_];
    if (nextBaud != currentBaud_) {
      if (CCM_GPS_SERIAL_LOGS) {
        Serial.printf("[GPS] no valid NMEA after hold at %lu baud; probing %lu baud\n",
                     static_cast<unsigned long>(currentBaud_),
                     static_cast<unsigned long>(nextBaud));
      }
      noRxProbePhase_ = 0;
      openSerial(nextBaud);
      return;
    }
  }
}

void UartGpsAdapter::poll() {
  const uint32_t nowMs = millis();
  bool sawByte = false;
  while (serial_.available() > 0) {
    sawByte = true;
    const uint8_t b = static_cast<uint8_t>(serial_.read());
    if (latest_.rawSampleLen < sizeof(latest_.rawSample)) {
      latest_.rawSample[latest_.rawSampleLen++] = b;
    } else {
      for (uint8_t i = 1; i < sizeof(latest_.rawSample); ++i) {
        latest_.rawSample[i - 1] = latest_.rawSample[i];
      }
      latest_.rawSample[sizeof(latest_.rawSample) - 1] = b;
    }
    ++latest_.rawBytes;
    if (b == '$') {
      ++latest_.rawDollarBytes;
    }
    if (b >= 0x20U && b <= 0x7EU) {
      ++latest_.rawPrintableBytes;
    }
    parser_.encode(static_cast<char>(b));
  }

  if (sawByte) {
    latest_.lastRxMs = nowMs;
  }

  latest_.speedKph = parser_.speed.kmph();
  latest_.latitude = parser_.location.lat();
  latest_.longitude = parser_.location.lng();
  latest_.altitudeM = parser_.altitude.meters();
  latest_.satellites = parser_.satellites.value();
  for (uint8_t i = 0; i < 2; ++i) {
    uint32_t parsed = 0;
    if (ggaFixQuality_[i].isUpdated() && parseUnsignedField(ggaFixQuality_[i].value(), parsed)) {
      latest_.fixQuality = static_cast<uint8_t>(parsed > 255U ? 255U : parsed);
      seenGgaFixQuality_ = true;
    }
    if (rmcStatus_[i].isUpdated()) {
      const char* status = rmcStatus_[i].value();
      if (status && status[0] != '\0') {
        seenRmcStatus_ = true;
        rmcStatusValid_ = (status[0] == 'A');
      }
    }
  }
  for (uint8_t i = 0; i < 6; ++i) {
    updateCustomField(gsvSatsInView_[i], latest_.satellitesInView);
    uint32_t parsed = 0;
    if (gsaFixMode_[i].isUpdated() && parseUnsignedField(gsaFixMode_[i].value(), parsed)) {
      latest_.fixMode = static_cast<uint8_t>(parsed > 255U ? 255U : parsed);
      seenGsaFixMode_ = true;
    }
  }
  latest_.baud = currentBaud_;
  latest_.charsProcessed = parser_.charsProcessed();
  latest_.passedChecksum = parser_.passedChecksum();
  latest_.failedChecksum = parser_.failedChecksum();
  latest_.sentencesWithFix = parser_.sentencesWithFix();
  latest_.hdopHundredths = parser_.hdop.isValid()
      ? static_cast<uint16_t>(parser_.hdop.value() > 65535UL ? 65535UL : parser_.hdop.value())
      : 0U;
  const bool hasFreshUtcTime =
      parser_.time.isValid() && parser_.date.isValid() && parser_.time.age() < 2000UL;
  if (hasFreshUtcTime) {
    latest_.utcTimeValid = true;
    latest_.utcHour = parser_.time.hour();
    latest_.utcMinute = parser_.time.minute();
    latest_.utcSecond = parser_.time.second();
    latest_.utcCentisecond = parser_.time.centisecond();
    latest_.utcDay = parser_.date.day();
    latest_.utcMonth = parser_.date.month();
    latest_.utcYear = parser_.date.year();
    latest_.lastTimeMs = nowMs;
  }
  const bool hasCurrentNmeaFix =
      (seenGgaFixQuality_ && latest_.fixQuality > 0U) ||
      (seenGsaFixMode_ && latest_.fixMode >= 2U);
  const bool hasCurrentRmcFix = seenRmcStatus_ && rmcStatusValid_;
  latest_.validFix = parser_.location.isValid() && (hasCurrentNmeaFix || hasCurrentRmcFix);
  if (latest_.passedChecksum != lastPassedChecksum_) {
    lastPassedChecksum_ = latest_.passedChecksum;
    lastGoodSentenceMs_ = nowMs;
    sendOptionalUbxTuning();
  }
  if (latest_.validFix) {
    latest_.lastFixMs = nowMs;
  }

  maybeAutoBaud(nowMs);
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
  initialized_ = ledcAttachChannel(pin_, kTachPwmInitialHz, kTachPwmResolutionBits, channel_);
#else
  ledcSetup(channel_, kTachPwmInitialHz, kTachPwmResolutionBits);
  ledcAttachPin(pin_, channel_);
  initialized_ = true;
#endif
  return initialized_;
}

void LedcTachAdapter::setFrequencyHz(uint32_t hz, uint8_t duty) {
  if (!initialized_) return;
  if (hz == 0) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(pin_, 0);
#else
    ledcWrite(channel_, 0);
#endif
    return;
  }
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  if (ledcChangeFrequency(pin_, hz, kTachPwmResolutionBits) != 0) {
    ledcWrite(pin_, duty);
  }
#else
  ledcWriteTone(channel_, hz);
  ledcWrite(channel_, duty);
#endif
}

bool BoardSensorAdapter::begin() {
  pinMode(config::kBatterySensePin, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(config::kBatterySensePin, ADC_11db);

#if CCM_IMU_ENABLED
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
#else
  gyroOnline_ = false;
#endif
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
  // analogReadMilliVolts uses the IDF ADC calibration (two-point/curve eFuse)
  // for accurate voltage — no manual raw/4095*3.3 approximation needed.
  const float adcVolts = analogReadMilliVolts(config::kBatterySensePin) / 1000.0f;
  const float dividerScale = (config::kBatteryDividerTopOhms + config::kBatteryDividerBottomOhms) / config::kBatteryDividerBottomOhms;
  p.batteryV = adcVolts * dividerScale;
  p.undervoltage = p.batteryV < config::kUndervoltageThreshold;
  return p;
}

}  // namespace ccm::hal
