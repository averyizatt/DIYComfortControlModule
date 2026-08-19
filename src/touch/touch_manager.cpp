#include "touch/touch_manager.h"

namespace touch {

bool TouchManager::begin(TwoWire& wire, uint8_t sdaPin, uint8_t sclPin, uint8_t rstPin, uint8_t intPin, uint8_t address) {
  wire_ = &wire;
  address_ = address;
  rstPin_ = rstPin;
  intPin_ = intPin;

  wire_->begin(sdaPin, sclPin);
  // Some FT6x36-compatible panels are unreliable at 400 kHz, especially on
  // longer display harnesses. Touch traffic is tiny, so prefer robustness.
  wire_->setClock(100000U);
  wire_->setTimeOut(50);
  if (rstPin_ != 255) {
    pinMode(rstPin_, OUTPUT);
    digitalWrite(rstPin_, LOW);
    delay(20);
    digitalWrite(rstPin_, HIGH);
    delay(120);
  }
  if (intPin_ != 255) pinMode(intPin_, INPUT_PULLUP);

  online_ = false;
  for (uint8_t attempt = 0; attempt < 5U && !online_; ++attempt) {
    online_ = probe();
    if (!online_) delay(25);
  }
  readFailCount_ = 0;
  lastProbeMs_ = millis();
  Serial.printf("[TOUCH] begin addr=0x%02X SDA=%u SCL=%u RST=%u INT=%u -> %s\n",
                static_cast<unsigned>(address_),
                static_cast<unsigned>(sdaPin),
                static_cast<unsigned>(sclPin),
                static_cast<unsigned>(rstPin_),
                static_cast<unsigned>(intPin_),
                online_ ? "OK" : "OFF");
  if (online_) {
    // Force normal operating mode after reset. A controller can ACK at 0x38
    // while remaining asleep, which looks "online" but never reports touches.
    writeRegister(0xA5, 0x00);  // power mode: active
    writeRegister(0x00, 0x00);  // device mode: normal
    writeRegister(0xA4, 0x00);  // G_MODE: polling (not one-shot interrupt)
    delay(10);

    uint8_t chip = 0xFF;
    uint8_t firmware = 0xFF;
    uint8_t vendor = 0xFF;
    uint8_t power = 0xFF;
    uint8_t mode = 0xFF;
    uint8_t reportMode = 0xFF;
    const bool registersOk =
        readRegister(0xA3, chip) &&
        readRegister(0xA6, firmware) &&
        readRegister(0xA8, vendor) &&
        readRegister(0xA5, power) &&
        readRegister(0x00, mode) &&
        readRegister(0xA4, reportMode);
    Serial.printf("[TOUCH] controller regs=%s chip=0x%02X fw=0x%02X vendor=0x%02X power=0x%02X mode=0x%02X report=0x%02X irq=%d\n",
                  registersOk ? "OK" : "READ-FAIL",
                  static_cast<unsigned>(chip),
                  static_cast<unsigned>(firmware),
                  static_cast<unsigned>(vendor),
                  static_cast<unsigned>(power),
                  static_cast<unsigned>(mode),
                  static_cast<unsigned>(reportMode),
                  intPin_ == 255 ? -1 : digitalRead(intPin_));
  }
  return online_;
}

bool TouchManager::probe() {
  if (!wire_) return false;
  wire_->beginTransmission(address_);
  return wire_->endTransmission(true) == 0;
}

bool TouchManager::readRegister(uint8_t reg, uint8_t& value) {
  if (!wire_) return false;
  wire_->beginTransmission(address_);
  wire_->write(reg);
  if (wire_->endTransmission(false) != 0) return false;
  if (wire_->requestFrom(static_cast<int>(address_), 1) != 1) return false;
  value = static_cast<uint8_t>(wire_->read());
  return true;
}

bool TouchManager::writeRegister(uint8_t reg, uint8_t value) {
  if (!wire_) return false;
  wire_->beginTransmission(address_);
  wire_->write(reg);
  wire_->write(value);
  return wire_->endTransmission(true) == 0;
}

void TouchManager::markOffline(const char* reason) {
  if (readFailCount_ < 255U) {
    ++readFailCount_;
  }
  if (readFailCount_ < 6U) {
    return;
  }
  if (online_) {
    online_ = false;
    const uint32_t nowMs = millis();
    if ((nowMs - lastStateLogMs_) >= 1000U) {
      lastStateLogMs_ = nowMs;
      Serial.printf("[TOUCH] offline: %s\n", reason ? reason : "read failed");
    }
  }
}

TouchSample TouchManager::read() {
  TouchSample s{};
  if (!wire_) return s;

  if (!online_) {
    const uint32_t nowMs = millis();
    if ((nowMs - lastProbeMs_) >= 1000U) {
      lastProbeMs_ = nowMs;
      online_ = probe();
      if (online_) {
        readFailCount_ = 0;
        Serial.printf("[TOUCH] online addr=0x%02X\n", static_cast<unsigned>(address_));
      }
    }
    if (!online_) return s;
  }

  wire_->beginTransmission(address_);
  wire_->write(0x02);
  // This controller revision requires a STOP after selecting TD_STATUS. Using
  // a repeated start here still ACKs but returns no usable touch samples.
  if (wire_->endTransmission(true) != 0) {
    markOffline("register select");
    return s;
  }

  constexpr uint8_t reqLen = 5;
  const uint8_t got = wire_->requestFrom(static_cast<int>(address_), static_cast<int>(reqLen));
  if (got < reqLen) {
    while (wire_->available()) wire_->read();
    markOffline("short read");
    return s;
  }

  const uint8_t touches = wire_->read() & 0x0F;
  const uint8_t xh = wire_->read();
  const uint8_t xl = wire_->read();
  const uint8_t yh = wire_->read();
  const uint8_t yl = wire_->read();

  s.touched = touches > 0 && touches <= 5;
  s.x = static_cast<uint16_t>(((xh & 0x0F) << 8) | xl);
  s.y = static_cast<uint16_t>(((yh & 0x0F) << 8) | yl);
  const uint32_t nowMs = millis();
  if (!s.touched && intPin_ != 255 && digitalRead(intPin_) == LOW &&
      (nowMs - lastIrqStatusLogMs_) >= 1000U) {
    lastIrqStatusLogMs_ = nowMs;
    Serial.printf("[TOUCH] IRQ active but TD_STATUS=%u\n",
                  static_cast<unsigned>(touches));
  }
  readFailCount_ = 0;
  return s;
}

}  // namespace touch
