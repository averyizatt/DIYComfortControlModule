#include "touch/touch_manager.h"

namespace touch {

bool TouchManager::begin(TwoWire& wire, uint8_t sdaPin, uint8_t sclPin, uint8_t rstPin, uint8_t intPin, uint8_t address) {
  wire_ = &wire;
  address_ = address;
  rstPin_ = rstPin;
  intPin_ = intPin;

  wire_->begin(sdaPin, sclPin);
  wire_->setClock(400000U);
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
  return online_;
}

bool TouchManager::probe() {
  if (!wire_) return false;
  wire_->beginTransmission(address_);
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
  readFailCount_ = 0;
  return s;
}

}  // namespace touch
