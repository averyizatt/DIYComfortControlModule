#include "touch/touch_manager.h"

namespace touch {

bool TouchManager::begin(TwoWire& wire, uint8_t sdaPin, uint8_t sclPin, uint8_t rstPin, uint8_t intPin, uint8_t address) {
  wire_ = &wire;
  address_ = address;
  rstPin_ = rstPin;
  intPin_ = intPin;

  wire_->begin(sdaPin, sclPin);
  if (rstPin_ != 255) {
    pinMode(rstPin_, OUTPUT);
    digitalWrite(rstPin_, LOW);
    delay(5);
    digitalWrite(rstPin_, HIGH);
  }
  if (intPin_ != 255) pinMode(intPin_, INPUT_PULLUP);

  wire_->beginTransmission(address_);
  online_ = (wire_->endTransmission() == 0);
  return online_;
}

TouchSample TouchManager::read() {
  TouchSample s{};
  if (!wire_ || !online_) return s;

  wire_->beginTransmission(address_);
  wire_->write(0x02);
  if (wire_->endTransmission(false) != 0) {
    online_ = false;
    return s;
  }

  const uint8_t reqLen = 5;
  const uint8_t got = wire_->requestFrom(static_cast<int>(address_), static_cast<int>(reqLen));
  if (got < reqLen) return s;

  const uint8_t touches = wire_->read() & 0x0F;
  const uint8_t xh = wire_->read();
  const uint8_t xl = wire_->read();
  const uint8_t yh = wire_->read();
  const uint8_t yl = wire_->read();

  s.touched = touches > 0;
  s.x = static_cast<uint16_t>(((xh & 0x0F) << 8) | xl);
  s.y = static_cast<uint16_t>(((yh & 0x0F) << 8) | yl);
  return s;
}

}  // namespace touch
