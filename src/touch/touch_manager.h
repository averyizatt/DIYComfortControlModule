#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "state/vehicle_state.h"

namespace touch {

struct TouchSample {
  bool touched = false;
  uint16_t x = 0;
  uint16_t y = 0;
};

class TouchManager {
 public:
  bool begin(TwoWire& wire, uint8_t sdaPin, uint8_t sclPin, uint8_t rstPin, uint8_t intPin, uint8_t address = 0x38);
  TouchSample read();
  bool online() const { return online_; }

 private:
  bool probe();

  TwoWire* wire_ = nullptr;
  uint8_t address_ = 0x38;
  uint8_t rstPin_ = 255;
  uint8_t intPin_ = 255;
  bool online_ = false;
};

}  // namespace touch
