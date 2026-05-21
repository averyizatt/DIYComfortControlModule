#pragma once

#include <cstdint>
#include <unordered_map>

#include "hal/TestInterfaces.hpp"

struct MockGpioWriter : public ccm::hal::IGpioWriter {
  std::unordered_map<uint8_t, bool> states;

  void write(uint8_t pin, bool high) override {
    states[pin] = high;
  }
};
