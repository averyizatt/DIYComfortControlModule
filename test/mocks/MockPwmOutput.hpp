#pragma once

#include <cstdint>
#include <unordered_map>

#include "hal/TestInterfaces.hpp"

struct MockPwmOutput : public ccm::hal::IPwmOutput {
  std::unordered_map<uint8_t, bool> enabled;
  std::unordered_map<uint8_t, uint8_t> duty;

  void setEnabled(uint8_t channel, bool value) override {
    enabled[channel] = value;
  }

  void setDutyPercent(uint8_t channel, uint8_t dutyPercent) override {
    duty[channel] = dutyPercent;
  }
};
