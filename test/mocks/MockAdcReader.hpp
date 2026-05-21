#pragma once

#include <cstdint>
#include <deque>
#include <unordered_map>

#include "hal/TestInterfaces.hpp"

struct MockAdcReader : public ccm::hal::IAdcReader {
  std::unordered_map<uint8_t, std::deque<int>> samples;

  void push(uint8_t pin, int value) {
    samples[pin].push_back(value);
  }

  int readRaw(uint8_t pin) override {
    auto& queue = samples[pin];
    if (queue.empty()) return 0;
    const int value = queue.front();
    queue.pop_front();
    return value;
  }
};
