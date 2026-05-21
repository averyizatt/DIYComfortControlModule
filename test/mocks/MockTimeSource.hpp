#pragma once

#include "hal/TestInterfaces.hpp"

struct MockTimeSource : public ccm::hal::ITimeSource {
  uint32_t now_ms = 0;
  uint32_t millis() const override { return now_ms; }
};
