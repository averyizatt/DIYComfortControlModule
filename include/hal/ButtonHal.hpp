#pragma once

#include <Arduino.h>

namespace ccm::hal {

enum class ButtonEventType : uint8_t {
  None,
  Up,
  Down,
  Select,
};

struct ButtonEvent {
  ButtonEventType type = ButtonEventType::None;
  uint32_t timestampMs = 0;
};

class ButtonHal {
 public:
  virtual ~ButtonHal() = default;
  virtual bool begin() = 0;
  virtual bool poll(ButtonEvent& event) = 0;
};

}  // namespace ccm::hal
