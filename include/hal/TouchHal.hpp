#pragma once

namespace ccm::hal {

struct TouchPoint {
  bool touched = false;
  uint16_t x = 0;
  uint16_t y = 0;
};

class TouchHal {
 public:
  virtual ~TouchHal() = default;
  virtual bool begin() = 0;
  virtual TouchPoint readPoint() = 0;
};

}  // namespace ccm::hal
