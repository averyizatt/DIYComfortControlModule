#pragma once

namespace ccm::hal {

class TachHal {
 public:
  virtual ~TachHal() = default;
  virtual bool begin(uint8_t pin, uint8_t channel) = 0;
  virtual void setFrequencyHz(uint32_t hz, uint8_t duty) = 0;
};

}  // namespace ccm::hal
