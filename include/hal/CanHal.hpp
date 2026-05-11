#pragma once

#include "can/CanProtocol.hpp"

namespace ccm::hal {

class CanHal {
 public:
  virtual ~CanHal() = default;
  virtual bool begin(uint32_t bitrate) = 0;
  virtual bool send(const can::CanFrame& frame) = 0;
  virtual bool receive(can::CanFrame& frame) = 0;
};

}  // namespace ccm::hal
