#pragma once

#include <cstdint>
#include <string>

#include "can/can_protocol.h"

namespace ccm::hal {

class IAdcReader {
 public:
  virtual ~IAdcReader() = default;
  virtual int readRaw(uint8_t pin) = 0;
};

class IGpioWriter {
 public:
  virtual ~IGpioWriter() = default;
  virtual void write(uint8_t pin, bool high) = 0;
};

class IPwmOutput {
 public:
  virtual ~IPwmOutput() = default;
  virtual void setEnabled(uint8_t channel, bool enabled) = 0;
  virtual void setDutyPercent(uint8_t channel, uint8_t duty_percent) = 0;
};

class ICanBus {
 public:
  virtual ~ICanBus() = default;
  virtual bool send(const can_protocol::CanFrame& frame) = 0;
  virtual bool receive(can_protocol::CanFrame& frame) = 0;
};

class ITimeSource {
 public:
  virtual ~ITimeSource() = default;
  virtual uint32_t millis() const = 0;
};

class ISettingsStore {
 public:
  virtual ~ISettingsStore() = default;
  virtual bool put(const std::string& key, const std::string& value) = 0;
  virtual std::string get(const std::string& key, const std::string& fallback = {}) const = 0;
};

class ILogWriter {
 public:
  virtual ~ILogWriter() = default;
  virtual bool appendLine(const std::string& path, const std::string& line) = 0;
  virtual bool flush(const std::string& path) = 0;
};

}  // namespace ccm::hal
