#pragma once

#include <Arduino.h>
#include <TinyGPS++.h>

#include "hal/ButtonHal.hpp"
#include "hal/CanHal.hpp"
#include "hal/DisplayHal.hpp"
#include "hal/GpsHal.hpp"
#include "hal/SensorHal.hpp"
#include "hal/TachHal.hpp"
#include "hal/TouchHal.hpp"

namespace ccm::hal {

class NullDisplay : public DisplayHal {
 public:
  bool begin() override { return true; }
  void setBrightness(uint8_t) override {}
  void renderDashboard(const core::DashboardData&) override {}
  void renderPopup(const char*) override {}
};

class NullTouch : public TouchHal {
 public:
  bool begin() override { return true; }
  TouchPoint readPoint() override { return {}; }
};

class TwaiCanAdapter : public CanHal {
 public:
  bool begin(uint32_t bitrate) override;
  bool send(const can::CanFrame& frame) override;
  bool receive(can::CanFrame& frame) override;

 private:
  uint32_t bitrate_ = 0;
};

class UartGpsAdapter : public GpsHal {
 public:
  explicit UartGpsAdapter(HardwareSerial& serial) : serial_(serial) {}
  bool begin(uint32_t baud) override;
  void poll() override;
  core::GpsData latest() const override { return latest_; }

 private:
  HardwareSerial& serial_;
  TinyGPSPlus parser_;
  core::GpsData latest_{};
};

class GpioButtonAdapter : public ButtonHal {
 public:
  GpioButtonAdapter(uint8_t upPin, uint8_t downPin, uint8_t selectPin)
      : upPin_(upPin), downPin_(downPin), selectPin_(selectPin) {}
  bool begin() override;
  bool poll(ButtonEvent& event) override;

 private:
  bool debounceRead(uint8_t pin, bool& lastState, uint32_t& lastMs);

  uint8_t upPin_;
  uint8_t downPin_;
  uint8_t selectPin_;
  bool upState_ = HIGH;
  bool downState_ = HIGH;
  bool selectState_ = HIGH;
  uint32_t upDebounceMs_ = 0;
  uint32_t downDebounceMs_ = 0;
  uint32_t selectDebounceMs_ = 0;
  static constexpr uint32_t kDebounceMs = 25;
};

class LedcTachAdapter : public TachHal {
 public:
  bool begin(uint8_t pin, uint8_t channel) override;
  void setFrequencyHz(uint32_t hz, uint8_t duty) override;

 private:
  uint8_t pin_ = 0;
  uint8_t channel_ = 0;
  bool initialized_ = false;
};

class StubSensorAdapter : public SensorHal {
 public:
  bool begin() override { return true; }
  core::EnvironmentData readEnvironment() override;
  core::PowerData readPower() override;
};

}  // namespace ccm::hal
