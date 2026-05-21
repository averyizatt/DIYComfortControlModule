#pragma once

#include <Arduino.h>

#include <cstddef>

namespace sensors {

struct ThermistorLutPoint {
  float resistance_ohms = 0.0f;
  float temperature_c = 0.0f;
};

enum class ThermistorFault : uint8_t {
  None = 0,
  Disabled = 1,
  OpenCircuit = 2,
  ShortToGround = 3,
  OutOfRange = 4,
  InvalidConfig = 5,
  StaleReading = 6,
  AdcError = 7,
};

struct ThermistorConfig {
  bool enabled = false;
  uint8_t adc_pin = 255;
  float pullup_ohms = 10000.0f;
  float adc_vref = 3.3f;
  uint16_t adc_max_count = 4095;
  uint8_t oversample_count = 8;
  float filter_alpha = 0.20f;
  uint16_t update_period_ms = 50;
  uint16_t stale_timeout_ms = 1000;
  float min_valid_temp_c = -50.0f;
  float max_valid_temp_c = 180.0f;
  float open_circuit_threshold_v = 3.15f;
  float short_threshold_v = 0.08f;
  bool use_steinhart_hart = true;
  float steinhart_a = 1.129148e-3f;
  float steinhart_b = 2.34125e-4f;
  float steinhart_c = 8.76741e-8f;
  const ThermistorLutPoint* lut = nullptr;
  size_t lut_size = 0;
};

class ThermistorSensor {
 public:
  ThermistorSensor() = default;
  explicit ThermistorSensor(const ThermistorConfig& config) : config_(config) {}

  void configure(const ThermistorConfig& config);
  const ThermistorConfig& config() const { return config_; }
  bool begin();
  void update(uint32_t now_ms);

  float valueC() const { return filtered_temp_c_; }
  float rawVoltage() const { return raw_voltage_v_; }
  float resistanceOhms() const { return resistance_ohms_; }
  bool valid() const { return valid_; }
  ThermistorFault fault() const { return fault_; }
  bool stale(uint32_t now_ms) const;
  uint32_t lastUpdateMs() const { return last_update_ms_; }

 private:
  float computeAverageVoltage() const;
  float steinhartHartTempC(float resistance_ohms) const;
  float lookupTempC(float resistance_ohms) const;

  ThermistorConfig config_{};
  float filtered_temp_c_ = NAN;
  float raw_voltage_v_ = 0.0f;
  float resistance_ohms_ = NAN;
  bool initialized_ = false;
  bool valid_ = false;
  ThermistorFault fault_ = ThermistorFault::Disabled;
  uint32_t last_update_ms_ = 0;
  uint32_t last_sample_attempt_ms_ = 0;
};

}  // namespace sensors
