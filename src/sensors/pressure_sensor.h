#pragma once

#include <Arduino.h>

namespace sensors {

enum class PressureFault : uint8_t {
  None = 0,
  Disabled = 1,
  OpenCircuit = 2,
  ShortToGround = 3,
  OutOfRange = 4,
  InvalidConfig = 5,
  StaleReading = 6,
  AdcError = 7,
  ShortToVcc = 8,
};

struct PressureSensorConfig {
  bool enabled = false;
  uint8_t adc_pin = 255;
  float adc_vref = 3.3f;
  uint16_t adc_max_count = 4095;
  uint8_t oversample_count = 8;
  float filter_alpha = 0.20f;
  uint16_t update_period_ms = 50;
  uint16_t stale_timeout_ms = 1000;

  // Divider: sensor_out -> R_top -> ADC -> R_bottom -> GND
  float divider_top_ohms = 10000.0f;
  float divider_bottom_ohms = 20000.0f;

  // Typical 0.5V to 4.5V transducer transfer function
  float sensor_min_v = 0.5f;
  float sensor_max_v = 4.5f;
  float pressure_min_psi = 0.0f;
  float pressure_max_psi = 100.0f;

  // Optional calibration
  float calibration_scale = 1.0f;
  float calibration_offset_psi = 0.0f;

  // Diagnostics thresholds
  float open_circuit_threshold_v = 4.9f;
  float short_threshold_v = 0.1f;
  float min_valid_psi = -5.0f;
  float max_valid_psi = 300.0f;
};

class PressureSensor {
 public:
  PressureSensor() = default;
  explicit PressureSensor(const PressureSensorConfig& config) : config_(config) {}

  void configure(const PressureSensorConfig& config);
  const PressureSensorConfig& config() const { return config_; }
  bool begin();
  void update(uint32_t now_ms);

  float valuePsi() const { return filtered_psi_; }
  float adcNodeVoltage() const { return adc_node_voltage_v_; }
  float sensorVoltage() const { return sensor_voltage_v_; }
  bool valid() const { return valid_; }
  PressureFault fault() const { return fault_; }
  bool stale(uint32_t now_ms) const;
  uint32_t lastUpdateMs() const { return last_update_ms_; }

 private:
  float computeAverageAdcNodeVoltage() const;

  PressureSensorConfig config_{};
  float filtered_psi_ = NAN;
  float adc_node_voltage_v_ = 0.0f;
  float sensor_voltage_v_ = 0.0f;
  bool initialized_ = false;
  bool valid_ = false;
  PressureFault fault_ = PressureFault::Disabled;
  uint32_t last_update_ms_ = 0;
  uint32_t last_sample_attempt_ms_ = 0;
};

}  // namespace sensors
