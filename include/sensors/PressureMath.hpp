#pragma once

#include <cmath>
#include <cstdint>

namespace sensors::pressure_math {

constexpr float kShortToVccMarginVolts = 0.15f;

enum class Fault : uint8_t {
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

struct Config {
  bool enabled = false;
  float adc_vref = 3.3f;
  float filter_alpha = 0.20f;
  float divider_top_ohms = 10000.0f;
  float divider_bottom_ohms = 20000.0f;
  float sensor_min_v = 0.5f;
  float sensor_max_v = 4.5f;
  float pressure_min_psi = 0.0f;
  float pressure_max_psi = 100.0f;
  float calibration_scale = 1.0f;
  float calibration_offset_psi = 0.0f;
  float open_circuit_threshold_v = 4.9f;
  float short_threshold_v = 0.1f;
  float min_valid_psi = -5.0f;
  float max_valid_psi = 300.0f;
};

struct Result {
  float adc_node_voltage_v = 0.0f;
  float sensor_voltage_v = 0.0f;
  float raw_psi = 0.0f;
  float filtered_psi = 0.0f;
  bool valid = false;
  Fault fault = Fault::Disabled;
};

inline float clampAlpha(float alpha) {
  if (alpha < 0.01f) return 0.01f;
  if (alpha > 1.0f) return 1.0f;
  return alpha;
}

inline bool validConfig(const Config& config) {
  return config.adc_vref > 0.1f && config.divider_top_ohms > 0.1f && config.divider_bottom_ohms > 0.1f &&
         config.sensor_max_v > config.sensor_min_v && config.pressure_max_psi > config.pressure_min_psi;
}

inline float sensorVoltageFromNode(float adcNodeVoltage, const Config& config) {
  const float dividerScale = (config.divider_top_ohms + config.divider_bottom_ohms) / config.divider_bottom_ohms;
  return adcNodeVoltage * dividerScale;
}

inline Result evaluate(float adcNodeVoltage, const Config& config, float previousFilteredPsi) {
  Result result{};
  result.adc_node_voltage_v = adcNodeVoltage;
  if (!config.enabled) {
    result.fault = Fault::Disabled;
    return result;
  }
  if (!validConfig(config)) {
    result.fault = Fault::InvalidConfig;
    return result;
  }
  if (!std::isfinite(adcNodeVoltage) || adcNodeVoltage < 0.0f || adcNodeVoltage > config.adc_vref + 0.01f) {
    result.fault = Fault::AdcError;
    return result;
  }

  result.sensor_voltage_v = sensorVoltageFromNode(adcNodeVoltage, config);
  if (!std::isfinite(result.sensor_voltage_v) || result.sensor_voltage_v < -0.01f || result.sensor_voltage_v > 5.5f) {
    result.fault = Fault::AdcError;
    return result;
  }
  if (result.sensor_voltage_v > config.open_circuit_threshold_v + kShortToVccMarginVolts) {
    result.fault = Fault::ShortToVcc;
    return result;
  }
  if (result.sensor_voltage_v >= config.open_circuit_threshold_v) {
    result.fault = Fault::OpenCircuit;
    return result;
  }
  if (result.sensor_voltage_v <= config.short_threshold_v) {
    result.fault = Fault::ShortToGround;
    return result;
  }

  const float normalized = (result.sensor_voltage_v - config.sensor_min_v) / (config.sensor_max_v - config.sensor_min_v);
  result.raw_psi = config.pressure_min_psi + normalized * (config.pressure_max_psi - config.pressure_min_psi);
  result.raw_psi = (result.raw_psi * config.calibration_scale) + config.calibration_offset_psi;
  if (!std::isfinite(result.raw_psi) || result.raw_psi < config.min_valid_psi || result.raw_psi > config.max_valid_psi) {
    result.fault = Fault::OutOfRange;
    return result;
  }

  if (!std::isfinite(previousFilteredPsi)) {
    result.filtered_psi = result.raw_psi;
  } else {
    result.filtered_psi = previousFilteredPsi + (result.raw_psi - previousFilteredPsi) * clampAlpha(config.filter_alpha);
  }
  result.valid = true;
  result.fault = Fault::None;
  return result;
}

}  // namespace sensors::pressure_math
