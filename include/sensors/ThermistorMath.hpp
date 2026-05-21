#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace sensors::thermistor_math {

struct LutPoint {
  float resistance_ohms = 0.0f;
  float temperature_c = 0.0f;
};

enum class Fault : uint8_t {
  None = 0,
  Disabled = 1,
  OpenCircuit = 2,
  ShortToGround = 3,
  OutOfRange = 4,
  InvalidConfig = 5,
  StaleReading = 6,
  AdcError = 7,
};

struct Config {
  bool enabled = false;
  float pullup_ohms = 10000.0f;
  float adc_vref = 3.3f;
  float filter_alpha = 0.20f;
  float min_valid_temp_c = -50.0f;
  float max_valid_temp_c = 180.0f;
  float open_circuit_threshold_v = 3.15f;
  float short_threshold_v = 0.08f;
  bool use_steinhart_hart = true;
  float steinhart_a = 1.129148e-3f;
  float steinhart_b = 2.34125e-4f;
  float steinhart_c = 8.76741e-8f;
  const LutPoint* lut = nullptr;
  size_t lut_size = 0;
};

struct Result {
  float raw_voltage_v = 0.0f;
  float resistance_ohms = 0.0f;
  float raw_temp_c = 0.0f;
  float filtered_temp_c = 0.0f;
  bool valid = false;
  Fault fault = Fault::Disabled;
};

inline float clampAlpha(float alpha) {
  if (alpha < 0.01f) return 0.01f;
  if (alpha > 1.0f) return 1.0f;
  return alpha;
}

inline bool validConfig(const Config& config) {
  return config.pullup_ohms > 1.0f && config.adc_vref > 0.1f;
}

inline float resistanceFromVoltage(float rawVoltage, const Config& config) {
  const float denom = config.adc_vref - rawVoltage;
  if (denom <= 0.0001f) return NAN;
  return (config.pullup_ohms * rawVoltage) / denom;
}

inline float steinhartHartTempC(float resistanceOhms, const Config& config) {
  if (resistanceOhms < 1.0f) return NAN;
  const float lnR = logf(resistanceOhms);
  const float invT = config.steinhart_a + config.steinhart_b * lnR + config.steinhart_c * lnR * lnR * lnR;
  if (invT <= 0.0f) return NAN;
  return (1.0f / invT) - 273.15f;
}

inline float lookupTempC(float resistanceOhms, const Config& config) {
  if (!config.lut || config.lut_size < 2) return NAN;
  for (size_t i = 0; i + 1 < config.lut_size; ++i) {
    const LutPoint& a = config.lut[i];
    const LutPoint& b = config.lut[i + 1];
    const bool inRange = (resistanceOhms <= a.resistance_ohms && resistanceOhms >= b.resistance_ohms) ||
                         (resistanceOhms >= a.resistance_ohms && resistanceOhms <= b.resistance_ohms);
    if (!inRange) continue;
    const float denom = b.resistance_ohms - a.resistance_ohms;
    if (fabsf(denom) < 1e-6f) return a.temperature_c;
    const float t = (resistanceOhms - a.resistance_ohms) / denom;
    return a.temperature_c + t * (b.temperature_c - a.temperature_c);
  }
  return NAN;
}

inline Result evaluate(float rawVoltage, const Config& config, float previousFilteredTempC) {
  Result result{};
  result.raw_voltage_v = rawVoltage;
  if (!config.enabled) {
    result.fault = Fault::Disabled;
    return result;
  }
  if (!validConfig(config)) {
    result.fault = Fault::InvalidConfig;
    return result;
  }
  if (!std::isfinite(rawVoltage) || rawVoltage < 0.0f || rawVoltage > config.adc_vref + 0.01f) {
    result.fault = Fault::AdcError;
    return result;
  }
  if (rawVoltage >= config.open_circuit_threshold_v) {
    result.fault = Fault::OpenCircuit;
    return result;
  }
  if (rawVoltage <= config.short_threshold_v) {
    result.fault = Fault::ShortToGround;
    return result;
  }

  result.resistance_ohms = resistanceFromVoltage(rawVoltage, config);
  if (!std::isfinite(result.resistance_ohms) || result.resistance_ohms < 1.0f) {
    result.fault = Fault::AdcError;
    return result;
  }

  result.raw_temp_c = config.use_steinhart_hart ? steinhartHartTempC(result.resistance_ohms, config)
                                                : lookupTempC(result.resistance_ohms, config);
  if (!std::isfinite(result.raw_temp_c) ||
      result.raw_temp_c < config.min_valid_temp_c ||
      result.raw_temp_c > config.max_valid_temp_c) {
    result.fault = Fault::OutOfRange;
    return result;
  }

  if (!std::isfinite(previousFilteredTempC)) {
    result.filtered_temp_c = result.raw_temp_c;
  } else {
    result.filtered_temp_c = previousFilteredTempC + (result.raw_temp_c - previousFilteredTempC) * clampAlpha(config.filter_alpha);
  }
  result.valid = true;
  result.fault = Fault::None;
  return result;
}

}  // namespace sensors::thermistor_math
