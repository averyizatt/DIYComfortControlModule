#include "sensors/pressure_sensor.h"

#include <cmath>

#include "sensors/PressureMath.hpp"

namespace sensors {

namespace {
constexpr float kAdcVrefTolerance = 0.01f;
constexpr float kMinFilterAlpha = 0.01f;
constexpr float kMaxFilterAlpha = 1.0f;
}  // namespace

void PressureSensor::configure(const PressureSensorConfig& config) {
  config_ = config;
  initialized_ = false;
  valid_ = false;
  fault_ = config_.enabled ? PressureFault::StaleReading : PressureFault::Disabled;
  filtered_psi_ = NAN;
  adc_node_voltage_v_ = 0.0f;
  sensor_voltage_v_ = 0.0f;
  last_update_ms_ = 0;
  last_sample_attempt_ms_ = 0;
}

bool PressureSensor::begin() {
  if (!config_.enabled) {
    fault_ = PressureFault::Disabled;
    initialized_ = true;
    return true;
  }

  if (config_.adc_pin == 255 || config_.adc_max_count == 0 || config_.adc_vref <= 0.1f ||
      config_.divider_top_ohms <= 0.1f || config_.divider_bottom_ohms <= 0.1f ||
      config_.sensor_max_v <= config_.sensor_min_v || config_.pressure_max_psi <= config_.pressure_min_psi) {
    fault_ = PressureFault::InvalidConfig;
    valid_ = false;
    initialized_ = true;
    return false;
  }

  pinMode(config_.adc_pin, INPUT);
  analogSetPinAttenuation(config_.adc_pin, ADC_11db);
  analogReadResolution(12);
  initialized_ = true;
  return true;
}

float PressureSensor::computeAverageAdcNodeVoltage() const {
  const uint8_t samples = config_.oversample_count == 0 ? 1 : config_.oversample_count;
  uint32_t acc = 0;
  for (uint8_t i = 0; i < samples; ++i) {
    acc += analogReadMilliVolts(config_.adc_pin);
  }
  return (static_cast<float>(acc) / static_cast<float>(samples)) / 1000.0f;
}

bool PressureSensor::stale(uint32_t now_ms) const {
  if (!config_.enabled) return false;
  if (last_update_ms_ == 0) return true;
  return (now_ms - last_update_ms_) > config_.stale_timeout_ms;
}

void PressureSensor::update(uint32_t now_ms) {
  if (!initialized_) begin();
  last_sample_attempt_ms_ = now_ms;

  if (!config_.enabled) {
    valid_ = false;
    fault_ = PressureFault::Disabled;
    return;
  }

  if (fault_ == PressureFault::InvalidConfig) return;

  if (last_update_ms_ != 0 && (now_ms - last_update_ms_) < config_.update_period_ms) {
    if (stale(now_ms)) {
      valid_ = false;
      fault_ = PressureFault::StaleReading;
    }
    return;
  }

  adc_node_voltage_v_ = computeAverageAdcNodeVoltage();
  pressure_math::Config mathConfig{};
  mathConfig.enabled = config_.enabled;
  mathConfig.adc_vref = config_.adc_vref;
  mathConfig.filter_alpha = constrain(config_.filter_alpha, kMinFilterAlpha, kMaxFilterAlpha);
  mathConfig.divider_top_ohms = config_.divider_top_ohms;
  mathConfig.divider_bottom_ohms = config_.divider_bottom_ohms;
  mathConfig.sensor_min_v = config_.sensor_min_v;
  mathConfig.sensor_max_v = config_.sensor_max_v;
  mathConfig.pressure_min_psi = config_.pressure_min_psi;
  mathConfig.pressure_max_psi = config_.pressure_max_psi;
  mathConfig.calibration_scale = config_.calibration_scale;
  mathConfig.calibration_offset_psi = config_.calibration_offset_psi;
  mathConfig.open_circuit_threshold_v = config_.open_circuit_threshold_v;
  mathConfig.short_threshold_v = config_.short_threshold_v;
  mathConfig.min_valid_psi = config_.min_valid_psi;
  mathConfig.max_valid_psi = config_.max_valid_psi;
  const pressure_math::Result result = pressure_math::evaluate(adc_node_voltage_v_, mathConfig, filtered_psi_);

  sensor_voltage_v_ = result.sensor_voltage_v;
  filtered_psi_ = result.filtered_psi;
  valid_ = result.valid;
  switch (result.fault) {
    case pressure_math::Fault::None: fault_ = PressureFault::None; break;
    case pressure_math::Fault::Disabled: fault_ = PressureFault::Disabled; break;
    case pressure_math::Fault::OpenCircuit: fault_ = PressureFault::OpenCircuit; break;
    case pressure_math::Fault::ShortToGround: fault_ = PressureFault::ShortToGround; break;
    case pressure_math::Fault::OutOfRange: fault_ = PressureFault::OutOfRange; break;
    case pressure_math::Fault::InvalidConfig: fault_ = PressureFault::InvalidConfig; break;
    case pressure_math::Fault::StaleReading: fault_ = PressureFault::StaleReading; break;
    case pressure_math::Fault::ShortToVcc: fault_ = PressureFault::ShortToVcc; break;
    case pressure_math::Fault::AdcError:
    default:
      fault_ = PressureFault::AdcError;
      break;
  }
  if (!valid_) return;
  last_update_ms_ = now_ms;
}

}  // namespace sensors
