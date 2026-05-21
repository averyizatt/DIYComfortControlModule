#include "sensors/pressure_sensor.h"

#include <cmath>

namespace sensors {

namespace {
constexpr float kAdcVrefTolerance = 0.01f;
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
#if defined(ADC_11db)
  analogSetPinAttenuation(config_.adc_pin, ADC_11db);
#endif
  analogReadResolution(12);
  initialized_ = true;
  return true;
}

float PressureSensor::computeAverageAdcNodeVoltage() const {
  const uint8_t samples = config_.oversample_count == 0 ? 1 : config_.oversample_count;
  uint32_t acc = 0;
  for (uint8_t i = 0; i < samples; ++i) {
    acc += static_cast<uint32_t>(analogRead(config_.adc_pin));
  }
  const float avg = static_cast<float>(acc) / static_cast<float>(samples);
  return (avg / static_cast<float>(config_.adc_max_count)) * config_.adc_vref;
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
  if (!isfinite(adc_node_voltage_v_) || adc_node_voltage_v_ < 0.0f || adc_node_voltage_v_ > config_.adc_vref + kAdcVrefTolerance) {
    valid_ = false;
    fault_ = PressureFault::AdcError;
    return;
  }

  const float divider_scale = (config_.divider_top_ohms + config_.divider_bottom_ohms) / config_.divider_bottom_ohms;
  sensor_voltage_v_ = adc_node_voltage_v_ * divider_scale;

  if (!isfinite(sensor_voltage_v_) || sensor_voltage_v_ < -0.01f || sensor_voltage_v_ > 5.5f) {
    valid_ = false;
    fault_ = PressureFault::AdcError;
    return;
  }

  if (sensor_voltage_v_ >= config_.open_circuit_threshold_v) {
    valid_ = false;
    fault_ = PressureFault::OpenCircuit;
    return;
  }

  if (sensor_voltage_v_ <= config_.short_threshold_v) {
    valid_ = false;
    fault_ = PressureFault::ShortToGround;
    return;
  }

  const float normalized = (sensor_voltage_v_ - config_.sensor_min_v) / (config_.sensor_max_v - config_.sensor_min_v);
  float psi = config_.pressure_min_psi + normalized * (config_.pressure_max_psi - config_.pressure_min_psi);
  psi = (psi * config_.calibration_scale) + config_.calibration_offset_psi;

  if (!isfinite(psi) || psi < config_.min_valid_psi || psi > config_.max_valid_psi) {
    valid_ = false;
    fault_ = PressureFault::OutOfRange;
    return;
  }

  if (!isfinite(filtered_psi_)) {
    filtered_psi_ = psi;
  } else {
    const float alpha = constrain(config_.filter_alpha, 0.01f, 1.0f);
    filtered_psi_ += (psi - filtered_psi_) * alpha;
  }

  valid_ = true;
  fault_ = PressureFault::None;
  last_update_ms_ = now_ms;
}

}  // namespace sensors
