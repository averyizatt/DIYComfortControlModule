#include "sensors/thermistor_sensor.h"

#include <cmath>

#include "sensors/ThermistorMath.hpp"

namespace sensors {

namespace {
constexpr float kKelvinOffset = 273.15f;
constexpr float kMinResistanceOhms = 1.0f;
constexpr float kAdcVrefTolerance = 0.01f;
constexpr float kMinFilterAlpha = 0.01f;
constexpr float kMaxFilterAlpha = 1.0f;
}  // namespace

void ThermistorSensor::configure(const ThermistorConfig& config) {
  config_ = config;
  initialized_ = false;
  valid_ = false;
  fault_ = config_.enabled ? ThermistorFault::StaleReading : ThermistorFault::Disabled;
  filtered_temp_c_ = NAN;
  raw_voltage_v_ = 0.0f;
  resistance_ohms_ = NAN;
  last_update_ms_ = 0;
  last_sample_attempt_ms_ = 0;
}

bool ThermistorSensor::begin() {
  if (!config_.enabled) {
    fault_ = ThermistorFault::Disabled;
    initialized_ = true;
    return true;
  }

  if (config_.adc_pin == 255 || config_.adc_max_count == 0 || config_.adc_vref <= 0.1f ||
      config_.pullup_ohms <= 1.0f) {
    fault_ = ThermistorFault::InvalidConfig;
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

float ThermistorSensor::computeAverageVoltage() const {
  const uint8_t samples = config_.oversample_count == 0 ? 1 : config_.oversample_count;
  uint32_t acc = 0;
  for (uint8_t i = 0; i < samples; ++i) {
    acc += static_cast<uint32_t>(analogRead(config_.adc_pin));
  }
  const float avg = static_cast<float>(acc) / static_cast<float>(samples);
  return (avg / static_cast<float>(config_.adc_max_count)) * config_.adc_vref;
}

float ThermistorSensor::steinhartHartTempC(float resistance_ohms) const {
  if (resistance_ohms < kMinResistanceOhms) return NAN;
  const float ln_r = logf(resistance_ohms);
  const float inv_t = config_.steinhart_a + config_.steinhart_b * ln_r +
                      config_.steinhart_c * ln_r * ln_r * ln_r;
  if (inv_t <= 0.0f) return NAN;
  return (1.0f / inv_t) - kKelvinOffset;
}

float ThermistorSensor::lookupTempC(float resistance_ohms) const {
  if (!config_.lut || config_.lut_size < 2) return NAN;

  const ThermistorLutPoint* lut = config_.lut;
  for (size_t i = 0; i + 1 < config_.lut_size; ++i) {
    const ThermistorLutPoint& a = lut[i];
    const ThermistorLutPoint& b = lut[i + 1];
    const bool in_range =
        (resistance_ohms <= a.resistance_ohms && resistance_ohms >= b.resistance_ohms) ||
        (resistance_ohms >= a.resistance_ohms && resistance_ohms <= b.resistance_ohms);
    if (!in_range) continue;

    const float denom = (b.resistance_ohms - a.resistance_ohms);
    if (fabsf(denom) < 1e-6f) return a.temperature_c;
    const float t = (resistance_ohms - a.resistance_ohms) / denom;
    return a.temperature_c + t * (b.temperature_c - a.temperature_c);
  }

  return NAN;
}

bool ThermistorSensor::stale(uint32_t now_ms) const {
  if (!config_.enabled) return false;
  if (last_update_ms_ == 0) return true;
  return (now_ms - last_update_ms_) > config_.stale_timeout_ms;
}

void ThermistorSensor::update(uint32_t now_ms) {
  if (!initialized_) begin();
  last_sample_attempt_ms_ = now_ms;

  if (!config_.enabled) {
    valid_ = false;
    fault_ = ThermistorFault::Disabled;
    return;
  }

  if (fault_ == ThermistorFault::InvalidConfig) return;

  if (last_update_ms_ != 0 && (now_ms - last_update_ms_) < config_.update_period_ms) {
    if (stale(now_ms)) {
      valid_ = false;
      fault_ = ThermistorFault::StaleReading;
    }
    return;
  }

  raw_voltage_v_ = computeAverageVoltage();
  thermistor_math::Config mathConfig{};
  mathConfig.enabled = config_.enabled;
  mathConfig.pullup_ohms = config_.pullup_ohms;
  mathConfig.adc_vref = config_.adc_vref;
  mathConfig.filter_alpha = constrain(config_.filter_alpha, kMinFilterAlpha, kMaxFilterAlpha);
  mathConfig.min_valid_temp_c = config_.min_valid_temp_c;
  mathConfig.max_valid_temp_c = config_.max_valid_temp_c;
  mathConfig.open_circuit_threshold_v = config_.open_circuit_threshold_v;
  mathConfig.short_threshold_v = config_.short_threshold_v;
  mathConfig.use_steinhart_hart = config_.use_steinhart_hart;
  mathConfig.steinhart_a = config_.steinhart_a;
  mathConfig.steinhart_b = config_.steinhart_b;
  mathConfig.steinhart_c = config_.steinhart_c;
  mathConfig.lut = reinterpret_cast<const thermistor_math::LutPoint*>(config_.lut);
  mathConfig.lut_size = config_.lut_size;
  const thermistor_math::Result result = thermistor_math::evaluate(raw_voltage_v_, mathConfig, filtered_temp_c_);

  resistance_ohms_ = result.resistance_ohms;
  filtered_temp_c_ = result.filtered_temp_c;
  valid_ = result.valid;
  switch (result.fault) {
    case thermistor_math::Fault::None: fault_ = ThermistorFault::None; break;
    case thermistor_math::Fault::Disabled: fault_ = ThermistorFault::Disabled; break;
    case thermistor_math::Fault::OpenCircuit: fault_ = ThermistorFault::OpenCircuit; break;
    case thermistor_math::Fault::ShortToGround: fault_ = ThermistorFault::ShortToGround; break;
    case thermistor_math::Fault::OutOfRange: fault_ = ThermistorFault::OutOfRange; break;
    case thermistor_math::Fault::InvalidConfig: fault_ = ThermistorFault::InvalidConfig; break;
    case thermistor_math::Fault::StaleReading: fault_ = ThermistorFault::StaleReading; break;
    case thermistor_math::Fault::AdcError:
    default:
      fault_ = ThermistorFault::AdcError;
      break;
  }
  if (!valid_) return;
  last_update_ms_ = now_ms;
}

}  // namespace sensors
