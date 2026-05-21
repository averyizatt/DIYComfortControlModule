#include "sensors/thermistor_sensor.h"

#include <cmath>

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

  if (!isfinite(raw_voltage_v_) || raw_voltage_v_ < 0.0f || raw_voltage_v_ > config_.adc_vref + kAdcVrefTolerance) {
    valid_ = false;
    fault_ = ThermistorFault::AdcError;
    return;
  }

  if (raw_voltage_v_ >= config_.open_circuit_threshold_v) {
    valid_ = false;
    fault_ = ThermistorFault::OpenCircuit;
    return;
  }

  if (raw_voltage_v_ <= config_.short_threshold_v) {
    valid_ = false;
    fault_ = ThermistorFault::ShortToGround;
    return;
  }

  const float denom = config_.adc_vref - raw_voltage_v_;
  if (denom <= 0.0001f) {
    valid_ = false;
    fault_ = ThermistorFault::OpenCircuit;
    return;
  }

  resistance_ohms_ = (config_.pullup_ohms * raw_voltage_v_) / denom;
  if (!isfinite(resistance_ohms_) || resistance_ohms_ < kMinResistanceOhms) {
    valid_ = false;
    fault_ = ThermistorFault::AdcError;
    return;
  }

  const float temp_c = config_.use_steinhart_hart ? steinhartHartTempC(resistance_ohms_) : lookupTempC(resistance_ohms_);
  if (!isfinite(temp_c) || temp_c < config_.min_valid_temp_c || temp_c > config_.max_valid_temp_c) {
    valid_ = false;
    fault_ = ThermistorFault::OutOfRange;
    return;
  }

  if (!isfinite(filtered_temp_c_)) {
    filtered_temp_c_ = temp_c;
  } else {
    const float alpha = constrain(config_.filter_alpha, kMinFilterAlpha, kMaxFilterAlpha);
    filtered_temp_c_ += (temp_c - filtered_temp_c_) * alpha;
  }

  valid_ = true;
  fault_ = ThermistorFault::None;
  last_update_ms_ = now_ms;
}

}  // namespace sensors
