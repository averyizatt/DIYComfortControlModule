#include "knock/knock_monitor.h"

#include <cmath>
#include <cstdio>
#include <cstring>

#include "can/can_manager.h"
#include "can/can_protocol.h"
#include "storage/log_manager.h"
#include "storage/sd_manager.h"

namespace knock {

namespace {
constexpr uint16_t kAdcMidpoint = 2048;
constexpr uint16_t kAdcLowClip = 5;
constexpr uint16_t kAdcHighClip = 4090;
constexpr uint8_t kSamplesPerTick = 10;
constexpr float kEnergyAlpha = 0.18f;
constexpr float kActivityAlpha = 0.08f;
constexpr float kBaselineAlphaSlow = 0.01f;
constexpr float kBaselineAlphaFast = 0.06f;
constexpr float kMinimumEnergyFloor = 2.0f;
constexpr float kLowActivityThreshold = 1.6f;
constexpr uint32_t kLowActivityFaultDelayMs = 2500;
constexpr uint32_t kClipWindowMs = 1000;
constexpr uint16_t kClipWindowThreshold = 20;
constexpr uint32_t kBaselineLearnMs = 3000;
constexpr uint32_t kKnockTaskIntervalMs = 5;
constexpr uint32_t kDemoSpikeMinGapMs = 1200;
constexpr uint32_t kDemoSpikeJitterMs = 2200;
constexpr uint32_t kBaselineSaveIntervalMs = 60000;
constexpr float kBaselineSaveDelta = 1.0f;
constexpr char kKnockProfileDir[] = "/config";
constexpr char kKnockProfilePath[] = "/config/knock_profile.txt";

constexpr uint8_t kKnockSeverityInfo = static_cast<uint8_t>(can_protocol::FaultSeverity::INFO);
constexpr uint8_t kKnockSeverityWarn = static_cast<uint8_t>(can_protocol::FaultSeverity::WARNING);
constexpr uint8_t kKnockSeverityCritical = static_cast<uint8_t>(can_protocol::FaultSeverity::CRITICAL);

bool isDemoBuildEnabled() {
#if defined(DEMO_MODE) && (DEMO_MODE == 1)
  return true;
#else
  return false;
#endif
}

uint8_t clampU8FromFloat(float v) {
  if (v <= 0.0f) return 0;
  if (v >= 255.0f) return 255;
  return static_cast<uint8_t>(v + 0.5f);
}

uint8_t encodeRpmDiv100(uint16_t rpm) {
  return static_cast<uint8_t>((rpm / 100U) > 255U ? 255U : (rpm / 100U));
}

bool parseFloatField(const char* text, const char* key, float& out) {
  const char* pos = strstr(text, key);
  if (!pos) return false;
  pos += strlen(key);
  return sscanf(pos, "%f", &out) == 1;
}

bool parseU32Field(const char* text, const char* key, uint32_t& out) {
  const char* pos = strstr(text, key);
  if (!pos) return false;
  pos += strlen(key);
  unsigned long value = 0;
  if (sscanf(pos, "%lu", &value) != 1) return false;
  out = static_cast<uint32_t>(value);
  return true;
}

}  // namespace

bool KnockMonitor::begin(state::VehicleStateStore* stateStore, settings::SettingsManager* settingsMgr,
                         storage::LogManager* logMgr, storage::SdManager* sdMgr, canbus::CanManager* canMgr) {
  stateStore_ = stateStore;
  settingsMgr_ = settingsMgr;
  logMgr_ = logMgr;
  sdMgr_ = sdMgr;
  canMgr_ = canMgr;
  if (!stateStore_ || !settingsMgr_ || !logMgr_ || !sdMgr_) return false;

  const state::VehicleState snapshot = stateStore_->read();
  loadConfigFromState(snapshot);
  configureAdc();

  if (sdMgr_->mounted()) {
    sdMgr_->ensureFolder("/logs/knock");
    sdMgr_->ensureFolder(kKnockProfileDir);
    loadBaselineProfile();
  }

  return true;
}

void KnockMonitor::configureAdc() {
  pinMode(cfg_.adc_pin, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(cfg_.adc_pin, ADC_11db);
  adcConfigured_ = true;
}

void KnockMonitor::loadConfigFromState(const state::VehicleState& s) {
  const bool profileSettingChanged =
      fabsf(cfg_.gain - s.knock_gain) > 0.001f ||
      fabsf(cfg_.threshold_multiplier - s.knock_threshold_multiplier) > 0.001f ||
      fabsf(cfg_.threshold_offset - s.knock_threshold_offset) > 0.001f;
  cfg_.enabled = s.knock_enabled;
  cfg_.adc_pin = s.knock_adc_pin;
  cfg_.gain = s.knock_gain;
  cfg_.boost_enable_kpa = s.knock_boost_enable_kpa;
  cfg_.rpm_enable_min = s.knock_rpm_enable_min;
  cfg_.threshold_multiplier = s.knock_threshold_multiplier;
  cfg_.threshold_offset = s.knock_threshold_offset;
  cfg_.event_cooldown_ms = s.knock_event_cooldown_ms;
  cfg_.warning_threshold_count = s.knock_warning_threshold_count;
  cfg_.critical_threshold_count = s.knock_critical_threshold_count;
  cfg_.baseline_learning_enabled = s.knock_baseline_learning_enabled;
  cfg_.demo_mode_enabled = s.knock_demo_mode_enabled;
  cfg_.response_mode = static_cast<ResponseMode>(s.knock_response_mode);
  if (profileSettingChanged && baselineLearned_) {
    baselineProfileDirty_ = true;
  }
}

void KnockMonitor::tick(uint32_t nowMs) {
  if (!stateStore_) return;

  state::VehicleState snapshot{};
  stateStore_->mutate([&](state::VehicleState& s) {
    applyStateCommands(s);
    loadConfigFromState(s);
    snapshot = s;
  });

  if (!adcConfigured_ || cfg_.adc_pin != snapshot.knock_adc_pin) {
    cfg_.adc_pin = snapshot.knock_adc_pin;
    configureAdc();
  }

  const float sampledEnergy = sampleEnergy(snapshot, nowMs) * cfg_.gain;
  knockEnergy_ += kEnergyAlpha * (sampledEnergy - knockEnergy_);
  if (knockEnergy_ < 0.0f) knockEnergy_ = 0.0f;

  const bool detectActive = cfg_.enabled && signalValid_ && !sensorFault_ &&
                            (snapshot.boost_kpa >= cfg_.boost_enable_kpa) &&
                            (snapshot.rpm >= cfg_.rpm_enable_min);

  maybeUpdateBaseline(detectActive);

  threshold_ = (baseline_ * cfg_.threshold_multiplier) + cfg_.threshold_offset;
  if (threshold_ < baseline_ + 1.0f) threshold_ = baseline_ + 1.0f;

  handleDetection(snapshot, nowMs, detectActive);
  maybeSaveBaselineProfile(nowMs);
  updateSharedState(nowMs);
}

void KnockMonitor::applyStateCommands(state::VehicleState& s) {
  if (s.knock_reset_baseline_request) {
    baseline_ = knockEnergy_;
    savedBaseline_ = 0.0f;
    baselineSampleCount_ = 0;
    savedBaselineSampleCount_ = 0;
    baselineLearned_ = false;
    baselineProfileLoaded_ = false;
    baselineProfileDirty_ = true;
    s.knock_reset_baseline_request = false;
  }

  if (s.knock_clear_event_count_request) {
    eventCountRolling_ = 0;
    eventWindowCount_ = 0;
    warningActive_ = false;
    criticalActive_ = false;
    s.knock_clear_event_count_request = false;
  }

  if (s.knock_simulate_event_request) {
    forceDemoSpike_ = true;
    s.knock_simulate_event_request = false;
  }
}

float KnockMonitor::sampleEnergy(const state::VehicleState& s, uint32_t nowMs) {
  float accum = 0.0f;

  if (isDemoBuildEnabled() || cfg_.demo_mode_enabled) {
    const float t = nowMs * 0.001f;
    float demo = 8.0f + 4.0f * (0.5f + 0.5f * sinf(t * 1.8f));
    const bool boostAboveThreshold = s.boost_kpa >= cfg_.boost_enable_kpa;
    const bool jitterElapsed = (nowMs - lastDemoSpikeMs_) >= nextDemoSpikeGapMs_;
    if (forceDemoSpike_ || (boostAboveThreshold && jitterElapsed)) {
      demo += 80.0f;
      lastDemoSpikeMs_ = nowMs;
      nextDemoSpikeGapMs_ = kDemoSpikeMinGapMs + (nowMs % kDemoSpikeJitterMs);
      forceDemoSpike_ = false;
    }
    activityEma_ += kActivityAlpha * (demo - activityEma_);
    signalValid_ = true;
    sensorFault_ = false;
    return demo;
  }

  for (uint8_t i = 0; i < kSamplesPerTick; ++i) {
    const uint16_t raw = static_cast<uint16_t>(analogRead(cfg_.adc_pin));
    const int16_t centered = static_cast<int16_t>(raw) - static_cast<int16_t>(kAdcMidpoint);
    const float absCentered = fabsf(static_cast<float>(centered));
    accum += absCentered;
    updateSignalHealth(raw, absCentered, nowMs);
  }

  return accum / static_cast<float>(kSamplesPerTick);
}

void KnockMonitor::updateSignalHealth(uint16_t sampleRaw, float absCentered, uint32_t nowMs) {
  activityEma_ += kActivityAlpha * (absCentered - activityEma_);

  if (clipWindowStartMs_ == 0) clipWindowStartMs_ = nowMs;
  if (sampleRaw >= kAdcHighClip) {
    ++clipHighWindowCount_;
    ++clipHighTotal_;
  }
  if (sampleRaw <= kAdcLowClip) {
    ++clipLowWindowCount_;
    ++clipLowTotal_;
  }

  if ((nowMs - clipWindowStartMs_) >= kClipWindowMs) {
    clippingDetected_ = (clipHighWindowCount_ >= kClipWindowThreshold) ||
                        (clipLowWindowCount_ >= kClipWindowThreshold);
    clipHighWindowCount_ = 0;
    clipLowWindowCount_ = 0;
    clipWindowStartMs_ = nowMs;
  }

  if (activityEma_ < kLowActivityThreshold) {
    if (lowActivitySinceMs_ == 0) lowActivitySinceMs_ = nowMs;
    if ((nowMs - lowActivitySinceMs_) >= kLowActivityFaultDelayMs) {
      signalValid_ = false;
      sensorFault_ = true;
    }
  } else {
    lowActivitySinceMs_ = 0;
    // Keep both flags explicit: signalValid_ informs CAN status bit packing,
    // sensorFault_ represents a latched diagnostic/fault condition.
    signalValid_ = true;
    sensorFault_ = false;
  }
}

void KnockMonitor::maybeUpdateBaseline(bool detectActive) {
  if (!cfg_.baseline_learning_enabled) return;

  const bool calmWindow = !detectActive && !warningActive_ && !criticalActive_;
  if (!calmWindow) return;

  const float alpha = baselineLearned_ ? kBaselineAlphaSlow : kBaselineAlphaFast;
  baseline_ += alpha * (knockEnergy_ - baseline_);
  if (baseline_ < kMinimumEnergyFloor) baseline_ = kMinimumEnergyFloor;

  ++baselineSampleCount_;
  if (!baselineLearned_ && baselineSampleCount_ > (kBaselineLearnMs / kKnockTaskIntervalMs)) {
    baselineLearned_ = true;
    baselineProfileDirty_ = true;
  }
}

void KnockMonitor::handleDetection(const state::VehicleState& snapshot, uint32_t nowMs, bool detectActive) {
  if (lastWindowDecayMs_ == 0) lastWindowDecayMs_ = nowMs;
  if ((nowMs - lastWindowDecayMs_) >= 1000) {
    if (eventWindowCount_ > 0) --eventWindowCount_;
    lastWindowDecayMs_ = nowMs;
  }

  warningActive_ = eventWindowCount_ >= cfg_.warning_threshold_count;
  criticalActive_ = eventWindowCount_ >= cfg_.critical_threshold_count;

  if (sensorFault_) {
    queueFault(can_protocol::knock_fault_code::SENSOR_DISCONNECTED, kKnockSeverityWarn,
               clampU8FromFloat(activityEma_), 0);
    if (lastFaultCode_ != can_protocol::knock_fault_code::SENSOR_DISCONNECTED ||
        (nowMs - lastFaultLogMs_) >= 2000) {
      logKnockEvent(nowMs, snapshot, can_protocol::knock_fault_code::SENSOR_DISCONNECTED, false);
      lastFaultCode_ = can_protocol::knock_fault_code::SENSOR_DISCONNECTED;
      lastFaultLogMs_ = nowMs;
    }
    return;
  }

  if (clippingDetected_) {
    queueFault(can_protocol::knock_fault_code::SIGNAL_CLIPPING, kKnockSeverityWarn,
               static_cast<uint8_t>(clipHighTotal_ & 0xFF), static_cast<uint8_t>(clipLowTotal_ & 0xFF));
    if (lastFaultCode_ != can_protocol::knock_fault_code::SIGNAL_CLIPPING ||
        (nowMs - lastFaultLogMs_) >= 2000) {
      logKnockEvent(nowMs, snapshot, can_protocol::knock_fault_code::SIGNAL_CLIPPING, false);
      lastFaultCode_ = can_protocol::knock_fault_code::SIGNAL_CLIPPING;
      lastFaultLogMs_ = nowMs;
    }
  }

  if (!baselineLearned_ && cfg_.enabled) {
    if (lastFaultCode_ != can_protocol::knock_fault_code::BASELINE_NOT_LEARNED ||
        (nowMs - lastFaultLogMs_) >= 2000) {
      queueFault(can_protocol::knock_fault_code::BASELINE_NOT_LEARNED, kKnockSeverityInfo,
                 clampU8FromFloat(baseline_), 0);
      lastFaultCode_ = can_protocol::knock_fault_code::BASELINE_NOT_LEARNED;
      lastFaultLogMs_ = nowMs;
    }
  }

  if (!detectActive) {
    applyResponseMode(snapshot, warningActive_, criticalActive_);
    return;
  }

  const bool aboveThreshold = knockEnergy_ > threshold_;
  const bool cooldownElapsed = (nowMs - lastEventMs_) >= cfg_.event_cooldown_ms;
  if (aboveThreshold && cooldownElapsed) {
    registerEvent(snapshot, nowMs);
  }

  warningActive_ = eventWindowCount_ >= cfg_.warning_threshold_count;
  criticalActive_ = eventWindowCount_ >= cfg_.critical_threshold_count;
  applyResponseMode(snapshot, warningActive_, criticalActive_);
}

void KnockMonitor::registerEvent(const state::VehicleState& snapshot, uint32_t nowMs) {
  lastEventMs_ = nowMs;
  ++eventCountRolling_;
  if (eventWindowCount_ < 255) ++eventWindowCount_;

  lastEventRpm_ = snapshot.rpm;
  const float boost = snapshot.boost_kpa;
  lastEventBoostKpa_ = clampU8FromFloat(boost);
  lastEventIatC_ = static_cast<int8_t>(snapshot.intake_temp);
  lastEventTimeMs_ = nowMs;

  const bool warning = eventWindowCount_ >= cfg_.warning_threshold_count;
  const bool critical = eventWindowCount_ >= cfg_.critical_threshold_count;

  if (critical) {
    queueFault(can_protocol::knock_fault_code::KNOCK_CRITICAL, kKnockSeverityCritical,
               encodeRpmDiv100(lastEventRpm_), lastEventBoostKpa_);
    logKnockEvent(nowMs, snapshot, can_protocol::knock_fault_code::KNOCK_CRITICAL, true);
  } else if (warning) {
    queueFault(can_protocol::knock_fault_code::KNOCK_WARNING, kKnockSeverityWarn,
               encodeRpmDiv100(lastEventRpm_), lastEventBoostKpa_);
    logKnockEvent(nowMs, snapshot, can_protocol::knock_fault_code::KNOCK_WARNING, true);
  } else {
    logKnockEvent(nowMs, snapshot, 0, true);
  }
  lastFaultCode_ = 0;
}

void KnockMonitor::applyResponseMode(const state::VehicleState& snapshot, bool warning, bool critical) {
  uint8_t faultSetMask = 0;
  if (warning) faultSetMask |= 0x02;
  if (critical) faultSetMask |= 0x04;

  stateStore_->mutate([faultSetMask](state::VehicleState& s) {
    s.knock_warning_active = (faultSetMask & 0x02) != 0;
    s.knock_critical_active = (faultSetMask & 0x04) != 0;

    if (s.knock_warning_active) s.fault_flags |= 0x0200;
    else s.fault_flags &= static_cast<uint16_t>(~0x0200U);

    if (s.knock_critical_active) s.fault_flags |= 0x0400;
    else s.fault_flags &= static_cast<uint16_t>(~0x0400U);
  });

  switch (cfg_.response_mode) {
    case ResponseMode::LOG_ONLY:
    case ResponseMode::WARN_ONLY:
      return;
    case ResponseMode::FORCE_METH_ENABLE_IF_ARMED:
      // Optional aggressive response: force meth enable as supplemental
      // octane/cooling support during sustained critical knock events.
      if (critical && canMgr_ && snapshot.meth_online && !snapshot.meth_desired_armed) {
        canMgr_->sendMethArm(true);
      }
      return;
    case ResponseMode::SAFETY_SHUTDOWN:
      // Conservative fallback mode for suspect sensor/system behavior where
      // continued meth operation should be stopped pending operator review.
      if (critical && canMgr_ && snapshot.meth_desired_armed) {
        canMgr_->sendMethArm(false);
      }
      return;
    default:
      return;
  }
}

void KnockMonitor::queueFault(uint8_t faultCode, uint8_t severity, uint8_t data0, uint8_t data1) {
  stateStore_->mutate([&](state::VehicleState& s) {
    s.knock_fault_pending = true;
    s.knock_fault_code_pending = faultCode;
    s.knock_fault_severity_pending = severity;
    s.knock_fault_data0_pending = data0;
    s.knock_fault_data1_pending = data1;
  });
}

void KnockMonitor::updateSharedState(uint32_t nowMs) {
  stateStore_->mutate([&](state::VehicleState& s) {
    s.knock_energy = knockEnergy_;
    s.knock_baseline = baseline_;
    s.knock_threshold = threshold_;
    s.knock_event_count = eventCountRolling_;
    s.knock_last_event_rpm = lastEventRpm_;
    s.knock_last_event_boost_kpa = lastEventBoostKpa_;
    s.knock_last_event_iat_c = lastEventIatC_;
    s.knock_last_event_time_ms = lastEventTimeMs_;
    s.knock_baseline_learned = baselineLearned_;
    s.knock_signal_valid = signalValid_;
    s.knock_sensor_fault = sensorFault_;
    s.knock_clipping_detected = clippingDetected_;
    s.knock_signal_clip_high_count = clipHighTotal_;
    s.knock_signal_clip_low_count = clipLowTotal_;
    s.knock_logging_active = (logMgr_ != nullptr) && (sdMgr_ != nullptr) && sdMgr_->mounted();
    s.last_knock_ms = nowMs;
    s.knock_online = true;
  });
}

void KnockMonitor::logKnockEvent(uint32_t nowMs, const state::VehicleState& s, uint8_t faultCode, bool knockEvent) {
  if (!logMgr_) return;

  char line[256];
  snprintf(line, sizeof(line),
           "%lu,%u,%.0f,%.1f,%.1f,%u,%u,%u,%.2f,%.2f,%.2f,%u,%u",
           static_cast<unsigned long>(nowMs),
           static_cast<unsigned>(s.rpm),
           static_cast<double>(s.boost_kpa),
           static_cast<double>(s.intake_temp),
           static_cast<double>(s.engine_bay_temp),
           static_cast<unsigned>(s.meth_state),
           static_cast<unsigned>(s.meth_pump_duty),
           static_cast<unsigned>(s.meth_tank_level),
           static_cast<double>(knockEnergy_),
           static_cast<double>(baseline_),
           static_cast<double>(threshold_),
           knockEvent ? 1U : 0U,
           static_cast<unsigned>(faultCode));
  logMgr_->enqueue("knock", line);
}

void KnockMonitor::loadBaselineProfile() {
  if (!sdMgr_ || !sdMgr_->mounted()) return;

  char text[256];
  if (!sdMgr_->readTextFile(kKnockProfilePath, text, sizeof(text))) {
    return;
  }
  if (strstr(text, "CCM_KNOCK_PROFILE_V1") == nullptr) {
    return;
  }

  float baseline = 0.0f;
  float gain = cfg_.gain;
  float multiplier = cfg_.threshold_multiplier;
  float offset = cfg_.threshold_offset;
  uint32_t samples = 0;
  if (!parseFloatField(text, "baseline=", baseline) ||
      !parseU32Field(text, "samples=", samples)) {
    return;
  }
  parseFloatField(text, "gain=", gain);
  parseFloatField(text, "multiplier=", multiplier);
  parseFloatField(text, "offset=", offset);
  if (baseline < kMinimumEnergyFloor || baseline > 255.0f || samples == 0U) {
    return;
  }

  cfg_.gain = gain;
  cfg_.threshold_multiplier = multiplier;
  cfg_.threshold_offset = offset;
  baseline_ = baseline;
  savedBaseline_ = baseline;
  baselineSampleCount_ = samples;
  savedBaselineSampleCount_ = samples;
  baselineLearned_ = true;
  baselineProfileLoaded_ = true;
  baselineProfileDirty_ = false;
  lastBaselineSaveMs_ = millis();
  stateStore_->mutate([&](state::VehicleState& s) {
    s.knock_gain = cfg_.gain;
    s.knock_threshold_multiplier = cfg_.threshold_multiplier;
    s.knock_threshold_offset = cfg_.threshold_offset;
  });

  Serial.printf("[KNOCK] loaded SD profile baseline=%.2f gain=%.2f samples=%lu\n",
                 static_cast<double>(baseline_),
                 static_cast<double>(cfg_.gain),
                 static_cast<unsigned long>(baselineSampleCount_));
}

void KnockMonitor::maybeSaveBaselineProfile(uint32_t nowMs) {
  if (!sdMgr_ || !sdMgr_->mounted() || !baselineLearned_) return;
  if (ESP.getFreeHeap() < 24000U) return;

  const bool firstSave = savedBaselineSampleCount_ == 0U;
  const bool intervalElapsed = (nowMs - lastBaselineSaveMs_) >= kBaselineSaveIntervalMs;
  const bool changedEnough = baselineProfileDirty_ ||
                             fabsf(baseline_ - savedBaseline_) >= kBaselineSaveDelta ||
                             (baselineSampleCount_ - savedBaselineSampleCount_) >= 5000U;
  if (!firstSave && (!intervalElapsed || !changedEnough)) return;
  saveBaselineProfile(nowMs);
}

bool KnockMonitor::saveBaselineProfile(uint32_t nowMs) {
  if (!sdMgr_ || !sdMgr_->mounted() || !baselineLearned_) return false;

  char text[256];
  snprintf(text, sizeof(text),
           "CCM_KNOCK_PROFILE_V1\nbaseline=%.3f\nsamples=%lu\ngain=%.3f\nmultiplier=%.3f\noffset=%.3f\nadc_pin=%u\nsaved_ms=%lu\n",
           static_cast<double>(baseline_),
           static_cast<unsigned long>(baselineSampleCount_),
           static_cast<double>(cfg_.gain),
           static_cast<double>(cfg_.threshold_multiplier),
           static_cast<double>(cfg_.threshold_offset),
           static_cast<unsigned>(cfg_.adc_pin),
           static_cast<unsigned long>(nowMs));

  const bool ok = sdMgr_->writeTextFile(kKnockProfilePath, text);
  if (ok) {
    lastBaselineSaveMs_ = nowMs;
    savedBaseline_ = baseline_;
    savedBaselineSampleCount_ = baselineSampleCount_;
    baselineProfileDirty_ = false;
    Serial.printf("[KNOCK] saved SD profile baseline=%.2f samples=%lu\n",
                   static_cast<double>(baseline_),
                   static_cast<unsigned long>(baselineSampleCount_));
  }
  return ok;
}

}  // namespace knock
