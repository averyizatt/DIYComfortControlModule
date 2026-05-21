#pragma once

#include <cstdint>

namespace knock::logic {

inline float computeThreshold(float baseline, float multiplier, float offset) {
  float threshold = (baseline * multiplier) + offset;
  if (threshold < baseline + 1.0f) threshold = baseline + 1.0f;
  return threshold;
}

inline bool detectionWindowActive(bool enabled, bool signalValid, bool sensorFault,
                                  float boostKpa, float boostEnableKpa,
                                  uint16_t rpm, uint16_t rpmEnableMin) {
  return enabled && signalValid && !sensorFault && boostKpa >= boostEnableKpa && rpm >= rpmEnableMin;
}

inline bool shouldRegisterEvent(float energy, float threshold, uint32_t nowMs,
                                uint32_t lastEventMs, uint16_t cooldownMs) {
  return energy > threshold && (nowMs - lastEventMs) >= cooldownMs;
}

inline uint8_t decayEventWindow(uint8_t eventWindowCount, uint32_t nowMs, uint32_t& lastDecayMs) {
  if (lastDecayMs == 0U) {
    lastDecayMs = nowMs;
    return eventWindowCount;
  }
  if ((nowMs - lastDecayMs) >= 1000U) {
    if (eventWindowCount > 0U) --eventWindowCount;
    lastDecayMs = nowMs;
  }
  return eventWindowCount;
}

inline bool warningActive(uint8_t count, uint8_t thresholdCount) {
  return count >= thresholdCount;
}

inline bool criticalActive(uint8_t count, uint8_t thresholdCount) {
  return count >= thresholdCount;
}

inline bool simulationAllowed(bool demoBuildEnabled, bool runtimeDemoEnabled) {
  return demoBuildEnabled || runtimeDemoEnabled;
}

inline bool forceMethEnable(uint8_t responseMode, bool critical, bool methArmed) {
  return responseMode == 2U && critical && methArmed;
}

inline bool safetyShutdown(uint8_t responseMode, bool critical) {
  return responseMode == 3U && critical;
}

struct SignalHealthState {
  float activity_ema = 0.0f;
  uint32_t low_activity_since_ms = 0;
  uint32_t clip_window_start_ms = 0;
  uint16_t clip_high_window_count = 0;
  uint16_t clip_low_window_count = 0;
  bool signal_valid = true;
  bool sensor_fault = false;
  bool clipping_detected = false;
};

inline void updateSignalHealth(SignalHealthState& state, uint16_t raw, float absCentered, uint32_t nowMs) {
  constexpr float kActivityAlpha = 0.08f;
  constexpr float kLowActivityThreshold = 1.6f;
  constexpr uint32_t kLowActivityFaultDelayMs = 2500;
  constexpr uint16_t kAdcLowClip = 5;
  constexpr uint16_t kAdcHighClip = 4090;
  constexpr uint32_t kClipWindowMs = 1000;
  constexpr uint16_t kClipWindowThreshold = 20;

  state.activity_ema += kActivityAlpha * (absCentered - state.activity_ema);
  if (state.clip_window_start_ms == 0U) state.clip_window_start_ms = nowMs;
  if (raw >= kAdcHighClip) ++state.clip_high_window_count;
  if (raw <= kAdcLowClip) ++state.clip_low_window_count;

  if ((nowMs - state.clip_window_start_ms) >= kClipWindowMs) {
    state.clipping_detected = state.clip_high_window_count >= kClipWindowThreshold ||
                              state.clip_low_window_count >= kClipWindowThreshold;
    state.clip_high_window_count = 0;
    state.clip_low_window_count = 0;
    state.clip_window_start_ms = nowMs;
  }

  if (state.activity_ema < kLowActivityThreshold) {
    if (state.low_activity_since_ms == 0U) state.low_activity_since_ms = nowMs;
    if ((nowMs - state.low_activity_since_ms) >= kLowActivityFaultDelayMs) {
      state.signal_valid = false;
      state.sensor_fault = true;
    }
  } else {
    state.low_activity_since_ms = 0;
    state.signal_valid = true;
    state.sensor_fault = false;
  }
}

}  // namespace knock::logic
