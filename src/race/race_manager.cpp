#include "race/race_manager.h"

#include <cmath>

namespace race {

namespace {
constexpr float kTarget30Mph = 30.0f;
constexpr float kTarget60Mph = 60.0f;
constexpr float kTarget130Mph = 130.0f;
constexpr float kTarget100Kph = 100.0f;
constexpr float kTarget150Kph = 150.0f;
constexpr float kEighthMileM = 201.168f;
constexpr float kQuarterMileM = 402.336f;
constexpr uint32_t kMinimumLapMs = 15000;
}  // namespace

bool RacePerformanceManager::begin(state::VehicleStateStore* stateStore, settings::SettingsManager* settingsMgr, storage::LogManager* logMgr) {
  stateStore_ = stateStore;
  settingsMgr_ = settingsMgr;
  logMgr_ = logMgr;
  return stateStore_ != nullptr && settingsMgr_ != nullptr && logMgr_ != nullptr;
}

void RacePerformanceManager::startRun(state::RaceMode mode, bool manualStart) {
  if (!stateStore_) return;
  const uint32_t nowMs = millis();
  stateStore_->mutate([&](state::VehicleState& s) {
    s.race_mode = mode;
    s.race_enabled = (mode != state::RaceMode::OFF);
    s.race_running = (mode != state::RaceMode::OFF);
    s.race_run_start_ms = nowMs;
    s.race_elapsed_ms = 0;
    s.race_distance_m = 0.0f;
    s.race_0_30_s = -1.0f;
    s.race_0_60_s = -1.0f;
    s.race_60_130_s = -1.0f;
    s.race_100_150_kph_s = -1.0f;
    s.race_eighth_mile_et_s = -1.0f;
    s.race_quarter_mile_et_s = -1.0f;
    s.race_eighth_mile_trap_mph = 0.0f;
    s.race_quarter_mile_trap_mph = 0.0f;
    s.race_data_valid = false;
    s.race_quality_percent = 0;
    s.race_validation_flags = 0;
    if (mode == state::RaceMode::LAP) {
      s.race_lap_count = 0;
      s.race_last_lap_s = -1.0f;
      s.race_lap_delta_s = 0.0f;
    }
  });
  prevSpeedMph_ = 0.0f;
  prevSpeedKph_ = 0.0f;
  prevDistanceM_ = 0.0f;
  lapArmed_ = false;
  lapStartMs_ = nowMs;
  lastCrossingMs_ = 0;
  lastTickMs_ = nowMs;
  if (manualStart) {
    logMgr_->enqueue("race", String("event=start,mode=") + static_cast<uint8_t>(mode));
  }
}

void RacePerformanceManager::stopRun() {
  if (!stateStore_) return;
  state::VehicleState snapshot = stateStore_->read();
  const uint8_t quality = snapshot.race_quality_percent;
  const uint8_t flags = snapshot.race_validation_flags;
  stateStore_->mutate([](state::VehicleState& s) { s.race_running = false; });
  completeRun(quality, flags);
}

void RacePerformanceManager::resetSession() {
  if (!stateStore_) return;
  stateStore_->mutate([](state::VehicleState& s) {
    s.race_enabled = false;
    s.race_running = false;
    s.race_mode = state::RaceMode::OFF;
    s.race_elapsed_ms = 0;
    s.race_distance_m = 0.0f;
    s.race_0_30_s = -1.0f;
    s.race_0_60_s = -1.0f;
    s.race_60_130_s = -1.0f;
    s.race_100_150_kph_s = -1.0f;
    s.race_eighth_mile_et_s = -1.0f;
    s.race_quarter_mile_et_s = -1.0f;
    s.race_eighth_mile_trap_mph = 0.0f;
    s.race_quarter_mile_trap_mph = 0.0f;
    s.race_lap_count = 0;
    s.race_last_lap_s = -1.0f;
    s.race_best_lap_s = -1.0f;
    s.race_lap_delta_s = 0.0f;
    s.race_data_valid = false;
    s.race_quality_percent = 0;
    s.race_validation_flags = 0;
  });
  logMgr_->enqueue("race", "event=reset");
}

void RacePerformanceManager::setStartFinishPointFromCurrentFix() {
  if (!stateStore_) return;
  state::VehicleState s = stateStore_->read();
  if (!s.gps_fix) return;
  stateStore_->mutate([&](state::VehicleState& st) {
    st.race_start_latitude = static_cast<float>(st.gps_latitude);
    st.race_start_longitude = static_cast<float>(st.gps_longitude);
    st.race_start_point_set = true;
  });
  state::VehicleState snapshot = stateStore_->read();
  settingsMgr_->updateFromState(snapshot);
  settingsMgr_->save();
  logMgr_->enqueue("race", "event=set_start_finish");
}

void RacePerformanceManager::markLap() {
  if (!stateStore_) return;
  const uint32_t nowMs = millis();
  stateStore_->mutate([&](state::VehicleState& s) {
    if (!s.race_running || s.race_mode != state::RaceMode::LAP) return;
    if (!lapArmed_) {
      lapArmed_ = true;
      lapStartMs_ = nowMs;
      lastCrossingMs_ = nowMs;
      return;
    }
    const float lapS = static_cast<float>(nowMs - lapStartMs_) / 1000.0f;
    lapStartMs_ = nowMs;
    lastCrossingMs_ = nowMs;
    s.race_lap_count++;
    s.race_last_lap_s = lapS;
    s.race_last_lap_ms = nowMs;
    if (s.race_best_lap_s < 0.0f || lapS < s.race_best_lap_s) {
      s.race_best_lap_s = lapS;
      s.race_lap_delta_s = 0.0f;
    } else {
      s.race_lap_delta_s = lapS - s.race_best_lap_s;
    }
  });
}

bool RacePerformanceManager::sampleValid(const state::VehicleState& s, uint32_t dtMs, uint8_t& qualityOut, uint8_t& flagsOut) const {
  enum : uint8_t {
    kFlagNoFix = 1 << 0,
    kFlagLowSat = 1 << 1,
    kFlagRateLow = 1 << 2,
    kFlagRateHigh = 1 << 3,
    kFlagNoisy = 1 << 4,
  };

  flagsOut = 0;
  qualityOut = 100;
  if (!s.gps_fix) flagsOut |= kFlagNoFix;
  if (s.gps_satellites < s.race_min_satellites) flagsOut |= kFlagLowSat;
  if (dtMs < s.race_sample_min_ms) flagsOut |= kFlagRateLow;
  if (dtMs > s.race_sample_max_ms) flagsOut |= kFlagRateHigh;

  const float speedDeltaMps = fabsf((s.speed - (prevSpeedKph_)) * kKphToMps);
  const float accelMps2 = (dtMs > 0) ? (speedDeltaMps / (static_cast<float>(dtMs) / 1000.0f)) : 0.0f;
  if (accelMps2 > 16.0f) flagsOut |= kFlagNoisy;

  if (flagsOut & kFlagNoFix) qualityOut = 0;
  if (flagsOut & kFlagLowSat) qualityOut = static_cast<uint8_t>(qualityOut > 35 ? qualityOut - 35 : 0);
  if (flagsOut & (kFlagRateLow | kFlagRateHigh)) qualityOut = static_cast<uint8_t>(qualityOut > 30 ? qualityOut - 30 : 0);
  if (flagsOut & kFlagNoisy) qualityOut = static_cast<uint8_t>(qualityOut > 20 ? qualityOut - 20 : 0);

  return flagsOut == 0;
}

float RacePerformanceManager::interpolateCrossTime(float prevValue, float nextValue, float targetValue, float prevTimeS, float nextTimeS) {
  if (nextValue <= prevValue) return nextTimeS;
  const float ratio = (targetValue - prevValue) / (nextValue - prevValue);
  return prevTimeS + (nextTimeS - prevTimeS) * ratio;
}

void RacePerformanceManager::tickAcceleration(const state::VehicleState& s, uint32_t nowMs, uint32_t dtMs, float speedMph, float speedKph) {
  if (!s.race_running) return;

  const float elapsedS = static_cast<float>(nowMs - s.race_run_start_ms) / 1000.0f;
  const float prevElapsedS = elapsedS - (static_cast<float>(dtMs) / 1000.0f);
  const float speedMps = speedMph * kMphToMps;
  const float prevSpeedMps = prevSpeedMph_ * kMphToMps;
  const float dist = prevDistanceM_ + ((prevSpeedMps + speedMps) * 0.5f * (static_cast<float>(dtMs) / 1000.0f));

  stateStore_->mutate([&](state::VehicleState& st) {
    st.race_elapsed_ms = nowMs - st.race_run_start_ms;
    st.race_distance_m = dist;

    if (st.race_0_30_s < 0.0f && prevSpeedMph_ < kTarget30Mph && speedMph >= kTarget30Mph) {
      st.race_0_30_s = interpolateCrossTime(prevSpeedMph_, speedMph, kTarget30Mph, prevElapsedS, elapsedS);
    }
    if (st.race_0_60_s < 0.0f && prevSpeedMph_ < kTarget60Mph && speedMph >= kTarget60Mph) {
      st.race_0_60_s = interpolateCrossTime(prevSpeedMph_, speedMph, kTarget60Mph, prevElapsedS, elapsedS);
    }
    if (st.race_60_130_s < 0.0f && prevSpeedMph_ < kTarget130Mph && speedMph >= kTarget130Mph && st.race_0_60_s >= 0.0f) {
      const float at130 = interpolateCrossTime(prevSpeedMph_, speedMph, kTarget130Mph, prevElapsedS, elapsedS);
      st.race_60_130_s = at130 - st.race_0_60_s;
    }
    if (st.race_100_150_kph_s < 0.0f && prevSpeedKph_ < kTarget150Kph && speedKph >= kTarget150Kph) {
      const float at150 = interpolateCrossTime(prevSpeedKph_, speedKph, kTarget150Kph, prevElapsedS, elapsedS);
      const float at100 = (st.race_0_60_s >= 0.0f && prevSpeedKph_ <= kTarget100Kph && speedKph >= kTarget100Kph)
                              ? interpolateCrossTime(prevSpeedKph_, speedKph, kTarget100Kph, prevElapsedS, elapsedS)
                              : -1.0f;
      if (at100 >= 0.0f) st.race_100_150_kph_s = at150 - at100;
    }

    if (st.race_eighth_mile_et_s < 0.0f && prevDistanceM_ < kEighthMileM && dist >= kEighthMileM) {
      st.race_eighth_mile_et_s = interpolateCrossTime(prevDistanceM_, dist, kEighthMileM, prevElapsedS, elapsedS);
      st.race_eighth_mile_trap_mph = speedMph;
    }
    if (st.race_quarter_mile_et_s < 0.0f && prevDistanceM_ < kQuarterMileM && dist >= kQuarterMileM) {
      st.race_quarter_mile_et_s = interpolateCrossTime(prevDistanceM_, dist, kQuarterMileM, prevElapsedS, elapsedS);
      st.race_quarter_mile_trap_mph = speedMph;
      st.race_running = false;
    }
  });

  prevDistanceM_ = dist;
}

float RacePerformanceManager::distanceMeters(double lat1, double lon1, double lat2, double lon2) {
  constexpr double kEarthRadiusM = 6371000.0;
  const double lat1Rad = lat1 * PI / 180.0;
  const double lat2Rad = lat2 * PI / 180.0;
  const double dLat = (lat2 - lat1) * PI / 180.0;
  const double dLon = (lon2 - lon1) * PI / 180.0;
  const double a = sin(dLat / 2.0) * sin(dLat / 2.0) + cos(lat1Rad) * cos(lat2Rad) * sin(dLon / 2.0) * sin(dLon / 2.0);
  const double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
  return static_cast<float>(kEarthRadiusM * c);
}

void RacePerformanceManager::tickLapMode(const state::VehicleState& s, uint32_t nowMs, uint32_t dtMs) {
  (void)dtMs;
  if (!s.race_running || s.race_mode != state::RaceMode::LAP || !s.race_start_point_set || !s.gps_fix) return;

  const float distToStart = distanceMeters(s.gps_latitude, s.gps_longitude, s.race_start_latitude, s.race_start_longitude);
  if (distToStart > s.race_start_finish_radius_m) return;
  if ((nowMs - lastCrossingMs_) < kMinimumLapMs) return;

  if (!lapArmed_) {
    lapArmed_ = true;
    lapStartMs_ = nowMs;
    lastCrossingMs_ = nowMs;
    return;
  }

  stateStore_->mutate([&](state::VehicleState& st) {
    const float lapS = static_cast<float>(nowMs - lapStartMs_) / 1000.0f;
    lapStartMs_ = nowMs;
    lastCrossingMs_ = nowMs;
    st.race_lap_count++;
    st.race_last_lap_s = lapS;
    st.race_last_lap_ms = nowMs;
    if (st.race_best_lap_s < 0.0f || lapS < st.race_best_lap_s) {
      st.race_best_lap_s = lapS;
      st.race_lap_delta_s = 0.0f;
    } else {
      st.race_lap_delta_s = lapS - st.race_best_lap_s;
    }
  });
}

void RacePerformanceManager::maybeAutoStart(const state::VehicleState& s, float speedMph) {
  if (!s.race_enabled || s.race_running || !s.race_auto_start) return;
  if (speedMph < 1.0f) return;
  if (s.race_mode == state::RaceMode::OFF) return;
  startRun(s.race_mode, false);
}

void RacePerformanceManager::completeRun(uint8_t quality, uint8_t flags) {
  if (!stateStore_) return;
  stateStore_->mutate([&](state::VehicleState& s) {
    s.race_data_valid = flags == 0;
    s.race_quality_percent = quality;
    s.race_validation_flags = flags;
  });
  const state::VehicleState s = stateStore_->read();
  pushHistory(s);
  String line = String("event=complete,mode=") + static_cast<uint8_t>(s.race_mode) + ",0_60=" + String(s.race_0_60_s, 3) +
                ",qtr=" + String(s.race_quarter_mile_et_s, 3) + ",trap=" + String(s.race_quarter_mile_trap_mph, 2) + ",quality=" + quality +
                ",flags=" + flags;
  logMgr_->enqueue("race", line);
}

void RacePerformanceManager::pushHistory(const state::VehicleState& s) {
  HistoryEntry& e = history_[historyHead_];
  e.timestampMs = millis();
  e.mode = static_cast<uint8_t>(s.race_mode);
  e.zeroToSixtyS = s.race_0_60_s;
  e.quarterEtS = s.race_quarter_mile_et_s;
  e.quarterTrapMph = s.race_quarter_mile_trap_mph;
  e.bestLapS = s.race_best_lap_s;
  e.laps = s.race_lap_count;
  e.quality = s.race_quality_percent;
  e.validationFlags = s.race_validation_flags;

  historyHead_ = static_cast<uint8_t>((historyHead_ + 1) % kHistorySize);
  if (historyCount_ < kHistorySize) historyCount_++;
}

void RacePerformanceManager::tick(uint32_t nowMs) {
  if (!stateStore_) return;
  if (lastTickMs_ == 0) {
    lastTickMs_ = nowMs;
    return;
  }

  const uint32_t dtMs = nowMs - lastTickMs_;
  lastTickMs_ = nowMs;

  state::VehicleState s = stateStore_->read();
  const float speedMph = s.speed * kKphToMph;
  maybeAutoStart(s, speedMph);
  s = stateStore_->read();

  if (!s.race_enabled) {
    prevSpeedMph_ = speedMph;
    prevSpeedKph_ = s.speed;
    return;
  }

  uint8_t quality = 0;
  uint8_t flags = 0;
  sampleValid(s, dtMs, quality, flags);
  stateStore_->mutate([&](state::VehicleState& st) {
    st.race_quality_percent = quality;
    st.race_validation_flags = flags;
    st.race_data_valid = flags == 0;
    if (st.race_running) st.race_elapsed_ms = nowMs - st.race_run_start_ms;
  });

  if (s.race_mode == state::RaceMode::ACCEL) {
    tickAcceleration(s, nowMs, dtMs, speedMph, s.speed);
  } else if (s.race_mode == state::RaceMode::LAP) {
    tickLapMode(s, nowMs, dtMs);
  }

  const state::VehicleState afterTick = stateStore_->read();
  if (s.race_running && !afterTick.race_running) {
    completeRun(quality, flags);
  }

  prevSpeedMph_ = speedMph;
  prevSpeedKph_ = s.speed;
}

String RacePerformanceManager::historyJson() const {
  DynamicJsonDocument doc(4096);
  JsonArray arr = doc.createNestedArray("history");
  for (uint8_t i = 0; i < historyCount_; ++i) {
    const int index = (static_cast<int>(historyHead_) - 1 - i + kHistorySize) % kHistorySize;
    const HistoryEntry& e = history_[index];
    JsonObject item = arr.createNestedObject();
    item["ts_ms"] = e.timestampMs;
    item["mode"] = e.mode;
    item["zero_to_sixty_s"] = e.zeroToSixtyS;
    item["quarter_et_s"] = e.quarterEtS;
    item["quarter_trap_mph"] = e.quarterTrapMph;
    item["best_lap_s"] = e.bestLapS;
    item["lap_count"] = e.laps;
    item["quality"] = e.quality;
    item["validation_flags"] = e.validationFlags;
  }
  String out;
  serializeJson(doc, out);
  return out;
}

String RacePerformanceManager::recordsJson() const {
  DynamicJsonDocument doc(512);
  float best060 = -1.0f;
  float bestQuarter = -1.0f;
  float bestLap = -1.0f;
  for (uint8_t i = 0; i < historyCount_; ++i) {
    const HistoryEntry& e = history_[i];
    if (e.zeroToSixtyS >= 0.0f && (best060 < 0.0f || e.zeroToSixtyS < best060)) best060 = e.zeroToSixtyS;
    if (e.quarterEtS >= 0.0f && (bestQuarter < 0.0f || e.quarterEtS < bestQuarter)) bestQuarter = e.quarterEtS;
    if (e.bestLapS >= 0.0f && (bestLap < 0.0f || e.bestLapS < bestLap)) bestLap = e.bestLapS;
  }
  doc["best_0_60_s"] = best060;
  doc["best_quarter_et_s"] = bestQuarter;
  doc["best_lap_s"] = bestLap;
  doc["history_count"] = historyCount_;
  String out;
  serializeJson(doc, out);
  return out;
}

uint16_t RacePerformanceManager::exportHistoryToLog() {
  if (!logMgr_) return 0;
  uint16_t written = 0;
  for (uint8_t i = 0; i < historyCount_; ++i) {
    const int index = (static_cast<int>(historyHead_) - historyCount_ + i + kHistorySize) % kHistorySize;
    const HistoryEntry& e = history_[index];
    String line = String("ts=") + e.timestampMs + ",mode=" + e.mode + ",0_60=" + String(e.zeroToSixtyS, 3) + ",qtr_et=" + String(e.quarterEtS, 3) +
                  ",qtr_trap=" + String(e.quarterTrapMph, 2) + ",best_lap=" + String(e.bestLapS, 3) + ",laps=" + e.laps + ",quality=" + e.quality +
                  ",flags=" + e.validationFlags;
    logMgr_->enqueue("race", line);
    ++written;
  }
  return written;
}

}  // namespace race
