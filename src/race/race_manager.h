#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>

#include "settings/settings_manager.h"
#include "state/vehicle_state.h"
#include "storage/log_manager.h"

namespace race {

class RacePerformanceManager {
 public:
  bool begin(state::VehicleStateStore* stateStore, settings::SettingsManager* settingsMgr, storage::LogManager* logMgr);
  void tick(uint32_t nowMs);

  void startRun(state::RaceMode mode, bool manualStart = true);
  void stopRun();
  void resetSession();
  void setStartFinishPointFromCurrentFix();
  void markLap();

  String historyJson() const;
  String recordsJson() const;
  uint16_t exportHistoryToLog();

 private:
  struct HistoryEntry {
    uint32_t timestampMs = 0;
    uint8_t mode = 0;
    float zeroToSixtyS = -1.0f;
    float quarterEtS = -1.0f;
    float quarterTrapMph = 0.0f;
    float bestLapS = -1.0f;
    uint16_t laps = 0;
    uint8_t quality = 0;
    uint8_t validationFlags = 0;
  };

  static constexpr uint8_t kHistorySize = 20;
  static constexpr float kMphToMps = 0.44704f;
  static constexpr float kKphToMph = 0.621371f;
  static constexpr float kKphToMps = 0.27777778f;

  bool sampleValid(const state::VehicleState& s, uint32_t dtMs, uint8_t& qualityOut, uint8_t& flagsOut) const;
  void tickAcceleration(const state::VehicleState& s, uint32_t nowMs, uint32_t dtMs, float speedMph, float speedKph);
  void tickLapMode(const state::VehicleState& s, uint32_t nowMs, uint32_t dtMs);
  void maybeAutoStart(const state::VehicleState& s, float speedMph);
  void completeRun(uint8_t quality, uint8_t flags);
  void pushHistory(const state::VehicleState& s);

  static float interpolateCrossTime(float prevValue, float nextValue, float targetValue, float prevTimeS, float nextTimeS);
  static float distanceMeters(double lat1, double lon1, double lat2, double lon2);

  state::VehicleStateStore* stateStore_ = nullptr;
  settings::SettingsManager* settingsMgr_ = nullptr;
  storage::LogManager* logMgr_ = nullptr;

  HistoryEntry history_[kHistorySize]{};
  uint8_t historyHead_ = 0;
  uint8_t historyCount_ = 0;

  uint32_t lastTickMs_ = 0;
  float prevSpeedMph_ = 0.0f;
  float prevSpeedKph_ = 0.0f;
  float prevDistanceM_ = 0.0f;
  bool lapArmed_ = false;
  uint32_t lapStartMs_ = 0;
  uint32_t lastCrossingMs_ = 0;
};

}  // namespace race
