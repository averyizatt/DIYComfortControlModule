#pragma once

#include <Arduino.h>

#include "settings/settings_manager.h"
#include "state/vehicle_state.h"

namespace canbus {
class CanManager;
}

namespace storage {
class LogManager;
class SdManager;
}

namespace knock {

enum class ResponseMode : uint8_t {
  LOG_ONLY = 0,
  WARN_ONLY = 1,
  FORCE_METH_ENABLE_IF_ARMED = 2,
  SAFETY_SHUTDOWN = 3,
};

struct RuntimeConfig {
  bool enabled = true;
  uint8_t adc_pin = 48;
  float boost_enable_kpa = 120.0f;
  uint16_t rpm_enable_min = 2500;
  float threshold_multiplier = 2.5f;
  float threshold_offset = 8.0f;
  uint16_t event_cooldown_ms = 250;
  uint8_t warning_threshold_count = 2;
  uint8_t critical_threshold_count = 4;
  bool baseline_learning_enabled = true;
  bool demo_mode_enabled = false;
  ResponseMode response_mode = ResponseMode::WARN_ONLY;
};

class KnockMonitor {
 public:
  bool begin(state::VehicleStateStore* stateStore, settings::SettingsManager* settingsMgr,
             storage::LogManager* logMgr, storage::SdManager* sdMgr, canbus::CanManager* canMgr);
  void tick(uint32_t nowMs);

 private:
  void configureAdc();
  void loadConfigFromState(const state::VehicleState& s);
  void applyStateCommands(state::VehicleState& s);
  float sampleEnergy(const state::VehicleState& s, uint32_t nowMs);
  void updateSignalHealth(uint16_t sampleRaw, float absCentered, uint32_t nowMs);
  void maybeUpdateBaseline(bool detectActive);
  void handleDetection(const state::VehicleState& snapshot, uint32_t nowMs, bool detectActive);
  void registerEvent(const state::VehicleState& snapshot, uint32_t nowMs);
  void applyResponseMode(const state::VehicleState& snapshot, bool warning, bool critical);
  void updateSharedState(uint32_t nowMs);
  void queueFault(uint8_t faultCode, uint8_t severity, uint8_t data0, uint8_t data1);
  void logKnockEvent(uint32_t nowMs, const state::VehicleState& s, uint8_t faultCode, bool knockEvent);

  state::VehicleStateStore* stateStore_ = nullptr;
  settings::SettingsManager* settingsMgr_ = nullptr;
  storage::LogManager* logMgr_ = nullptr;
  storage::SdManager* sdMgr_ = nullptr;
  canbus::CanManager* canMgr_ = nullptr;

  RuntimeConfig cfg_{};
  bool adcConfigured_ = false;

  // Smoothed signal metrics in ADC counts around the midpoint.
  float knockEnergy_ = 0.0f;
  float baseline_ = 12.0f;
  float threshold_ = 32.0f;
  float activityEma_ = 0.0f;

  uint8_t eventCountRolling_ = 0;
  uint8_t eventWindowCount_ = 0;
  uint16_t lastEventRpm_ = 0;
  uint8_t lastEventBoostKpa_ = 0;

  uint32_t lastEventMs_ = 0;
  uint32_t lastWindowDecayMs_ = 0;
  uint32_t lowActivitySinceMs_ = 0;
  uint32_t clipWindowStartMs_ = 0;
  uint32_t lastFaultLogMs_ = 0;
  uint8_t lastFaultCode_ = 0;
  uint32_t lastDemoSpikeMs_ = 0;
  uint32_t nextDemoSpikeGapMs_ = 1200;
  uint16_t clipHighWindowCount_ = 0;
  uint16_t clipLowWindowCount_ = 0;
  uint16_t clipHighTotal_ = 0;
  uint16_t clipLowTotal_ = 0;
  uint32_t baselineSampleCount_ = 0;

  int8_t lastEventIatC_ = 0;
  uint32_t lastEventTimeMs_ = 0;

  bool baselineLearned_ = false;
  bool signalValid_ = true;
  bool sensorFault_ = false;
  bool clippingDetected_ = false;
  bool warningActive_ = false;
  bool criticalActive_ = false;
  bool forceDemoSpike_ = false;
};

}  // namespace knock
