#pragma once

#include <Arduino.h>

#include "can/can_protocol.h"
#include "state/vehicle_state.h"

namespace meth {

constexpr uint8_t kMinBoostTriggerKpa = 20;
constexpr uint8_t kMaxBoostTriggerKpa = 250;
constexpr int8_t kMinIatThresholdC = -20;
constexpr int8_t kMaxIatThresholdC = 120;

enum class RatioPreset : uint8_t {
  WATER_ONLY = 0,
  METH_25 = 25,
  METH_30 = 30,
  METH_50 = 50,
  METH_75 = 75,
  METH_100 = 100,
  CUSTOM = 255,
};

struct DesiredConfig {
  uint8_t version = 0;
  bool armed = false;
  uint8_t ratio_percent = 50;
  uint8_t boost_trigger_kpa = 120;
  int8_t iat_threshold_c = 55;
  uint8_t max_pump_duty = 200;
  uint8_t failsafe_flags = 0;
};

DesiredConfig fromVehicleState(const state::VehicleState& s);
can_protocol::CanFrame toCanBroadcast(const DesiredConfig& cfg);
bool parseAck(const can_protocol::CanFrame& frame, can_protocol::MethConfigAck& ack);
bool parseRequest(const can_protocol::CanFrame& frame, can_protocol::MethConfigRequest& req);
uint8_t sanitizeRatio(uint8_t ratio);
bool isSafeConfig(const DesiredConfig& cfg);

}  // namespace meth
