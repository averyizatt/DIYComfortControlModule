#pragma once

#include <Arduino.h>

#include "can/can_protocol.h"
#include "state/vehicle_state.h"

namespace meth {

// CCM only controls armed state and mixture ratio; all injection thresholds
// are owned by the meth module's own firmware.
struct DesiredConfig {
  uint8_t version = 0;
  bool armed = false;
  uint8_t ratio_percent = 50;  // 0–100 % methanol in tank
};

DesiredConfig fromVehicleState(const state::VehicleState& s);
can_protocol::CanFrame toCanBroadcast(const DesiredConfig& cfg);
bool parseAck(const can_protocol::CanFrame& frame, can_protocol::MethConfigAck& ack);
bool parseRequest(const can_protocol::CanFrame& frame, can_protocol::MethConfigRequest& req);
uint8_t sanitizeRatio(uint8_t ratio);

}  // namespace meth
