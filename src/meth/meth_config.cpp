#include "meth/meth_config.h"

namespace meth {

uint8_t sanitizeRatio(uint8_t ratio) {
  // Clamp to 0–100. 255 is no longer used as a sentinel.
  if (ratio > 100) return 100;
  return ratio;
}

DesiredConfig fromVehicleState(const state::VehicleState& s) {
  DesiredConfig cfg{};
  cfg.version = s.meth_config_version;
  cfg.armed = s.meth_desired_armed;
  cfg.ratio_percent = sanitizeRatio(s.meth_selected_ratio_percent);
  return cfg;
}

can_protocol::CanFrame toCanBroadcast(const DesiredConfig& cfg) {
  can_protocol::MethConfigBroadcast m{};
  m.version = cfg.version;
  m.desired_armed = cfg.armed ? 1 : 0;
  m.ratio_percent = sanitizeRatio(cfg.ratio_percent);
  m.boost_trigger_kpa = 114;  // ~3.5 psi boost above a 90 kPa local baro default
  m.iat_threshold_offset40 = can_protocol::tempToOffset40(50);
  m.max_pump_duty = 100;
  m.failsafe_flags = 0x03;  // low tank + sensor validity required
  return can_protocol::packMethConfigBroadcast(m);
}

bool parseAck(const can_protocol::CanFrame& frame, can_protocol::MethConfigAck& ack) {
  return can_protocol::unpackMethConfigAck(frame, ack);
}

bool parseRequest(const can_protocol::CanFrame& frame, can_protocol::MethConfigRequest& req) {
  return can_protocol::unpackMethConfigRequest(frame, req);
}

}  // namespace meth
