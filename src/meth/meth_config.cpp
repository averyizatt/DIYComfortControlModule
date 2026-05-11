#include "meth/meth_config.h"

namespace meth {

uint8_t sanitizeRatio(uint8_t ratio) {
  if (ratio == static_cast<uint8_t>(RatioPreset::CUSTOM)) return ratio;
  if (ratio > 100) return 100;
  return ratio;
}

DesiredConfig fromVehicleState(const state::VehicleState& s) {
  DesiredConfig cfg{};
  cfg.version = s.meth_config_version;
  cfg.armed = s.meth_desired_armed;
  cfg.ratio_percent = sanitizeRatio(s.meth_selected_ratio_percent);
  cfg.boost_trigger_kpa = s.meth_boost_trigger_kpa;
  cfg.iat_threshold_c = s.meth_iat_safety_threshold;
  cfg.max_pump_duty = s.meth_max_pump_duty;
  cfg.failsafe_flags = (s.meth_can_loss_behavior == state::MethCanLossBehavior::HOLD_LAST_VALID) ? 0x01 : 0x00;
  return cfg;
}

bool isSafeConfig(const DesiredConfig& cfg) {
  if (cfg.boost_trigger_kpa < 20 || cfg.boost_trigger_kpa > 250) return false;
  if (cfg.max_pump_duty > 255) return false;
  if (cfg.iat_threshold_c < -20 || cfg.iat_threshold_c > 120) return false;
  if (cfg.ratio_percent != 255 && cfg.ratio_percent > 100) return false;
  return true;
}

can_protocol::CanFrame toCanBroadcast(const DesiredConfig& cfg) {
  can_protocol::MethConfigBroadcast m{};
  m.version = cfg.version;
  m.desired_armed = cfg.armed ? 1 : 0;
  m.ratio_percent = sanitizeRatio(cfg.ratio_percent);
  m.boost_trigger_kpa = cfg.boost_trigger_kpa;
  m.iat_threshold_offset40 = can_protocol::tempToOffset40(cfg.iat_threshold_c);
  m.max_pump_duty = cfg.max_pump_duty;
  m.failsafe_flags = cfg.failsafe_flags;
  return can_protocol::packMethConfigBroadcast(m);
}

bool parseAck(const can_protocol::CanFrame& frame, can_protocol::MethConfigAck& ack) {
  return can_protocol::unpackMethConfigAck(frame, ack);
}

bool parseRequest(const can_protocol::CanFrame& frame, can_protocol::MethConfigRequest& req) {
  return can_protocol::unpackMethConfigRequest(frame, req);
}

}  // namespace meth
