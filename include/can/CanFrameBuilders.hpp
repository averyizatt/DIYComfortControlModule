#pragma once

#include "can/can_protocol.h"
#include "state/VehicleStateData.hpp"

namespace canbus {

inline void packMasterHeartbeat(const state::VehicleState& s, can_protocol::CanFrame& out) {
  out.id = can_protocol::ID_MASTER_HEARTBEAT;
  out.dlc = 8;
  out.data[0] = s.master_state;
  out.data[1] = s.ui_page;
  out.data[2] = s.input_flags;
  out.data[3] = can_protocol::tempToOffset40(static_cast<int>(s.cabin_temp));
  out.data[4] = can_protocol::tempToOffset40(static_cast<int>(s.outside_temp));
  out.data[5] = can_protocol::voltsTo10(s.battery_voltage);
  out.data[6] = static_cast<uint8_t>(s.fault_flags & 0xFFU);
  out.data[7] = static_cast<uint8_t>((s.uptime_ms / 1000UL) & 0xFFU);
}

inline void packTachState(const state::VehicleState& s, can_protocol::CanFrame& out) {
  out.id = can_protocol::ID_TACH_RPM_STATE;
  out.dlc = 8;
  can_protocol::encodeU16BE(s.rpm, out.data[0], out.data[1]);
  can_protocol::encodeU16BE(s.generated_tach_hz10, out.data[2], out.data[3]);
  out.data[4] = s.tach_source;
  out.data[5] = s.tach_status_flags;
  out.data[6] = s.pulses_per_rev10;
  out.data[7] = 0;
}

inline void packGpsState(const state::VehicleState& s, can_protocol::CanFrame& out) {
  out.id = can_protocol::ID_GPS_STATE;
  out.dlc = 8;
  can_protocol::encodeU16BE(static_cast<uint16_t>(s.speed * 10.0f), out.data[0], out.data[1]);
  can_protocol::encodeU16BE(static_cast<uint16_t>(s.gps_altitude_m), out.data[2], out.data[3]);
  out.data[4] = s.gps_satellites;
  out.data[5] = s.gps_fix_type;
  out.data[6] = s.gps_status_flags;
  out.data[7] = 0;
}

inline void packKnockState(const state::VehicleState& s, can_protocol::CanFrame& out) {
  can_protocol::EngineKnockState ks{};
  ks.status_flags |= s.knock_enabled ? (1U << 0) : 0U;
  ks.status_flags |= s.knock_signal_valid ? (1U << 1) : 0U;
  ks.status_flags |= s.knock_warning_active ? (1U << 2) : 0U;
  ks.status_flags |= s.knock_critical_active ? (1U << 3) : 0U;
  ks.status_flags |= s.knock_baseline_learned ? (1U << 4) : 0U;
  ks.status_flags |= s.knock_sensor_fault ? (1U << 5) : 0U;
  ks.status_flags |= s.knock_clipping_detected ? (1U << 6) : 0U;
  ks.energy = can_protocol::clampU8(static_cast<int>(s.knock_energy));
  ks.baseline = can_protocol::clampU8(static_cast<int>(s.knock_baseline));
  ks.threshold = can_protocol::clampU8(static_cast<int>(s.knock_threshold));
  ks.event_count = s.knock_event_count;
  ks.last_event_rpm_div100 = can_protocol::clampU8(static_cast<int>(s.knock_last_event_rpm / 100U));
  ks.last_event_boost_kpa = s.knock_last_event_boost_kpa;
  ks.reserved = 0;
  out = can_protocol::packEngineKnockState(ks);
}

inline void packEngineSensorExt(const state::VehicleState& s, can_protocol::CanFrame& out) {
  can_protocol::EngineSensorExt ext{};
  ext.oil_pressure_psi = can_protocol::clampU8(static_cast<int>(s.oil_pressure_psi));
  ext.fuel_pressure_psi = can_protocol::clampU8(static_cast<int>(s.fuel_pressure_psi));
  ext.meth_pressure_psi = can_protocol::clampU8(static_cast<int>(s.meth_pressure_psi));
  ext.boost_ref_pressure_psi = can_protocol::clampU8(static_cast<int>(s.boost_ref_pressure_psi));
  ext.ambient_temp_c = static_cast<int8_t>(s.outside_temp);
  ext.cabin_temp_c = static_cast<int8_t>(s.cabin_temp);
  ext.analog_fault_flags = s.analog_sensor_fault_flags;
  out = can_protocol::packEngineSensorExt(ext);
}

inline can_protocol::CanFrame packMethConfigState(const state::VehicleState& s) {
  can_protocol::MethConfigBroadcast msg{};
  msg.version = s.meth_config_version;
  msg.desired_armed = s.meth_desired_armed ? 1U : 0U;
  msg.ratio_percent = s.meth_selected_ratio_percent > 100U ? 100U : s.meth_selected_ratio_percent;
  return can_protocol::packMethConfigBroadcast(msg);
}

}  // namespace canbus
