#pragma once

#include <Arduino.h>

namespace can_protocol {

// -----------------------------------------------------------------------------
// CAN network: Standard 11-bit IDs @ 500 kbit/s.
// Compatibility note: Taillight IDs 0x100, 0x101, 0x102 are preserved exactly.
// -----------------------------------------------------------------------------

constexpr uint32_t CAN_BITRATE = 500000;

// Reserved blocks
constexpr uint16_t ID_BLOCK_TAILLIGHT_BASE = 0x100;
constexpr uint16_t ID_BLOCK_MASTER_BASE = 0x200;
constexpr uint16_t ID_BLOCK_ENGINE_METH_BASE = 0x300;
constexpr uint16_t ID_BLOCK_GPS_BASE = 0x400;
constexpr uint16_t ID_BLOCK_COMFORT_BASE = 0x500;
constexpr uint16_t ID_BLOCK_FUTURE_BASE = 0x600;

// Taillight protocol (existing, do not change)
constexpr uint16_t ID_TAILLIGHT_STATE = 0x100;   // TX every 100ms, DLC 7
constexpr uint16_t ID_TAILLIGHT_COMMAND = 0x101; // RX, DLC varies
constexpr uint16_t ID_TAILLIGHT_FAULT = 0x102;   // TX on demand, DLC 4

// Cabin master frames
constexpr uint16_t ID_MASTER_HEARTBEAT = 0x200;  // TX every 100ms, DLC 8
constexpr uint16_t ID_MASTER_COMMAND = 0x201;    // RX, DLC varies
constexpr uint16_t ID_TACH_RPM_STATE = 0x202;    // TX every 20ms, DLC 8
constexpr uint16_t ID_GPS_STATE = 0x203;         // TX every 250ms, DLC 8

// Engine/water meth frames
constexpr uint16_t ID_ENGINE_METH_STATE = 0x300;    // TX every 50ms, DLC 8
constexpr uint16_t ID_ENGINE_METH_COMMAND = 0x301;  // RX, DLC varies
constexpr uint16_t ID_ENGINE_METH_FAULT = 0x302;    // TX on demand, DLC 4
constexpr uint16_t ID_ENGINE_SENSOR_EXT = 0x303;    // TX every 250ms, DLC 8
constexpr uint16_t ID_METH_CONFIG_BROADCAST = 0x304; // TX every 500ms, DLC 8
constexpr uint16_t ID_METH_CONFIG_REQUEST = 0x305;   // RX/TX as needed, DLC 1
constexpr uint16_t ID_METH_CONFIG_ACK = 0x306;       // RX/TX as needed, DLC 4

enum class MasterState : uint8_t { BOOT = 0, RUN = 1, WARN = 2, FAULT = 3, CONFIG = 4 };
enum class UiPage : uint8_t { DASH = 0, ENVIRONMENT = 1, METH = 2, LIGHTING = 3, DIAGNOSTICS = 4, SETTINGS = 5 };
enum class TachSource : uint8_t { CAN = 0, GPIO_INPUT = 1, TEST = 2, DEMO = 3 };
enum class MethState : uint8_t { OFF = 0, ARMED = 1, SPRAYING = 2, FAULT = 3, TEST = 4 };
enum class FlowStatus : uint8_t { UNKNOWN = 0, OK = 1, LOW_FLOW = 2, NO_FLOW = 3 };
enum class FaultSeverity : uint8_t { INFO = 0, WARNING = 1, CRITICAL = 2 };

namespace input_flag {
constexpr uint8_t UP = 1 << 0;
constexpr uint8_t DOWN = 1 << 1;
constexpr uint8_t ENTER = 1 << 2;
constexpr uint8_t BACK = 1 << 3;
constexpr uint8_t TOUCH = 1 << 4;
}  // namespace input_flag

namespace master_command {
constexpr uint8_t SET_UI_PAGE = 0x01;          // DLC 2, B1 page
constexpr uint8_t SET_BRIGHTNESS = 0x02;       // DLC 2, B1 0..255
constexpr uint8_t TRIGGER_TACH_SWEEP = 0x03;   // DLC 1
constexpr uint8_t SET_DRIVE_MODE = 0x04;       // DLC 2, B1 mode
}  // namespace master_command

namespace taillight_command {
constexpr uint8_t SET_BRIGHTNESS = 0x01;
constexpr uint8_t SET_OVERRIDE = 0x02;
constexpr uint8_t CLEAR_OVERRIDE = 0x03;
constexpr uint8_t TRIGGER_CUSTOM_ANIMATION = 0x04;
}  // namespace taillight_command

namespace meth_command {
constexpr uint8_t ARM = 0x01;                  // DLC 2, B1 0/1
constexpr uint8_t MANUAL_TEST_DUTY = 0x02;     // DLC 2, B1 duty
constexpr uint8_t STOP_MANUAL_TEST = 0x03;     // DLC 1
constexpr uint8_t SET_BOOST_TRIGGER = 0x04;    // DLC 2, B1 kPa
constexpr uint8_t SET_IAT_THRESHOLD = 0x05;    // DLC 2, B1 temp + 40
constexpr uint8_t CLEAR_FAULTS = 0x06;         // DLC 1
}  // namespace meth_command

namespace meth_fault_code {
constexpr uint8_t LOW_TANK = 0x01;
constexpr uint8_t NO_FLOW = 0x02;
constexpr uint8_t LOW_FLOW = 0x03;
constexpr uint8_t PUMP_OVERCURRENT = 0x04;
constexpr uint8_t SENSOR_FAIL = 0x05;
constexpr uint8_t OVER_TEMP = 0x06;
constexpr uint8_t CAN_TIMEOUT = 0x07;
constexpr uint8_t CONFIG_INVALID = 0x08;
constexpr uint8_t SAFETY_SHUTDOWN = 0x09;
}  // namespace meth_fault_code

struct CanFrame {
  uint16_t id = 0;
  uint8_t dlc = 0;
  uint8_t data[8]{};
};

inline uint8_t clampU8(int value) {
  return static_cast<uint8_t>(value < 0 ? 0 : (value > 255 ? 255 : value));
}

inline uint8_t tempToOffset40(int celsius) {
  return clampU8(celsius + 40);
}

inline int8_t offset40ToTemp(uint8_t encoded) {
  return static_cast<int8_t>(static_cast<int16_t>(encoded) - 40);
}

inline uint8_t voltsTo10(float volts) {
  const int voltsScaledBy10 = static_cast<int>(volts * 10.0f + 0.5f);
  return clampU8(voltsScaledBy10);
}

inline uint16_t decodeU16BE(uint8_t high, uint8_t low) {
  return static_cast<uint16_t>((static_cast<uint16_t>(high) << 8) | low);
}

inline void encodeU16BE(uint16_t value, uint8_t& high, uint8_t& low) {
  high = static_cast<uint8_t>((value >> 8) & 0xFF);
  low = static_cast<uint8_t>(value & 0xFF);
}

struct TaillightState {
  // 0x100, DLC 7 (compat contract)
  // B0 left state, B1 right state, B2 raw input flags, B3 brightness,
  // B4 die temp +40, B5 thermal derate %, B6 status flags.
  uint8_t left_state = 0;
  uint8_t right_state = 0;
  uint8_t input_flags = 0;
  uint8_t brightness = 0;
  int8_t die_temp_c = 0;
  uint8_t thermal_derate = 0;
  uint8_t status_flags = 0;
};

inline bool unpackTaillightState(const CanFrame& frame, TaillightState& out) {
  if (frame.id != ID_TAILLIGHT_STATE || frame.dlc < 7) return false;
  out.left_state = frame.data[0];
  out.right_state = frame.data[1];
  out.input_flags = frame.data[2];
  out.brightness = frame.data[3];
  out.die_temp_c = offset40ToTemp(frame.data[4]);
  out.thermal_derate = frame.data[5];
  out.status_flags = frame.data[6];
  return true;
}

struct TaillightFault {
  // 0x102, DLC 4: B0 code, B1 severity, B2 data0, B3 data1
  uint8_t code = 0;
  uint8_t severity = 0;
  uint8_t data0 = 0;
  uint8_t data1 = 0;
};

inline bool unpackTaillightFault(const CanFrame& frame, TaillightFault& out) {
  if (frame.id != ID_TAILLIGHT_FAULT || frame.dlc < 4) return false;
  out.code = frame.data[0];
  out.severity = frame.data[1];
  out.data0 = frame.data[2];
  out.data1 = frame.data[3];
  return true;
}

struct EngineMethState {
  // 0x300, DLC 8
  // B0 meth state, B1 pump duty, B2 tank %, B3 flow status,
  // B4 MAP/boost kPa, B5 IAT+40, B6 engine bay+40, B7 fault flags.
  uint8_t meth_state = 0;
  uint8_t pump_duty = 0;
  uint8_t tank_level = 0;
  uint8_t flow_status = 0;
  uint8_t boost_kpa = 0;
  int8_t iat_c = 0;
  int8_t engine_bay_c = 0;
  uint8_t fault_flags = 0;
};

inline bool unpackEngineMethState(const CanFrame& frame, EngineMethState& out) {
  if (frame.id != ID_ENGINE_METH_STATE || frame.dlc < 8) return false;
  out.meth_state = frame.data[0];
  out.pump_duty = frame.data[1];
  out.tank_level = frame.data[2];
  out.flow_status = frame.data[3];
  out.boost_kpa = frame.data[4];
  out.iat_c = offset40ToTemp(frame.data[5]);
  out.engine_bay_c = offset40ToTemp(frame.data[6]);
  out.fault_flags = frame.data[7];
  return true;
}

struct EngineMethFault {
  // 0x302, DLC 4
  uint8_t code = 0;
  uint8_t severity = 0;
  uint8_t data0 = 0;
  uint8_t data1 = 0;
};

inline bool unpackEngineMethFault(const CanFrame& frame, EngineMethFault& out) {
  if (frame.id != ID_ENGINE_METH_FAULT || frame.dlc < 4) return false;
  out.code = frame.data[0];
  out.severity = frame.data[1];
  out.data0 = frame.data[2];
  out.data1 = frame.data[3];
  return true;
}

struct MethConfigBroadcast {
  uint8_t version = 0;
  uint8_t desired_armed = 0;
  uint8_t ratio_percent = 255;    // 255 = unknown/custom
  uint8_t boost_trigger_kpa = 0;
  uint8_t iat_threshold_offset40 = 0;
  uint8_t max_pump_duty = 0;
  uint8_t failsafe_flags = 0;
  uint8_t checksum = 0;
};

struct MethConfigRequest {
  uint8_t reason = 0;  // 0 boot, 1 config expired, 2 user requested, 3 checksum mismatch
};

struct MethConfigAck {
  uint8_t accepted_version = 0;
  uint8_t status = 0;  // 0 OK, 1 rejected, 2 partial, 3 fault locked
  uint8_t reject_reason = 0;
  uint8_t active_ratio_percent = 255;
};

inline uint8_t simpleXorChecksum(const uint8_t* data, uint8_t len) {
  uint8_t checksum = 0;
  for (uint8_t i = 0; i < len; ++i) {
    checksum ^= data[i];
  }
  return checksum;
}

inline bool validateMethConfigChecksum(const MethConfigBroadcast& cfg) {
  const uint8_t data[7] = {
      cfg.version, cfg.desired_armed, cfg.ratio_percent, cfg.boost_trigger_kpa, cfg.iat_threshold_offset40, cfg.max_pump_duty, cfg.failsafe_flags};
  return cfg.checksum == simpleXorChecksum(data, 7);
}

inline bool unpackMethConfigBroadcast(const CanFrame& frame, MethConfigBroadcast& out) {
  if (frame.id != ID_METH_CONFIG_BROADCAST || frame.dlc < 8) return false;
  out.version = frame.data[0];
  out.desired_armed = frame.data[1];
  out.ratio_percent = frame.data[2];
  out.boost_trigger_kpa = frame.data[3];
  out.iat_threshold_offset40 = frame.data[4];
  out.max_pump_duty = frame.data[5];
  out.failsafe_flags = frame.data[6];
  out.checksum = frame.data[7];
  return true;
}

inline bool unpackMethConfigRequest(const CanFrame& frame, MethConfigRequest& out) {
  if (frame.id != ID_METH_CONFIG_REQUEST || frame.dlc < 1) return false;
  out.reason = frame.data[0];
  return true;
}

inline bool unpackMethConfigAck(const CanFrame& frame, MethConfigAck& out) {
  if (frame.id != ID_METH_CONFIG_ACK || frame.dlc < 4) return false;
  out.accepted_version = frame.data[0];
  out.status = frame.data[1];
  out.reject_reason = frame.data[2];
  out.active_ratio_percent = frame.data[3];
  return true;
}

inline CanFrame packMethConfigBroadcast(const MethConfigBroadcast& cfgInput) {
  MethConfigBroadcast cfg = cfgInput;
  const uint8_t data[7] = {
      cfg.version, cfg.desired_armed, cfg.ratio_percent, cfg.boost_trigger_kpa, cfg.iat_threshold_offset40, cfg.max_pump_duty, cfg.failsafe_flags};
  cfg.checksum = simpleXorChecksum(data, 7);

  CanFrame frame{};
  frame.id = ID_METH_CONFIG_BROADCAST;
  frame.dlc = 8;
  frame.data[0] = cfg.version;
  frame.data[1] = cfg.desired_armed;
  frame.data[2] = cfg.ratio_percent;
  frame.data[3] = cfg.boost_trigger_kpa;
  frame.data[4] = cfg.iat_threshold_offset40;
  frame.data[5] = cfg.max_pump_duty;
  frame.data[6] = cfg.failsafe_flags;
  frame.data[7] = cfg.checksum;
  return frame;
}

inline CanFrame packMethConfigRequest(uint8_t reason) {
  CanFrame frame{};
  frame.id = ID_METH_CONFIG_REQUEST;
  frame.dlc = 1;
  frame.data[0] = reason;
  return frame;
}

inline CanFrame packMethConfigAck(uint8_t acceptedVersion, uint8_t status, uint8_t rejectReason, uint8_t activeRatioPercent) {
  CanFrame frame{};
  frame.id = ID_METH_CONFIG_ACK;
  frame.dlc = 4;
  frame.data[0] = acceptedVersion;
  frame.data[1] = status;
  frame.data[2] = rejectReason;
  frame.data[3] = activeRatioPercent;
  return frame;
}

inline CanFrame packTaillightBrightness(uint8_t brightness) {
  CanFrame frame{};
  frame.id = ID_TAILLIGHT_COMMAND;
  frame.dlc = 2;
  frame.data[0] = taillight_command::SET_BRIGHTNESS;
  frame.data[1] = brightness;
  return frame;
}

inline CanFrame packTaillightOverride(uint8_t leftState, uint8_t rightState) {
  CanFrame frame{};
  frame.id = ID_TAILLIGHT_COMMAND;
  frame.dlc = 3;
  frame.data[0] = taillight_command::SET_OVERRIDE;
  frame.data[1] = leftState;
  frame.data[2] = rightState;
  return frame;
}

inline CanFrame packTaillightClearOverride() {
  CanFrame frame{};
  frame.id = ID_TAILLIGHT_COMMAND;
  frame.dlc = 1;
  frame.data[0] = taillight_command::CLEAR_OVERRIDE;
  return frame;
}

inline CanFrame packTaillightCustomAnimation(uint8_t animId, uint16_t durationMs, uint8_t param0, uint8_t param1) {
  CanFrame frame{};
  frame.id = ID_TAILLIGHT_COMMAND;
  frame.dlc = 6;
  frame.data[0] = taillight_command::TRIGGER_CUSTOM_ANIMATION;
  frame.data[1] = animId;
  encodeU16BE(durationMs, frame.data[2], frame.data[3]);
  frame.data[4] = param0;
  frame.data[5] = param1;
  return frame;
}

inline CanFrame packMethArm(bool armed) {
  CanFrame frame{};
  frame.id = ID_ENGINE_METH_COMMAND;
  frame.dlc = 2;
  frame.data[0] = meth_command::ARM;
  frame.data[1] = armed ? 1 : 0;
  return frame;
}

inline CanFrame packMethManualTest(uint8_t duty) {
  CanFrame frame{};
  frame.id = ID_ENGINE_METH_COMMAND;
  frame.dlc = 2;
  frame.data[0] = meth_command::MANUAL_TEST_DUTY;
  frame.data[1] = duty;
  return frame;
}

inline CanFrame packMethStopManualTest() {
  CanFrame frame{};
  frame.id = ID_ENGINE_METH_COMMAND;
  frame.dlc = 1;
  frame.data[0] = meth_command::STOP_MANUAL_TEST;
  return frame;
}

inline CanFrame packMethSetBoostThreshold(uint8_t kpa) {
  CanFrame frame{};
  frame.id = ID_ENGINE_METH_COMMAND;
  frame.dlc = 2;
  frame.data[0] = meth_command::SET_BOOST_TRIGGER;
  frame.data[1] = kpa;
  return frame;
}

inline CanFrame packMethSetIatThreshold(int iatC) {
  CanFrame frame{};
  frame.id = ID_ENGINE_METH_COMMAND;
  frame.dlc = 2;
  frame.data[0] = meth_command::SET_IAT_THRESHOLD;
  frame.data[1] = tempToOffset40(iatC);
  return frame;
}

inline CanFrame packMethClearFaults() {
  CanFrame frame{};
  frame.id = ID_ENGINE_METH_COMMAND;
  frame.dlc = 1;
  frame.data[0] = meth_command::CLEAR_FAULTS;
  return frame;
}

}  // namespace can_protocol
