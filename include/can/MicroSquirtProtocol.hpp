#pragma once

#include <cstdint>

#include "can_contract/can_protocol.h"

namespace microsquirt {

constexpr uint16_t kDashBaseId = 0x5E8;
constexpr uint16_t kDashLastId = 0x5EC;
constexpr uint16_t kDefaultRealtimeBaseId = 0x5F0;
constexpr uint16_t kRecommendedRealtimeBaseId = 0x700;
constexpr uint8_t kRealtimeGroupCount = 64;
constexpr uint32_t kFreshTimeoutMs = 500;

enum ValidField : uint64_t {
  VALID_RPM = 1ULL << 0,
  VALID_MAP = 1ULL << 1,
  VALID_BARO = 1ULL << 2,
  VALID_CLT = 1ULL << 3,
  VALID_MAT = 1ULL << 4,
  VALID_TPS = 1ULL << 5,
  VALID_BATTERY = 1ULL << 6,
  VALID_AFR1 = 1ULL << 7,
  VALID_AFR2 = 1ULL << 8,
  VALID_AFR_TARGET1 = 1ULL << 9,
  VALID_SPARK = 1ULL << 10,
  VALID_PW1 = 1ULL << 11,
  VALID_PW2 = 1ULL << 12,
  VALID_KNOCK = 1ULL << 13,
  VALID_EGO1 = 1ULL << 14,
  VALID_EGO2 = 1ULL << 15,
  VALID_WARMUP_COR = 1ULL << 16,
  VALID_TOTAL_COR = 1ULL << 17,
  VALID_VE1 = 1ULL << 18,
  VALID_VE2 = 1ULL << 19,
  VALID_DWELL = 1ULL << 20,
  VALID_FUEL_LOAD = 1ULL << 21,
  VALID_IGN_LOAD = 1ULL << 22,
  VALID_ECU_SECONDS = 1ULL << 23,
};

struct LiveData {
  uint64_t valid = 0;
  uint16_t rpm = 0;
  uint16_t ecu_seconds = 0;
  float map_kpa = 0.0f;
  float baro_kpa = 0.0f;
  float coolant_f = 0.0f;
  float mat_f = 0.0f;
  float tps_percent = 0.0f;
  float battery_v = 0.0f;
  float afr1 = 0.0f;
  float afr2 = 0.0f;
  float afr_target1 = 0.0f;
  float spark_advance_deg = 0.0f;
  float pulse_width_1_ms = 0.0f;
  float pulse_width_2_ms = 0.0f;
  float knock_percent = 0.0f;
  float ego_correction_1 = 0.0f;
  float ego_correction_2 = 0.0f;
  float warmup_correction = 0.0f;
  float total_correction = 0.0f;
  float ve1 = 0.0f;
  float ve2 = 0.0f;
  float dwell_ms = 0.0f;
  float fuel_load = 0.0f;
  float ignition_load = 0.0f;
  uint8_t engine_flags = 0;
  uint8_t status[7]{};
  int16_t generic_sensor_x10[8]{};
  uint32_t last_update_ms = 0;
  uint32_t last_group_ms[kRealtimeGroupCount]{};
  uint16_t last_id = 0;
  uint32_t frame_count = 0;
  uint32_t invalid_count = 0;
};

inline uint16_t u16be(const uint8_t* data) {
  return static_cast<uint16_t>((static_cast<uint16_t>(data[0]) << 8U) | data[1]);
}

inline int16_t s16be(const uint8_t* data) {
  return static_cast<int16_t>(u16be(data));
}

inline float x10(int16_t raw) { return static_cast<float>(raw) / 10.0f; }
inline float x1000(uint16_t raw) { return static_cast<float>(raw) / 1000.0f; }

inline bool isDashId(uint16_t id) {
  return id >= kDashBaseId && id <= kDashLastId;
}

inline bool isRealtimeId(uint16_t id, uint16_t baseId) {
  return baseId <= 0x7C0U && id >= baseId &&
         id < static_cast<uint16_t>(baseId + kRealtimeGroupCount);
}

inline bool isMicroSquirtId(uint16_t id, uint16_t configuredBase = kRecommendedRealtimeBaseId) {
  return isDashId(id) || isRealtimeId(id, configuredBase) ||
         isRealtimeId(id, kDefaultRealtimeBaseId);
}

inline bool decodeDash(const can_protocol::CanFrame& frame, LiveData& out, uint32_t nowMs) {
  if (!isDashId(frame.id) || frame.dlc != 8U) {
    if (isDashId(frame.id)) ++out.invalid_count;
    return false;
  }

  switch (static_cast<uint16_t>(frame.id - kDashBaseId)) {
    case 0:
      out.map_kpa = x10(s16be(&frame.data[0]));
      out.rpm = u16be(&frame.data[2]);
      out.coolant_f = x10(s16be(&frame.data[4]));
      out.tps_percent = x10(s16be(&frame.data[6]));
      out.valid |= VALID_MAP | VALID_RPM | VALID_CLT | VALID_TPS;
      break;
    case 1:
      out.pulse_width_1_ms = x1000(u16be(&frame.data[0]));
      out.pulse_width_2_ms = x1000(u16be(&frame.data[2]));
      out.mat_f = x10(s16be(&frame.data[4]));
      out.spark_advance_deg = x10(s16be(&frame.data[6]));
      out.valid |= VALID_PW1 | VALID_PW2 | VALID_MAT | VALID_SPARK;
      break;
    case 2:
      out.afr_target1 = static_cast<float>(frame.data[0]) / 10.0f;
      out.afr1 = static_cast<float>(frame.data[1]) / 10.0f;
      out.ego_correction_1 = x10(s16be(&frame.data[2]));
      out.valid |= VALID_AFR_TARGET1 | VALID_AFR1 | VALID_EGO1;
      break;
    case 3:
      out.battery_v = x10(s16be(&frame.data[0]));
      out.generic_sensor_x10[0] = s16be(&frame.data[2]);
      out.generic_sensor_x10[1] = s16be(&frame.data[4]);
      out.valid |= VALID_BATTERY;
      break;
    case 4:
      // VSS is not supplied by MS2/MicroSquirt in the simplified dataset.
      break;
    default:
      return false;
  }

  out.last_group_ms[frame.id - kDashBaseId] = nowMs;
  out.last_update_ms = nowMs;
  out.last_id = frame.id;
  ++out.frame_count;
  return true;
}

inline bool decodeRealtime(const can_protocol::CanFrame& frame, uint16_t baseId,
                           LiveData& out, uint32_t nowMs) {
  if (!isRealtimeId(frame.id, baseId) || frame.dlc != 8U) {
    if (isRealtimeId(frame.id, baseId)) ++out.invalid_count;
    return false;
  }
  const uint8_t group = static_cast<uint8_t>(frame.id - baseId);

  switch (group) {
    case 0:
      out.ecu_seconds = u16be(&frame.data[0]);
      out.pulse_width_1_ms = x1000(u16be(&frame.data[2]));
      out.pulse_width_2_ms = x1000(u16be(&frame.data[4]));
      out.rpm = u16be(&frame.data[6]);
      out.valid |= VALID_ECU_SECONDS | VALID_PW1 | VALID_PW2 | VALID_RPM;
      break;
    case 1:
      out.spark_advance_deg = x10(s16be(&frame.data[0]));
      out.engine_flags = frame.data[3];
      out.afr_target1 = static_cast<float>(frame.data[4]) / 10.0f;
      out.valid |= VALID_SPARK | VALID_AFR_TARGET1;
      break;
    case 2:
      out.baro_kpa = x10(s16be(&frame.data[0]));
      out.map_kpa = x10(s16be(&frame.data[2]));
      out.mat_f = x10(s16be(&frame.data[4]));
      out.coolant_f = x10(s16be(&frame.data[6]));
      out.valid |= VALID_BARO | VALID_MAP | VALID_MAT | VALID_CLT;
      break;
    case 3:
      out.tps_percent = x10(s16be(&frame.data[0]));
      out.battery_v = x10(s16be(&frame.data[2]));
      out.afr1 = x10(s16be(&frame.data[4]));
      out.afr2 = x10(s16be(&frame.data[6]));
      out.valid |= VALID_TPS | VALID_BATTERY | VALID_AFR1 | VALID_AFR2;
      break;
    case 4:
      out.knock_percent = x10(s16be(&frame.data[0]));
      out.ego_correction_1 = x10(s16be(&frame.data[2]));
      out.ego_correction_2 = x10(s16be(&frame.data[4]));
      out.valid |= VALID_KNOCK | VALID_EGO1 | VALID_EGO2;
      break;
    case 5:
      out.warmup_correction = x10(s16be(&frame.data[0]));
      out.valid |= VALID_WARMUP_COR;
      break;
    case 6:
      out.total_correction = x10(s16be(&frame.data[0]));
      out.ve1 = x10(s16be(&frame.data[2]));
      out.ve2 = x10(s16be(&frame.data[4]));
      out.valid |= VALID_TOTAL_COR | VALID_VE1 | VALID_VE2;
      break;
    case 8:
      out.fuel_load = x10(s16be(&frame.data[2]));
      out.valid |= VALID_FUEL_LOAD;
      break;
    case 9:
      out.dwell_ms = x10(static_cast<int16_t>(u16be(&frame.data[4])));
      out.valid |= VALID_DWELL;
      break;
    case 10:
      for (uint8_t i = 0; i < 7; ++i) out.status[i] = frame.data[i];
      break;
    case 11:
      out.ignition_load = x10(s16be(&frame.data[2]));
      out.valid |= VALID_IGN_LOAD;
      break;
    case 13:
    case 14: {
      const uint8_t first = static_cast<uint8_t>((group - 13U) * 4U);
      for (uint8_t i = 0; i < 4; ++i) {
        out.generic_sensor_x10[first + i] = s16be(&frame.data[i * 2U]);
      }
      break;
    }
    default:
      // The raw recorder preserves every other MS2-supported group losslessly.
      break;
  }

  out.last_group_ms[group] = nowMs;
  out.last_update_ms = nowMs;
  out.last_id = frame.id;
  ++out.frame_count;
  return true;
}

inline bool fresh(const LiveData& data, uint32_t nowMs,
                  uint32_t timeoutMs = kFreshTimeoutMs) {
  return data.last_update_ms != 0U && (nowMs - data.last_update_ms) <= timeoutMs;
}

inline float fahrenheitToCelsius(float valueF) {
  return (valueF - 32.0f) * (5.0f / 9.0f);
}

inline float gaugeBoostKpa(const LiveData& data) {
  if ((data.valid & (VALID_MAP | VALID_BARO)) != (VALID_MAP | VALID_BARO)) return 0.0f;
  return data.map_kpa - data.baro_kpa;
}

}  // namespace microsquirt
