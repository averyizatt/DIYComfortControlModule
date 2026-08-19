#include <Unity.h>

#include "can/CanFrameBuilders.hpp"
#include "can/MicroSquirtProtocol.hpp"
#include "can_contract/can_protocol.h"

void setUp() {}
void tearDown() {}

void test_taillight_and_meth_frames_remain_compatible() {
  const auto brightness = can_protocol::packTaillightBrightness(120);
  const auto manual = can_protocol::packMethManualTest(55);
  TEST_ASSERT_EQUAL_HEX16(0x101, brightness.id);
  TEST_ASSERT_EQUAL_UINT8(2, brightness.dlc);
  TEST_ASSERT_EQUAL_UINT8(120, brightness.data[1]);
  TEST_ASSERT_EQUAL_HEX16(0x301, manual.id);
  TEST_ASSERT_EQUAL_UINT8(2, manual.dlc);
  TEST_ASSERT_EQUAL_UINT8(can_protocol::meth_command::MANUAL_TEST_DUTY, manual.data[0]);
}

void test_invalid_dlc_and_endian_handling() {
  can_protocol::EngineMethState meth{};
  can_protocol::CanFrame bad{};
  bad.id = can_protocol::ID_ENGINE_METH_STATE;
  bad.dlc = 7;
  TEST_ASSERT_FALSE(can_protocol::unpackEngineMethState(bad, meth));

  uint8_t hi = 0;
  uint8_t lo = 0;
  can_protocol::encodeU16BE(0x1234, hi, lo);
  TEST_ASSERT_EQUAL_HEX8(0x12, hi);
  TEST_ASSERT_EQUAL_HEX8(0x34, lo);
  TEST_ASSERT_EQUAL_HEX16(0x1234, can_protocol::decodeU16BE(hi, lo));
}

void test_knock_and_sensor_frames_pack_expected_bytes() {
  state::VehicleState s{};
  s.knock_enabled = true;
  s.knock_signal_valid = true;
  s.knock_warning_active = true;
  s.knock_energy = 999.0f;
  s.knock_baseline = 25.0f;
  s.knock_threshold = 40.0f;
  s.knock_event_count = 3;
  s.knock_last_event_rpm = 6800;
  s.knock_last_event_boost_kpa = 180;
  s.oil_pressure_psi = 61.0f;
  s.fuel_pressure_psi = 49.0f;
  s.meth_pressure_psi = 125.0f;
  s.boost_ref_pressure_psi = 12.0f;
  s.outside_temp = 22.0f;
  s.cabin_temp = 24.0f;
  s.analog_sensor_fault_flags = 0x34;

  can_protocol::CanFrame knock{};
  can_protocol::CanFrame sensor{};
  canbus::packKnockState(s, knock);
  canbus::packEngineSensorExt(s, sensor);
  TEST_ASSERT_EQUAL_HEX16(0x307, knock.id);
  TEST_ASSERT_EQUAL_UINT8(8, knock.dlc);
  TEST_ASSERT_EQUAL_UINT8(255, knock.data[1]);
  TEST_ASSERT_EQUAL_UINT8(68, knock.data[5]);
  TEST_ASSERT_EQUAL_HEX16(0x303, sensor.id);
  TEST_ASSERT_EQUAL_UINT8(61, sensor.data[0]);
  TEST_ASSERT_EQUAL_UINT8(0x34, sensor.data[6]);
}

void test_meth_config_broadcast_has_required_fields_and_checksum() {
  state::VehicleState s{};
  s.meth_config_version = 7;
  s.meth_desired_armed = true;
  s.meth_selected_ratio_percent = 60;

  const auto frame = canbus::packMethConfigState(s);
  can_protocol::MethConfigBroadcast msg{};
  TEST_ASSERT_TRUE(can_protocol::unpackMethConfigBroadcast(frame, msg));
  TEST_ASSERT_EQUAL_UINT8(7, msg.version);
  TEST_ASSERT_EQUAL_UINT8(1, msg.desired_armed);
  TEST_ASSERT_EQUAL_UINT8(60, msg.ratio_percent);
  TEST_ASSERT_EQUAL_UINT8(114, msg.boost_trigger_kpa);
  TEST_ASSERT_EQUAL_UINT8(can_protocol::tempToOffset40(50), msg.iat_threshold_offset40);
  TEST_ASSERT_EQUAL_UINT8(100, msg.max_pump_duty);
  TEST_ASSERT_EQUAL_UINT8(0x03, msg.failsafe_flags);
  TEST_ASSERT_TRUE(can_protocol::validateMethConfigChecksum(msg));
}

void test_microsquirt_dash_broadcast_decodes_big_endian_values() {
  can_protocol::CanFrame frame{};
  frame.id = microsquirt::kDashBaseId;
  frame.dlc = 8;
  frame.data[0] = 0x05; frame.data[1] = 0xDC;  // 150.0 kPa
  frame.data[2] = 0x0D; frame.data[3] = 0xAC;  // 3500 rpm
  frame.data[4] = 0x03; frame.data[5] = 0x56;  // 85.4 F
  frame.data[6] = 0x02; frame.data[7] = 0x26;  // 55.0 %
  microsquirt::LiveData data{};
  TEST_ASSERT_TRUE(microsquirt::decodeDash(frame, data, 100));
  TEST_ASSERT_EQUAL_UINT16(3500, data.rpm);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 150.0f, data.map_kpa);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 85.4f, data.coolant_f);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 55.0f, data.tps_percent);
}

void test_microsquirt_realtime_boost_uses_baro_and_rejects_bad_dlc() {
  can_protocol::CanFrame frame{};
  frame.id = microsquirt::kRecommendedRealtimeBaseId + 2;
  frame.dlc = 8;
  frame.data[0] = 0x03; frame.data[1] = 0x70;  // 88.0 kPa baro
  frame.data[2] = 0x06; frame.data[3] = 0x40;  // 160.0 kPa MAP
  frame.data[4] = 0x02; frame.data[5] = 0xEE;  // 75.0 F MAT
  frame.data[6] = 0x03; frame.data[7] = 0x84;  // 90.0 F CLT
  microsquirt::LiveData data{};
  TEST_ASSERT_TRUE(microsquirt::decodeRealtime(
      frame, microsquirt::kRecommendedRealtimeBaseId, data, 200));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 72.0f, microsquirt::gaugeBoostKpa(data));
  frame.dlc = 7;
  TEST_ASSERT_FALSE(microsquirt::decodeRealtime(
      frame, microsquirt::kRecommendedRealtimeBaseId, data, 201));
  TEST_ASSERT_EQUAL_UINT32(1, data.invalid_count);
}

int main(int argc, char** argv) {
  UNITY_BEGIN();
  RUN_TEST(test_taillight_and_meth_frames_remain_compatible);
  RUN_TEST(test_invalid_dlc_and_endian_handling);
  RUN_TEST(test_knock_and_sensor_frames_pack_expected_bytes);
  RUN_TEST(test_meth_config_broadcast_has_required_fields_and_checksum);
  RUN_TEST(test_microsquirt_dash_broadcast_decodes_big_endian_values);
  RUN_TEST(test_microsquirt_realtime_boost_uses_baro_and_rejects_bad_dlc);
  return UNITY_END();
}
