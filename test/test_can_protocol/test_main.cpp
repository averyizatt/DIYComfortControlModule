#include <Unity.h>

#include "can/CanFrameBuilders.hpp"
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

int main(int argc, char** argv) {
  UNITY_BEGIN();
  RUN_TEST(test_taillight_and_meth_frames_remain_compatible);
  RUN_TEST(test_invalid_dlc_and_endian_handling);
  RUN_TEST(test_knock_and_sensor_frames_pack_expected_bytes);
  return UNITY_END();
}
