#include <Unity.h>

#include "meth/MethSafetyLogic.hpp"

void setUp() {}
void tearDown() {}

void test_boots_off_disarmed() {
  state::VehicleState s{};
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(state::MethState::OFF), static_cast<uint8_t>(s.meth_state));
  TEST_ASSERT_FALSE(s.meth_desired_armed);
}

void test_arming_only_works_when_sensors_valid() {
  state::VehicleState s{};
  s.meth_online = true;
  s.meth_tank_level = 50;
  TEST_ASSERT_FALSE(meth::canArm(s));
  s.intake_temp_valid = true;
  s.meth_pressure_valid = true;
  TEST_ASSERT_TRUE(meth::canArm(s));
}

void test_pump_duty_zero_when_disarmed_or_faulted() {
  TEST_ASSERT_EQUAL_UINT8(0, meth::progressivePumpDuty(false, false, false, true, 150.0f, 100.0f, 200.0f, 80));
  TEST_ASSERT_EQUAL_UINT8(0, meth::progressivePumpDuty(true, true, false, true, 150.0f, 100.0f, 200.0f, 80));
}

void test_pump_ramps_progressively_with_boost() {
  TEST_ASSERT_EQUAL_UINT8(0, meth::progressivePumpDuty(true, false, false, true, 100.0f, 100.0f, 200.0f, 80));
  TEST_ASSERT_EQUAL_UINT8(40, meth::progressivePumpDuty(true, false, false, true, 150.0f, 100.0f, 200.0f, 80));
  TEST_ASSERT_EQUAL_UINT8(80, meth::progressivePumpDuty(true, false, false, true, 220.0f, 100.0f, 200.0f, 80));
}

void test_manual_test_requires_confirmation_and_times_out() {
  state::VehicleState s{};
  s.meth_online = true;
  auto decision = meth::evaluateManualTestRequest(s, 30, false, 5000, 0);
  TEST_ASSERT_FALSE(decision.allowed);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(meth::ManualTestRejectReason::CONFIRMATION_REQUIRED),
                          static_cast<uint8_t>(decision.reason));
  TEST_ASSERT_TRUE(meth::manualTestTimedOut(true, 6001, 0));
}

void test_low_tank_and_fault_latching_are_safe() {
  TEST_ASSERT_EQUAL_UINT8(0, meth::progressivePumpDuty(true, false, true, true, 180.0f, 100.0f, 200.0f, 80));
  meth::FaultLatchState state{};
  state = meth::updateFaultLatch(state, false, true, false, false, false, false);
  TEST_ASSERT_TRUE(state.fault_active);
  TEST_ASSERT_TRUE(state.critical_latched);
  state = meth::updateFaultLatch(state, false, false, false, false, true, false);
  TEST_ASSERT_TRUE(state.critical_latched);
  state = meth::updateFaultLatch(state, false, false, false, false, true, true);
  TEST_ASSERT_FALSE(state.critical_latched);
}

void test_can_loss_disarms_and_fault_inputs_work() {
  state::VehicleState s{};
  s.last_meth_ms = 100;
  TEST_ASSERT_TRUE(state::methCanLossDisarms(s, 400, 250));
  auto state = meth::updateFaultLatch({}, false, false, true, false, false, false);
  TEST_ASSERT_TRUE(state.critical_latched);
  state = meth::updateFaultLatch({}, false, false, false, true, false, false);
  TEST_ASSERT_TRUE(state.critical_latched);
}

int main(int argc, char** argv) {
  UNITY_BEGIN();
  RUN_TEST(test_boots_off_disarmed);
  RUN_TEST(test_arming_only_works_when_sensors_valid);
  RUN_TEST(test_pump_duty_zero_when_disarmed_or_faulted);
  RUN_TEST(test_pump_ramps_progressively_with_boost);
  RUN_TEST(test_manual_test_requires_confirmation_and_times_out);
  RUN_TEST(test_low_tank_and_fault_latching_are_safe);
  RUN_TEST(test_can_loss_disarms_and_fault_inputs_work);
  return UNITY_END();
}
