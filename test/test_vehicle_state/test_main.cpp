#include <Unity.h>

#include "state/StateHelpers.hpp"
#include "state/UiStateModel.hpp"

void setUp() {}
void tearDown() {}

void test_default_state_values_and_validity_flags() {
  state::VehicleState s{};
  TEST_ASSERT_EQUAL_UINT16(0, s.rpm);
  TEST_ASSERT_FALSE(s.intake_temp_valid);
  TEST_ASSERT_FALSE(s.meth_online);
}

void test_node_timeout_and_stale_detection() {
  TEST_ASSERT_TRUE(state::nodeTimedOut(1000, 100, 500));
  state::VehicleState s{};
  s.last_meth_ms = 100;
  TEST_ASSERT_TRUE(state::methCanLossDisarms(s, 500, 250));
}

void test_fault_helpers_and_ui_model() {
  state::VehicleState s{};
  s.can_online = true;
  s.meth_online = true;
  s.taillight_online = true;
  s.intake_temp_valid = true;
  s.meth_pressure_valid = true;
  s.meth_state = state::MethState::SPRAYING;
  s.knock_warning_active = true;
  s.meth_pump_duty = 55;
  s.meth_tank_level = 80;
  const auto ui = state::buildUiStateData(s);
  TEST_ASSERT_TRUE(state::methSafetyInputsValid(s));
  TEST_ASSERT_TRUE(ui.meth_available);
  TEST_ASSERT_TRUE(ui.meth_active);
  TEST_ASSERT_TRUE(ui.knock_alert);
  TEST_ASSERT_EQUAL_UINT8(55, ui.pump_duty);
}

int main(int argc, char** argv) {
  UNITY_BEGIN();
  RUN_TEST(test_default_state_values_and_validity_flags);
  RUN_TEST(test_node_timeout_and_stale_detection);
  RUN_TEST(test_fault_helpers_and_ui_model);
  return UNITY_END();
}
