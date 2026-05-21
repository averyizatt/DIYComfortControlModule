#include <string>
#include <Unity.h>

#include "web/StateApiModel.hpp"
#include "web/WebApiLogic.hpp"
#include "../mocks/MockWebRequestHandler.hpp"

void setUp() {}
void tearDown() {}

void test_state_model_contains_expected_fields() {
  state::VehicleState s{};
  s.rpm = 3200;
  s.battery_voltage = 13.8f;
  s.meth_state = state::MethState::ARMED;
  s.meth_pump_duty = 20;
  s.knock_enabled = true;
  s.analog_sensor_fault_flags = 0x20;
  s.meth_manual_test_reject_reason = 3;
  const auto api = web::buildStateApiData(s);
  TEST_ASSERT_EQUAL_UINT16(3200, api.rpm);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 13.8f, api.battery_voltage);
  TEST_ASSERT_EQUAL_UINT8(20, api.pump_duty);
  TEST_ASSERT_EQUAL_STRING("cooldown", api.meth_manual_test_reject_reason);
}

void test_command_validation_enforces_safety() {
  TEST_ASSERT_TRUE(web::manualTestRequiresConfirmation(25, false));
  TEST_ASSERT_TRUE(web::unsafeKnockResponseRequiresConfirmation(2));
  TEST_ASSERT_TRUE(web::ledChannelInRange(2));
  TEST_ASSERT_FALSE(web::ledChannelInRange(4));
  TEST_ASSERT_TRUE(web::knockSimulationAllowed(false, true));
  TEST_ASSERT_FALSE(web::knockSimulationAllowed(false, false));
  TEST_ASSERT_EQUAL_STRING("confirmation_required", web::manualTestRejectReasonText(6));
}

void test_malformed_json_is_rejected_safely() {
  MockWebRequestHandler handler{};
  handler.rejectMalformedJson("not-json");
  TEST_ASSERT_EQUAL_INT(400, handler.status);
  TEST_ASSERT_EQUAL_STRING("{\"error\":\"invalid_json\"}", handler.body.c_str());
}

int main(int argc, char** argv) {
  UNITY_BEGIN();
  RUN_TEST(test_state_model_contains_expected_fields);
  RUN_TEST(test_command_validation_enforces_safety);
  RUN_TEST(test_malformed_json_is_rejected_safely);
  return UNITY_END();
}
