#include <cmath>
#include <Unity.h>

#include "sensors/PressureMath.hpp"

namespace {
float adcNode(float sensorVoltage) { return sensorVoltage / 1.5f; }
}

void setUp() {}
void tearDown() {}

void test_voltage_to_psi_and_divider_scaling() {
  sensors::pressure_math::Config cfg{};
  cfg.enabled = true;
  auto zero = sensors::pressure_math::evaluate(adcNode(0.5f), cfg, NAN);
  auto full = sensors::pressure_math::evaluate(adcNode(4.5f), cfg, NAN);
  auto mid = sensors::pressure_math::evaluate(adcNode(2.5f), cfg, NAN);
  TEST_ASSERT_TRUE(zero.valid);
  TEST_ASSERT_FLOAT_WITHIN(0.2f, 0.0f, zero.raw_psi);
  TEST_ASSERT_FLOAT_WITHIN(0.2f, 100.0f, full.raw_psi);
  TEST_ASSERT_FLOAT_WITHIN(0.2f, 50.0f, mid.raw_psi);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 4.5f, sensors::pressure_math::sensorVoltageFromNode(adcNode(4.5f), cfg));
}

void test_fault_detection_and_smoothing() {
  sensors::pressure_math::Config cfg{};
  cfg.enabled = true;
  auto open = sensors::pressure_math::evaluate(adcNode(4.95f), cfg, NAN);
  auto shortGnd = sensors::pressure_math::evaluate(adcNode(0.05f), cfg, NAN);
  auto shortVcc = sensors::pressure_math::evaluate(adcNode(5.2f), cfg, NAN);
  auto smooth = sensors::pressure_math::evaluate(adcNode(3.5f), cfg, 20.0f);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(sensors::pressure_math::Fault::OpenCircuit), static_cast<uint8_t>(open.fault));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(sensors::pressure_math::Fault::ShortToGround), static_cast<uint8_t>(shortGnd.fault));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(sensors::pressure_math::Fault::ShortToVcc), static_cast<uint8_t>(shortVcc.fault));
  TEST_ASSERT_TRUE(smooth.valid);
  TEST_ASSERT_TRUE(smooth.filtered_psi > 20.0f);
}

void test_calibration_and_invalid_adc_are_safe() {
  sensors::pressure_math::Config cfg{};
  cfg.enabled = true;
  cfg.calibration_scale = 1.1f;
  cfg.calibration_offset_psi = 5.0f;
  auto calibrated = sensors::pressure_math::evaluate(adcNode(2.5f), cfg, NAN);
  auto badAdc = sensors::pressure_math::evaluate(4.0f, cfg, NAN);
  TEST_ASSERT_TRUE(calibrated.valid);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 60.0f, calibrated.raw_psi);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(sensors::pressure_math::Fault::AdcError), static_cast<uint8_t>(badAdc.fault));
}

int main(int argc, char** argv) {
  UNITY_BEGIN();
  RUN_TEST(test_voltage_to_psi_and_divider_scaling);
  RUN_TEST(test_fault_detection_and_smoothing);
  RUN_TEST(test_calibration_and_invalid_adc_are_safe);
  return UNITY_END();
}
