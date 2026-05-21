#include <cmath>
#include <Unity.h>

#include "sensors/ThermistorMath.hpp"

void setUp() {}
void tearDown() {}

void test_voltage_to_resistance_and_temperature() {
  sensors::thermistor_math::Config cfg{};
  cfg.enabled = true;
  const float resistance = sensors::thermistor_math::resistanceFromVoltage(1.65f, cfg);
  TEST_ASSERT_FLOAT_WITHIN(5.0f, 10000.0f, resistance);
  const float cold = sensors::thermistor_math::steinhartHartTempC(30000.0f, cfg);
  const float hot = sensors::thermistor_math::steinhartHartTempC(3000.0f, cfg);
  TEST_ASSERT_TRUE(cold < hot);
}

void test_faults_and_smoothing_work() {
  sensors::thermistor_math::Config cfg{};
  cfg.enabled = true;
  auto open = sensors::thermistor_math::evaluate(3.2f, cfg, NAN);
  auto shorted = sensors::thermistor_math::evaluate(0.05f, cfg, NAN);
  auto smooth = sensors::thermistor_math::evaluate(1.2f, cfg, 10.0f);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(sensors::thermistor_math::Fault::OpenCircuit), static_cast<uint8_t>(open.fault));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(sensors::thermistor_math::Fault::ShortToGround), static_cast<uint8_t>(shorted.fault));
  TEST_ASSERT_TRUE(smooth.valid);
  TEST_ASSERT_TRUE(fabsf(smooth.filtered_temp_c - 10.0f) > 0.1f);
}

void test_out_of_range_and_lut_interpolation() {
  sensors::thermistor_math::LutPoint lut[] = {
      {30000.0f, -10.0f},
      {10000.0f, 25.0f},
      {3000.0f, 80.0f},
  };
  sensors::thermistor_math::Config cfg{};
  cfg.enabled = true;
  cfg.use_steinhart_hart = false;
  cfg.lut = lut;
  cfg.lut_size = 3;
  auto lutResult = sensors::thermistor_math::evaluate(1.65f, cfg, NAN);
  cfg.max_valid_temp_c = 20.0f;
  auto bad = sensors::thermistor_math::evaluate(1.65f, cfg, NAN);
  TEST_ASSERT_TRUE(lutResult.valid);
  TEST_ASSERT_FLOAT_WITHIN(2.0f, 25.0f, lutResult.raw_temp_c);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(sensors::thermistor_math::Fault::OutOfRange), static_cast<uint8_t>(bad.fault));
}

int main(int argc, char** argv) {
  UNITY_BEGIN();
  RUN_TEST(test_voltage_to_resistance_and_temperature);
  RUN_TEST(test_faults_and_smoothing_work);
  RUN_TEST(test_out_of_range_and_lut_interpolation);
  return UNITY_END();
}
