#include <Unity.h>

#include "tach/TachMath.hpp"

void setUp() {}
void tearDown() {}

void test_rpm_and_frequency_conversion_modes() {
  TEST_ASSERT_EQUAL_UINT32(400, ccm::tach::math::rpmToFrequencyHz(6000, 0));
  TEST_ASSERT_EQUAL_UINT32(200, ccm::tach::math::rpmToFrequencyHz(6000, 1));
  TEST_ASSERT_EQUAL_UINT16(6000, ccm::tach::math::frequencyHzToRpm(400, 0));
  TEST_ASSERT_EQUAL_UINT16(6000, ccm::tach::math::frequencyHzToRpm(200, 1));
}

void test_sweep_generation_and_clamping() {
  const auto sweep = ccm::tach::math::generateSweep(3000, 1000);
  TEST_ASSERT_EQUAL_UINT(8, sweep.size());
  TEST_ASSERT_EQUAL_UINT16(0, sweep.front());
  TEST_ASSERT_EQUAL_UINT16(0, sweep.back());
  TEST_ASSERT_EQUAL_UINT16(9000, ccm::tach::math::clampRpm(12000));
}

void test_output_enable_and_smoothing() {
  TEST_ASSERT_EQUAL_UINT32(0, ccm::tach::math::rpmToFrequencyHz(3000, 0, false));
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 1500.0f, ccm::tach::math::applySmoothing(0.0f, 3000, 0.5f));
}

int main(int argc, char** argv) {
  UNITY_BEGIN();
  RUN_TEST(test_rpm_and_frequency_conversion_modes);
  RUN_TEST(test_sweep_generation_and_clamping);
  RUN_TEST(test_output_enable_and_smoothing);
  return UNITY_END();
}
