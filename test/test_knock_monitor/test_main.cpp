#include <Unity.h>

#include "knock/KnockLogic.hpp"

void setUp() {}
void tearDown() {}

void test_threshold_and_gating_logic() {
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 33.0f, knock::logic::computeThreshold(10.0f, 2.5f, 8.0f));
  TEST_ASSERT_FALSE(knock::logic::detectionWindowActive(true, true, false, 100.0f, 120.0f, 3000, 2500));
  TEST_ASSERT_FALSE(knock::logic::detectionWindowActive(true, true, false, 130.0f, 120.0f, 2000, 2500));
  TEST_ASSERT_TRUE(knock::logic::detectionWindowActive(true, true, false, 130.0f, 120.0f, 3000, 2500));
}

void test_cooldown_warning_and_critical_behavior() {
  uint32_t lastDecayMs = 0;
  TEST_ASSERT_TRUE(knock::logic::shouldRegisterEvent(40.0f, 30.0f, 1000, 0, 250));
  TEST_ASSERT_FALSE(knock::logic::shouldRegisterEvent(40.0f, 30.0f, 100, 0, 250));
  TEST_ASSERT_EQUAL_UINT8(3, knock::logic::decayEventWindow(3, 1000, lastDecayMs));
  TEST_ASSERT_EQUAL_UINT8(2, knock::logic::decayEventWindow(3, 2000, lastDecayMs));
  TEST_ASSERT_TRUE(knock::logic::warningActive(2, 2));
  TEST_ASSERT_TRUE(knock::logic::criticalActive(4, 4));
}

void test_signal_health_and_demo_modes() {
  knock::logic::SignalHealthState state{};
  for (int i = 0; i < 25; ++i) {
    knock::logic::updateSignalHealth(state, 4095, 400.0f, 1200);
  }
  knock::logic::updateSignalHealth(state, 4095, 400.0f, 2200);
  TEST_ASSERT_TRUE(state.clipping_detected);

  knock::logic::SignalHealthState disconnected{};
  for (int ms = 0; ms <= 3000; ms += 100) {
    knock::logic::updateSignalHealth(disconnected, 2048, 0.1f, static_cast<uint32_t>(ms));
  }
  TEST_ASSERT_TRUE(disconnected.sensor_fault);
  TEST_ASSERT_TRUE(knock::logic::simulationAllowed(true, false));
  TEST_ASSERT_TRUE(knock::logic::forceMethEnable(2, true, true));
  TEST_ASSERT_TRUE(knock::logic::safetyShutdown(3, true));
}

int main(int argc, char** argv) {
  UNITY_BEGIN();
  RUN_TEST(test_threshold_and_gating_logic);
  RUN_TEST(test_cooldown_warning_and_critical_behavior);
  RUN_TEST(test_signal_health_and_demo_modes);
  return UNITY_END();
}
