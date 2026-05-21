#include <cstring>
#include <Unity.h>

#include "settings/AppSettings.hpp"
#include "settings/SettingsValidation.hpp"

void setUp() {}
void tearDown() {}

void test_default_settings_are_safe() {
  settings::AppSettings app{};
  const auto normalized = settings::normalizeSettings(app);
  TEST_ASSERT_EQUAL_UINT8(50, normalized.meth_selected_ratio_percent);
  TEST_ASSERT_EQUAL_UINT8(1, normalized.knock_response_mode);
  TEST_ASSERT_TRUE(normalized.knock_critical_threshold_count >= normalized.knock_warning_threshold_count);
}

void test_invalid_settings_are_clamped() {
  settings::AppSettings app{};
  app.meth_selected_ratio_percent = 200;
  app.knock_warning_threshold_count = 0;
  app.knock_critical_threshold_count = 0;
  app.knock_event_cooldown_ms = 1;
  app.pressure_sensor_min_v = 0.0f;
  app.pressure_sensor_max_v = 0.0f;
  const auto normalized = settings::normalizeSettings(app);
  TEST_ASSERT_EQUAL_UINT8(100, normalized.meth_selected_ratio_percent);
  TEST_ASSERT_EQUAL_UINT8(1, normalized.knock_warning_threshold_count);
  TEST_ASSERT_TRUE(normalized.knock_critical_threshold_count >= 1);
  TEST_ASSERT_EQUAL_UINT16(50, normalized.knock_event_cooldown_ms);
  TEST_ASSERT_TRUE(normalized.pressure_sensor_max_v > normalized.pressure_sensor_min_v);
}

void test_unsafe_modes_require_confirmation_and_values_persist() {
  settings::AppSettings current{};
  settings::AppSettings proposed = current;
  proposed.knock_response_mode = 3;
  proposed.thermistor_pullup_ohms = 15000.0f;
  proposed.led_ch_color[0] = 0x123456;
  proposed.tach_scaling_mode = 1;
  TEST_ASSERT_TRUE(settings::unsafeSettingsRequireConfirmation(current, proposed));
  const auto normalized = settings::normalizeSettings(proposed);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 15000.0f, normalized.thermistor_pullup_ohms);
  TEST_ASSERT_EQUAL_HEX32(0x123456, normalized.led_ch_color[0]);
  TEST_ASSERT_EQUAL_UINT8(1, normalized.tach_scaling_mode);
}

int main(int argc, char** argv) {
  UNITY_BEGIN();
  RUN_TEST(test_default_settings_are_safe);
  RUN_TEST(test_invalid_settings_are_clamped);
  RUN_TEST(test_unsafe_modes_require_confirmation_and_values_persist);
  return UNITY_END();
}
