#include <cstring>
#include <Unity.h>

#include "storage/LogFormatting.hpp"
#include "../mocks/MockLogWriter.hpp"

void setUp() {}
void tearDown() {}

void test_csv_header_and_row_formatting() {
  TEST_ASSERT_EQUAL_STRING("timestamp_ms,category,payload", storage::logfmt::csvHeader().c_str());
  const auto row = storage::logfmt::formatCsvLine(42, "meth", "pump=25");
  TEST_ASSERT_EQUAL_STRING("42,meth,pump=25", row.c_str());
}

void test_fault_formatting_and_immediate_flush() {
  const auto row = storage::logfmt::formatFaultLine(100, "NO_FLOW", "pump commanded", true);
  TEST_ASSERT_NOT_NULL(strstr(row.c_str(), "CRITICAL:NO_FLOW"));
  TEST_ASSERT_TRUE(storage::logfmt::shouldFlushImmediately(true));
}

void test_missing_sd_writer_does_not_crash_and_payload_is_clamped() {
  MockLogWriter writer{};
  writer.allow_writes = false;
  TEST_ASSERT_FALSE(writer.appendLine("/logs/test.csv", "line"));
  std::string payload(600, 'x');
  const auto clamped = storage::logfmt::clampPayload(payload);
  TEST_ASSERT_EQUAL_UINT(512, clamped.size());
}

int main(int argc, char** argv) {
  UNITY_BEGIN();
  RUN_TEST(test_csv_header_and_row_formatting);
  RUN_TEST(test_fault_formatting_and_immediate_flush);
  RUN_TEST(test_missing_sd_writer_does_not_crash_and_payload_is_clamped);
  return UNITY_END();
}
