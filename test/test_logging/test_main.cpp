#include <cstring>
#include <Unity.h>

#include "storage/LogFormatting.hpp"
#include "storage/TelemetryFormat.hpp"
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

void test_binary_telemetry_crc_detects_partial_or_corrupt_records() {
  storage::telemetry::Record record{};
  record.sequence = 42;
  record.timestamp_ms = 1234;
  record.can_id = 0x700;
  record.dlc = 8;
  for (uint8_t i = 0; i < 8; ++i) record.data[i] = i;
  storage::telemetry::finalize(record);
  TEST_ASSERT_TRUE(storage::telemetry::valid(record));
  record.data[4] ^= 0x80;
  TEST_ASSERT_FALSE(storage::telemetry::valid(record));
}

void test_binary_segment_header_is_one_sector_and_crc_protected() {
  storage::telemetry::SegmentHeader header{};
  header.session_id = 7;
  header.realtime_base_id = 0x700;
  storage::telemetry::initializeHeader(header);
  TEST_ASSERT_EQUAL_UINT32(512, sizeof(header));
  TEST_ASSERT_TRUE(storage::telemetry::valid(header));
  header.session_id++;
  TEST_ASSERT_FALSE(storage::telemetry::valid(header));
}

int main(int argc, char** argv) {
  UNITY_BEGIN();
  RUN_TEST(test_csv_header_and_row_formatting);
  RUN_TEST(test_fault_formatting_and_immediate_flush);
  RUN_TEST(test_missing_sd_writer_does_not_crash_and_payload_is_clamped);
  RUN_TEST(test_binary_telemetry_crc_detects_partial_or_corrupt_records);
  RUN_TEST(test_binary_segment_header_is_one_sector_and_crc_protected);
  return UNITY_END();
}
