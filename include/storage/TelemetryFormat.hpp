#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace storage::telemetry {

constexpr uint16_t kRecordMagic = 0xC352;
constexpr uint8_t kFormatVersion = 2;
constexpr uint8_t kRecordCan = 1;
constexpr uint8_t kRecordFooter = 0xFE;
constexpr uint8_t kFlagProtected = 1U << 0;

inline uint32_t crc32(const void* data, size_t len) {
  const auto* bytes = static_cast<const uint8_t*>(data);
  uint32_t crc = 0xFFFFFFFFUL;
  for (size_t i = 0; i < len; ++i) {
    crc ^= bytes[i];
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc >> 1U) ^ (0xEDB88320UL &
            static_cast<uint32_t>(-static_cast<int32_t>(crc & 1U)));
    }
  }
  return ~crc;
}

#pragma pack(push, 1)
struct Record {
  uint16_t magic = kRecordMagic;
  uint8_t version = kFormatVersion;
  uint8_t type = kRecordCan;
  uint32_t sequence = 0;
  uint32_t timestamp_ms = 0;
  uint16_t can_id = 0;
  uint8_t dlc = 0;
  uint8_t flags = 0;
  uint8_t data[8]{};
  uint32_t dropped_before = 0;
  uint32_t crc = 0;
};

struct SegmentHeader {
  char magic[8]{};
  uint16_t format_version = kFormatVersion;
  uint16_t header_size = 512;
  uint32_t session_id = 0;
  uint16_t segment_index = 0;
  uint16_t flags = 0;
  uint32_t start_ms = 0;
  uint16_t realtime_base_id = 0;
  uint8_t reset_reason = 0;
  uint8_t record_size = sizeof(Record);
  uint32_t protocol_schema = 1;
  uint8_t reserved[476]{};
  uint32_t crc = 0;
};
#pragma pack(pop)

static_assert(sizeof(Record) == 32, "Telemetry record must divide a 512-byte sector");
static_assert(sizeof(SegmentHeader) == 512, "Telemetry header must occupy one sector");

inline void finalize(Record& record) {
  record.crc = 0;
  record.crc = crc32(&record, offsetof(Record, crc));
}

inline bool valid(const Record& record) {
  return record.magic == kRecordMagic && record.version == kFormatVersion &&
         record.crc == crc32(&record, offsetof(Record, crc));
}

inline void initializeHeader(SegmentHeader& header) {
  static constexpr char kMagic[8] = {'C', 'C', 'M', 'L', 'O', 'G', '2', '\0'};
  memcpy(header.magic, kMagic, sizeof(kMagic));
  header.crc = 0;
  header.crc = crc32(&header, offsetof(SegmentHeader, crc));
}

inline bool valid(const SegmentHeader& header) {
  static constexpr char kMagic[8] = {'C', 'C', 'M', 'L', 'O', 'G', '2', '\0'};
  return memcmp(header.magic, kMagic, sizeof(kMagic)) == 0 &&
         header.format_version == kFormatVersion &&
         header.header_size == sizeof(SegmentHeader) &&
         header.record_size == sizeof(Record) &&
         header.crc == crc32(&header, offsetof(SegmentHeader, crc));
}

}  // namespace storage::telemetry
