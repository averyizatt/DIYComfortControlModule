#pragma once

#include <Arduino.h>
#include <FS.h>
#include <SD.h>

#include "can/MicroSquirtProtocol.hpp"
#include "storage/TelemetryFormat.hpp"
#include "storage/sd_manager.h"

namespace storage {

struct TelemetryRecorderStats {
  bool enabled = false;
  bool recording = false;
  bool scan_complete = false;
  bool shutdown_clean = false;
  bool active_protected = false;
  uint16_t queue_depth = 0;
  uint16_t queue_high_water = 0;
  uint32_t captured = 0;
  uint32_t written = 0;
  uint32_t dropped = 0;
  uint32_t bytes_written = 0;
  uint32_t write_errors = 0;
  uint32_t recovery_count = 0;
  uint32_t deleted_count = 0;
  uint32_t max_write_us = 0;
  uint64_t managed_bytes = 0;
  char active_file[72]{};
};

class TelemetryRecorder {
 public:
  bool begin(SdManager* sd, uint8_t resetReason,
             uint16_t realtimeBaseId = microsquirt::kRecommendedRealtimeBaseId);
  bool capture(const can_protocol::CanFrame& frame, uint32_t timestampMs);
  void service(uint32_t nowMs);
  void protectEvent();
  void requestShutdown();
  void resume(uint32_t nowMs);
  TelemetryRecorderStats stats() const;

 private:
  static constexpr uint16_t kRingCapacity = 512;
  static constexpr uint32_t kSegmentDurationMs = 15UL * 60UL * 1000UL;
  static constexpr uint32_t kSegmentMaxBytes = 16UL * 1024UL * 1024UL;
  static constexpr uint32_t kSyncIntervalMs = 1000;
  static constexpr uint32_t kRetryIntervalMs = 5000;
  static constexpr uint8_t kRecordsPerService = 16;

  bool openSegment(uint32_t nowMs);
  bool writeBytes(const uint8_t* data, size_t len);
  bool writePending(bool force);
  void closeSegment(uint32_t nowMs, bool clean);
  bool pop(telemetry::Record& out);
  uint16_t queueDepth() const;
  void startScan();
  void scanOneEntry();
  void runRetention();
  bool renamePath(const char* from, const char* to);
  bool removePath(const char* path);
  void setWriteFailure();

  SdManager* sd_ = nullptr;
  uint8_t resetReason_ = 0;
  uint16_t realtimeBaseId_ = microsquirt::kRecommendedRealtimeBaseId;
  uint32_t sessionId_ = 0;
  uint16_t segmentIndex_ = 0;
  uint32_t nextSequence_ = 1;

  telemetry::Record ring_[kRingCapacity]{};
  mutable portMUX_TYPE ringMux_ = portMUX_INITIALIZER_UNLOCKED;
  uint16_t ringHead_ = 0;
  uint16_t ringTail_ = 0;
  uint16_t ringCount_ = 0;
  uint16_t ringHighWater_ = 0;
  uint32_t pendingDrops_ = 0;
  uint32_t captured_ = 0;
  uint32_t dropped_ = 0;

  File activeFile_;
  File scanDir_;
  uint8_t sector_[512]{};
  uint16_t sectorFill_ = 0;
  uint32_t segmentStartMs_ = 0;
  uint32_t segmentBytes_ = 0;
  uint32_t lastSyncMs_ = 0;
  uint32_t retryAfterMs_ = 0;
  uint32_t written_ = 0;
  uint32_t bytesWritten_ = 0;
  uint32_t writeErrors_ = 0;
  uint32_t recoveryCount_ = 0;
  uint32_t deletedCount_ = 0;
  uint32_t maxWriteUs_ = 0;
  uint64_t managedBytes_ = 0;
  uint64_t scanBytes_ = 0;
  uint32_t oldestSize_ = 0;
  bool enabled_ = false;
  bool recording_ = false;
  bool shutdownRequested_ = false;
  bool shutdownClean_ = false;
  bool protectRequested_ = false;
  bool activeProtected_ = false;
  bool spacePressure_ = false;
  bool scanStarted_ = false;
  bool scanComplete_ = false;
  char activePath_[72]{};
  char previousClosedPath_[72]{};
  char oldestPath_[72]{};
};

}  // namespace storage
