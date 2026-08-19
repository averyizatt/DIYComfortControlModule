#include "storage/telemetry_recorder.h"

#include <Preferences.h>
#include <cstring>

#include "hal/SharedSpiBus.hpp"

namespace storage {
namespace {

constexpr const char* kLogDir = "/logs/ecu";

bool hasSuffix(const char* value, const char* suffix) {
  if (!value || !suffix) return false;
  const size_t valueLen = strlen(value);
  const size_t suffixLen = strlen(suffix);
  return valueLen >= suffixLen && strcmp(value + valueLen - suffixLen, suffix) == 0;
}

const char* leafName(const char* path) {
  if (!path) return "";
  const char* slash = strrchr(path, '/');
  return slash ? slash + 1 : path;
}

}  // namespace

bool TelemetryRecorder::begin(SdManager* sd, uint8_t resetReason, uint16_t realtimeBaseId) {
  sd_ = sd;
  resetReason_ = resetReason;
  realtimeBaseId_ = realtimeBaseId;
  // There is no hot-plug/remount path in this firmware. Do not consume the
  // fixed CAN ring forever when boot completed without a usable card.
  enabled_ = sd_ != nullptr && sd_->mounted();
  if (!enabled_) return false;

  Preferences prefs;
  if (prefs.begin("ccm_log", false)) {
    sessionId_ = prefs.getULong("session", 0) + 1U;
    prefs.putULong("session", sessionId_);
    prefs.end();
  } else {
    sessionId_ = esp_random();
  }
  return true;
}

bool TelemetryRecorder::capture(const can_protocol::CanFrame& frame, uint32_t timestampMs) {
  if (!enabled_ || shutdownRequested_ ||
      !microsquirt::isMicroSquirtId(frame.id, realtimeBaseId_)) {
    return false;
  }

  telemetry::Record record{};
  record.timestamp_ms = timestampMs;
  record.can_id = frame.id;
  record.dlc = frame.dlc > 8U ? 8U : frame.dlc;
  for (uint8_t i = 0; i < record.dlc; ++i) record.data[i] = frame.data[i];

  portENTER_CRITICAL(&ringMux_);
  if (ringCount_ >= kRingCapacity) {
    pendingDrops_ = pendingDrops_ + 1U;
    dropped_ = dropped_ + 1U;
    portEXIT_CRITICAL(&ringMux_);
    return false;
  }
  record.sequence = nextSequence_++;
  record.dropped_before = pendingDrops_;
  pendingDrops_ = 0;
  telemetry::finalize(record);
  ring_[ringTail_] = record;
  ringTail_ = static_cast<uint16_t>((ringTail_ + 1U) % kRingCapacity);
  ringCount_ = ringCount_ + 1U;
  if (ringCount_ > ringHighWater_) ringHighWater_ = ringCount_;
  captured_ = captured_ + 1U;
  portEXIT_CRITICAL(&ringMux_);
  return true;
}

bool TelemetryRecorder::pop(telemetry::Record& out) {
  portENTER_CRITICAL(&ringMux_);
  if (ringCount_ == 0U) {
    portEXIT_CRITICAL(&ringMux_);
    return false;
  }
  out = ring_[ringHead_];
  ringHead_ = static_cast<uint16_t>((ringHead_ + 1U) % kRingCapacity);
  ringCount_ = ringCount_ - 1U;
  portEXIT_CRITICAL(&ringMux_);
  return true;
}

uint16_t TelemetryRecorder::queueDepth() const {
  portENTER_CRITICAL(&ringMux_);
  const uint16_t result = ringCount_;
  portEXIT_CRITICAL(&ringMux_);
  return result;
}

bool TelemetryRecorder::openSegment(uint32_t nowMs) {
  if (!sd_ || !sd_->mounted() || nowMs < retryAfterMs_) return false;
  sd_->ensureFolder("/logs");
  if (!sd_->ensureFolder(kLogDir)) {
    setWriteFailure();
    return false;
  }

  snprintf(activePath_, sizeof(activePath_), "%s/S%08lX_%03u.tmp", kLogDir,
           static_cast<unsigned long>(sessionId_), static_cast<unsigned>(segmentIndex_));
  {
    hal::SharedSpiBusLock lock("SD:ecu-open", pdMS_TO_TICKS(50));
    if (!lock.locked()) return false;
    activeFile_ = SD.open(activePath_, FILE_WRITE);
  }
  if (!activeFile_) {
    // A full filesystem commonly presents as an open failure. Retention will
    // reclaim one old, unprotected segment after the incremental scan.
    spacePressure_ = true;
    setWriteFailure();
    return false;
  }

  telemetry::SegmentHeader header{};
  header.session_id = sessionId_;
  header.segment_index = segmentIndex_;
  header.start_ms = nowMs;
  header.realtime_base_id = realtimeBaseId_;
  header.reset_reason = resetReason_;
  telemetry::initializeHeader(header);
  if (!writeBytes(reinterpret_cast<const uint8_t*>(&header), sizeof(header))) {
    setWriteFailure();
    return false;
  }
  {
    hal::SharedSpiBusLock lock("SD:ecu-sync", pdMS_TO_TICKS(50));
    if (lock.locked()) activeFile_.flush();
  }
  segmentStartMs_ = nowMs;
  segmentBytes_ = sizeof(header);
  sectorFill_ = 0;
  lastSyncMs_ = nowMs;
  recording_ = true;
  shutdownClean_ = false;
  activeProtected_ = false;
  return true;
}

bool TelemetryRecorder::writeBytes(const uint8_t* data, size_t len) {
  if (!activeFile_ || !data || len == 0U) return false;
  const uint32_t startUs = micros();
  size_t wrote = 0;
  {
    hal::SharedSpiBusLock lock("SD:ecu-write", pdMS_TO_TICKS(50));
    if (!lock.locked()) return false;
    wrote = activeFile_.write(data, len);
  }
  const uint32_t elapsed = micros() - startUs;
  if (elapsed > maxWriteUs_) maxWriteUs_ = elapsed;
  if (wrote != len) {
    spacePressure_ = true;
    return false;
  }
  bytesWritten_ += static_cast<uint32_t>(wrote);
  segmentBytes_ += static_cast<uint32_t>(wrote);
  return true;
}

bool TelemetryRecorder::writePending(bool force) {
  if (sectorFill_ == 0U) return true;
  if (!force && sectorFill_ < sizeof(sector_)) return true;
  if (!writeBytes(sector_, sectorFill_)) {
    setWriteFailure();
    return false;
  }
  sectorFill_ = 0;
  return true;
}

void TelemetryRecorder::closeSegment(uint32_t nowMs, bool clean) {
  if (!recording_) return;
  if (!writePending(true)) clean = false;

  if (clean && activeFile_) {
    telemetry::Record footer{};
    footer.type = telemetry::kRecordFooter;
    footer.timestamp_ms = nowMs;
    footer.flags = activeProtected_ ? telemetry::kFlagProtected : 0U;
    portENTER_CRITICAL(&ringMux_);
    footer.sequence = nextSequence_++;
    footer.dropped_before = pendingDrops_;
    pendingDrops_ = 0;
    portEXIT_CRITICAL(&ringMux_);
    telemetry::finalize(footer);
    if (!writeBytes(reinterpret_cast<const uint8_t*>(&footer), sizeof(footer))) clean = false;
  }

  if (activeFile_) {
    hal::SharedSpiBusLock lock("SD:ecu-close", pdMS_TO_TICKS(100));
    if (lock.locked()) {
      activeFile_.flush();
      activeFile_.close();
    } else {
      clean = false;
      activeFile_.close();
    }
  }

  char closedPath[72]{};
  snprintf(closedPath, sizeof(closedPath), "%s/S%08lX_%03u.%s", kLogDir,
           static_cast<unsigned long>(sessionId_), static_cast<unsigned>(segmentIndex_),
           activeProtected_ ? "keep" : (clean ? "ccmlog" : "recover"));
  if (!renamePath(activePath_, closedPath)) clean = false;
  strncpy(previousClosedPath_, closedPath, sizeof(previousClosedPath_) - 1);
  previousClosedPath_[sizeof(previousClosedPath_) - 1] = '\0';
  activePath_[0] = '\0';
  recording_ = false;
  shutdownClean_ = clean;
  ++segmentIndex_;
  scanComplete_ = false;
  scanStarted_ = false;
  if (scanDir_) scanDir_.close();
}

void TelemetryRecorder::setWriteFailure() {
  ++writeErrors_;
  retryAfterMs_ = millis() + kRetryIntervalMs;
  recording_ = false;
  const bool hadActiveFile = static_cast<bool>(activeFile_);
  if (hadActiveFile) {
    hal::SharedSpiBusLock lock("SD:ecu-fail-close", pdMS_TO_TICKS(50));
    activeFile_.close();
  }
  if (activePath_[0] != '\0' && hadActiveFile) {
    char recovered[72]{};
    snprintf(recovered, sizeof(recovered), "%s/S%08lX_%03u.recover", kLogDir,
             static_cast<unsigned long>(sessionId_), static_cast<unsigned>(segmentIndex_));
    renamePath(activePath_, recovered);
    strncpy(previousClosedPath_, recovered, sizeof(previousClosedPath_) - 1);
    previousClosedPath_[sizeof(previousClosedPath_) - 1] = '\0';
    activePath_[0] = '\0';
    ++segmentIndex_;
  }
  activePath_[0] = '\0';
  sectorFill_ = 0;
  scanComplete_ = false;
  scanStarted_ = false;
  if (scanDir_) scanDir_.close();
}

bool TelemetryRecorder::renamePath(const char* from, const char* to) {
  if (!from || !*from || !to || !*to) return false;
  hal::SharedSpiBusLock lock("SD:ecu-rename", pdMS_TO_TICKS(100));
  if (!lock.locked()) return false;
  if (SD.exists(to)) SD.remove(to);
  return SD.rename(from, to);
}

bool TelemetryRecorder::removePath(const char* path) {
  if (!path || !*path) return false;
  hal::SharedSpiBusLock lock("SD:ecu-remove", pdMS_TO_TICKS(100));
  return lock.locked() && SD.remove(path);
}

void TelemetryRecorder::startScan() {
  if (!sd_ || !sd_->mounted() || scanStarted_) return;
  hal::SharedSpiBusLock lock("SD:ecu-scan-open", pdMS_TO_TICKS(50));
  if (!lock.locked()) return;
  scanDir_ = SD.open(kLogDir);
  if (!scanDir_ || !scanDir_.isDirectory()) {
    if (scanDir_) scanDir_.close();
    return;
  }
  scanStarted_ = true;
  scanComplete_ = false;
  scanBytes_ = 0;
  oldestPath_[0] = '\0';
  oldestSize_ = 0;
}

void TelemetryRecorder::scanOneEntry() {
  if (!scanStarted_ || !scanDir_) return;
  File entry;
  {
    hal::SharedSpiBusLock lock("SD:ecu-scan", pdMS_TO_TICKS(50));
    if (!lock.locked()) return;
    entry = scanDir_.openNextFile();
  }
  if (!entry) {
    scanDir_.close();
    scanStarted_ = false;
    scanComplete_ = true;
    managedBytes_ = scanBytes_;
    return;
  }

  char path[72]{};
  const char* name = leafName(entry.name());
  snprintf(path, sizeof(path), "%s/%s", kLogDir, name);
  const uint32_t size = entry.isDirectory() ? 0U : static_cast<uint32_t>(entry.size());
  entry.close();
  if (strcmp(path, activePath_) == 0) return;

  if (hasSuffix(path, ".tmp")) {
    char recovered[72]{};
    strncpy(recovered, path, sizeof(recovered) - 1);
    char* suffix = strrchr(recovered, '.');
    if (suffix) snprintf(suffix, static_cast<size_t>(recovered + sizeof(recovered) - suffix), ".recover");
    if (renamePath(path, recovered)) ++recoveryCount_;
    strncpy(path, recovered, sizeof(path) - 1);
  }
  if (hasSuffix(path, ".ccmlog") || hasSuffix(path, ".keep") || hasSuffix(path, ".recover")) {
    scanBytes_ += size;
    if (hasSuffix(path, ".ccmlog") &&
        (oldestPath_[0] == '\0' || strcmp(leafName(path), leafName(oldestPath_)) < 0)) {
      strncpy(oldestPath_, path, sizeof(oldestPath_) - 1);
      oldestPath_[sizeof(oldestPath_) - 1] = '\0';
      oldestSize_ = size;
    }
  }
}

void TelemetryRecorder::runRetention() {
  if (!scanComplete_ || !sd_ || sd_->totalBytes() == 0U) return;
  const uint64_t quota = (sd_->totalBytes() * 85ULL) / 100ULL;
  const bool overQuota = managedBytes_ + segmentBytes_ > quota;
  if ((!overQuota && !spacePressure_) || oldestPath_[0] == '\0') return;
  if (removePath(oldestPath_)) {
    managedBytes_ = managedBytes_ > oldestSize_ ? managedBytes_ - oldestSize_ : 0U;
    // Reclaim one segment per pressure event. If the filesystem still cannot
    // accept the next segment, the following failed open/write requests one
    // more bounded deletion.
    spacePressure_ = false;
    ++deletedCount_;
    scanComplete_ = false;
    scanStarted_ = false;
    oldestPath_[0] = '\0';
  }
}

void TelemetryRecorder::service(uint32_t nowMs) {
  if (!enabled_ || !sd_ || !sd_->mounted()) return;
  if (!recording_ && !shutdownRequested_) openSegment(nowMs);
  if (!recording_) {
    if (!scanStarted_ && !scanComplete_) startScan();
    scanOneEntry();
    runRetention();
    return;
  }

  if (protectRequested_) {
    protectRequested_ = false;
    activeProtected_ = true;
    if (previousClosedPath_[0] != '\0' && hasSuffix(previousClosedPath_, ".ccmlog")) {
      char keepPath[72]{};
      strncpy(keepPath, previousClosedPath_, sizeof(keepPath) - 1);
      char* suffix = strrchr(keepPath, '.');
      if (suffix) strcpy(suffix, ".keep");
      if (renamePath(previousClosedPath_, keepPath)) {
        strncpy(previousClosedPath_, keepPath, sizeof(previousClosedPath_) - 1);
      }
    }
  }

  const uint8_t budget = shutdownRequested_ ? 64U : kRecordsPerService;
  for (uint8_t i = 0; i < budget; ++i) {
    telemetry::Record record{};
    if (!pop(record)) break;
    memcpy(&sector_[sectorFill_], &record, sizeof(record));
    sectorFill_ = static_cast<uint16_t>(sectorFill_ + sizeof(record));
    ++written_;
    if (sectorFill_ == sizeof(sector_) && !writePending(false)) break;
  }

  if ((nowMs - lastSyncMs_) >= kSyncIntervalMs && activeFile_) {
    writePending(true);
    hal::SharedSpiBusLock lock("SD:ecu-sync", pdMS_TO_TICKS(100));
    if (lock.locked()) activeFile_.flush();
    lastSyncMs_ = nowMs;
  }

  if (shutdownRequested_ && queueDepth() == 0U) {
    closeSegment(nowMs, true);
  } else if (!shutdownRequested_ &&
             ((nowMs - segmentStartMs_) >= kSegmentDurationMs ||
              segmentBytes_ >= kSegmentMaxBytes)) {
    closeSegment(nowMs, true);
  }

  if (!scanStarted_ && !scanComplete_) startScan();
  scanOneEntry();
  runRetention();
}

void TelemetryRecorder::protectEvent() {
  protectRequested_ = true;
}

void TelemetryRecorder::requestShutdown() {
  shutdownRequested_ = true;
}

void TelemetryRecorder::resume(uint32_t nowMs) {
  if (!shutdownRequested_) return;
  shutdownRequested_ = false;
  shutdownClean_ = false;
  retryAfterMs_ = nowMs;
}

TelemetryRecorderStats TelemetryRecorder::stats() const {
  TelemetryRecorderStats result{};
  result.enabled = enabled_;
  result.recording = recording_;
  result.scan_complete = scanComplete_;
  result.shutdown_clean = shutdownClean_;
  result.active_protected = activeProtected_;
  portENTER_CRITICAL(&ringMux_);
  result.queue_depth = ringCount_;
  result.queue_high_water = ringHighWater_;
  result.captured = captured_;
  result.dropped = dropped_;
  portEXIT_CRITICAL(&ringMux_);
  result.written = written_;
  result.bytes_written = bytesWritten_;
  result.write_errors = writeErrors_;
  result.recovery_count = recoveryCount_;
  result.deleted_count = deletedCount_;
  result.max_write_us = maxWriteUs_;
  result.managed_bytes = managedBytes_ + segmentBytes_;
  strncpy(result.active_file, activePath_, sizeof(result.active_file) - 1);
  return result;
}

}  // namespace storage
