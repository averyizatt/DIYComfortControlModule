#include "storage/log_manager.h"

#include <cstring>

namespace storage {
namespace {

#ifndef CCM_SD_LOGGING_ENABLED
#define CCM_SD_LOGGING_ENABLED 0
#endif

#ifndef CCM_SD_LOG_DRAIN_INTERVAL_MS
#define CCM_SD_LOG_DRAIN_INTERVAL_MS 1000
#endif

#ifndef CCM_SD_LOG_DRAIN_MAX_WRITES
#define CCM_SD_LOG_DRAIN_MAX_WRITES 1
#endif

#ifndef CCM_SD_LOG_MIN_FREE_HEAP
#define CCM_SD_LOG_MIN_FREE_HEAP 24000
#endif

constexpr bool kSdLoggingEnabled = CCM_SD_LOGGING_ENABLED != 0;
constexpr uint32_t kSdLogDrainIntervalMs =
    (CCM_SD_LOG_DRAIN_INTERVAL_MS < 200) ? 200U : static_cast<uint32_t>(CCM_SD_LOG_DRAIN_INTERVAL_MS);
constexpr uint8_t kSdLogDrainMaxWrites =
    (CCM_SD_LOG_DRAIN_MAX_WRITES < 1) ? 1U : static_cast<uint8_t>(CCM_SD_LOG_DRAIN_MAX_WRITES);
constexpr uint32_t kSdLogMinFreeHeap =
    (CCM_SD_LOG_MIN_FREE_HEAP < 8000) ? 8000U : static_cast<uint32_t>(CCM_SD_LOG_MIN_FREE_HEAP);

}  // namespace

bool LogManager::begin(SdManager* sd) {
  sd_ = sd;
  return sd_ != nullptr;
}

void LogManager::setSessionPrefix(const char* prefix) {
  snprintf(sessionPrefix_, sizeof(sessionPrefix_), "%s", prefix ? prefix : "boot");
}

void LogManager::enqueue(const char* category, const char* payload) {
  if (!kSdLoggingEnabled) {
    return;
  }
  if (qCount_ >= kMaxQueueSize) {
    // Drop oldest entry to make room
    qHead_ = static_cast<uint8_t>((qHead_ + 1) % kMaxQueueSize);
    --qCount_;
    ++droppedCount_;
  }
  const char* cat = (category && *category) ? category : "misc";
  snprintf(s_queue_[qTail_], kMaxLineLen, "%lu,%s,%s",
           static_cast<unsigned long>(millis()), cat,
           payload ? payload : "");
  qTail_ = static_cast<uint8_t>((qTail_ + 1) % kMaxQueueSize);
  ++qCount_;
}

// Extract the category field from a stored "timestamp,category,payload" line.
// Writes into catBuf (max catBufLen bytes incl NUL). Returns catBuf.
static const char* extractCategory(const char* entry, char* catBuf, size_t catBufLen) {
  const char* p1 = strchr(entry, ',');
  if (!p1) { snprintf(catBuf, catBufLen, "misc"); return catBuf; }
  const char* p2 = strchr(p1 + 1, ',');
  if (!p2) { snprintf(catBuf, catBufLen, "misc"); return catBuf; }
  const size_t len = static_cast<size_t>(p2 - p1 - 1);
  const size_t copy = (len < catBufLen - 1) ? len : catBufLen - 1;
  memcpy(catBuf, p1 + 1, copy);
  catBuf[copy] = '\0';
  return catBuf;
}

void LogManager::tick(uint32_t nowMs) {
  if (!kSdLoggingEnabled) return;
  if (!sd_ || !sd_->mounted()) return;
  if ((nowMs - lastFlushMs_) < kSdLogDrainIntervalMs) return;
  lastFlushMs_ = nowMs;
  if (qCount_ == 0) return;
  if (ESP.getFreeHeap() < kSdLogMinFreeHeap) {
    ++droppedCount_;
    return;
  }

  for (uint8_t i = 0; i < kSdLogDrainMaxWrites && qCount_ > 0; ++i) {
    const char* entry = s_queue_[qHead_];
    char catBuf[24];
    extractCategory(entry, catBuf, sizeof(catBuf));
    snprintf(currentFile_, sizeof(currentFile_), "/logs/%s/%s.csv", catBuf, sessionPrefix_);
    sd_->appendLine(currentFile_, entry);
    qHead_ = static_cast<uint8_t>((qHead_ + 1) % kMaxQueueSize);
    --qCount_;
  }
}

void LogManager::flushCritical() {
  if (!kSdLoggingEnabled) return;
  if (!sd_ || !sd_->mounted()) return;
  while (qCount_ > 0) {
    const char* entry = s_queue_[qHead_];
    char catBuf[24];
    extractCategory(entry, catBuf, sizeof(catBuf));
    snprintf(currentFile_, sizeof(currentFile_), "/logs/%s/%s.csv", catBuf, sessionPrefix_);
    sd_->appendLine(currentFile_, entry);
    qHead_ = static_cast<uint8_t>((qHead_ + 1) % kMaxQueueSize);
    --qCount_;
  }
}

}  // namespace storage
