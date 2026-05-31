#include "storage/log_manager.h"

#include <cstring>

namespace storage {

bool LogManager::begin(SdManager* sd) {
  sd_ = sd;
  return sd_ != nullptr;
}

void LogManager::setSessionPrefix(const char* prefix) {
  snprintf(sessionPrefix_, sizeof(sessionPrefix_), "%s", prefix ? prefix : "boot");
}

void LogManager::enqueue(const char* category, const char* payload) {
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
  if (!sd_ || !sd_->mounted()) return;
  if ((nowMs - lastFlushMs_) < 200) return;
  lastFlushMs_ = nowMs;
  if (qCount_ == 0) return;

  for (uint8_t i = 0; i < 4 && qCount_ > 0; ++i) {
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
