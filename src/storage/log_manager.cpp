#include "storage/log_manager.h"

namespace storage {

namespace {
constexpr size_t kMaxLogLineLength = 512;
constexpr size_t kMaxQueueSize = 300;
constexpr uint8_t kMaxWritesPerTick = 4;
}  // namespace

bool LogManager::begin(SdManager* sd) {
  sd_ = sd;
  return sd_ != nullptr;
}

void LogManager::setSessionPrefix(const String& prefix) {
  sessionPrefix_ = prefix;
}

void LogManager::enqueue(const char* category, const String& payload) {
  const char* cat = category ? category : "misc";
  String line = payload;
  if (line.length() > kMaxLogLineLength) line = line.substring(0, kMaxLogLineLength);

  String framed;
  framed.reserve(line.length() + 48);
  framed += String(millis());
  framed += ",";
  framed += cat;
  framed += ",";
  framed += line;

  if (queue_.size() > kMaxQueueSize) {
    queue_.pop_front();
    ++droppedCount_;
  }
  queue_.push_back(framed);
}

void LogManager::tick(uint32_t nowMs) {
  if (!sd_ || !sd_->mounted()) return;
  if ((nowMs - lastFlushMs_) < 200) return;
  lastFlushMs_ = nowMs;

  if (queue_.empty()) return;

  for (uint8_t i = 0; i < kMaxWritesPerTick && !queue_.empty(); ++i) {
    String entry = queue_.front();
    queue_.pop_front();

    const int firstComma = entry.indexOf(',');
    const int secondComma = entry.indexOf(',', firstComma + 1);
    String cat = "misc";
    if (firstComma > 0 && secondComma > firstComma) {
      cat = entry.substring(firstComma + 1, secondComma);
    }

    currentFile_ = "/logs/" + cat + "/" + sessionPrefix_ + ".csv";
    sd_->appendLine(currentFile_.c_str(), entry);
  }
}

void LogManager::flushCritical() {
  if (!sd_ || !sd_->mounted()) return;
  while (!queue_.empty()) {
    String entry = queue_.front();
    queue_.pop_front();
    const int firstComma = entry.indexOf(',');
    const int secondComma = entry.indexOf(',', firstComma + 1);
    String cat = "faults";
    if (firstComma > 0 && secondComma > firstComma) {
      cat = entry.substring(firstComma + 1, secondComma);
    }
    currentFile_ = "/logs/" + cat + "/" + sessionPrefix_ + ".csv";
    sd_->appendLine(currentFile_.c_str(), entry);
  }
}

}  // namespace storage
