#pragma once

#include <Arduino.h>

#include "storage/sd_manager.h"

namespace storage {

// Zero-heap log queue: fixed static ring buffer of char arrays.
// No std::deque, no Arduino String, no ostringstream — zero heap allocations.
class LogManager {
 public:
  bool begin(SdManager* sd);
  void setSessionPrefix(const char* prefix);
  // Overload for callers still passing Arduino String (avoids changing all call sites)
  void setSessionPrefix(const String& prefix) { setSessionPrefix(prefix.c_str()); }
  void enqueue(const char* category, const char* payload);
  // Overload for callers still passing Arduino String
  void enqueue(const char* category, const String& payload) { enqueue(category, payload.c_str()); }
  void tick(uint32_t nowMs);
  void flushCritical();
  const char* currentFile() const { return currentFile_; }
  uint32_t droppedCount() const { return droppedCount_; }

 private:
  static constexpr uint8_t  kMaxQueueSize = 24;   // 4 s headroom at 6 enqueues/s
  static constexpr uint16_t kMaxLineLen   = 512;  // timestamp,category,payload

  SdManager* sd_ = nullptr;
  char s_queue_[kMaxQueueSize][kMaxLineLen];
  uint8_t qHead_  = 0;
  uint8_t qTail_  = 0;
  uint8_t qCount_ = 0;
  char sessionPrefix_[32]  = "boot";
  char currentFile_[72]    = {};
  uint32_t lastFlushMs_    = 0;
  uint32_t droppedCount_   = 0;
};

}  // namespace storage
