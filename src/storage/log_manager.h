#pragma once

#include <Arduino.h>

#include <deque>

#include "storage/sd_manager.h"

namespace storage {

class LogManager {
 public:
  bool begin(SdManager* sd);
  void setSessionPrefix(const String& prefix);
  void enqueue(const char* category, const String& payload);
  void tick(uint32_t nowMs);
  void flushCritical();
  const char* currentFile() const { return currentFile_.c_str(); }

 private:
  SdManager* sd_ = nullptr;
  std::deque<String> queue_;
  String sessionPrefix_ = "boot";
  String currentFile_;
  uint32_t lastFlushMs_ = 0;
};

}  // namespace storage
