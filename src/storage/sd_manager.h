#pragma once

#include <Arduino.h>
#include <SD.h>
#include <SPI.h>

namespace storage {

class SdManager {
 public:
  // SPI bus must be initialised externally (SPI.begin) before calling begin().
  bool begin(uint8_t lcdCsPin, uint8_t sdCsPin);
  bool mounted() const { return mounted_; }
  uint64_t totalBytes() const;
  uint64_t usedBytes() const;
  const char* lastStatus() const { return lastStatus_; }
  uint32_t errorCount() const { return errorCount_; }

  bool ensureFolder(const char* path);
  bool appendLine(const char* path, const char* line);

 private:
  void setStatus(const char* s, bool isError);

  bool mounted_ = false;
  uint8_t lcdCsPin_ = 255;
  uint8_t sdCsPin_ = 255;
  char lastStatus_[32] = "not_mounted";
  uint32_t errorCount_ = 0;
};

}  // namespace storage
