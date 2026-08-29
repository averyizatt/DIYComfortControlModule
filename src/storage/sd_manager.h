#pragma once

#include <Arduino.h>
#include <SD.h>
#include <SPI.h>

namespace storage {

struct SdFileEntry {
  char name[40] = {};
  uint32_t sizeBytes = 0;
  bool isDirectory = false;
};

class SdManager {
 public:
  // The shared Arduino SPI instance must already be started by setup().
  bool begin(uint8_t lcdCsPin, uint8_t sdCsPin);
  void service(uint32_t nowMs);
  void noteIoSuccess() { recordIoResult(true, nullptr); }
  void noteIoFailure(const char* status) { recordIoResult(false, status); }
  bool mounted() const { return mounted_; }
  uint64_t totalBytes() const;
  uint64_t usedBytes() const;
  const char* lastStatus() const { return lastStatus_; }
  uint32_t errorCount() const { return errorCount_; }

  bool exists(const char* path) const;
  bool ensureFolder(const char* path);
  bool appendLine(const char* path, const char* line);
  bool writeTextFile(const char* path, const char* text);
  bool readTextFile(const char* path, char* out, size_t outLen) const;
  bool listDirectory(const char* path, SdFileEntry* entries, size_t maxEntries,
                     size_t offset, size_t& totalEntriesOut) const;

 private:
  void setStatus(const char* s, bool isError);
  void recordIoResult(bool ok, const char* failureStatus);

  bool mounted_ = false;
  uint64_t totalBytes_ = 0;
  uint8_t lcdCsPin_ = 255;
  uint8_t sdCsPin_ = 255;
  char lastStatus_[32] = "not_mounted";
  uint32_t errorCount_ = 0;
  uint32_t lastMountAttemptMs_ = 0;
  uint8_t consecutiveIoErrors_ = 0;
  bool configured_ = false;
};

}  // namespace storage
