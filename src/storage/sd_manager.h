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

  bool mounted_ = false;
  uint8_t lcdCsPin_ = 255;
  uint8_t sdCsPin_ = 255;
  char lastStatus_[32] = "not_mounted";
  uint32_t errorCount_ = 0;
};

}  // namespace storage
