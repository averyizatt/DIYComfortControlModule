#include "storage/sd_manager.h"

#include <cstring>

namespace storage {

bool SdManager::begin(uint8_t lcdCsPin, uint8_t sdCsPin) {
  lcdCsPin_ = lcdCsPin;
  sdCsPin_ = sdCsPin;

  // SPI bus is already initialised by the caller (SPI.begin in main.cpp).
  // Just ensure all CS pins are deasserted before the SD init transaction.
  pinMode(lcdCsPin_, OUTPUT);
  pinMode(sdCsPin_, OUTPUT);
  digitalWrite(lcdCsPin_, HIGH);
  digitalWrite(sdCsPin_, HIGH);

  // SD.begin() manages CS internally; it needs CS HIGH at call time so the
  // card can complete the 74-clock power-up sequence before selection.
  mounted_ = SD.begin(sdCsPin_, SPI, 4000000U);
  Serial0.printf("[SD] mount %s\n", mounted_ ? "OK" : "FAILED (no card?)");

  if (!mounted_) {
    setStatus("mount_failed", true);
    return false;
  }

  ensureFolder("/logs");
  ensureFolder("/logs/can");
  ensureFolder("/logs/gps");
  ensureFolder("/logs/meth");
  ensureFolder("/logs/knock");
  ensureFolder("/logs/faults");
  ensureFolder("/logs/tach");
  ensureFolder("/logs/race");
  ensureFolder("/ui");
  ensureFolder("/ui/images");
  ensureFolder("/ui/icons");
  ensureFolder("/ui/themes");
  ensureFolder("/ui/splash");

  setStatus("mounted", false);
  return true;
}

uint64_t SdManager::totalBytes() const {
  return mounted_ ? SD.totalBytes() : 0;
}

uint64_t SdManager::usedBytes() const {
  return mounted_ ? SD.usedBytes() : 0;
}

void SdManager::setStatus(const char* s, bool isError) {
  strncpy(lastStatus_, s ? s : "unknown", sizeof(lastStatus_) - 1);
  lastStatus_[sizeof(lastStatus_) - 1] = '\0';
  if (isError) ++errorCount_;
}

bool SdManager::ensureFolder(const char* path) {
  if (!mounted_ || !path) return false;
  if (SD.exists(path)) return true;
  if (!SD.mkdir(path)) {
    setStatus("mkdir_failed", true);
    return false;
  }
  return true;
}

bool SdManager::appendLine(const char* path, const String& line) {
  if (!mounted_ || !path) return false;

  digitalWrite(lcdCsPin_, HIGH);
  digitalWrite(sdCsPin_, LOW);

  File f = SD.open(path, FILE_APPEND);
  if (!f) {
    digitalWrite(sdCsPin_, HIGH);
    setStatus("open_failed", true);
    return false;
  }

  const bool ok = f.println(line) > 0;
  f.close();
  digitalWrite(sdCsPin_, HIGH);
  setStatus(ok ? "write_ok" : "write_failed", !ok);
  return ok;
}

}  // namespace storage
