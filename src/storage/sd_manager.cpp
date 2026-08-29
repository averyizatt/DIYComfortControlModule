#include "storage/sd_manager.h"

#include <cstring>

#include "hal/SharedSpiBus.hpp"

#ifndef CCM_SD_ENABLED
#define CCM_SD_ENABLED 1
#endif

#ifndef CCM_SD_AUTOMOUNT
#define CCM_SD_AUTOMOUNT 0
#endif

#ifndef CCM_SD_MAX_SPI_HZ
#define CCM_SD_MAX_SPI_HZ 1000000UL
#endif

#ifndef CCM_SD_CREATE_FOLDERS_ON_BOOT
#define CCM_SD_CREATE_FOLDERS_ON_BOOT 0
#endif

#ifndef CCM_SD_MIN_FILEOP_HEAP
#define CCM_SD_MIN_FILEOP_HEAP 24000
#endif

namespace storage {
namespace {

constexpr uint32_t kSdMaxSpiHz = CCM_SD_MAX_SPI_HZ;
constexpr bool kSdCreateFoldersOnBoot = CCM_SD_CREATE_FOLDERS_ON_BOOT != 0;
constexpr uint32_t kSdMinFileOpHeap =
    (CCM_SD_MIN_FILEOP_HEAP < 8000) ? 8000U : static_cast<uint32_t>(CCM_SD_MIN_FILEOP_HEAP);
constexpr uint32_t kSdRemountIntervalMs = 10000U;
constexpr uint8_t kSdIoErrorsBeforeOffline = 3U;

bool heapAllowsFileOp() {
  return ESP.getFreeHeap() >= kSdMinFileOpHeap;
}

const char* baseName(const char* path) {
  if (!path || path[0] == '\0') {
    return "";
  }
  const char* slash = strrchr(path, '/');
  if (!slash) {
    return path;
  }
  return slash[1] ? slash + 1 : path;
}

void copyEntryName(char* dst, size_t dstLen, const char* src) {
  if (!dst || dstLen == 0) return;
  const char* name = baseName(src);
  if (!name || name[0] == '\0') {
    name = src ? src : "";
  }
  strncpy(dst, name, dstLen - 1);
  dst[dstLen - 1] = '\0';
}

}  // namespace

bool SdManager::begin(uint8_t lcdCsPin, uint8_t sdCsPin) {
  lcdCsPin_ = lcdCsPin;
  sdCsPin_ = sdCsPin;
  totalBytes_ = 0;
  configured_ = true;
  lastMountAttemptMs_ = millis();

#if !CCM_SD_ENABLED
  mounted_ = false;
  setStatus("disabled", false);
  Serial.printf("[SD] disabled by build flag (cs=%u)\n", static_cast<unsigned>(sdCsPin_));
  return false;
#elif !CCM_SD_AUTOMOUNT
  mounted_ = false;
  setStatus("mount_deferred", false);
  Serial.printf("[SD] auto-mount skipped by build flag (cs=%u)\n",
                 static_cast<unsigned>(sdCsPin_));
  return false;
#else
  // setup() starts the shared Arduino SPI instance on the board's custom pins.
  // Keep both chip-selects deasserted before the SD init transaction.
  pinMode(lcdCsPin_, OUTPUT);
  pinMode(sdCsPin_, OUTPUT);
  digitalWrite(lcdCsPin_, HIGH);
  digitalWrite(sdCsPin_, HIGH);

  {
    Serial.printf("[SD] waiting for SPI lock owner=%s\n", hal::sharedSpiBusOwner());
    hal::SharedSpiBusLock spiLock("SD:begin", pdMS_TO_TICKS(1500));
    if (!spiLock.locked()) {
      mounted_ = false;
      setStatus("spi_lock_timeout", true);
      Serial.printf("[SD] SPI lock timeout owner=%s; continuing without SD\n",
                    hal::sharedSpiBusOwner());
      return false;
    }
    Serial.println("[SD] SPI lock acquired; probing at 400kHz");
    // SD.begin() manages CS internally; start at the safest speed on the
    // shared SPI bus. Faster retries are capped so SD cannot dominate the
    // same wires the TFT depends on for stable redraws.
    mounted_ = SD.begin(sdCsPin_, SPI, 400000U);
    if (!mounted_ && kSdMaxSpiHz >= 1000000UL) {
      Serial.println("[SD] 400kHz failed; retrying at 1MHz");
      delay(50);
      mounted_ = SD.begin(sdCsPin_, SPI, 1000000U);
    }
    if (!mounted_ && kSdMaxSpiHz >= 4000000UL) {
      delay(50);
      mounted_ = SD.begin(sdCsPin_, SPI, 4000000U);
    }
    digitalWrite(lcdCsPin_, HIGH);
    digitalWrite(sdCsPin_, HIGH);
  }
  Serial.printf("[SD] mount %s (cs=%u)\n", mounted_ ? "OK" : "FAILED (no card?)", static_cast<unsigned>(sdCsPin_));

  if (!mounted_) {
    setStatus("mount_failed", true);
    return false;
  }

  uint64_t cardSize = 0;
  {
    hal::SharedSpiBusLock spiLock("SD:size");
    digitalWrite(lcdCsPin_, HIGH);
    cardSize = SD.cardSize();
    digitalWrite(sdCsPin_, HIGH);
  }
  Serial.printf("[SD] card size=%lu MB\n",
                 static_cast<unsigned long>(cardSize / (1024ULL * 1024ULL)));
  totalBytes_ = cardSize;
  consecutiveIoErrors_ = 0;

  bool foldersOk = true;
  if (kSdCreateFoldersOnBoot) {
    foldersOk &= ensureFolder("/logs");
    foldersOk &= ensureFolder("/logs/can");
    foldersOk &= ensureFolder("/logs/gps");
    foldersOk &= ensureFolder("/logs/meth");
    foldersOk &= ensureFolder("/logs/knock");
    foldersOk &= ensureFolder("/logs/faults");
    foldersOk &= ensureFolder("/logs/tach");
    foldersOk &= ensureFolder("/logs/race");
    foldersOk &= ensureFolder("/ui");
    foldersOk &= ensureFolder("/ui/images");
    foldersOk &= ensureFolder("/ui/icons");
    foldersOk &= ensureFolder("/ui/themes");
    foldersOk &= ensureFolder("/ui/splash");
    Serial.printf("[SD] folders %s\n", foldersOk ? "ready" : "incomplete");
  } else {
    Serial.println("[SD] boot folder scan skipped");
  }

  setStatus(foldersOk ? "mounted" : "folder_failed", !foldersOk);
  return true;
#endif
}

void SdManager::service(uint32_t nowMs) {
#if CCM_SD_ENABLED && CCM_SD_AUTOMOUNT
  if (!configured_ || mounted_ ||
      (nowMs - lastMountAttemptMs_) < kSdRemountIntervalMs) {
    return;
  }

  lastMountAttemptMs_ = nowMs;
  Serial.println("[SD] offline; attempting remount");
  {
    hal::SharedSpiBusLock spiLock("SD:remount-end", pdMS_TO_TICKS(500));
    if (!spiLock.locked()) {
      setStatus("remount_lock_timeout", true);
      return;
    }
    SD.end();
    digitalWrite(lcdCsPin_, HIGH);
    digitalWrite(sdCsPin_, HIGH);
  }
  (void)begin(lcdCsPin_, sdCsPin_);
#else
  (void)nowMs;
#endif
}

uint64_t SdManager::totalBytes() const {
  // Card capacity is static while mounted and was already queried at boot.
  // Avoid a FAT/SD call from the periodic storage-status path: on a busy
  // shared SPI bus that query can block long enough to trip the task WDT.
  return mounted_ ? totalBytes_ : 0;
}

uint64_t SdManager::usedBytes() const {
  if (!mounted_) return 0;
  hal::SharedSpiBusLock spiLock("SD:used");
  digitalWrite(lcdCsPin_, HIGH);
  const uint64_t used = SD.usedBytes();
  digitalWrite(sdCsPin_, HIGH);
  return used;
}

void SdManager::setStatus(const char* s, bool isError) {
  strncpy(lastStatus_, s ? s : "unknown", sizeof(lastStatus_) - 1);
  lastStatus_[sizeof(lastStatus_) - 1] = '\0';
  if (isError) ++errorCount_;
}

bool SdManager::ensureFolder(const char* path) {
  if (!mounted_ || !path) return false;
  hal::SharedSpiBusLock spiLock("SD:mkdir");
  digitalWrite(lcdCsPin_, HIGH);
  if (SD.exists(path)) {
    digitalWrite(sdCsPin_, HIGH);
    recordIoResult(true, nullptr);
    return true;
  }
  if (!SD.mkdir(path)) {
    digitalWrite(sdCsPin_, HIGH);
    recordIoResult(false, "mkdir_failed");
    return false;
  }
  digitalWrite(sdCsPin_, HIGH);
  recordIoResult(true, nullptr);
  return true;
}

void SdManager::recordIoResult(bool ok, const char* failureStatus) {
  if (ok) {
    consecutiveIoErrors_ = 0;
    return;
  }

  setStatus(failureStatus, true);
  if (consecutiveIoErrors_ < 255U) ++consecutiveIoErrors_;
  if (mounted_ && consecutiveIoErrors_ >= kSdIoErrorsBeforeOffline) {
    bool cardPresent = false;
    {
      hal::SharedSpiBusLock spiLock("SD:health", pdMS_TO_TICKS(100));
      cardPresent = spiLock.locked() && SD.cardType() != CARD_NONE;
    }
    if (cardPresent) {
      // A present but full card is handled by telemetry retention; remounting
      // it would only interrupt that recovery path.
      consecutiveIoErrors_ = 0;
      return;
    }
    mounted_ = false;
    totalBytes_ = 0;
    setStatus("io_offline", false);
    Serial.println("[SD] repeated I/O failures; marked offline and remount scheduled");
  }
}

bool SdManager::exists(const char* path) const {
  if (!mounted_ || !path) return false;
  if (!heapAllowsFileOp()) return false;
  hal::SharedSpiBusLock spiLock("SD:exists");
  digitalWrite(lcdCsPin_, HIGH);
  const bool ok = SD.exists(path);
  digitalWrite(sdCsPin_, HIGH);
  return ok;
}

bool SdManager::appendLine(const char* path, const char* line) {
  if (!mounted_ || !path) return false;
  if (!heapAllowsFileOp()) {
    setStatus("low_heap", true);
    return false;
  }

  hal::SharedSpiBusLock spiLock("SD:append");
  digitalWrite(lcdCsPin_, HIGH);
  digitalWrite(sdCsPin_, LOW);

  File f = SD.open(path, FILE_APPEND);
  if (!f) {
    digitalWrite(sdCsPin_, HIGH);
    recordIoResult(false, "open_failed");
    return false;
  }

  const bool ok = (f.println(line) > 0);
  f.close();
  digitalWrite(sdCsPin_, HIGH);
  if (ok) setStatus("write_ok", false);
  recordIoResult(ok, "write_failed");
  return ok;
}

bool SdManager::writeTextFile(const char* path, const char* text) {
  if (!mounted_ || !path || !text) return false;
  if (!heapAllowsFileOp()) {
    setStatus("low_heap", true);
    return false;
  }

  hal::SharedSpiBusLock spiLock("SD:write");
  digitalWrite(lcdCsPin_, HIGH);
  digitalWrite(sdCsPin_, LOW);
  if (SD.exists(path)) {
    SD.remove(path);
  }

  File f = SD.open(path, FILE_WRITE);
  if (!f) {
    digitalWrite(sdCsPin_, HIGH);
    recordIoResult(false, "open_failed");
    return false;
  }

  const bool ok = f.print(text) > 0;
  f.close();
  digitalWrite(sdCsPin_, HIGH);
  if (ok) setStatus("write_ok", false);
  recordIoResult(ok, "write_failed");
  return ok;
}

bool SdManager::readTextFile(const char* path, char* out, size_t outLen) const {
  if (!mounted_ || !path || !out || outLen == 0) return false;
  if (!heapAllowsFileOp()) return false;
  out[0] = '\0';

  hal::SharedSpiBusLock spiLock("SD:read");
  digitalWrite(lcdCsPin_, HIGH);
  digitalWrite(sdCsPin_, LOW);
  File f = SD.open(path, FILE_READ);
  if (!f) {
    digitalWrite(sdCsPin_, HIGH);
    return false;
  }

  size_t n = 0;
  while (f.available() && n + 1 < outLen) {
    out[n++] = static_cast<char>(f.read());
  }
  out[n] = '\0';
  f.close();
  digitalWrite(sdCsPin_, HIGH);
  return n > 0;
}

bool SdManager::listDirectory(const char* path, SdFileEntry* entries, size_t maxEntries,
                              size_t offset, size_t& totalEntriesOut) const {
  totalEntriesOut = 0;
  if (!mounted_ || !path || !entries) return false;
  if (!heapAllowsFileOp()) return false;
  for (size_t i = 0; i < maxEntries; ++i) {
    entries[i] = SdFileEntry{};
  }

  hal::SharedSpiBusLock spiLock("SD:list");
  digitalWrite(lcdCsPin_, HIGH);

  File dir = SD.open(path);
  if (!dir || !dir.isDirectory()) {
    if (dir) dir.close();
    digitalWrite(sdCsPin_, HIGH);
    return false;
  }

  size_t stored = 0;
  while (true) {
    File entry = dir.openNextFile();
    if (!entry) {
      break;
    }

    const size_t index = totalEntriesOut++;
    if (index >= offset && stored < maxEntries) {
      SdFileEntry& dst = entries[stored++];
      copyEntryName(dst.name, sizeof(dst.name), entry.name());
      dst.sizeBytes = entry.isDirectory() ? 0U : static_cast<uint32_t>(entry.size());
      dst.isDirectory = entry.isDirectory();
    }
    entry.close();
  }

  dir.close();
  digitalWrite(sdCsPin_, HIGH);
  return true;
}

}  // namespace storage
