#include "hal/SharedSpiBus.hpp"

#include "pin_map.h"

#ifndef CCM_SPI_DIAG_VERBOSE
#define CCM_SPI_DIAG_VERBOSE 0
#endif

#ifndef CCM_SPI_DIAG_SAMPLE
#define CCM_SPI_DIAG_SAMPLE 32
#endif

#ifndef CCM_SPI_LCD_RELEASE_GUARD_US
#define CCM_SPI_LCD_RELEASE_GUARD_US 300
#endif

#ifndef CCM_SPI_RELEASE_GUARD_US
#define CCM_SPI_RELEASE_GUARD_US 20
#endif

#ifndef CCM_SPI_LCD_ACQUIRE_GUARD_US
#define CCM_SPI_LCD_ACQUIRE_GUARD_US 300
#endif

#ifndef CCM_SPI_NONLCD_AFTER_LCD_GUARD_US
#define CCM_SPI_NONLCD_AFTER_LCD_GUARD_US 500
#endif

namespace hal {
namespace {

StaticSemaphore_t s_spiMutexStorage;
SemaphoreHandle_t s_spiMutex = nullptr;
portMUX_TYPE s_initMux = portMUX_INITIALIZER_UNLOCKED;
bool s_csPinsConfigured = false;
volatile bool s_spiLocked = false;
const char* s_spiOwner = nullptr;
uint32_t s_spiSeq = 0;
uint32_t s_spiLockDepth = 0;
bool s_lastReleaseKnown = false;
bool s_lastReleaseWasLcd = false;
const char* s_lastReleaseOwner = nullptr;

void writeCsHigh(uint8_t pin) {
  if (pin == 255U) return;
  digitalWrite(pin, HIGH);
}

void configureSharedChipSelects() {
  if (s_csPinsConfigured) {
    return;
  }

  if (pins::kLcdCs != 255U) {
    pinMode(pins::kLcdCs, OUTPUT);
  }
  if (pins::kSdCs != 255U) {
    pinMode(pins::kSdCs, OUTPUT);
  }
  if (pins::kCanSpiCs != 255U) {
    pinMode(pins::kCanSpiCs, OUTPUT);
  }

  writeCsHigh(pins::kLcdCs);
  writeCsHigh(pins::kSdCs);
  writeCsHigh(pins::kCanSpiCs);
  s_csPinsConfigured = true;
}

void deassertSharedChipSelects() {
  configureSharedChipSelects();
  writeCsHigh(pins::kLcdCs);
  writeCsHigh(pins::kSdCs);
  writeCsHigh(pins::kCanSpiCs);
}

bool shouldLogSpi(uint32_t seq) {
  if (!CCM_SPI_DIAG_VERBOSE) {
    return false;
  }
  constexpr uint32_t sample =
      (CCM_SPI_DIAG_SAMPLE < 1) ? 1U : static_cast<uint32_t>(CCM_SPI_DIAG_SAMPLE);
  return seq <= 12U || (seq % sample) == 0U;
}

bool ownerIsLcd(const char* owner) {
  return owner && owner[0] == 'L' && owner[1] == 'C' && owner[2] == 'D' && owner[3] == ':';
}

void applyReleaseGuard(const char* owner) {
  const uint32_t guardUs = ownerIsLcd(owner)
      ? static_cast<uint32_t>(CCM_SPI_LCD_RELEASE_GUARD_US)
      : static_cast<uint32_t>(CCM_SPI_RELEASE_GUARD_US);
  if (guardUs > 0U) {
    delayMicroseconds(guardUs);
  }
}

void applyAcquireGuard(const char* owner) {
  if (!s_lastReleaseKnown) {
    return;
  }

  const bool acquiringLcd = ownerIsLcd(owner);
  uint32_t guardUs = 0;
  if (acquiringLcd && !s_lastReleaseWasLcd) {
    guardUs = static_cast<uint32_t>(CCM_SPI_LCD_ACQUIRE_GUARD_US);
  } else if (!acquiringLcd && s_lastReleaseWasLcd) {
    guardUs = static_cast<uint32_t>(CCM_SPI_NONLCD_AFTER_LCD_GUARD_US);
  }

  if (guardUs > 0U) {
    deassertSharedChipSelects();
    delayMicroseconds(guardUs);
    deassertSharedChipSelects();
  }
}

SemaphoreHandle_t spiMutex() {
  if (s_spiMutex != nullptr) {
    return s_spiMutex;
  }

  portENTER_CRITICAL(&s_initMux);
  if (s_spiMutex == nullptr) {
    s_spiMutex = xSemaphoreCreateRecursiveMutexStatic(&s_spiMutexStorage);
  }
  portEXIT_CRITICAL(&s_initMux);
  return s_spiMutex;
}

}  // namespace

void initSharedSpiBusLock() {
  configureSharedChipSelects();
  (void)spiMutex();
}

void deassertSharedSpiDevices() {
  deassertSharedChipSelects();
}

bool sharedSpiBusLocked() {
  return s_spiLocked;
}

const char* sharedSpiBusOwner() {
  return s_spiOwner ? s_spiOwner : "none";
}

SharedSpiBusLock::SharedSpiBusLock(const char* owner) : owner_(owner ? owner : "?") {
  SemaphoreHandle_t mutex = spiMutex();
  if (mutex == nullptr) {
    return;
  }

  const TickType_t waitTicks =
      (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) ? portMAX_DELAY : 0;
  locked_ = (xSemaphoreTakeRecursive(mutex, waitTicks) == pdTRUE);
  if (locked_) {
    const bool outerLock = (s_spiLockDepth == 0);
    ++s_spiLockDepth;
    if (outerLock) {
      applyAcquireGuard(owner_);
      s_spiLocked = true;
      s_spiOwner = owner_;
      ++s_spiSeq;
    }
    deassertSharedChipSelects();
    if (outerLock && shouldLogSpi(s_spiSeq)) {
      Serial.printf("[SPI:LOCK] #%lu owner=%s start depth=%lu CS lcd=%d sd=%d can=%d\n",
                     static_cast<unsigned long>(s_spiSeq),
                     owner_,
                     static_cast<unsigned long>(s_spiLockDepth),
                     digitalRead(pins::kLcdCs),
                     digitalRead(pins::kSdCs),
                     digitalRead(pins::kCanSpiCs));
    }
  }
}

SharedSpiBusLock::~SharedSpiBusLock() {
  if (locked_ && s_spiMutex != nullptr) {
    const uint32_t seq = s_spiSeq;
    const bool finalLock = (s_spiLockDepth <= 1U);
    deassertSharedChipSelects();
    if (finalLock && shouldLogSpi(seq)) {
      const uint32_t guardUs = ownerIsLcd(s_spiOwner)
          ? static_cast<uint32_t>(CCM_SPI_LCD_RELEASE_GUARD_US)
          : static_cast<uint32_t>(CCM_SPI_RELEASE_GUARD_US);
      Serial.printf("[SPI:LOCK] #%lu owner=%s end depth=%lu guard=%luus CS lcd=%d sd=%d can=%d\n",
                     static_cast<unsigned long>(seq),
                     owner_ ? owner_ : "?",
                     static_cast<unsigned long>(s_spiLockDepth),
                     static_cast<unsigned long>(guardUs),
                     digitalRead(pins::kLcdCs),
                     digitalRead(pins::kSdCs),
                     digitalRead(pins::kCanSpiCs));
    }
    if (s_spiLockDepth > 0) {
      --s_spiLockDepth;
    }
    if (finalLock) {
      const char* releasedOwner = s_spiOwner;
      const bool releasedWasLcd = ownerIsLcd(releasedOwner);
      applyReleaseGuard(releasedOwner);
      s_lastReleaseOwner = releasedOwner;
      s_lastReleaseWasLcd = releasedWasLcd;
      s_lastReleaseKnown = true;
      s_spiOwner = nullptr;
      s_spiLocked = false;
      s_spiLockDepth = 0;
    }
    xSemaphoreGiveRecursive(s_spiMutex);
  }
}

}  // namespace hal
