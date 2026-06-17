#include "hal/SharedSpiBus.hpp"

namespace hal {
namespace {

StaticSemaphore_t s_spiMutexStorage;
SemaphoreHandle_t s_spiMutex = nullptr;
portMUX_TYPE s_initMux = portMUX_INITIALIZER_UNLOCKED;

SemaphoreHandle_t spiMutex() {
  if (s_spiMutex != nullptr) {
    return s_spiMutex;
  }

  portENTER_CRITICAL(&s_initMux);
  if (s_spiMutex == nullptr) {
    s_spiMutex = xSemaphoreCreateMutexStatic(&s_spiMutexStorage);
  }
  portEXIT_CRITICAL(&s_initMux);
  return s_spiMutex;
}

}  // namespace

void initSharedSpiBusLock() {
  (void)spiMutex();
}

SharedSpiBusLock::SharedSpiBusLock() {
  SemaphoreHandle_t mutex = spiMutex();
  if (mutex == nullptr) {
    return;
  }

  const TickType_t waitTicks =
      (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) ? portMAX_DELAY : 0;
  locked_ = (xSemaphoreTake(mutex, waitTicks) == pdTRUE);
}

SharedSpiBusLock::~SharedSpiBusLock() {
  if (locked_ && s_spiMutex != nullptr) {
    xSemaphoreGive(s_spiMutex);
  }
}

}  // namespace hal
