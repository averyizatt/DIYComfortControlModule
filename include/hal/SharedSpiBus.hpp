#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

namespace hal {

void initSharedSpiBusLock();
void deassertSharedSpiDevices();
bool sharedSpiBusLocked();
const char* sharedSpiBusOwner();

class SharedSpiBusLock {
 public:
  explicit SharedSpiBusLock(const char* owner = nullptr, TickType_t waitTicks = portMAX_DELAY);
  ~SharedSpiBusLock();

  SharedSpiBusLock(const SharedSpiBusLock&) = delete;
  SharedSpiBusLock& operator=(const SharedSpiBusLock&) = delete;

  bool locked() const { return locked_; }

 private:
  const char* owner_ = nullptr;
  bool locked_ = false;
};

}  // namespace hal
