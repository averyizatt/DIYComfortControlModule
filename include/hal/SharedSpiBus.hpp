#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

namespace hal {

void initSharedSpiBusLock();

class SharedSpiBusLock {
 public:
  SharedSpiBusLock();
  ~SharedSpiBusLock();

  SharedSpiBusLock(const SharedSpiBusLock&) = delete;
  SharedSpiBusLock& operator=(const SharedSpiBusLock&) = delete;

  bool locked() const { return locked_; }

 private:
  bool locked_ = false;
};

}  // namespace hal
