#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include "can/CanProtocol.hpp"
#include "config/SystemConfig.hpp"
#include "core/SystemState.hpp"
#include "core/TaskContracts.hpp"
#include "gps/GpsService.hpp"
#include "hal/HardwareAdapters.hpp"
#include "safety/SafetyManager.hpp"
#include "sensors/EnvironmentService.hpp"
#include "tach/TachController.hpp"
#include "ui/UiManager.hpp"

namespace ccm::core {

class Application {
 public:
  Application();
  void begin();

 private:
  static void canTaskEntry(void* ctx);
  static void sensorTaskEntry(void* ctx);
  static void gpsTaskEntry(void* ctx);
  static void tachTaskEntry(void* ctx);
  static void uiTaskEntry(void* ctx);
  static void diagnosticsTaskEntry(void* ctx);

  void runCanTask();
  void runSensorTask();
  void runGpsTask();
  void runTachTask();
  void runUiTask();
  void runDiagnosticsTask();

  bool queueOverwrite(QueueHandle_t queue, const void* item);

  SystemState state_;

  hal::NullDisplay display_;
  hal::NullTouch touch_;
  hal::TwaiCanAdapter can_;
  hal::UartGpsAdapter gpsHal_;
  hal::LedcTachAdapter tachHal_;
  hal::StubSensorAdapter sensorsHal_;

  gps::GpsService gps_;
  sensors::EnvironmentService env_;
  safety::SafetyManager safety_;
  tach::TachController tach_;
  ui::UiManager ui_;
  can::CanScheduler canScheduler_;

  QueueHandle_t qUiActions_ = nullptr;
  QueueHandle_t qSensor_ = nullptr;
  QueueHandle_t qRpm_ = nullptr;
  QueueHandle_t qHealth_ = nullptr;
  QueueHandle_t qCanCmd_ = nullptr;
};

}  // namespace ccm::core
