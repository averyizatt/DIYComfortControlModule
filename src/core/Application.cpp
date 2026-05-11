#include "core/Application.hpp"

namespace ccm::core {

Application::Application()
    : gpsHal_(Serial1),
      buttons_(config::kButtonUpPin, config::kButtonDownPin, config::kButtonSelectPin),
      gps_(gpsHal_),
      env_(sensorsHal_),
      tach_(tachHal_),
      ui_(display_, touch_, buttons_) {}

void Application::begin() {
  qUiActions_ = xQueueCreate(8, sizeof(UiAction));
  qSensor_ = xQueueCreate(4, sizeof(SensorFrame));
  qRpm_ = xQueueCreate(4, sizeof(RpmFrame));
  qHealth_ = xQueueCreate(4, sizeof(HealthFrame));
  qCanCmd_ = xQueueCreate(8, sizeof(CanCommand));

  can_.begin(config::kCanBitrate);
  gps_.begin(config::kGpsBaud);
  env_.begin();
  ui_.begin();
  safety_.begin(config::kUndervoltageThreshold);
  tach_.begin(config::kTachOutPin, config::kTachOutChannel, config::kTachDuty);
  canScheduler_.begin();

  xTaskCreatePinnedToCore(canTaskEntry, "can", 4096, this, 3, nullptr, 0);
  xTaskCreatePinnedToCore(sensorTaskEntry, "sensors", 4096, this, 2, nullptr, 0);
  xTaskCreatePinnedToCore(gpsTaskEntry, "gps", 4096, this, 2, nullptr, 0);
  xTaskCreatePinnedToCore(tachTaskEntry, "tach", 4096, this, 3, nullptr, 0);
  xTaskCreatePinnedToCore(uiTaskEntry, "ui", 6144, this, 1, nullptr, 1);
  xTaskCreatePinnedToCore(diagnosticsTaskEntry, "diag", 4096, this, 1, nullptr, 1);
}

void Application::canTaskEntry(void* ctx) { static_cast<Application*>(ctx)->runCanTask(); }
void Application::sensorTaskEntry(void* ctx) { static_cast<Application*>(ctx)->runSensorTask(); }
void Application::gpsTaskEntry(void* ctx) { static_cast<Application*>(ctx)->runGpsTask(); }
void Application::tachTaskEntry(void* ctx) { static_cast<Application*>(ctx)->runTachTask(); }
void Application::uiTaskEntry(void* ctx) { static_cast<Application*>(ctx)->runUiTask(); }
void Application::diagnosticsTaskEntry(void* ctx) { static_cast<Application*>(ctx)->runDiagnosticsTask(); }

void Application::runCanTask() {
  uint32_t nodeBitmask = 0;
  uint32_t lastHeartbeat = 0;

  while (true) {
    const uint32_t now = millis();
    can::CanFrame incoming{};
    if (can_.receive(incoming)) {
      if (incoming.id == static_cast<uint16_t>(can::CanId::RpmTelemetry) && incoming.dlc >= 2) {
        RpmFrame frame{};
        frame.timestampMs = now;
        frame.source = RpmSource::CanBus;
        frame.rpm = static_cast<uint16_t>((incoming.data[0] << 8) | incoming.data[1]);
        queueOverwrite(qRpm_, &frame, 0);
      }
      nodeBitmask |= 1;
    }

    if ((now - lastHeartbeat) >= config::kCanHeartbeatMs) {
      can::CanFrame hb{};
      hb.id = static_cast<uint16_t>(can::CanId::MasterHeartbeat);
      hb.dlc = 2;
      hb.data[0] = can::kProtocolVersion;
      hb.data[1] = static_cast<uint8_t>(can::ModuleOwner::Master);
      can_.send(hb);
      lastHeartbeat = now;
    }

    CanCommand cmd{};
    while (xQueueReceive(qCanCmd_, &cmd, 0) == pdTRUE) {
      can::CanFrame tx{};
      tx.id = static_cast<uint16_t>(cmd.id);
      tx.dlc = cmd.len;
      for (uint8_t i = 0; i < cmd.len && i < 8; ++i) {
        tx.data[i] = cmd.data[i];
      }
      can_.send(tx);
    }

    HealthFrame health{};
    health.timestampMs = now;
    health.canOnline = true;
    health.gpsOnline = false;
    health.nodeBitmask = nodeBitmask;
    queueOverwrite(qHealth_, &health, 0);

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void Application::runSensorTask() {
  while (true) {
    SensorFrame frame{};
    frame.timestampMs = millis();
    frame.environment = env_.readEnvironment();
    frame.power = env_.readPower();
    queueOverwrite(qSensor_, &frame, pdMS_TO_TICKS(5));
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void Application::runGpsTask() {
  while (true) {
    gps_.poll();
    const auto gpsData = gps_.data();

    DashboardData d = state_.readDashboard();
    d.gpsOnline = gps_.online(millis(), 2000);
    d.speedKph = static_cast<uint16_t>(gpsData.speedKph);
    state_.updateDashboard(d);

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void Application::runTachTask() {
  tach_.startupSweep(2500, 250, 20);

  while (true) {
    RpmFrame rpm{};
    if (xQueueReceive(qRpm_, &rpm, pdMS_TO_TICKS(50)) == pdTRUE) {
      tach_.updateRpm(rpm.rpm);

      DashboardData d = state_.readDashboard();
      d.rpm = rpm.rpm;
      state_.updateDashboard(d);
    }
  }
}

void Application::runUiTask() {
  while (true) {
    DashboardData dashboard = state_.readDashboard();

    SensorFrame sensor{};
    if (xQueueReceive(qSensor_, &sensor, 0) == pdTRUE) {
      dashboard.environment = sensor.environment;
      dashboard.power = sensor.power;
    }

    HealthFrame health{};
    if (xQueueReceive(qHealth_, &health, 0) == pdTRUE) {
      dashboard.canOnline = health.canOnline;
    }

    state_.updateDashboard(dashboard);
    ui_.tick(dashboard);

    UiAction action{};
    if (ui_.pollAction(action)) {
      CanCommand cmd{};
      switch (action.type) {
        case UiActionType::ChangeMethMix:
          cmd.id = can::CanId::MethCommand;
          cmd.len = 2;
          cmd.data[0] = 0x01;
          cmd.data[1] = action.value;
          xQueueSend(qCanCmd_, &cmd, 0);
          break;
        case UiActionType::ToggleMethEnable:
          cmd.id = can::CanId::MethCommand;
          cmd.len = 2;
          cmd.data[0] = 0x02;
          cmd.data[1] = action.value;
          xQueueSend(qCanCmd_, &cmd, 0);
          break;
        case UiActionType::SetTaillightMode:
          cmd.id = can::CanId::TailLightCommand;
          cmd.len = 2;
          cmd.data[0] = 0x01;
          cmd.data[1] = action.value;
          xQueueSend(qCanCmd_, &cmd, 0);
          break;
        default:
          break;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(config::kDashboardRefreshMs));
  }
}

void Application::runDiagnosticsTask() {
  while (true) {
    const DashboardData d = state_.readDashboard();
    const auto faults = safety_.evaluate(d.power, d.canOnline, d.gpsOnline);
    state_.setFaults(faults);

    if (hasFault(faults, SystemFault::Undervoltage)) {
      display_.setBrightness(80);
    } else {
      display_.setBrightness(180);
    }

    vTaskDelay(pdMS_TO_TICKS(250));
  }
}

bool Application::queueOverwrite(QueueHandle_t queue, const void* item, TickType_t timeoutTicks) {
  if (!queue || !item) return false;
#if CONFIG_FREERTOS_UNICORE
  return xQueueOverwrite(queue, item) == pdTRUE;
#else
  if (uxQueueSpacesAvailable(queue) == 0) {
    uint8_t scratch[sizeof(SensorFrame)]{};
    xQueueReceive(queue, scratch, 0);
  }
  return xQueueSend(queue, item, timeoutTicks) == pdTRUE;
#endif
}

}  // namespace ccm::core
