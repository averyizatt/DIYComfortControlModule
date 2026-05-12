#include <Arduino.h>
#include <WiFi.h>
#include <esp_system.h>
#include <esp_task_wdt.h>

#include <cstring>

#include "can/can_manager.h"
#include "led/led_manager.h"
#include "race/race_manager.h"
#include "settings/settings_manager.h"
#include "state/vehicle_state.h"
#include "storage/log_manager.h"
#include "storage/sd_manager.h"
#include "touch/touch_manager.h"
#include "ui/asset_manager.h"
#include "web/web_server.h"

namespace {
canbus::CanManager g_can;
settings::SettingsManager g_settings;
led::LedManager g_led;
web::WebServerManager g_web;
storage::SdManager g_sd;
storage::LogManager g_logs;
race::RacePerformanceManager g_race;
touch::TouchManager g_touch;
ui::AssetManager g_assets;

// Hosyond 4.0" ST7796S + CTP + SD wiring placeholders.
constexpr uint8_t kPinLcdCs = 10;
constexpr uint8_t kPinLcdRst = 9;
constexpr uint8_t kPinLcdDc = 8;
constexpr uint8_t kPinLcdBacklight = 7;
constexpr uint8_t kPinSpiMosi = 11;
constexpr uint8_t kPinSpiMiso = 13;
constexpr uint8_t kPinSpiSck = 12;
constexpr uint8_t kPinTouchScl = 47;
constexpr uint8_t kPinTouchSda = 48;
constexpr uint8_t kPinTouchRst = 14;
constexpr uint8_t kPinTouchInt = 15;
constexpr uint8_t kPinSdCs = 16;

constexpr uint8_t kPinLedData1 = 38;
constexpr uint8_t kPinLedData2 = 39;
constexpr uint8_t kPinLedData3 = 40;
constexpr uint32_t kTaskWatchdogTimeoutS = 6;

void initTaskWatchdog() {
#if defined(ESP_IDF_VERSION_MAJOR) && (ESP_IDF_VERSION_MAJOR >= 5)
  esp_task_wdt_config_t cfg{};
  cfg.timeout_ms = kTaskWatchdogTimeoutS * 1000;
  cfg.idle_core_mask = (1U << portNUM_PROCESSORS) - 1U;
  cfg.trigger_panic = true;
  esp_task_wdt_init(&cfg);
#else
  esp_task_wdt_init(kTaskWatchdogTimeoutS, true);
#endif
}

void registerTaskWatchdog() {
  esp_task_wdt_add(nullptr);
}

void feedTaskWatchdog() {
  esp_task_wdt_reset();
}

void applySettingsToState() {
  state::g_vehicle_state.mutate([](state::VehicleState& s) {
    const esp_reset_reason_t reason = esp_reset_reason();
    s.reset_reason = static_cast<uint8_t>(reason);
    if (reason == ESP_RST_BROWNOUT) {
      s.brownout_reset_count++;
    }
#if defined(ESP_RST_TASK_WDT)
    if (reason == ESP_RST_TASK_WDT) {
      s.watchdog_reset_count++;
    }
#endif
#if defined(ESP_RST_INT_WDT)
    if (reason == ESP_RST_INT_WDT) {
      s.watchdog_reset_count++;
    }
#endif
#if defined(ESP_RST_WDT)
    if (reason == ESP_RST_WDT) {
      s.watchdog_reset_count++;
    }
#endif
    s.heap_free_bytes = ESP.getFreeHeap();
    s.esp_die_temp_c = static_cast<int8_t>(temperatureRead());
  });

  state::VehicleState current = state::g_vehicle_state.read();
  g_settings.loadIntoState(current);
  state::g_vehicle_state.write(current);
}

void setupWifiFromSettings() {
  const auto& cfg = g_settings.data();
  if (cfg.wifi_ap_mode || cfg.wifi_ssid[0] == '\0') {
    WiFi.mode(WIFI_AP);
    const char* pass = cfg.wifi_password[0] ? cfg.wifi_password : nullptr;
    WiFi.softAP("FoxbodyCabinMaster", pass);
    state::g_vehicle_state.mutate([](state::VehicleState& s) {
      s.wifi_ap_mode = true;
      s.wifi_connected = true;
    });
  } else {
    WiFi.mode(WIFI_STA);
    WiFi.begin(cfg.wifi_ssid, cfg.wifi_password);
    state::g_vehicle_state.mutate([](state::VehicleState& s) { s.wifi_ap_mode = false; });
  }
}

void canTask(void*) {
  registerTaskWatchdog();
  while (true) {
    g_can.tick();
    feedTaskWatchdog();
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void ledTask(void*) {
  registerTaskWatchdog();
  while (true) {
    const state::VehicleState s = state::g_vehicle_state.read();
    g_led.tick(s);
    state::g_vehicle_state.mutate([](state::VehicleState& st) {
      if (st.led_startup_preview) st.led_startup_preview = false;
    });
    feedTaskWatchdog();
    vTaskDelay(pdMS_TO_TICKS(8));
  }
}

void webTask(void*) {
  registerTaskWatchdog();
  while (true) {
    g_web.tick();
    feedTaskWatchdog();
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void raceTask(void*) {
  registerTaskWatchdog();
  while (true) {
    g_race.tick(millis());
    feedTaskWatchdog();
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void touchTask(void*) {
  registerTaskWatchdog();
  while (true) {
    const touch::TouchSample t = g_touch.read();
    state::g_vehicle_state.mutate([&](state::VehicleState& s) {
      s.touch_online = g_touch.online();
      if (t.touched) {
        s.input_flags |= can_protocol::input_flag::TOUCH;
      }
    });
    feedTaskWatchdog();
    vTaskDelay(pdMS_TO_TICKS(25));
  }
}

void storageTask(void*) {
  registerTaskWatchdog();
  uint32_t lastLogMs = 0;
  uint32_t frameCounter = 0;
  uint32_t fpsStartMs = millis();

  while (true) {
    const uint32_t nowMs = millis();
    g_logs.tick(nowMs);

    if ((nowMs - lastLogMs) >= 1000) {
      lastLogMs = nowMs;
      const state::VehicleState s = state::g_vehicle_state.read();
      char canLine[96];
      char gpsLine[96];
      char methLine[96];
      char raceLine[160];
      snprintf(canLine, sizeof(canLine), "rx=%lu,tx=%lu,last_id=%u", static_cast<unsigned long>(s.can_rx_count), static_cast<unsigned long>(s.can_tx_count), s.can_last_rx_id);
      snprintf(gpsLine, sizeof(gpsLine), "fix=%u,sat=%u,speed=%.1f", s.gps_fix ? 1U : 0U, s.gps_satellites, static_cast<double>(s.speed));
      snprintf(methLine, sizeof(methLine), "state=%u,ratio=%u,duty=%u", static_cast<uint8_t>(s.meth_state), s.meth_selected_ratio_percent, s.meth_pump_duty);
      snprintf(raceLine, sizeof(raceLine), "mode=%u,running=%u,0_60=%.3f,qtr=%.3f,lap_best=%.3f,quality=%u", static_cast<uint8_t>(s.race_mode),
               s.race_running ? 1U : 0U, static_cast<double>(s.race_0_60_s), static_cast<double>(s.race_quarter_mile_et_s),
               static_cast<double>(s.race_best_lap_s), s.race_quality_percent);
      g_logs.enqueue("can", canLine);
      g_logs.enqueue("gps", gpsLine);
      g_logs.enqueue("meth", methLine);
      g_logs.enqueue("race", raceLine);
      if (s.fault_flags != 0) {
        char faultLine[48];
        snprintf(faultLine, sizeof(faultLine), "fault_flags=%u", s.fault_flags);
        g_logs.enqueue("faults", faultLine);
        g_logs.flushCritical();
      }
    }

    frameCounter++;
    if ((nowMs - fpsStartMs) >= 1000) {
      const float fps = frameCounter * 1000.0f / static_cast<float>(nowMs - fpsStartMs);
      frameCounter = 0;
      fpsStartMs = nowMs;

      state::g_vehicle_state.mutate([&](state::VehicleState& s) {
        s.sd_mounted = g_sd.mounted();
        s.sd_size_bytes = g_sd.totalBytes();
        s.sd_used_bytes = g_sd.usedBytes();
        s.sd_write_error_count = g_sd.errorCount() + g_logs.droppedCount();
        strncpy(s.last_sd_write_status, g_sd.lastStatus(), sizeof(s.last_sd_write_status) - 1);
        s.last_sd_write_status[sizeof(s.last_sd_write_status) - 1] = '\0';
        strncpy(s.current_log_file, g_logs.currentFile(), sizeof(s.current_log_file) - 1);
        s.current_log_file[sizeof(s.current_log_file) - 1] = '\0';
        s.ui_fps = fps;
        s.heap_free_bytes = ESP.getFreeHeap();
        s.esp_die_temp_c = static_cast<int8_t>(temperatureRead());
      });
    }

    feedTaskWatchdog();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void heartbeatTask(void*) {
  registerTaskWatchdog();
  while (true) {
    state::g_vehicle_state.mutate([](state::VehicleState& s) {
      s.input_flags = 0;
      if (s.fault_flags != 0) {
        s.master_state = 2;
      } else {
        s.master_state = 1;
      }
    });
    feedTaskWatchdog();
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}
}  // namespace

void setup() {
  Serial.begin(115200);
  delay(50);

  pinMode(kPinLcdRst, OUTPUT);
  pinMode(kPinLcdDc, OUTPUT);
  pinMode(kPinLcdBacklight, OUTPUT);
  digitalWrite(kPinLcdRst, HIGH);
  digitalWrite(kPinLcdDc, HIGH);

  state::g_vehicle_state.begin();
  initTaskWatchdog();
  registerTaskWatchdog();
  g_settings.begin();
  applySettingsToState();
  analogWrite(kPinLcdBacklight, state::g_vehicle_state.read().display_brightness);
  setupWifiFromSettings();

  g_can.begin(true);

  g_touch.begin(Wire, kPinTouchSda, kPinTouchScl, kPinTouchRst, kPinTouchInt);
  g_sd.begin(kPinSpiSck, kPinSpiMiso, kPinSpiMosi, kPinLcdCs, kPinSdCs);
  g_assets.begin(&g_sd);
  g_logs.begin(&g_sd);
  g_logs.setSessionPrefix(String("boot_") + String(millis()));
  g_race.begin(&state::g_vehicle_state, &g_settings, &g_logs);

  g_led.begin(kPinLedData1, kPinLedData2, kPinLedData3, 18);
  g_web.begin(&state::g_vehicle_state, &g_settings, &g_can, &g_race);

  xTaskCreatePinnedToCore(canTask, "can_task", 6144, nullptr, 3, nullptr, 0);
  xTaskCreatePinnedToCore(ledTask, "led_task", 4096, nullptr, 2, nullptr, 1);
  xTaskCreatePinnedToCore(webTask, "web_task", 6144, nullptr, 1, nullptr, 1);
  xTaskCreatePinnedToCore(storageTask, "storage_task", 6144, nullptr, 1, nullptr, 1);
  xTaskCreatePinnedToCore(raceTask, "race_task", 4096, nullptr, 1, nullptr, 1);
  xTaskCreatePinnedToCore(touchTask, "touch_task", 4096, nullptr, 1, nullptr, 1);
  xTaskCreatePinnedToCore(heartbeatTask, "hb_task", 3072, nullptr, 1, nullptr, 1);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
