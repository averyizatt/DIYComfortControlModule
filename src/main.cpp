#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <esp_system.h>
#include <esp_task_wdt.h>

#include <cstring>

#include "can/can_manager.h"
#include "hal/HardwareAdapters.hpp"
#include "gps/GpsService.hpp"
#include "led/led_manager.h"
#include "knock/knock_monitor.h"
#include "pin_map.h"
#include "race/race_manager.h"
#include "settings/settings_manager.h"
#include "state/vehicle_state.h"
#include "storage/log_manager.h"
#include "storage/sd_manager.h"
#include "touch/touch_manager.h"
#include "ui/asset_manager.h"
#include "ui/screen_dashboard.h"
#include "web/web_server.h"

namespace {
canbus::CanManager g_can;
settings::SettingsManager g_settings;

// GPS: UartGpsAdapter owns Serial2 (GPIO41=RX, GPIO42=TX @ 9600 baud)
static ccm::hal::UartGpsAdapter g_gpsHal(Serial2);
static ccm::gps::GpsService     g_gps(g_gpsHal);

led::LedManager g_led;
web::WebServerManager g_web;
storage::SdManager g_sd;
storage::LogManager g_logs;
race::RacePerformanceManager g_race;
knock::KnockMonitor g_knock;
touch::TouchManager g_touch;
ui::AssetManager g_assets;
ui::ScreenDashboard g_screen;

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
    g_screen.handleTouch(t, millis());
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

void screenTask(void*) {
  registerTaskWatchdog();
  uint32_t lastFpsMs = millis();
  uint32_t frameCount = 0;

  while (true) {
    const state::VehicleState s = state::g_vehicle_state.read();
    g_screen.tick(s, millis());
    frameCount++;

    const uint32_t nowMs = millis();
    if ((nowMs - lastFpsMs) >= 1000) {
      const float fps = frameCount * 1000.0f / static_cast<float>(nowMs - lastFpsMs);
      frameCount = 0;
      lastFpsMs = nowMs;
      state::g_vehicle_state.mutate([&](state::VehicleState& st) { st.ui_fps = fps; });
    }

    feedTaskWatchdog();
    vTaskDelay(pdMS_TO_TICKS(33));
  }
}

void storageTask(void*) {
  registerTaskWatchdog();
  uint32_t lastLogMs = 0;
  uint32_t lastStorageStatusMs = 0;
  uint8_t lastKnockEventCount = 0;

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

      char knockLine[220];
      const bool knockEventCountChanged = s.knock_event_count != lastKnockEventCount;
      lastKnockEventCount = s.knock_event_count;
      snprintf(knockLine, sizeof(knockLine),
               "%lu,%u,%.0f,%.1f,%.1f,%u,%u,%u,%.2f,%.2f,%.2f,%u,%u",
               static_cast<unsigned long>(nowMs), static_cast<unsigned>(s.rpm),
               static_cast<double>(s.boost_kpa), static_cast<double>(s.intake_temp),
               static_cast<double>(s.engine_bay_temp), static_cast<unsigned>(s.meth_state),
               static_cast<unsigned>(s.meth_pump_duty), static_cast<unsigned>(s.meth_tank_level),
               static_cast<double>(s.knock_energy), static_cast<double>(s.knock_baseline),
               static_cast<double>(s.knock_threshold), knockEventCountChanged ? 1U : 0U,
               static_cast<unsigned>(s.knock_fault_code_pending));
      g_logs.enqueue("knock", knockLine);

      if (s.fault_flags != 0) {
        char faultLine[48];
        snprintf(faultLine, sizeof(faultLine), "fault_flags=%u", s.fault_flags);
        g_logs.enqueue("faults", faultLine);
        g_logs.flushCritical();
      }
    }

    if ((nowMs - lastStorageStatusMs) >= 1000) {
      lastStorageStatusMs = nowMs;
      state::g_vehicle_state.mutate([&](state::VehicleState& s) {
        s.sd_mounted = g_sd.mounted();
        s.sd_size_bytes = g_sd.totalBytes();
        s.sd_used_bytes = g_sd.usedBytes();
        s.sd_write_error_count = g_sd.errorCount() + g_logs.droppedCount();
        strncpy(s.last_sd_write_status, g_sd.lastStatus(), sizeof(s.last_sd_write_status) - 1);
        s.last_sd_write_status[sizeof(s.last_sd_write_status) - 1] = '\0';
        strncpy(s.current_log_file, g_logs.currentFile(), sizeof(s.current_log_file) - 1);
        s.current_log_file[sizeof(s.current_log_file) - 1] = '\0';
        s.heap_free_bytes = ESP.getFreeHeap();
        s.esp_die_temp_c = static_cast<int8_t>(temperatureRead());
      });
    }

    feedTaskWatchdog();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void knockTask(void*) {
  registerTaskWatchdog();
  while (true) {
    g_knock.tick(millis());
    feedTaskWatchdog();
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void gpsTask(void*) {
  registerTaskWatchdog();

  Serial0.println("[GPS] task started — waiting for fix");

  bool lastFix = false;
  uint32_t lastSearchMs = 0;

  while (true) {
    g_gps.poll();
    const ccm::core::GpsData d = g_gps.data();
    const uint32_t nowMs = millis();

    if (d.validFix != lastFix) {
      lastFix = d.validFix;
      if (d.validFix) {
        Serial0.printf("[GPS] fix acquired — sats=%lu lat=%.6f lon=%.6f\n",
          static_cast<unsigned long>(d.satellites), d.latitude, d.longitude);
      } else {
        Serial0.println("[GPS] fix lost — searching...");
      }
    } else if (!d.validFix && (nowMs - lastSearchMs) >= 10000U) {
      lastSearchMs = nowMs;
      Serial0.println("[GPS] searching...");
    }

    state::g_vehicle_state.mutate([&](state::VehicleState& s) {
      s.gps_fix        = d.validFix;
      s.speed          = d.speedKph;
      s.gps_satellites = static_cast<uint8_t>(d.satellites > 255U ? 255U : d.satellites);
      s.gps_latitude   = d.latitude;
      s.gps_longitude  = d.longitude;
      s.gps_altitude_m = static_cast<int16_t>(d.altitudeM);
      s.gps_stale      = !d.validFix || ((nowMs - d.lastFixMs) > 5000U);
      if (d.validFix) s.last_gps_ms = nowMs;
    });
    feedTaskWatchdog();
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void heartbeatTask(void*) {
  registerTaskWatchdog();
  while (true) {
    state::g_vehicle_state.mutate([](state::VehicleState& s) {
      s.input_flags = 0;
      if (s.fault_flags != 0) {
        s.master_state = static_cast<uint8_t>(can_protocol::MasterState::WARN);
      } else {
        s.master_state = static_cast<uint8_t>(can_protocol::MasterState::RUN);
      }
    });
    feedTaskWatchdog();
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}
}  // namespace

void setup() {
  Serial.begin(115200);
  Serial0.begin(115200);  // UART0 -> COM8 (CH343 bridge) for hardware diagnostics
  delay(200);  // give COM8 monitor time to connect before first prints

  pinMode(pins::kLcdRst, OUTPUT);
  pinMode(pins::kLcdDc, OUTPUT);
  digitalWrite(pins::kLcdRst, HIGH);
  digitalWrite(pins::kLcdDc, HIGH);
  if (pins::kLcdBacklight != 255) {
    pinMode(pins::kLcdBacklight, OUTPUT);
  }

  state::g_vehicle_state.begin();
  initTaskWatchdog();
  registerTaskWatchdog();
  g_settings.begin();
  applySettingsToState();
  if (pins::kLcdBacklight != 255) {
    analogWrite(pins::kLcdBacklight, state::g_vehicle_state.read().display_brightness);
  }
  setupWifiFromSettings();

  g_can.begin(true);
  g_gps.begin(pins::kGpsBaud);  // Serial2 GPIO41 RX / GPIO42 TX @ 9600 baud (no-op: already begun above)

  // The display uses HSPI (SPI3) via Arduino_ESP32SPI – no SPI.begin() needed
  g_touch.begin(Wire, pins::kTouchSda, pins::kTouchScl, pins::kTouchRst, pins::kTouchInt);
  // SD uses FSPI (SPI2). Must explicitly init with our custom pins before
  // SD.begin() — otherwise the library falls back to ESP32-S3 defaults (11/12/13).
  SPI.begin(pins::kSpiSck, pins::kSpiMiso, pins::kSpiMosi, pins::kSdCs);
  g_sd.begin(pins::kLcdCs, pins::kSdCs);
  g_assets.begin(&g_sd);
  g_logs.begin(&g_sd);
  g_logs.setSessionPrefix(String("boot_") + String(millis()));
  state::g_vehicle_state.mutate([](state::VehicleState& s) {
    if (s.knock_adc_pin == 0) s.knock_adc_pin = pins::kKnockAdc;
  });
  g_knock.begin(&state::g_vehicle_state, &g_settings, &g_logs, &g_sd, &g_can);
  g_race.begin(&state::g_vehicle_state, &g_settings, &g_logs);
  g_screen.attach(&g_can, &g_race, &g_settings);
  Serial0.printf("[SETUP] heap free before screen init: %lu bytes\n",
    static_cast<unsigned long>(ESP.getFreeHeap()));
  Serial0.printf("[SCREEN] pins  CS=%d RST=%d DC=%d SCK=%d MOSI=%d MISO=%d\n",
    pins::kLcdCs, pins::kLcdRst, pins::kLcdDc,
    pins::kSpiSck, pins::kSpiMosi, pins::kSpiMiso);
  const bool screenOk = g_screen.begin(
    pins::kLcdCs, pins::kLcdRst, pins::kLcdDc,
    pins::kSpiSck, pins::kSpiMosi, pins::kSpiMiso);
  Serial0.printf("[SCREEN] begin() -> %s\n", screenOk ? "OK" : "FAILED");

  g_led.begin(pins::kLedData1, pins::kLedData2, pins::kLedData3, 18, 7);
  g_web.begin(&state::g_vehicle_state, &g_settings, &g_can, &g_race);

  xTaskCreatePinnedToCore(canTask,     "can_task",     6144, nullptr, 3, nullptr, 0);
  xTaskCreatePinnedToCore(gpsTask,     "gps_task",     6144, nullptr, 2, nullptr, 0);
  xTaskCreatePinnedToCore(ledTask, "led_task", 4096, nullptr, 2, nullptr, 1);
  xTaskCreatePinnedToCore(webTask, "web_task", 6144, nullptr, 1, nullptr, 1);
  xTaskCreatePinnedToCore(storageTask, "storage_task", 6144, nullptr, 1, nullptr, 1);
  xTaskCreatePinnedToCore(knockTask, "knock_task", 6144, nullptr, 2, nullptr, 0);
  xTaskCreatePinnedToCore(raceTask, "race_task", 4096, nullptr, 1, nullptr, 1);
  xTaskCreatePinnedToCore(touchTask, "touch_task", 4096, nullptr, 1, nullptr, 1);
  xTaskCreatePinnedToCore(screenTask, "screen_task", 8192, nullptr, 1, nullptr, 1);
  xTaskCreatePinnedToCore(heartbeatTask, "hb_task", 3072, nullptr, 1, nullptr, 1);
}

void loop() {
  feedTaskWatchdog();
  vTaskDelay(pdMS_TO_TICKS(1000));
}
