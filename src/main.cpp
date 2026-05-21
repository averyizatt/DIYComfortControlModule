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
#include "sensors/pressure_sensor.h"
#include "sensors/thermistor_sensor.h"
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
sensors::ThermistorSensor g_iatThermistor;
sensors::ThermistorSensor g_engineBayThermistor;
sensors::ThermistorSensor g_cabinThermistor;
sensors::ThermistorSensor g_ambientThermistor;
sensors::PressureSensor g_oilPressureSensor;
sensors::PressureSensor g_fuelPressureSensor;
sensors::PressureSensor g_methPressureSensor;
sensors::PressureSensor g_boostRefPressureSensor;
sensors::PressureSensor g_sparePressure1Sensor;
sensors::PressureSensor g_sparePressure2Sensor;

constexpr uint32_t kTaskWatchdogTimeoutS = 6;
constexpr uint16_t kAnalogFaultIat = 1U << 0;
constexpr uint16_t kAnalogFaultEngineBay = 1U << 1;
constexpr uint16_t kAnalogFaultCabin = 1U << 2;
constexpr uint16_t kAnalogFaultAmbient = 1U << 3;
constexpr uint16_t kAnalogFaultOil = 1U << 4;
constexpr uint16_t kAnalogFaultFuel = 1U << 5;
constexpr uint16_t kAnalogFaultMeth = 1U << 6;
constexpr uint16_t kAnalogFaultBoostRef = 1U << 7;
constexpr uint16_t kAnalogFaultSpare1 = 1U << 8;
constexpr uint16_t kAnalogFaultSpare2 = 1U << 9;
constexpr uint16_t kFaultFlagTempSensors = 0x0800;
constexpr uint16_t kFaultFlagPressureSensors = 0x1000;

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

void configureAnalogSensorsFromState(const state::VehicleState& cfg) {
  const bool sensorsEnabled = cfg.analog_sensors_enabled;
  const uint16_t sampleMs = cfg.analog_sensor_sample_ms < 10 ? 10 : cfg.analog_sensor_sample_ms;

  sensors::ThermistorConfig tc{};
  tc.pullup_ohms = cfg.thermistor_pullup_ohms;
  tc.update_period_ms = sampleMs;
  tc.stale_timeout_ms = static_cast<uint16_t>(sampleMs * 6U);
  tc.filter_alpha = 0.20f;

  tc.enabled = sensorsEnabled && cfg.iat_sensor_enabled;
  tc.adc_pin = cfg.iat_adc_pin;
  g_iatThermistor.configure(tc);
  g_iatThermistor.begin();

  tc.enabled = sensorsEnabled && cfg.engine_bay_sensor_enabled;
  tc.adc_pin = cfg.engine_bay_adc_pin;
  tc.max_valid_temp_c = 200.0f;
  g_engineBayThermistor.configure(tc);
  g_engineBayThermistor.begin();

  tc.enabled = sensorsEnabled && cfg.cabin_temp_sensor_enabled;
  tc.adc_pin = cfg.cabin_temp_adc_pin;
  tc.max_valid_temp_c = 120.0f;
  g_cabinThermistor.configure(tc);
  g_cabinThermistor.begin();

  tc.enabled = sensorsEnabled && cfg.ambient_temp_sensor_enabled;
  tc.adc_pin = cfg.ambient_temp_adc_pin;
  tc.max_valid_temp_c = 120.0f;
  g_ambientThermistor.configure(tc);
  g_ambientThermistor.begin();

  sensors::PressureSensorConfig pc{};
  pc.sensor_min_v = cfg.pressure_sensor_min_v;
  pc.sensor_max_v = cfg.pressure_sensor_max_v;
  pc.pressure_max_psi = cfg.pressure_sensor_max_psi;
  pc.update_period_ms = sampleMs;
  pc.stale_timeout_ms = static_cast<uint16_t>(sampleMs * 6U);
  pc.filter_alpha = 0.20f;
  pc.max_valid_psi = cfg.pressure_sensor_max_psi + 25.0f;

  pc.enabled = sensorsEnabled && cfg.oil_pressure_sensor_enabled;
  pc.adc_pin = cfg.oil_pressure_adc_pin;
  g_oilPressureSensor.configure(pc);
  g_oilPressureSensor.begin();

  pc.enabled = sensorsEnabled && cfg.fuel_pressure_sensor_enabled;
  pc.adc_pin = cfg.fuel_pressure_adc_pin;
  g_fuelPressureSensor.configure(pc);
  g_fuelPressureSensor.begin();

  pc.enabled = sensorsEnabled && cfg.meth_pressure_sensor_enabled;
  pc.adc_pin = cfg.meth_pressure_adc_pin;
  g_methPressureSensor.configure(pc);
  g_methPressureSensor.begin();

  pc.enabled = sensorsEnabled && cfg.boost_ref_pressure_sensor_enabled;
  pc.adc_pin = cfg.boost_ref_pressure_adc_pin;
  g_boostRefPressureSensor.configure(pc);
  g_boostRefPressureSensor.begin();

  pc.enabled = sensorsEnabled && cfg.spare_pressure_1_sensor_enabled;
  pc.adc_pin = cfg.spare_pressure_1_adc_pin;
  g_sparePressure1Sensor.configure(pc);
  g_sparePressure1Sensor.begin();

  pc.enabled = sensorsEnabled && cfg.spare_pressure_2_sensor_enabled;
  pc.adc_pin = cfg.spare_pressure_2_adc_pin;
  g_sparePressure2Sensor.configure(pc);
  g_sparePressure2Sensor.begin();
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

void analogSensorTask(void*) {
  registerTaskWatchdog();
  uint32_t lastConfigRefreshMs = 0;
  state::VehicleState lastConfig{};

  while (true) {
    const uint32_t nowMs = millis();
    if ((nowMs - lastConfigRefreshMs) >= 1000U) {
      lastConfigRefreshMs = nowMs;
      const state::VehicleState cfg = state::g_vehicle_state.read();
      const bool configChanged = cfg.analog_sensors_enabled != lastConfig.analog_sensors_enabled ||
                                 cfg.analog_sensor_sample_ms != lastConfig.analog_sensor_sample_ms ||
                                 cfg.thermistor_pullup_ohms != lastConfig.thermistor_pullup_ohms ||
                                 cfg.iat_adc_pin != lastConfig.iat_adc_pin ||
                                 cfg.engine_bay_adc_pin != lastConfig.engine_bay_adc_pin ||
                                 cfg.cabin_temp_adc_pin != lastConfig.cabin_temp_adc_pin ||
                                 cfg.ambient_temp_adc_pin != lastConfig.ambient_temp_adc_pin ||
                                 cfg.oil_pressure_adc_pin != lastConfig.oil_pressure_adc_pin ||
                                 cfg.fuel_pressure_adc_pin != lastConfig.fuel_pressure_adc_pin ||
                                 cfg.meth_pressure_adc_pin != lastConfig.meth_pressure_adc_pin ||
                                 cfg.boost_ref_pressure_adc_pin != lastConfig.boost_ref_pressure_adc_pin ||
                                 cfg.spare_pressure_1_adc_pin != lastConfig.spare_pressure_1_adc_pin ||
                                 cfg.spare_pressure_2_adc_pin != lastConfig.spare_pressure_2_adc_pin ||
                                 cfg.iat_sensor_enabled != lastConfig.iat_sensor_enabled ||
                                 cfg.engine_bay_sensor_enabled != lastConfig.engine_bay_sensor_enabled ||
                                 cfg.cabin_temp_sensor_enabled != lastConfig.cabin_temp_sensor_enabled ||
                                 cfg.ambient_temp_sensor_enabled != lastConfig.ambient_temp_sensor_enabled ||
                                 cfg.oil_pressure_sensor_enabled != lastConfig.oil_pressure_sensor_enabled ||
                                 cfg.fuel_pressure_sensor_enabled != lastConfig.fuel_pressure_sensor_enabled ||
                                 cfg.meth_pressure_sensor_enabled != lastConfig.meth_pressure_sensor_enabled ||
                                 cfg.boost_ref_pressure_sensor_enabled != lastConfig.boost_ref_pressure_sensor_enabled ||
                                 cfg.spare_pressure_1_sensor_enabled != lastConfig.spare_pressure_1_sensor_enabled ||
                                 cfg.spare_pressure_2_sensor_enabled != lastConfig.spare_pressure_2_sensor_enabled ||
                                 cfg.pressure_sensor_min_v != lastConfig.pressure_sensor_min_v ||
                                 cfg.pressure_sensor_max_v != lastConfig.pressure_sensor_max_v ||
                                 cfg.pressure_sensor_max_psi != lastConfig.pressure_sensor_max_psi;
      if (configChanged) {
        configureAnalogSensorsFromState(cfg);
        lastConfig = cfg;
      }
    }

    g_iatThermistor.update(nowMs);
    g_engineBayThermistor.update(nowMs);
    g_cabinThermistor.update(nowMs);
    g_ambientThermistor.update(nowMs);
    g_oilPressureSensor.update(nowMs);
    g_fuelPressureSensor.update(nowMs);
    g_methPressureSensor.update(nowMs);
    g_boostRefPressureSensor.update(nowMs);
    g_sparePressure1Sensor.update(nowMs);
    g_sparePressure2Sensor.update(nowMs);

    state::g_vehicle_state.mutate([&](state::VehicleState& s) {
      uint16_t analogFaultFlags = 0;

      s.intake_temp_valid = g_iatThermistor.valid();
      s.engine_bay_temp_valid = g_engineBayThermistor.valid();
      s.cabin_temp_valid = g_cabinThermistor.valid();
      s.outside_temp_valid = g_ambientThermistor.valid();
      s.oil_pressure_valid = g_oilPressureSensor.valid();
      s.fuel_pressure_valid = g_fuelPressureSensor.valid();
      s.meth_pressure_valid = g_methPressureSensor.valid();
      s.boost_ref_pressure_valid = g_boostRefPressureSensor.valid();
      s.spare_pressure_1_valid = g_sparePressure1Sensor.valid();
      s.spare_pressure_2_valid = g_sparePressure2Sensor.valid();

      if (s.intake_temp_valid) s.intake_temp = g_iatThermistor.valueC();
      else if (s.iat_sensor_enabled) analogFaultFlags |= kAnalogFaultIat;

      if (s.engine_bay_temp_valid) s.engine_bay_temp = g_engineBayThermistor.valueC();
      else if (s.engine_bay_sensor_enabled) analogFaultFlags |= kAnalogFaultEngineBay;

      if (s.cabin_temp_valid) s.cabin_temp = g_cabinThermistor.valueC();
      else if (s.cabin_temp_sensor_enabled) analogFaultFlags |= kAnalogFaultCabin;

      if (s.outside_temp_valid) s.outside_temp = g_ambientThermistor.valueC();
      else if (s.ambient_temp_sensor_enabled) analogFaultFlags |= kAnalogFaultAmbient;

      if (s.oil_pressure_valid) s.oil_pressure_psi = g_oilPressureSensor.valuePsi();
      else if (s.oil_pressure_sensor_enabled) analogFaultFlags |= kAnalogFaultOil;

      if (s.fuel_pressure_valid) s.fuel_pressure_psi = g_fuelPressureSensor.valuePsi();
      else if (s.fuel_pressure_sensor_enabled) analogFaultFlags |= kAnalogFaultFuel;

      if (s.meth_pressure_valid) s.meth_pressure_psi = g_methPressureSensor.valuePsi();
      else if (s.meth_pressure_sensor_enabled) analogFaultFlags |= kAnalogFaultMeth;

      if (s.boost_ref_pressure_valid) s.boost_ref_pressure_psi = g_boostRefPressureSensor.valuePsi();
      else if (s.boost_ref_pressure_sensor_enabled) analogFaultFlags |= kAnalogFaultBoostRef;

      if (s.spare_pressure_1_valid) s.spare_pressure_1_psi = g_sparePressure1Sensor.valuePsi();
      else if (s.spare_pressure_1_sensor_enabled) analogFaultFlags |= kAnalogFaultSpare1;

      if (s.spare_pressure_2_valid) s.spare_pressure_2_psi = g_sparePressure2Sensor.valuePsi();
      else if (s.spare_pressure_2_sensor_enabled) analogFaultFlags |= kAnalogFaultSpare2;

      s.analog_sensor_fault_flags = analogFaultFlags;
      s.last_analog_sensor_ms = nowMs;

      const bool tempFault = (analogFaultFlags & (kAnalogFaultIat | kAnalogFaultEngineBay | kAnalogFaultCabin | kAnalogFaultAmbient)) != 0U;
      const bool pressureFault = (analogFaultFlags & (kAnalogFaultOil | kAnalogFaultFuel | kAnalogFaultMeth | kAnalogFaultBoostRef |
                                                       kAnalogFaultSpare1 | kAnalogFaultSpare2)) != 0U;
      if (tempFault) s.fault_flags |= kFaultFlagTempSensors;
      else s.fault_flags &= static_cast<uint16_t>(~kFaultFlagTempSensors);
      if (pressureFault) s.fault_flags |= kFaultFlagPressureSensors;
      else s.fault_flags &= static_cast<uint16_t>(~kFaultFlagPressureSensors);
    });

    feedTaskWatchdog();
    vTaskDelay(pdMS_TO_TICKS(20));
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

      char analogLine[512];
      snprintf(analogLine, sizeof(analogLine),
               "intake_air_temp_c=%.2f,engine_bay_temp_c=%.2f,cabin_temp_c=%.2f,ambient_temp_c=%.2f,oil_pressure_psi=%.2f,fuel_pressure_psi=%.2f,meth_pressure_psi=%.2f,boost_ref_pressure_psi=%.2f,spare_pressure_1_psi=%.2f,spare_pressure_2_psi=%.2f,sensor_fault_flags=0x%04X",
               static_cast<double>(s.intake_temp), static_cast<double>(s.engine_bay_temp),
               static_cast<double>(s.cabin_temp), static_cast<double>(s.outside_temp),
               static_cast<double>(s.oil_pressure_psi), static_cast<double>(s.fuel_pressure_psi),
               static_cast<double>(s.meth_pressure_psi), static_cast<double>(s.boost_ref_pressure_psi),
               static_cast<double>(s.spare_pressure_1_psi), static_cast<double>(s.spare_pressure_2_psi),
               static_cast<unsigned>(s.analog_sensor_fault_flags));
      g_logs.enqueue("sensors", analogLine);

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
  configureAnalogSensorsFromState(state::g_vehicle_state.read());
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
  xTaskCreatePinnedToCore(analogSensorTask, "analog_sensor_task", 6144, nullptr, 2, nullptr, 0);
  xTaskCreatePinnedToCore(raceTask, "race_task", 4096, nullptr, 1, nullptr, 1);
  xTaskCreatePinnedToCore(touchTask, "touch_task", 4096, nullptr, 1, nullptr, 1);
  xTaskCreatePinnedToCore(screenTask, "screen_task", 8192, nullptr, 1, nullptr, 1);
  xTaskCreatePinnedToCore(heartbeatTask, "hb_task", 3072, nullptr, 1, nullptr, 1);
}

void loop() {
  feedTaskWatchdog();
  vTaskDelay(pdMS_TO_TICKS(1000));
}
