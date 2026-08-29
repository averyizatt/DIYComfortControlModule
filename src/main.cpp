#include <Arduino.h>
#include <SPI.h>

#ifndef CCM_SCREEN_TASK_PERIOD_MS
#define CCM_SCREEN_TASK_PERIOD_MS 17
#endif

#ifndef CCM_CAN_TASK_PERIOD_MS
#define CCM_CAN_TASK_PERIOD_MS 10
#endif

#ifndef CCM_DISPLAY_LCD_ONLY_TEST
#define CCM_DISPLAY_LCD_ONLY_TEST 0
#endif

#ifndef CCM_CAN_ONLY_DEBUG
#define CCM_CAN_ONLY_DEBUG 0
#endif

#ifndef CCM_SCREEN_SERIAL_LOGS
#define CCM_SCREEN_SERIAL_LOGS 1
#endif

#ifndef CCM_GPS_SERIAL_LOGS
#define CCM_GPS_SERIAL_LOGS 1
#endif

#ifndef CCM_HEAP_SERIAL_LOGS
#define CCM_HEAP_SERIAL_LOGS 1
#endif

#ifndef CCM_SCREEN_SETUP_FIRST_PAINT_MS
#define CCM_SCREEN_SETUP_FIRST_PAINT_MS 0
#endif

#ifndef CCM_LED_ENABLED
#define CCM_LED_ENABLED 1
#endif

#ifndef CCM_LED_SHOW_SHARED_LOCK
#define CCM_LED_SHOW_SHARED_LOCK 1
#endif

#ifndef CCM_LED_TASK_PERIOD_MS
#define CCM_LED_TASK_PERIOD_MS 33
#endif

#ifndef CCM_LED_TASK_STACK_BYTES
#define CCM_LED_TASK_STACK_BYTES 49152
#endif

#ifndef CCM_SD_LOGGING_ENABLED
#define CCM_SD_LOGGING_ENABLED 0
#endif

#ifndef CCM_SD_LOG_SNAPSHOT_PERIOD_MS
#define CCM_SD_LOG_SNAPSHOT_PERIOD_MS 10000
#endif

#ifndef CCM_SD_STATUS_PERIOD_MS
#define CCM_SD_STATUS_PERIOD_MS 60000
#endif

#ifndef CCM_STORAGE_TASK_STACK_BYTES
#define CCM_STORAGE_TASK_STACK_BYTES 8192
#endif

#ifndef CCM_CAN_TASK_STACK_BYTES
#define CCM_CAN_TASK_STACK_BYTES 10240
#endif

#ifndef CCM_STORAGE_TASK_PERIOD_MS
#define CCM_STORAGE_TASK_PERIOD_MS 100
#endif

#ifndef CCM_KNOCK_TASK_PERIOD_MS
#define CCM_KNOCK_TASK_PERIOD_MS 5
#endif

#ifndef CCM_LOCAL_KNOCK_ENABLED
#define CCM_LOCAL_KNOCK_ENABLED 1
#endif

#ifndef CCM_TACH_INPUT_ENABLED
#define CCM_TACH_INPUT_ENABLED 1
#endif

#ifndef CCM_TACH_INPUT_EDGE
#define CCM_TACH_INPUT_EDGE RISING
#endif

#ifndef CCM_TACH_INPUT_MIN_PERIOD_US
#define CCM_TACH_INPUT_MIN_PERIOD_US 1000UL
#endif

#ifndef CCM_TACH_INPUT_STALE_MS
#define CCM_TACH_INPUT_STALE_MS 250
#endif

#ifndef CCM_TACH_INPUT_TASK_PERIOD_MS
#define CCM_TACH_INPUT_TASK_PERIOD_MS 50
#endif

#ifndef CCM_GPS_ZERO_CLAMP_MPH
#define CCM_GPS_ZERO_CLAMP_MPH 5.0f
#endif

#ifndef CCM_GPS_ZERO_HOLD_MS
#define CCM_GPS_ZERO_HOLD_MS 600
#endif

#ifndef CCM_GPS_LOW_SPEED_SMOOTH_ALPHA
#define CCM_GPS_LOW_SPEED_SMOOTH_ALPHA 0.16f
#endif

#ifndef CCM_GPS_HIGH_SPEED_SMOOTH_ALPHA
#define CCM_GPS_HIGH_SPEED_SMOOTH_ALPHA 0.38f
#endif

#ifndef CCM_GPS_MAX_ACCEL_MPS2
#define CCM_GPS_MAX_ACCEL_MPS2 8.0f
#endif

#ifndef CCM_GPS_SPIKE_GRACE_MPH
#define CCM_GPS_SPIKE_GRACE_MPH 6.0f
#endif

#ifndef CCM_GPS_DEAD_RECKON_MS
#define CCM_GPS_DEAD_RECKON_MS 2500
#endif

#ifndef CCM_GPS_DEAD_RECKON_DECAY_MPH_PER_S
#define CCM_GPS_DEAD_RECKON_DECAY_MPH_PER_S 7.0f
#endif

#ifndef CCM_GPS_IMU_DEAD_RECKON
#define CCM_GPS_IMU_DEAD_RECKON 1
#endif

#ifndef CCM_GPS_IMU_ACCEL_CONFIRM
#define CCM_GPS_IMU_ACCEL_CONFIRM 1
#endif

#ifndef CCM_GPS_IMU_ACCEL_MARGIN_MPS2
#define CCM_GPS_IMU_ACCEL_MARGIN_MPS2 2.5f
#endif

#ifndef CCM_GPS_IMU_ACCEL_DEADBAND_MPS2
#define CCM_GPS_IMU_ACCEL_DEADBAND_MPS2 0.6f
#endif

#ifndef CCM_IMU_ENABLED
#define CCM_IMU_ENABLED 0
#endif

#include <esp_system.h>
#include <esp_task_wdt.h>

#include <cmath>
#include <cstring>

#include "can/can_manager.h"
#include "hal/SharedSpiBus.hpp"
#include "hal/HardwareAdapters.hpp"
#include "gps/GpsService.hpp"
#include "knock/knock_monitor.h"
#include "led/led_manager.h"
#include "pin_map.h"
#include "race/race_manager.h"
#include "sensors/pressure_sensor.h"
#include "sensors/thermistor_sensor.h"
#include "settings/settings_manager.h"
#include "state/vehicle_state.h"
#include "storage/log_manager.h"
#include "storage/sd_manager.h"
#include "storage/telemetry_recorder.h"
#include "touch/touch_manager.h"
#include "ui/asset_manager.h"
#include "ui/screen_dashboard.h"
#include "imu/imu_service.h"

namespace {
canbus::CanManager g_can;
settings::SettingsManager g_settings;

#ifndef CCM_MAIN_LED_COUNT
#define CCM_MAIN_LED_COUNT 18
#endif

#ifndef CCM_CASE_LED_COUNT
#define CCM_CASE_LED_COUNT 7
#endif

#ifndef CCM_INTERIOR_LED_SEND_COUNT
#define CCM_INTERIOR_LED_SEND_COUNT 60
#endif

#ifndef CCM_CASE_LED_OFFSET
#define CCM_CASE_LED_OFFSET 0
#endif

constexpr uint16_t kMainLedCount = CCM_MAIN_LED_COUNT;
constexpr uint16_t kInteriorLedSendCount = CCM_INTERIOR_LED_SEND_COUNT;

constexpr float kGpsKphToMph = 0.621371f;
constexpr float kGpsMphToKph = 1.609344f;
constexpr float kGpsMpsToKph = 3.6f;
constexpr float kGpsGravityMps2 = 9.80665f;
constexpr float kGpsZeroClampMph = CCM_GPS_ZERO_CLAMP_MPH;
constexpr float kGpsLowSpeedBandMph = 12.0f;
constexpr float kGpsLowSpeedAlpha = CCM_GPS_LOW_SPEED_SMOOTH_ALPHA;
constexpr float kGpsHighSpeedAlpha = CCM_GPS_HIGH_SPEED_SMOOTH_ALPHA;
constexpr float kGpsMaxAccelMps2 = CCM_GPS_MAX_ACCEL_MPS2;
constexpr float kGpsSpikeGraceMph = CCM_GPS_SPIKE_GRACE_MPH;
constexpr float kGpsDeadReckonDecayMphPerS = CCM_GPS_DEAD_RECKON_DECAY_MPH_PER_S;
constexpr float kGpsImuAccelMarginMps2 = CCM_GPS_IMU_ACCEL_MARGIN_MPS2;
constexpr float kGpsImuAccelDeadbandMps2 = CCM_GPS_IMU_ACCEL_DEADBAND_MPS2;
constexpr uint32_t kGpsZeroHoldMs = CCM_GPS_ZERO_HOLD_MS;
constexpr uint32_t kGpsDeadReckonMs = CCM_GPS_DEAD_RECKON_MS;
constexpr float kGpsMaxValidMph = 180.0f;

constexpr uint8_t kGpsStatusRxLive = 0x01;
constexpr uint8_t kGpsStatusChecksumOk = 0x02;
constexpr uint8_t kGpsStatusChecksumWarn = 0x04;
constexpr uint8_t kGpsStatusSatsInView = 0x08;
constexpr uint8_t kGpsStatusFixQuality = 0x10;
constexpr uint8_t kGpsStatusDeadReckoned = 0x20;
constexpr uint8_t kGpsStatusSpikeRejected = 0x40;
constexpr uint8_t kGpsStatusZeroClamped = 0x80;
constexpr uint32_t kTachInputMinPeriodUs = CCM_TACH_INPUT_MIN_PERIOD_US;
constexpr uint32_t kTachInputStaleMs = CCM_TACH_INPUT_STALE_MS;
constexpr uint32_t kTachInputTaskPeriodMs =
    (CCM_TACH_INPUT_TASK_PERIOD_MS < 10) ? 10U : static_cast<uint32_t>(CCM_TACH_INPUT_TASK_PERIOD_MS);
constexpr uint8_t kTachStatusInputLive = 0x01;
constexpr uint8_t kTachStatusSignalStale = 0x02;
constexpr uint8_t kTachStatusNoiseRejected = 0x04;

float clampFloat(float value, float lo, float hi) {
  if (value < lo) return lo;
  if (value > hi) return hi;
  return value;
}

class GpsSpeedConditioner {
 public:
  struct Result {
    float speedKph = 0.0f;
    bool deadReckoned = false;
    bool spikeRejected = false;
    bool zeroClamped = false;
  };

  Result update(const ccm::core::GpsData& gps, bool fixLive, uint32_t nowMs,
                const state::VehicleState& current) {
    const float dtSec = lastUpdateMs_ == 0
        ? 0.05f
        : clampFloat(static_cast<float>(nowMs - lastUpdateMs_) / 1000.0f, 0.02f, 0.75f);
    lastUpdateMs_ = nowMs;

    Result result{};
    const bool rawPlausible =
        fixLive && gps.speedKph >= 0.0f && (gps.speedKph * kGpsKphToMph) <= kGpsMaxValidMph;

    if (!rawPlausible) {
      result.speedKph = deadReckon(nowMs, dtSec, current);
      result.deadReckoned =
          initialized_ && (nowMs - lastGoodFixMs_) <= kGpsDeadReckonMs && result.speedKph > 0.0f;
      result.zeroClamped = result.speedKph <= 0.0f;
      filteredKph_ = result.speedKph;
      return result;
    }

    float targetKph = gps.speedKph;
    const float targetMph = targetKph * kGpsKphToMph;
    if (targetMph < kGpsZeroClampMph) {
      if (lowSpeedSinceMs_ == 0) {
        lowSpeedSinceMs_ = nowMs;
      }
      if ((nowMs - lowSpeedSinceMs_) >= kGpsZeroHoldMs ||
          !initialized_ ||
          (filteredKph_ * kGpsKphToMph) < kGpsZeroClampMph) {
        targetKph = 0.0f;
        result.zeroClamped = true;
      }
    } else {
      lowSpeedSinceMs_ = 0;
    }

    if (initialized_ && !result.zeroClamped) {
      const float deltaKph = targetKph - lastAcceptedKph_;
      const float deltaMph = fabsf(deltaKph) * kGpsKphToMph;
      const float requiredAccelMps2 = dtSec > 0.0f ? (deltaKph / kGpsMpsToKph) / dtSec : 0.0f;
      const float allowedAccelMps2 = allowedGpsAccelMps2(current, requiredAccelMps2);
      const float maxDeltaMph = kGpsSpikeGraceMph + (allowedAccelMps2 * dtSec * 2.23694f);
      if (deltaMph > maxDeltaMph) {
        if (spikeRejectStreak_ < 3U) {
          ++spikeRejectStreak_;
          targetKph = filteredKph_;
          result.spikeRejected = true;
        } else {
          spikeRejectStreak_ = 0;
        }
      } else {
        spikeRejectStreak_ = 0;
      }
    } else {
      spikeRejectStreak_ = 0;
    }

    if (!initialized_ || result.zeroClamped) {
      filteredKph_ = targetKph;
      initialized_ = true;
    } else if (!result.spikeRejected) {
      const float alpha = (targetKph * kGpsKphToMph) < kGpsLowSpeedBandMph
          ? kGpsLowSpeedAlpha
          : kGpsHighSpeedAlpha;
      filteredKph_ += (targetKph - filteredKph_) * clampFloat(alpha, 0.02f, 1.0f);
    }

    if ((filteredKph_ * kGpsKphToMph) < 0.5f || result.zeroClamped) {
      filteredKph_ = 0.0f;
      result.zeroClamped = true;
    }

    if (!result.spikeRejected) {
      lastAcceptedKph_ = targetKph;
      lastGoodFixMs_ = nowMs;
    }
    result.speedKph = (filteredKph_ * kGpsKphToMph) < kGpsZeroClampMph ? 0.0f : filteredKph_;
    if (result.speedKph <= 0.0f) {
      result.zeroClamped = true;
    }
    return result;
  }

 private:
  float deadReckon(uint32_t nowMs, float dtSec, const state::VehicleState& current) {
    if (!initialized_ || lastGoodFixMs_ == 0 || (nowMs - lastGoodFixMs_) > kGpsDeadReckonMs) {
      return decayTowardZero(filteredKph_, dtSec);
    }

    float estimateKph = filteredKph_;
#if CCM_GPS_IMU_DEAD_RECKON
    if (current.imu_online && (estimateKph * kGpsKphToMph) >= kGpsZeroClampMph) {
      const float accelMps2 =
          clampFloat(current.imu_g_longitudinal, -0.45f, 0.45f) * kGpsGravityMps2;
      estimateKph += accelMps2 * dtSec * kGpsMpsToKph;
    }
#else
    (void)current;
#endif

    estimateKph = clampFloat(estimateKph, 0.0f, kGpsMaxValidMph * kGpsMphToKph);
    if ((estimateKph * kGpsKphToMph) < kGpsZeroClampMph) {
      return 0.0f;
    }
    return estimateKph;
  }

  static float imuLongitudinalAccelMps2(const state::VehicleState& current) {
    if (!current.imu_online) {
      return 0.0f;
    }
    return clampFloat(current.imu_g_longitudinal, -0.60f, 0.60f) * kGpsGravityMps2;
  }

  static bool imuSupportsGpsDelta(float requiredAccelMps2, float imuAccelMps2) {
    if (fabsf(requiredAccelMps2) < kGpsImuAccelDeadbandMps2) {
      return true;
    }
    if (requiredAccelMps2 > 0.0f) {
      return imuAccelMps2 >= -kGpsImuAccelDeadbandMps2;
    }
    return imuAccelMps2 <= kGpsImuAccelDeadbandMps2;
  }

  static float allowedGpsAccelMps2(const state::VehicleState& current, float requiredAccelMps2) {
    float allowed = kGpsMaxAccelMps2;
#if CCM_GPS_IMU_ACCEL_CONFIRM
    if (current.imu_online) {
      const float imuAccel = imuLongitudinalAccelMps2(current);
      if (imuSupportsGpsDelta(requiredAccelMps2, imuAccel)) {
        allowed = clampFloat(fabsf(imuAccel) + kGpsImuAccelMarginMps2,
                             kGpsImuAccelMarginMps2,
                             kGpsMaxAccelMps2);
      } else {
        allowed = kGpsImuAccelMarginMps2;
      }
    }
#else
    (void)current;
    (void)requiredAccelMps2;
#endif
    return allowed;
  }

  static float decayTowardZero(float speedKph, float dtSec) {
    const float decayKph = kGpsDeadReckonDecayMphPerS * kGpsMphToKph * dtSec;
    if (speedKph <= decayKph) {
      return 0.0f;
    }
    return speedKph - decayKph;
  }

  bool initialized_ = false;
  float filteredKph_ = 0.0f;
  float lastAcceptedKph_ = 0.0f;
  uint32_t lastUpdateMs_ = 0;
  uint32_t lastGoodFixMs_ = 0;
  uint32_t lowSpeedSinceMs_ = 0;
  uint8_t spikeRejectStreak_ = 0;
};

// GPS: UartGpsAdapter owns Serial2 (GPIO42=RX, GPIO41=TX @ 9600 baud)
static ccm::hal::UartGpsAdapter g_gpsHal(Serial2);
static ccm::gps::GpsService     g_gps(g_gpsHal);
GpsSpeedConditioner g_gpsSpeedConditioner;

portMUX_TYPE g_tachMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t g_tachLastEdgeUs = 0;
volatile uint32_t g_tachLastPeriodUs = 0;
volatile uint32_t g_tachPulseCount = 0;
volatile uint32_t g_tachRejectCount = 0;

void IRAM_ATTR tachInputIsr() {
  const uint32_t nowUs = micros();
  portENTER_CRITICAL_ISR(&g_tachMux);
  const uint32_t prevUs = g_tachLastEdgeUs;
  const uint32_t periodUs = nowUs - prevUs;

  if (prevUs != 0U && periodUs < kTachInputMinPeriodUs) {
    ++g_tachRejectCount;
    portEXIT_CRITICAL_ISR(&g_tachMux);
    return;
  }

  g_tachLastEdgeUs = nowUs;
  if (prevUs != 0U) {
    g_tachLastPeriodUs = periodUs;
  }
  ++g_tachPulseCount;
  portEXIT_CRITICAL_ISR(&g_tachMux);
}

led::LedManager g_led;
storage::SdManager g_sd;
storage::LogManager g_logs;
storage::TelemetryRecorder g_telemetry;
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
imu::ImuService g_imu;

constexpr uint32_t kTaskWatchdogTimeoutS = 6;
constexpr BaseType_t kCoreIo = 0;
constexpr BaseType_t kCoreUi = 1;
constexpr UBaseType_t kPrioHighIo = 3;
constexpr UBaseType_t kPrioUi = 3;
constexpr UBaseType_t kPrioSensors = 2;
constexpr UBaseType_t kPrioBackground = 1;
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

const char* resetReasonName(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_POWERON: return "POWERON";
    case ESP_RST_EXT: return "EXT";
    case ESP_RST_SW: return "SW";
    case ESP_RST_PANIC: return "PANIC";
    case ESP_RST_INT_WDT: return "INT_WDT";
    case ESP_RST_TASK_WDT: return "TASK_WDT";
    case ESP_RST_WDT: return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT: return "BROWNOUT";
    case ESP_RST_SDIO: return "SDIO";
    default: return "UNKNOWN";
  }
}

void initTaskWatchdog() {
#if defined(ESP_IDF_VERSION_MAJOR) && (ESP_IDF_VERSION_MAJOR >= 5)
  esp_task_wdt_config_t cfg{};
  cfg.timeout_ms = kTaskWatchdogTimeoutS * 1000;
  // We explicitly register critical tasks with TWDT; do not watch idle tasks.
  // setup() performs blocking peripheral bring-up (e.g. SD mount), which can
  // legitimately starve IDLE1 long enough to trigger false-positive resets.
  cfg.idle_core_mask = 0;
  cfg.trigger_panic = true;
  // Reconfigure first to avoid noisy "already initialized" logs when Arduino
  // core has already brought up TWDT before setup().
  esp_err_t err = esp_task_wdt_reconfigure(&cfg);
  if (err == ESP_ERR_INVALID_STATE) {
    err = esp_task_wdt_init(&cfg);
  }
  if (err != ESP_OK) {
    Serial.printf("[WDT] configure failed: %d\n", static_cast<int>(err));
  }
#else
  const esp_err_t err = esp_task_wdt_init(kTaskWatchdogTimeoutS, true);
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
    Serial.printf("[WDT] init failed: %d\n", static_cast<int>(err));
  }
#endif
}

bool registerTaskWatchdog() {
  if (esp_task_wdt_status(nullptr) == ESP_OK) {
    return true;
  }
  const esp_err_t err = esp_task_wdt_add(nullptr);
  if (err != ESP_OK) {
    Serial.printf("[WDT] task add failed: %d\n", static_cast<int>(err));
    return false;
  }
  return true;
}

void feedTaskWatchdog() {
  if (esp_task_wdt_status(nullptr) == ESP_OK) {
    const esp_err_t err = esp_task_wdt_reset();
    if (err != ESP_OK) {
      Serial.printf("[WDT] reset failed: %d\n", static_cast<int>(err));
    }
  }
}

bool createPinnedTask(TaskFunction_t taskFn, const char* name, uint32_t stackBytes,
                      UBaseType_t priority, BaseType_t coreId) {
  const BaseType_t ok = xTaskCreatePinnedToCore(taskFn, name, stackBytes, nullptr,
                                                priority, nullptr, coreId);
  if (ok != pdPASS) {
    Serial.printf("[TASK] create failed name=%s stack=%lu core=%d err=%ld free_heap=%lu\n",
                   name,
                   static_cast<unsigned long>(stackBytes),
                   static_cast<int>(coreId),
                   static_cast<long>(ok),
                   static_cast<unsigned long>(ESP.getFreeHeap()));
    return false;
  }
  return true;
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

void canTask(void*) {
  registerTaskWatchdog();
  constexpr uint32_t kCanTaskPeriodMs =
      (CCM_CAN_TASK_PERIOD_MS < 5) ? 5U : static_cast<uint32_t>(CCM_CAN_TASK_PERIOD_MS);
  uint32_t lastStackReportMs = 0;
  while (true) {
    g_can.tick();
    const uint32_t nowMs = millis();
    if (nowMs - lastStackReportMs >= 30000U) {
      lastStackReportMs = nowMs;
      Serial.printf("[STACK] can free=%lu bytes\n",
                    static_cast<unsigned long>(uxTaskGetStackHighWaterMark(nullptr)));
    }
    feedTaskWatchdog();
    vTaskDelay(pdMS_TO_TICKS(kCanTaskPeriodMs));
  }
}

void tachInputTask(void*) {
#if !CCM_TACH_INPUT_ENABLED
  vTaskDelete(nullptr);
#else
  registerTaskWatchdog();
  pinMode(pins::kTachIn, INPUT);
  attachInterrupt(digitalPinToInterrupt(pins::kTachIn), tachInputIsr, CCM_TACH_INPUT_EDGE);
  Serial.printf("[TACH] input GPIO=%u edge=%u min_period=%luus stale=%lums\n",
                static_cast<unsigned>(pins::kTachIn),
                static_cast<unsigned>(CCM_TACH_INPUT_EDGE),
                static_cast<unsigned long>(kTachInputMinPeriodUs),
                static_cast<unsigned long>(kTachInputStaleMs));

  uint32_t lastRejectCount = 0;
  while (true) {
    uint32_t lastEdgeUs = 0;
    uint32_t lastPeriodUs = 0;
    uint32_t pulseCount = 0;
    uint32_t rejectCount = 0;
    portENTER_CRITICAL(&g_tachMux);
    lastEdgeUs = g_tachLastEdgeUs;
    lastPeriodUs = g_tachLastPeriodUs;
    pulseCount = g_tachPulseCount;
    rejectCount = g_tachRejectCount;
    portEXIT_CRITICAL(&g_tachMux);

    const uint32_t nowMs = millis();
    const uint32_t nowUs = micros();
    const bool noiseRejected = rejectCount != lastRejectCount;
    lastRejectCount = rejectCount;
    const bool live = lastEdgeUs != 0U && ((nowUs - lastEdgeUs) / 1000UL) <= kTachInputStaleMs;
    (void)pulseCount;

    state::g_vehicle_state.mutate([&](state::VehicleState& s) {
      if (s.bench_test_mode) return;

      // The engine ECU is the authoritative RPM source while its broadcast is
      // fresh. Keep the GPIO tach input as an automatic failover only.
      if (s.microsquirt_online &&
          (nowMs - s.microsquirt_last_ms) <= microsquirt::kFreshTimeoutMs) {
        s.tach_source = static_cast<uint8_t>(can_protocol::TachSource::CAN);
        return;
      }

      const uint8_t ppr10 = s.pulses_per_rev10 == 0U ? 20U : s.pulses_per_rev10;
      uint16_t rawHz10 = 0;
      uint16_t rpm = 0;
      uint8_t status = 0;

      if (live && lastPeriodUs > 0U) {
        const uint32_t hz10 = (10000000UL + (lastPeriodUs / 2UL)) / lastPeriodUs;
        const uint32_t rpmCalc = (hz10 * 60UL + (ppr10 / 2U)) / ppr10;
        rawHz10 = static_cast<uint16_t>(hz10 > 65535UL ? 65535UL : hz10);
        rpm = static_cast<uint16_t>(rpmCalc > 9000UL ? 9000UL : rpmCalc);
        status |= kTachStatusInputLive;
      } else {
        status |= kTachStatusSignalStale;
      }

      if (noiseRejected) {
        status |= kTachStatusNoiseRejected;
      }

      s.rpm = rpm;
      s.raw_tach_hz10 = rawHz10;
      const uint16_t tachDivisor = s.tach_scaling_mode == 0U ? 15U : 30U;
      s.generated_tach_hz10 =
          static_cast<uint16_t>((static_cast<uint32_t>(rpm) * 10UL) / tachDivisor);
      s.tach_source = static_cast<uint8_t>(can_protocol::TachSource::GPIO_INPUT);
      s.tach_status_flags = status;
      s.tach_input_frequency_hz = rawHz10 / 10.0f;
      s.tach_generated_frequency_hz = s.generated_tach_hz10 / 10.0f;
      s.uptime_ms = nowMs;
    });

    feedTaskWatchdog();
    vTaskDelay(pdMS_TO_TICKS(kTachInputTaskPeriodMs));
  }
#endif
}

void ledTask(void*) {
#if !CCM_LED_ENABLED
  vTaskDelete(nullptr);
#else
  registerTaskWatchdog();
  constexpr uint32_t kLedTaskPeriodMs =
      (CCM_LED_TASK_PERIOD_MS < 8) ? 8U : static_cast<uint32_t>(CCM_LED_TASK_PERIOD_MS);
  uint32_t lastLedFrameMs = 0;
  while (true) {
    const uint32_t nowMs = millis();
    const state::VehicleState s = state::g_vehicle_state.read();
    if (s.led_startup_preview) {
      g_led.triggerStartupSweep();
    }
    const bool ledFrameDue =
        s.led_startup_preview || lastLedFrameMs == 0 ||
        static_cast<uint32_t>(nowMs - lastLedFrameMs) >= kLedTaskPeriodMs;
    if (ledFrameDue) {
#if CCM_LED_SHOW_SHARED_LOCK
      {
        hal::SharedSpiBusLock quietDisplayTiming("LED:show");
        g_led.tick(s);
      }
#else
      g_led.tick(s);
#endif
      lastLedFrameMs = nowMs;
    }
    if (s.led_startup_preview) {
      state::g_vehicle_state.mutate([](state::VehicleState& st) {
        st.led_startup_preview = false;
      });
    }
    feedTaskWatchdog();
    vTaskDelay(pdMS_TO_TICKS(kLedTaskPeriodMs));
  }
#endif
}

void knockTask(void*) {
  registerTaskWatchdog();
  while (true) {
    g_knock.tick(millis());
    feedTaskWatchdog();
    vTaskDelay(pdMS_TO_TICKS(CCM_KNOCK_TASK_PERIOD_MS));
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
      else if (g_iatThermistor.config().enabled) analogFaultFlags |= kAnalogFaultIat;

      if (s.engine_bay_temp_valid) s.engine_bay_temp = g_engineBayThermistor.valueC();
      else if (g_engineBayThermistor.config().enabled) analogFaultFlags |= kAnalogFaultEngineBay;

      if (s.cabin_temp_valid) s.cabin_temp = g_cabinThermistor.valueC();
      else if (g_cabinThermistor.config().enabled) analogFaultFlags |= kAnalogFaultCabin;

      if (s.outside_temp_valid) s.outside_temp = g_ambientThermistor.valueC();
      else if (g_ambientThermistor.config().enabled) analogFaultFlags |= kAnalogFaultAmbient;

      if (s.oil_pressure_valid) s.oil_pressure_psi = g_oilPressureSensor.valuePsi();
      else if (g_oilPressureSensor.config().enabled) analogFaultFlags |= kAnalogFaultOil;

      if (s.fuel_pressure_valid) s.fuel_pressure_psi = g_fuelPressureSensor.valuePsi();
      else if (g_fuelPressureSensor.config().enabled) analogFaultFlags |= kAnalogFaultFuel;

      if (s.meth_pressure_valid) s.meth_pressure_psi = g_methPressureSensor.valuePsi();
      else if (g_methPressureSensor.config().enabled) analogFaultFlags |= kAnalogFaultMeth;

      if (s.boost_ref_pressure_valid) s.boost_ref_pressure_psi = g_boostRefPressureSensor.valuePsi();
      else if (g_boostRefPressureSensor.config().enabled) analogFaultFlags |= kAnalogFaultBoostRef;

      if (s.spare_pressure_1_valid) s.spare_pressure_1_psi = g_sparePressure1Sensor.valuePsi();
      else if (g_sparePressure1Sensor.config().enabled) analogFaultFlags |= kAnalogFaultSpare1;

      if (s.spare_pressure_2_valid) s.spare_pressure_2_psi = g_sparePressure2Sensor.valuePsi();
      else if (g_sparePressure2Sensor.config().enabled) analogFaultFlags |= kAnalogFaultSpare2;

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
  uint32_t lastFpsMs = millis();
  uint32_t frameCount = 0;
  bool lastTouchOnline = g_touch.online();
  constexpr uint32_t kScreenTaskPeriodMs =
      (CCM_SCREEN_TASK_PERIOD_MS < 1) ? 1 : CCM_SCREEN_TASK_PERIOD_MS;
  if (CCM_SCREEN_SERIAL_LOGS) {
    Serial.printf("[SCREEN] task period=%lu ms fps_cap=%.1f\n",
                   static_cast<unsigned long>(kScreenTaskPeriodMs),
                   static_cast<double>(1000.0f / static_cast<float>(kScreenTaskPeriodMs)));
  }
  TickType_t lastWake = xTaskGetTickCount();

  while (true) {
    const state::VehicleState s = state::g_vehicle_state.read();
    g_screen.tick(s, millis());
#if CCM_IMU_ENABLED
    // Touch and IMU share one I2C bus. Sampling both from screenTask keeps bus
    // ownership serialized and lets ImuService run its reconnect state machine.
    g_imu.update();
#endif
    const bool touchOnline = g_touch.online();
    if (touchOnline != lastTouchOnline) {
      lastTouchOnline = touchOnline;
      state::g_vehicle_state.mutate([touchOnline](state::VehicleState& st) {
        st.touch_online = touchOnline;
      });
    }
    frameCount++;

    const uint32_t nowMs = millis();
    if ((nowMs - lastFpsMs) >= 1000) {
      const float fps = frameCount * 1000.0f / static_cast<float>(nowMs - lastFpsMs);
      frameCount = 0;
      lastFpsMs = nowMs;
      state::g_vehicle_state.mutate([&](state::VehicleState& st) {
        st.ui_fps = fps;
        st.heap_free_bytes = ESP.getFreeHeap();
        st.heap_min_free_bytes = ESP.getMinFreeHeap();
      });
    }

    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(kScreenTaskPeriodMs));
  }
}

void storageTask(void*) {
  // SD/FAT operations can block below this task while a card is busy. Storage
  // is noncritical, so it must not reboot the controller if a slow or damaged
  // card exceeds the global watchdog timeout.
  constexpr bool kSdLoggingEnabled = CCM_SD_LOGGING_ENABLED != 0;
  constexpr uint32_t kSdLogSnapshotPeriodMs = CCM_SD_LOG_SNAPSHOT_PERIOD_MS;
  constexpr uint32_t kStorageStatusPeriodMs =
      (CCM_SD_STATUS_PERIOD_MS < 10000) ? 10000U : static_cast<uint32_t>(CCM_SD_STATUS_PERIOD_MS);
  constexpr uint32_t kStorageTaskPeriodMs =
      (CCM_STORAGE_TASK_PERIOD_MS < 20) ? 20U : static_cast<uint32_t>(CCM_STORAGE_TASK_PERIOD_MS);
  uint32_t lastLogMs = 0;
  uint32_t lastStorageStatusMs = 0;
  uint8_t lastKnockEventCount = 0;
  uint8_t lastProtectedKnockEventCount = 0;
  uint16_t lastProtectedFaultFlags = 0;
  uint32_t lastTelemetryStatusMs = 0;
  bool lastRaceRunning = false;
#ifndef CCM_IGNITION_SENSE_ACTIVE_HIGH
#define CCM_IGNITION_SENSE_ACTIVE_HIGH 1
#endif
  bool ignitionStable = true;
  bool ignitionRawLast = true;
  uint32_t ignitionChangeMs = millis();

  while (true) {
    const uint32_t nowMs = millis();
    g_sd.service(nowMs);
    if (pins::kIgnitionSense != 255U) {
      const bool rawLevel = digitalRead(pins::kIgnitionSense) == HIGH;
      const bool ignitionOn = CCM_IGNITION_SENSE_ACTIVE_HIGH ? rawLevel : !rawLevel;
      if (ignitionOn != ignitionRawLast) {
        ignitionRawLast = ignitionOn;
        ignitionChangeMs = nowMs;
      }
      if (ignitionOn != ignitionStable && (nowMs - ignitionChangeMs) >= 100U) {
        ignitionStable = ignitionOn;
        if (ignitionOn) g_telemetry.resume(nowMs);
        else g_telemetry.requestShutdown();
      }
    }

    const state::VehicleState eventState = state::g_vehicle_state.read();
    if ((eventState.fault_flags != 0U && eventState.fault_flags != lastProtectedFaultFlags) ||
        eventState.knock_event_count != lastProtectedKnockEventCount ||
        (eventState.race_running && !lastRaceRunning)) {
      g_telemetry.protectEvent();
    }
    lastProtectedFaultFlags = eventState.fault_flags;
    lastProtectedKnockEventCount = eventState.knock_event_count;
    lastRaceRunning = eventState.race_running;
    g_telemetry.service(nowMs);

    if (kSdLoggingEnabled) {
      g_logs.tick(nowMs);
    }

    if (kSdLoggingEnabled && kSdLogSnapshotPeriodMs > 0U &&
        (nowMs - lastLogMs) >= kSdLogSnapshotPeriodMs) {
      lastLogMs = nowMs;
      const state::VehicleState s = state::g_vehicle_state.read();
      static char canLine[96];
      static char gpsLine[96];
      static char methLine[96];
      static char raceLine[160];
      snprintf(canLine, sizeof(canLine), "rx=%lu,tx=%lu,last_id=%u", static_cast<unsigned long>(s.can_rx_count), static_cast<unsigned long>(s.can_tx_count), s.can_last_rx_id);
      snprintf(gpsLine, sizeof(gpsLine), "fix=%u,used=%u,view=%u,q=%u,mode=%u,speed=%.1f",
               s.gps_fix ? 1U : 0U,
               s.gps_satellites,
               s.gps_satellites_in_view,
               s.gps_fix_quality,
               s.gps_fix_mode,
               static_cast<double>(s.speed));
      snprintf(methLine, sizeof(methLine), "state=%u,ratio=%u,duty=%u", static_cast<uint8_t>(s.meth_state), s.meth_selected_ratio_percent, s.meth_pump_duty);
      snprintf(raceLine, sizeof(raceLine), "mode=%u,running=%u,0_60=%.3f,qtr=%.3f,lap_best=%.3f,quality=%u", static_cast<uint8_t>(s.race_mode),
               s.race_running ? 1U : 0U, static_cast<double>(s.race_0_60_s), static_cast<double>(s.race_quarter_mile_et_s),
               static_cast<double>(s.race_best_lap_s), s.race_quality_percent);
      g_logs.enqueue("can", canLine);
      g_logs.enqueue("gps", gpsLine);
      g_logs.enqueue("meth", methLine);
      g_logs.enqueue("race", raceLine);

      static char knockLine[220];
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

      // 11 key/value fields + delimiters + float precision margin.
      static char analogLine[512];
      snprintf(analogLine, sizeof(analogLine),
               "intake_air_temp_c=%.2f,engine_bay_temp_c=%.2f,cabin_temp_c=%.2f,ambient_temp_c=%.2f,oil_pressure_psi=%.2f,fuel_pressure_psi=%.2f,meth_pressure_psi=%.2f,boost_ref_pressure_psi=%.2f,spare_pressure_1_psi=%.2f,spare_pressure_2_psi=%.2f,sensor_fault_flags=0x%04X",
               static_cast<double>(s.intake_temp), static_cast<double>(s.engine_bay_temp),
               static_cast<double>(s.cabin_temp), static_cast<double>(s.outside_temp),
               static_cast<double>(s.oil_pressure_psi), static_cast<double>(s.fuel_pressure_psi),
               static_cast<double>(s.meth_pressure_psi), static_cast<double>(s.boost_ref_pressure_psi),
               static_cast<double>(s.spare_pressure_1_psi), static_cast<double>(s.spare_pressure_2_psi),
               static_cast<unsigned>(s.analog_sensor_fault_flags));
      g_logs.enqueue("sensors", analogLine);

      const uint16_t curFaultFlags = s.fault_flags;
      if (curFaultFlags != 0) {
        static char faultLine[48];
        snprintf(faultLine, sizeof(faultLine), "fault_flags=%u", curFaultFlags);
        g_logs.enqueue("faults", faultLine);
        // Avoid synchronous flushes here; queued SD writes are intentionally
        // drained slowly so logging cannot stall the shared SPI bus.
      }
    }

    if ((nowMs - lastTelemetryStatusMs) >= 1000U) {
      lastTelemetryStatusMs = nowMs;
      const storage::TelemetryRecorderStats ts = g_telemetry.stats();
      state::g_vehicle_state.mutate([&](state::VehicleState& s) {
        s.telemetry_recording = ts.recording;
        s.telemetry_shutdown_clean = ts.shutdown_clean;
        s.telemetry_active_protected = ts.active_protected;
        s.telemetry_queue_depth = ts.queue_depth;
        s.telemetry_queue_high_water = ts.queue_high_water;
        s.telemetry_captured_count = ts.captured;
        s.telemetry_written_count = ts.written;
        s.telemetry_dropped_count = ts.dropped;
        s.telemetry_bytes_written = ts.bytes_written;
        s.telemetry_write_errors = ts.write_errors;
        s.telemetry_recovery_count = ts.recovery_count;
        s.telemetry_deleted_count = ts.deleted_count;
        s.telemetry_max_write_us = ts.max_write_us;
        s.telemetry_managed_bytes = ts.managed_bytes;
        if (ts.active_file[0] != '\0') {
          strncpy(s.current_log_file, ts.active_file, sizeof(s.current_log_file) - 1);
          s.current_log_file[sizeof(s.current_log_file) - 1] = '\0';
        }
      });
    }

    if ((nowMs - lastStorageStatusMs) >= kStorageStatusPeriodMs) {
      lastStorageStatusMs = nowMs;
      state::g_vehicle_state.mutate([&](state::VehicleState& s) {
        s.sd_mounted = g_sd.mounted();
        s.sd_size_bytes = g_sd.totalBytes();
        // SD.usedBytes() scans every FAT cluster — can take 10-30 seconds on large cards.
        // Calling it every second would starve the task watchdog (6 s timeout) and cause
        // a panic reboot. Field left as 0; total bytes is sufficient for the dash display.
        s.sd_used_bytes = 0;
        s.sd_write_error_count = g_sd.errorCount() + g_logs.droppedCount();
        strncpy(s.last_sd_write_status, g_sd.lastStatus(), sizeof(s.last_sd_write_status) - 1);
        s.last_sd_write_status[sizeof(s.last_sd_write_status) - 1] = '\0';
        const storage::TelemetryRecorderStats ts = g_telemetry.stats();
        const char* activeLog = ts.active_file[0] != '\0' ? ts.active_file : g_logs.currentFile();
        strncpy(s.current_log_file, activeLog, sizeof(s.current_log_file) - 1);
        s.current_log_file[sizeof(s.current_log_file) - 1] = '\0';
        s.heap_free_bytes = ESP.getFreeHeap();
        if (s.heap_free_bytes < s.heap_min_free_bytes) {
          s.heap_min_free_bytes = s.heap_free_bytes;
        }
        s.esp_die_temp_c = static_cast<int8_t>(temperatureRead());
      });
      // Periodic heap report to UART so it's visible on the monitor
      const state::VehicleState heapState = state::g_vehicle_state.read();
      const uint32_t psramTotal = ESP.getPsramSize();
      const uint32_t psramFree = psramTotal == 0 ? 0 : ESP.getFreePsram();
      if (CCM_HEAP_SERIAL_LOGS) {
        Serial.printf("[HEAP] free=%lu min=%lu max_block=%lu psram=%lu/%lu\n",
          static_cast<unsigned long>(heapState.heap_free_bytes),
          static_cast<unsigned long>(heapState.heap_min_free_bytes),
          static_cast<unsigned long>(ESP.getMaxAllocHeap()),
          static_cast<unsigned long>(psramFree),
          static_cast<unsigned long>(psramTotal));
      }
    }

    vTaskDelay(pdMS_TO_TICKS(kStorageTaskPeriodMs));
  }
}

void gpsTask(void*) {
  // HardwareSerial operations (including driver-level reconfiguration/flush)
  // can block independently of our parser loop. GPS is non-critical, so do
  // not let a disconnected or noisy receiver reboot the entire dashboard.

  if (CCM_GPS_SERIAL_LOGS) Serial.println("[GPS] task started - waiting for fix");

  bool lastFix = false;
  uint32_t lastSearchMs = 0;

  while (true) {
    g_gps.poll();
    const ccm::core::GpsData d = g_gps.data();
    const uint32_t nowMs = millis();
    const bool rxLive = d.lastRxMs != 0 && ((nowMs - d.lastRxMs) <= 5000U);
    const bool fixLive = d.validFix && d.lastFixMs != 0 && ((nowMs - d.lastFixMs) <= 5000U);
    const state::VehicleState gpsStateBefore = state::g_vehicle_state.read();
    const GpsSpeedConditioner::Result gpsSpeed =
        gpsStateBefore.bench_test_mode
            ? GpsSpeedConditioner::Result{gpsStateBefore.speed, false, false, false}
            : g_gpsSpeedConditioner.update(d, fixLive, nowMs, gpsStateBefore);

    if (fixLive != lastFix) {
      lastFix = fixLive;
      if (fixLive) {
        if (CCM_GPS_SERIAL_LOGS) {
          Serial.printf("[GPS] fix acquired - baud=%lu sats=%lu view=%lu q=%u mode=%u hdop=%.1f lat=%.6f lon=%.6f\n",
            static_cast<unsigned long>(d.baud),
            static_cast<unsigned long>(d.satellites),
            static_cast<unsigned long>(d.satellitesInView),
            static_cast<unsigned>(d.fixQuality),
            static_cast<unsigned>(d.fixMode),
            static_cast<double>(d.hdopHundredths / 100.0f),
            d.latitude, d.longitude);
        }
      } else {
        if (CCM_GPS_SERIAL_LOGS) Serial.println("[GPS] fix lost - searching...");
      }
    } else if (!fixLive && (nowMs - lastSearchMs) >= 10000U) {
      lastSearchMs = nowMs;
      const uint32_t rxAge = d.lastRxMs == 0 ? 0xFFFFFFFFUL : (nowMs - d.lastRxMs);
      char rawHex[sizeof(d.rawSample) * 3U + 1U] = {};
      size_t rawHexPos = 0;
      for (uint8_t i = 0; i < d.rawSampleLen && rawHexPos + 3U < sizeof(rawHex); ++i) {
        rawHexPos += snprintf(rawHex + rawHexPos, sizeof(rawHex) - rawHexPos,
                              "%s%02X", i == 0 ? "" : " ", d.rawSample[i]);
      }
      const char* gpsSearchStatus =
          !rxLive ? "NO_UART_RX" :
          (d.passedChecksum == 0U && d.failedChecksum == 0U) ? "RX_NO_NMEA" :
          (d.failedChecksum > d.passedChecksum && d.passedChecksum == 0U) ? "BAD_NMEA" :
          (d.satellitesInView == 0U && d.satellites == 0U) ? "NMEA_NO_SATS" :
          "NMEA_NO_FIX";
      if (CCM_GPS_SERIAL_LOGS) {
        Serial.printf("[GPS] searching status=%s baud=%lu rx=%u rx_idle=%u rx_age=%lu raw=%lu printable=%lu dollar=%lu sample=%s chars=%lu ok=%lu err=%lu fix_sent=%lu sats=%lu view=%lu q=%u mode=%u hdop=%.1f\n",
          gpsSearchStatus,
          static_cast<unsigned long>(d.baud),
          rxLive ? 1U : 0U,
          static_cast<unsigned>(d.rxIdleLevel),
          static_cast<unsigned long>(rxAge),
          static_cast<unsigned long>(d.rawBytes),
          static_cast<unsigned long>(d.rawPrintableBytes),
          static_cast<unsigned long>(d.rawDollarBytes),
          d.rawSampleLen == 0 ? "-" : rawHex,
          static_cast<unsigned long>(d.charsProcessed),
          static_cast<unsigned long>(d.passedChecksum),
          static_cast<unsigned long>(d.failedChecksum),
          static_cast<unsigned long>(d.sentencesWithFix),
          static_cast<unsigned long>(d.satellites),
          static_cast<unsigned long>(d.satellitesInView),
          static_cast<unsigned>(d.fixQuality),
          static_cast<unsigned>(d.fixMode),
          static_cast<double>(d.hdopHundredths / 100.0f));
      }
    }

    state::g_vehicle_state.mutate([&](state::VehicleState& s) {
      // In bench test mode the CAN demo generator provides spoofed GPS data;
      // skip hardware GPS writes so they don't overwrite those values.
      if (s.bench_test_mode) return;
      s.gps_fix        = fixLive;
      s.speed          = gpsSpeed.speedKph;
      s.gps_satellites = static_cast<uint8_t>(d.satellites > 255U ? 255U : d.satellites);
      s.gps_satellites_in_view = static_cast<uint8_t>(d.satellitesInView > 255U ? 255U : d.satellitesInView);
      s.gps_fix_quality = d.fixQuality;
      s.gps_fix_mode = d.fixMode;
      s.gps_hdop_x10 = static_cast<uint16_t>((d.hdopHundredths + 5U) / 10U);
      if (!s.gps_time_valid && fixLive && d.utcTimeValid) {
        s.gps_time_valid = true;
        s.gps_utc_seconds_of_day =
            static_cast<uint32_t>(d.utcHour) * 3600UL +
            static_cast<uint32_t>(d.utcMinute) * 60UL +
            static_cast<uint32_t>(d.utcSecond);
        s.gps_time_sync_ms = d.lastTimeMs != 0U ? d.lastTimeMs : nowMs;
      }
      s.gps_fix_type = fixLive
          ? (d.fixQuality > 0U ? d.fixQuality : (d.fixMode > 0U ? d.fixMode : 2U))
          : (d.passedChecksum > 0 ? 1U : 0U);
      s.gps_status_flags = 0;
      if (rxLive) s.gps_status_flags |= kGpsStatusRxLive;
      if (d.passedChecksum > 0) s.gps_status_flags |= kGpsStatusChecksumOk;
      if (d.failedChecksum > d.passedChecksum && d.failedChecksum > 0) {
        s.gps_status_flags |= kGpsStatusChecksumWarn;
      }
      if (d.satellitesInView > 0U) s.gps_status_flags |= kGpsStatusSatsInView;
      if (d.fixQuality > 0U || d.fixMode >= 2U) s.gps_status_flags |= kGpsStatusFixQuality;
      if (gpsSpeed.deadReckoned) s.gps_status_flags |= kGpsStatusDeadReckoned;
      if (gpsSpeed.spikeRejected) s.gps_status_flags |= kGpsStatusSpikeRejected;
      if (gpsSpeed.zeroClamped) s.gps_status_flags |= kGpsStatusZeroClamped;
      if (d.validFix) {
        s.gps_latitude   = d.latitude;
        s.gps_longitude  = d.longitude;
        s.gps_altitude_m = static_cast<int16_t>(d.altitudeM);
      }
      s.gps_stale = !rxLive;
      if (d.lastRxMs != 0) s.last_gps_ms = d.lastRxMs;
    });
    feedTaskWatchdog();
    vTaskDelay(pdMS_TO_TICKS(fixLive ? 50U : 20U));
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  const uint32_t serialStartMs = millis();
  while (!Serial && (millis() - serialStartMs) < 2000U) {
    delay(10);
  }
  delay(100);
  initTaskWatchdog();

  const esp_reset_reason_t resetReason = esp_reset_reason();
  Serial.printf("[BOOT] reset=%s free_heap=%lu die_temp=%d\n",
                 resetReasonName(resetReason),
                 static_cast<unsigned long>(ESP.getFreeHeap()),
                 static_cast<int>(temperatureRead()));

  pinMode(pins::kLcdRst, OUTPUT);
  pinMode(pins::kLcdDc, OUTPUT);
#if CCM_CAN_ONLY_DEBUG
  digitalWrite(pins::kLcdRst, LOW);
  digitalWrite(pins::kLcdDc, LOW);
#else
  digitalWrite(pins::kLcdRst, HIGH);
  digitalWrite(pins::kLcdDc, HIGH);
#endif
  if (pins::kLcdBacklight != 255) {
    pinMode(pins::kLcdBacklight, OUTPUT);
#if CCM_CAN_ONLY_DEBUG
    digitalWrite(pins::kLcdBacklight, LOW);
#endif
  }
  hal::initSharedSpiBusLock();

  state::g_vehicle_state.begin();

#if CCM_CAN_ONLY_DEBUG
  pinMode(pins::kCanSpiCs, OUTPUT);
  pinMode(pins::kSdCs, OUTPUT);
  pinMode(pins::kLcdCs, OUTPUT);
  digitalWrite(pins::kCanSpiCs, HIGH);
  digitalWrite(pins::kSdCs, HIGH);
  digitalWrite(pins::kLcdCs, HIGH);

  Serial.printf("[CAN-ONLY] SPI SCK=%u MISO=%u MOSI=%u CAN_CS=%u CAN_INT=%u CAN_RST=%u\n",
                 static_cast<unsigned>(pins::kSpiSck),
                 static_cast<unsigned>(pins::kSpiMiso),
                 static_cast<unsigned>(pins::kSpiMosi),
                 static_cast<unsigned>(pins::kCanSpiCs),
                 static_cast<unsigned>(pins::kCanSpiInt),
                 static_cast<unsigned>(pins::kCanSpiRst));
  SPI.begin(pins::kSpiSck, pins::kSpiMiso, pins::kSpiMosi, -1);
  delay(5);
  Serial.println("[CAN-ONLY] screen/GPS/touch/SD/LED/sensors disabled");
  g_can.begin(true);
  Serial.println("[TASK] core0=CAN; heartbeat uses loopTask");
  createPinnedTask(canTask, "can_task", CCM_CAN_TASK_STACK_BYTES, kPrioHighIo, kCoreIo);
  return;
#endif

  g_settings.begin();
  applySettingsToState();
  configureAnalogSensorsFromState(state::g_vehicle_state.read());
  feedTaskWatchdog();
  if (pins::kLcdBacklight != 255) {
    analogWrite(pins::kLcdBacklight, state::g_vehicle_state.read().display_brightness);
  }

  pinMode(pins::kCanSpiCs, OUTPUT);
  pinMode(pins::kSdCs, OUTPUT);
  pinMode(pins::kLcdCs, OUTPUT);
  digitalWrite(pins::kCanSpiCs, HIGH);
  digitalWrite(pins::kSdCs, HIGH);
  digitalWrite(pins::kLcdCs, HIGH);

  // Start the one shared Arduino SPI instance used by LCD, CAN, and SD.
  // Keeping all three devices on the same SPI driver avoids ESP32-S3/Arduino 3.x
  // host reconfiguration glitches on the shared bus.
  Serial.printf("[SPI] shared Arduino SPI SCK=%u MISO=%u MOSI=%u SD_CS=%u CAN_CS=%u LCD_CS=%u\n",
                 static_cast<unsigned>(pins::kSpiSck),
                 static_cast<unsigned>(pins::kSpiMiso),
                 static_cast<unsigned>(pins::kSpiMosi),
                 static_cast<unsigned>(pins::kSdCs),
                 static_cast<unsigned>(pins::kCanSpiCs),
                 static_cast<unsigned>(pins::kLcdCs));
  SPI.begin(pins::kSpiSck, pins::kSpiMiso, pins::kSpiMosi, -1);
  digitalWrite(pins::kCanSpiCs, HIGH);
  digitalWrite(pins::kSdCs, HIGH);
  digitalWrite(pins::kLcdCs, HIGH);
  delay(5);
  feedTaskWatchdog();

  g_screen.attach(&g_can, &g_race, &g_settings, &g_sd, &g_touch);
  if (CCM_SCREEN_SERIAL_LOGS) {
    Serial.printf("[SETUP] heap free before screen init: %lu bytes\n",
      static_cast<unsigned long>(ESP.getFreeHeap()));
    Serial.printf("[SCREEN] pins  CS=%d RST=%d DC=%d SCK=%d MOSI=%d MISO=%d\n",
      pins::kLcdCs, pins::kLcdRst, pins::kLcdDc,
      pins::kSpiSck, pins::kSpiMosi, pins::kSpiMiso);
  }
  const bool screenOk = g_screen.begin(
    pins::kLcdCs, pins::kLcdRst, pins::kLcdDc,
    pins::kSpiSck, pins::kSpiMosi, pins::kSpiMiso);
  if (CCM_SCREEN_SERIAL_LOGS) Serial.printf("[SCREEN] begin() -> %s\n", screenOk ? "OK" : "FAILED");
  if (screenOk && CCM_SCREEN_SETUP_FIRST_PAINT_MS > 0) {
    Serial.println("[SCREEN] first paint start");
    const uint32_t firstPaintStartMs = millis();
    while ((millis() - firstPaintStartMs) < static_cast<uint32_t>(CCM_SCREEN_SETUP_FIRST_PAINT_MS)) {
      const state::VehicleState s = state::g_vehicle_state.read();
      g_screen.tick(s, millis());
      feedTaskWatchdog();
      delay(5);
    }
    Serial.printf("[SCREEN] first paint done in %lums\n",
                  static_cast<unsigned long>(millis() - firstPaintStartMs));
  }
  feedTaskWatchdog();

#if CCM_DISPLAY_LCD_ONLY_TEST
  Serial.println("[LCD-ONLY] enabled: skipping CAN, SD, touch, sensors, storage, race, web, and LED tasks");
  Serial.println("[TASK] core1=screen; heartbeat uses loopTask");
  createPinnedTask(screenTask, "screen_task", 12288, kPrioUi, kCoreUi);
  return;
#endif

  // CAN (CS=11), SD (CS=5), and LCD (CS=10) share the same SPI pins and the
  // same Arduino SPI driver. All bus users take SharedSpiBusLock before IO.
  const bool touchOk = g_touch.begin(Wire, pins::kTouchSda, pins::kTouchScl, pins::kTouchRst, pins::kTouchInt);
  // Publish the startup probe result before worker tasks begin. The touch task
  // must never wait on the general vehicle-state mutex in its 10 ms input loop.
  state::g_vehicle_state.mutate([touchOk](state::VehicleState& s) {
    s.touch_online = touchOk;
  });
#if CCM_IMU_ENABLED
  const bool imuOk = g_imu.begin(Wire);  // MPU-6050 shares the same I2C bus
  const char* imuStatus = imuOk ? "OK" : "OFF";
#else
  const char* imuStatus = "SKIP";
#endif
  Serial.printf("[I2C] touch=%s imu=%s SDA=%u SCL=%u\n",
                 touchOk ? "OK" : "OFF",
                 imuStatus,
                 static_cast<unsigned>(pins::kTouchSda),
                 static_cast<unsigned>(pins::kTouchScl));
  // Keep the shared SPI bus single-threaded until CAN and SD finish their
  // synchronous startup. Starting screenTask here can race SD.begin() and
  // leave setup blocked forever waiting for the display's SPI lock.
  g_can.begin(true);
  g_can.requestKnockConfig();
  // GPS starts at the configured baud and auto-probes common NMEA baud rates.
  g_gps.begin(pins::kGpsBaud);  // Serial2 GPIO42 RX / GPIO41 TX
  Serial.println("[SD] begin start");
  g_sd.begin(pins::kLcdCs, pins::kSdCs);
  Serial.println("[SD] begin done");
  g_assets.begin(&g_sd);
  g_logs.begin(&g_sd);
  g_telemetry.begin(&g_sd, static_cast<uint8_t>(esp_reset_reason()),
                    microsquirt::kRecommendedRealtimeBaseId);
  g_can.attachTelemetryRecorder(&g_telemetry);
  if (pins::kIgnitionSense != 255U) {
    pinMode(pins::kIgnitionSense, INPUT_PULLDOWN);
  }
  { char pfx[32]; snprintf(pfx, sizeof(pfx), "boot_%lu", static_cast<unsigned long>(millis())); g_logs.setSessionPrefix(pfx); }
  g_race.begin(&state::g_vehicle_state, &g_settings, &g_logs);
#if CCM_LOCAL_KNOCK_ENABLED
  g_knock.begin(&state::g_vehicle_state, &g_settings, &g_logs, &g_sd, &g_can);
#else
  Serial.println("[KNOCK] local ADC disabled; waiting for CAN knock state");
  state::g_vehicle_state.mutate([](state::VehicleState& s) {
    s.knock_online = false;
    s.last_knock_ms = 0;
    s.knock_signal_valid = false;
    s.knock_warning_active = false;
    s.knock_critical_active = false;
    s.knock_sensor_fault = false;
    s.knock_clipping_detected = false;
  });
#endif
  feedTaskWatchdog();

  // Channel 1 is the RPM strip. Channels 2/3 are solid interior strips, so they
  // over-send a safe number of LED slots instead of needing an exact count.
#if CCM_LED_ENABLED
  g_led.begin(pins::kLedData1, pins::kLedData2, pins::kLedData3,
              kMainLedCount, kInteriorLedSendCount, kInteriorLedSendCount, 0);
#else
  Serial.println("[LED] disabled by build flag");
#endif
  Serial.println("[TASK] core0=CAN/GPS/sensors/storage core1=screen/LED; race/hb use loopTask; touch owned by screen");
  createPinnedTask(screenTask, "screen_task", 10240, kPrioUi, kCoreUi);
  createPinnedTask(canTask, "can_task", CCM_CAN_TASK_STACK_BYTES, kPrioHighIo, kCoreIo);
  createPinnedTask(tachInputTask, "tach_in_task", 3072, kPrioHighIo, kCoreIo);
  createPinnedTask(gpsTask, "gps_task", 3584, kPrioSensors, kCoreIo);
#if CCM_LED_ENABLED
  createPinnedTask(ledTask, "led_task", CCM_LED_TASK_STACK_BYTES, kPrioSensors, kCoreIo);
#endif
#if CCM_SD_ENABLED
  createPinnedTask(storageTask, "storage_task", CCM_STORAGE_TASK_STACK_BYTES,
                   kPrioBackground, kCoreIo);
#endif
  createPinnedTask(analogSensorTask, "analog_sensor_task", 3584, kPrioSensors, kCoreIo);
#if CCM_LOCAL_KNOCK_ENABLED
  createPinnedTask(knockTask, "knock_task", 3584, kPrioSensors, kCoreIo);
#endif
  // setup() can block on peripheral bring-up; register loopTask only once loop() starts.
}

void loop() {
  // loopTask is intentionally not part of TWDT. The functional work in this
  // firmware happens in pinned FreeRTOS tasks that already register and feed
  // the watchdog themselves. Registering loopTask adds a low-priority CPU1
  // observer that can be starved by normal UI activity and cause false resets
  // while the real tasks are still healthy.
  static uint32_t lastHeartbeatMs = 0;
  static uint32_t lastRaceMs = 0;
  const uint32_t nowMs = millis();

  if ((nowMs - lastHeartbeatMs) >= 20U) {
    lastHeartbeatMs = nowMs;
    state::g_vehicle_state.mutate([](state::VehicleState& s) {
      s.input_flags = 0;
      s.master_state = static_cast<uint8_t>(
          s.fault_flags != 0 ? can_protocol::MasterState::WARN
                             : can_protocol::MasterState::RUN);
    });
  }

#if !CCM_CAN_ONLY_DEBUG && !CCM_DISPLAY_LCD_ONLY_TEST
  if ((nowMs - lastRaceMs) >= 50U) {
    lastRaceMs = nowMs;
    g_race.tick(nowMs);
  }
#else
  (void)lastRaceMs;
#endif

  vTaskDelay(pdMS_TO_TICKS(10));
}
