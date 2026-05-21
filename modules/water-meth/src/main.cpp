#include <Arduino.h>
#include <Preferences.h>
#include <stdlib.h>

#include "actuators.h"
#include "app_config.h"
#include "can_bridge.h"
#include "injection_controller.h"
#include "pins.h"
#include "sensors.h"

namespace {
AppConfig config = defaultConfig();
TankBlend blend = computeTankBlend(1.5f, 0.5f);

MapSensor mapSensor;
FloatSensor floatSensor;
ThermistorSensor iatSensor;
ThermistorSensor engineBaySensor;
ThermistorSensor cabinSensor;
ThermistorSensor ambientSensor;
PressureSensor oilPressure;
PressureSensor fuelPressure;
PressureSensor methPressure;
PressureSensor boostRefPressure;
PressureSensor sparePressure1;
PressureSensor sparePressure2;
PumpDriver pumpDriver;
WarningOutput warningOutput;
InjectionController controller;
CanBridge canBridge;
Preferences preferences;

uint32_t lastLoopMs = 0;
uint32_t lastDebugMs = 0;
FailsafeReason lastFailsafe = FailsafeReason::None;
FailsafeReason lastReportedCanFault = FailsafeReason::None;
String serialLine;

constexpr char kPrefsNamespace[] = "wmix";
constexpr char kPrefsKeyWater[] = "water_l";
constexpr char kPrefsKeyMeth[] = "meth_l";

constexpr uint16_t kFaultIat = 1U << 0;
constexpr uint16_t kFaultEngineBay = 1U << 1;
constexpr uint16_t kFaultCabin = 1U << 2;
constexpr uint16_t kFaultAmbient = 1U << 3;
constexpr uint16_t kFaultOil = 1U << 4;
constexpr uint16_t kFaultFuel = 1U << 5;
constexpr uint16_t kFaultMeth = 1U << 6;
constexpr uint16_t kFaultBoostRef = 1U << 7;
constexpr uint16_t kFaultSpare1 = 1U << 8;
constexpr uint16_t kFaultSpare2 = 1U << 9;

inline bool elapsed(uint32_t now, uint32_t start, uint32_t intervalMs) {
  return static_cast<uint32_t>(now - start) >= intervalMs;
}

void printHelp() {
  Serial.println("Commands:");
  Serial.println("  WATER <liters>   - set water volume");
  Serial.println("  METH <liters>    - set methanol volume");
  Serial.println("  SHOW             - print current config/blend");
  Serial.println("  MODE OFF|BOOST|PRIME");
  Serial.println("  HELP");
}

const char *modeName(InjectionMode mode) {
  switch (mode) {
  case InjectionMode::Off: return "OFF";
  case InjectionMode::BoostOnly: return "BOOST";
  case InjectionMode::Prime: return "PRIME";
  default: return "UNKNOWN";
  }
}

const char *failsafeName(FailsafeReason reason) {
  switch (reason) {
  case FailsafeReason::None: return "NONE";
  case FailsafeReason::LowFluid: return "LOW_FLUID";
  case FailsafeReason::MapInvalid: return "MAP_INVALID";
  case FailsafeReason::InvalidBlend: return "INVALID_BLEND";
  case FailsafeReason::InvalidBoostConfig: return "INVALID_BOOST_CONFIG";
  default: return "UNKNOWN";
  }
}

void saveBlend() {
  if (!preferences.begin(kPrefsNamespace, false)) return;
  preferences.putFloat(kPrefsKeyWater, blend.waterLiters);
  preferences.putFloat(kPrefsKeyMeth, blend.methLiters);
  preferences.end();
}

void loadBlend() {
  if (!preferences.begin(kPrefsNamespace, true)) return;
  const float water = preferences.getFloat(kPrefsKeyWater, blend.waterLiters);
  const float meth = preferences.getFloat(kPrefsKeyMeth, blend.methLiters);
  preferences.end();
  blend = computeTankBlend(water, meth);
}

void printSetupSummary() {
  Serial.println("---- Water/Meth Controller ----");
  Serial.print("Mode: "); Serial.println(modeName(config.mode));
  Serial.print("MAP calibration (V): ");
  Serial.print(config.map.vMin, 2); Serial.print(" to "); Serial.print(config.map.vMax, 2);
  Serial.print(" -> (kPa): "); Serial.print(config.map.kpaMin, 1);
  Serial.print(" to "); Serial.println(config.map.kpaMax, 1);
  Serial.print("Boost start/full (psi): ");
  Serial.print(config.boost.startPsi, 1); Serial.print(" / "); Serial.println(config.boost.fullPsi, 1);
  Serial.print("Blend water/meth (L): ");
  Serial.print(blend.waterLiters, 2); Serial.print(" / "); Serial.print(blend.methLiters, 2);
  Serial.print("  Meth%: "); Serial.println(blend.methPercent, 1);
  Serial.println("--------------------------------");
}

void configureAnalogSensors() {
  ThermistorConfig t{};
  t.enabled = true;
  t.pullupOhms = 10000.0f;
  t.filterAlpha = 0.2f;

  t.pin = pins::IAT_THERM_PIN;
  iatSensor.begin(t);
  t.pin = pins::ENGINE_BAY_THERM_PIN;
  t.maxValidTempC = 200.0f;
  engineBaySensor.begin(t);
  t.pin = pins::CABIN_THERM_PIN;
  t.maxValidTempC = 120.0f;
  cabinSensor.begin(t);
  t.pin = pins::AMBIENT_THERM_PIN;
  ambientSensor.begin(t);

  PressureConfig p{};
  p.enabled = true;
  p.sensorMinV = 0.5f;
  p.sensorMaxV = 4.5f;
  p.pressureMaxPsi = 100.0f;
  p.maxValidPsi = 250.0f;
  p.dividerTopOhms = 10000.0f;
  p.dividerBottomOhms = 20000.0f;

  p.pin = pins::OIL_PRESSURE_ADC;
  oilPressure.begin(p);
  p.pin = pins::FUEL_PRESSURE_ADC;
  fuelPressure.begin(p);
  p.pin = pins::METH_PRESSURE_ADC;
  methPressure.begin(p);
  p.pin = pins::BOOST_REF_PRESSURE_ADC;
  boostRefPressure.begin(p);
  p.pin = pins::SPARE_PRESSURE_1_ADC;
  sparePressure1.begin(p);
  p.pin = pins::SPARE_PRESSURE_2_ADC;
  sparePressure2.begin(p);
}

void updateAnalogReadings(SensorReadings &r, uint32_t nowMs) {
  iatSensor.update(nowMs);
  engineBaySensor.update(nowMs);
  cabinSensor.update(nowMs);
  ambientSensor.update(nowMs);
  oilPressure.update(nowMs);
  fuelPressure.update(nowMs);
  methPressure.update(nowMs);
  boostRefPressure.update(nowMs);
  sparePressure1.update(nowMs);
  sparePressure2.update(nowMs);

  r.analogFaultFlags = 0;
  r.iatValid = iatSensor.valid();
  r.engineBayValid = engineBaySensor.valid();
  r.cabinValid = cabinSensor.valid();
  r.ambientValid = ambientSensor.valid();
  r.oilPressureValid = oilPressure.valid();
  r.fuelPressureValid = fuelPressure.valid();
  r.methPressureValid = methPressure.valid();
  r.boostRefPressureValid = boostRefPressure.valid();
  r.sparePressure1Valid = sparePressure1.valid();
  r.sparePressure2Valid = sparePressure2.valid();

  if (r.iatValid) r.iatC = iatSensor.valueC(); else if (iatSensor.config().enabled) r.analogFaultFlags |= kFaultIat;
  if (r.engineBayValid) r.engineBayC = engineBaySensor.valueC(); else if (engineBaySensor.config().enabled) r.analogFaultFlags |= kFaultEngineBay;
  if (r.cabinValid) r.cabinC = cabinSensor.valueC(); else if (cabinSensor.config().enabled) r.analogFaultFlags |= kFaultCabin;
  if (r.ambientValid) r.ambientC = ambientSensor.valueC(); else if (ambientSensor.config().enabled) r.analogFaultFlags |= kFaultAmbient;
  if (r.oilPressureValid) r.oilPressurePsi = oilPressure.valuePsi(); else if (oilPressure.config().enabled) r.analogFaultFlags |= kFaultOil;
  if (r.fuelPressureValid) r.fuelPressurePsi = fuelPressure.valuePsi(); else if (fuelPressure.config().enabled) r.analogFaultFlags |= kFaultFuel;
  if (r.methPressureValid) r.methPressurePsi = methPressure.valuePsi(); else if (methPressure.config().enabled) r.analogFaultFlags |= kFaultMeth;
  if (r.boostRefPressureValid) r.boostRefPressurePsi = boostRefPressure.valuePsi(); else if (boostRefPressure.config().enabled) r.analogFaultFlags |= kFaultBoostRef;
  if (r.sparePressure1Valid) r.sparePressure1Psi = sparePressure1.valuePsi(); else if (sparePressure1.config().enabled) r.analogFaultFlags |= kFaultSpare1;
  if (r.sparePressure2Valid) r.sparePressure2Psi = sparePressure2.valuePsi(); else if (sparePressure2.config().enabled) r.analogFaultFlags |= kFaultSpare2;
}

bool parsePositiveFloat(const String &token, float &valueOut) {
  char buffer[32];
  token.toCharArray(buffer, sizeof(buffer));
  char *endPtr = nullptr;
  const float parsed = strtof(buffer, &endPtr);
  if (endPtr == buffer) return false;
  valueOut = parsed;
  return true;
}

void parseCommand(const String &line) {
  String cmd = line;
  cmd.trim();
  if (cmd.length() == 0) return;

  if (cmd.equalsIgnoreCase("HELP")) { printHelp(); return; }
  if (cmd.equalsIgnoreCase("SHOW")) { printSetupSummary(); return; }

  if (cmd.startsWith("WATER ") || cmd.startsWith("water ")) {
    float value = 0.0f;
    if (parsePositiveFloat(cmd.substring(6), value) && value >= 0.0f) {
      blend = computeTankBlend(value, blend.methLiters);
      saveBlend();
      Serial.print("Water updated. Meth% = "); Serial.println(blend.methPercent, 1);
    } else Serial.println("Invalid WATER value.");
    return;
  }

  if (cmd.startsWith("METH ") || cmd.startsWith("meth ")) {
    float value = 0.0f;
    if (parsePositiveFloat(cmd.substring(5), value) && value >= 0.0f) {
      blend = computeTankBlend(blend.waterLiters, value);
      saveBlend();
      Serial.print("Meth updated. Meth% = "); Serial.println(blend.methPercent, 1);
    } else Serial.println("Invalid METH value.");
    return;
  }

  if (cmd.startsWith("MODE ") || cmd.startsWith("mode ")) {
    const String value = cmd.substring(5);
    if (value.equalsIgnoreCase("OFF")) config.mode = InjectionMode::Off;
    else if (value.equalsIgnoreCase("BOOST")) config.mode = InjectionMode::BoostOnly;
    else if (value.equalsIgnoreCase("PRIME")) config.mode = InjectionMode::Prime;
    else {
      Serial.println("Invalid MODE. Use OFF|BOOST|PRIME");
      return;
    }
    Serial.print("Mode set to "); Serial.println(modeName(config.mode));
    return;
  }

  Serial.println("Unknown command. Type HELP.");
}

void handleSerialCommands() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r') continue;
    if (c == '\n') {
      parseCommand(serialLine);
      serialLine = "";
      continue;
    }
    if (serialLine.length() < 120) serialLine += c;
  }
}
} // namespace

void setup() {
  Serial.begin(config.serialBaud);
  delay(200);

  loadBlend();
  mapSensor.begin(pins::MAP_SENSOR_ADC, config.map);
  floatSensor.begin(pins::FLOAT_SENSOR_DIGITAL, config.floatActiveLow, config.floatDebounceMs);
  configureAnalogSensors();
  pumpDriver.begin(pins::PUMP_PWM, config.pwmFrequencyHz, config.pwmResolutionBits);
  warningOutput.begin(pins::WARNING_LED, true);

  if (canBridge.begin(pins::CAN_TX, pins::CAN_RX)) Serial.println("CAN: online");
  else Serial.println("CAN: TWAI init failed — running serial-only");

  printSetupSummary();
  printHelp();
}

void loop() {
  handleSerialCommands();
  const uint32_t now = millis();
  if (!elapsed(now, lastLoopMs, config.loopPeriodMs)) return;
  lastLoopMs = now;

  canBridge.poll();

  if (canBridge.hasRemoteRatio()) {
    const uint8_t pct = canBridge.remoteRatioPercent();
    blend = computeTankBlend(static_cast<float>(100 - pct), static_cast<float>(pct));
    canBridge.clearRemoteRatio();
  }
  if (canBridge.hasClearFaultsRequest()) {
    lastFailsafe = FailsafeReason::None;
    lastReportedCanFault = FailsafeReason::None;
    canBridge.clearFaultsRequest();
  }

  SensorReadings readings = mapSensor.read();
  readings.tankLow = floatSensor.update();
  updateAnalogReadings(readings, now);

  AppConfig effectiveConfig = config;
  if (!canBridge.isArmed() && canBridge.isOnline()) effectiveConfig.mode = InjectionMode::Off;

  ControlResult result{};
  if (canBridge.hasPendingManualTest()) {
    result.pump.enabled = true;
    result.pump.dutyPercent = constrain(canBridge.manualTestDuty(), 0, 100);
    result.finalDutyPercent = result.pump.dutyPercent;
  } else {
    result = controller.update(readings, effectiveConfig, blend);
  }

  pumpDriver.apply(result.pump);
  warningOutput.set(result.failsafe != FailsafeReason::None);

  if (result.failsafe != lastReportedCanFault) {
    if (result.failsafe != FailsafeReason::None) {
      uint8_t code = 0;
      switch (result.failsafe) {
        case FailsafeReason::LowFluid: code = 0x01; break;
        case FailsafeReason::MapInvalid: code = 0x05; break;
        case FailsafeReason::InvalidBlend: code = 0x08; break;
        case FailsafeReason::InvalidBoostConfig: code = 0x08; break;
        default: code = 0x09; break;
      }
      canBridge.sendFault(code, 1, 0, 0);
    }
    lastReportedCanFault = result.failsafe;
  }

  if (result.failsafe != lastFailsafe) {
    Serial.print("Failsafe state changed: ");
    Serial.println(failsafeName(result.failsafe));
    lastFailsafe = result.failsafe;
  }

  canBridge.sendStateIfDue(readings, result, effectiveConfig, now);

  if (!elapsed(now, lastDebugMs, config.debugPeriodMs)) return;
  lastDebugMs = now;

  Serial.print("boostPsi="); Serial.print(readings.boostPsi, 2);
  Serial.print(" iatC="); Serial.print(readings.iatC, 1);
  Serial.print(" bayC="); Serial.print(readings.engineBayC, 1);
  Serial.print(" ambC="); Serial.print(readings.ambientC, 1);
  Serial.print(" oil/fuel/meth=");
  Serial.print(readings.oilPressurePsi, 1); Serial.print("/");
  Serial.print(readings.fuelPressurePsi, 1); Serial.print("/");
  Serial.print(readings.methPressurePsi, 1);
  Serial.print(" dutyOut="); Serial.print(result.finalDutyPercent, 1);
  Serial.print(" fs="); Serial.print(failsafeName(result.failsafe));
  Serial.print(" analogFault=0x"); Serial.print(readings.analogFaultFlags, HEX);
  Serial.println();
}

