#include <Arduino.h>
#include <SPI.h>
#include <Preferences.h>
#include <mcp2515.h>
#include <Adafruit_NeoPixel.h>
#include <esp_ota_ops.h>
#include "pin_map.h"
#include "config.h"
#include "logic.h"
#include "steering.h"
#include <can_contract/gateway_protocol.h>
#include <can_contract/firmware_update.h>

// Identifies this build to the dash. The published firmware is built with the first
// eight hex digits of its commit; a build from the editor reports 0.
#ifndef CCM_FW_BUILD
#define CCM_FW_BUILD 0
#endif

// A firmware received over CAN stays on trial until the dash confirms it (see loop()).
// Without this the Arduino core would accept it the moment it starts.
extern "C" bool verifyRollbackLater() { return true; }

// NeoPixel 1.12.3 uses a stack-based RMT symbol array on Arduino 3.x.
// Only one strip is transmitted at a time; retain 16 KiB for the rest of loop().
SET_LOOP_TASK_STACK_SIZE(16384 + gateway_config::ledCount * 24 * sizeof(rmt_data_t));

namespace {
using namespace gateway_config;
namespace wire = can_protocol::gateway;
namespace fw = can_protocol::firmware;
constexpr bool validPins() {
  const int used[] = {vssPin, tachPin, fuelPin, steeringOnPin, steeringLadderPin, upperLedPin, lowerLedPin,
      spareLed1Pin, spareLed2Pin,
      CCM_PIN_SPI_SCK, CCM_PIN_SPI_MOSI, CCM_PIN_SPI_MISO,
      CCM_PIN_CAN_SPI_CS, CCM_PIN_CAN_SPI_INT, CCM_PIN_CAN_SPI_RST};
  for (unsigned i = 0; i < sizeof(used) / sizeof(used[0]); ++i) {
    if (used[i] < 0 || used[i] > 48 || used[i] == 19 || used[i] == 20 ||
        (used[i] >= 22 && used[i] <= 37)) return false;
    for (unsigned j = 0; j < i; ++j) if (used[i] == used[j]) return false;
  }
  return fuelPin >= 1 && fuelPin <= 10 && steeringLadderPin >= 1 && steeringLadderPin <= 10 &&
      steeringOnPin >= 1 && steeringOnPin <= 10;
}
static_assert(validPins(), "Gateway pins must be unique, USB/flash/PSRAM-safe; fuel and steering ladder need ADC1");
// Supplying &SPI avoids the library starting the default pin bus in its constructor.
MCP2515 can(CCM_PIN_CAN_SPI_CS, 1000000, &SPI);
Adafruit_NeoPixel upper(ledCount, upperLedPin, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel lower(ledCount, lowerLedPin, NEO_GRB + NEO_KHZ800);
gateway::SteeringButtons buttons;
wire::Buttons buttonState{0, 31, 0};
wire::Sensors sensors;
uint8_t tachCalibration = tachPulsesPerRev10;
volatile gateway::Pulse vss, tach;
portMUX_TYPE pulseLock = portMUX_INITIALIZER_UNLOCKED;
struct LightState { wire::Light value{}; gateway::CommandLease lease; };
LightState lights[2];
bool lightsDirty = true, canReady = false, txPending = false, txBackoff = false;
bool buttonsDirty = true;
uint32_t txStart = 0, txFailure = 0, lastTx = 0, lastFuel = 0, lastLed = 0;
uint8_t txSlot = 0;
float filteredFuel = 0;
bool fuelFilterReady = false;
bool onHigh = false;

// Firmware update over CAN (can_contract/firmware_update.h). The image is written to
// the spare application slot; the running firmware is untouched until the restart.
struct OtaFlash {
  esp_ota_handle_t handle = 0;
  const esp_partition_t* slot = nullptr;
  bool open = false;
  bool begin(uint32_t size) {
    slot = esp_ota_get_next_update_partition(nullptr);
    if (!slot || size > slot->size) return false;
    // Sequential: each sector is erased as the writing reaches it, never one long pause.
    open = esp_ota_begin(slot, OTA_WITH_SEQUENTIAL_WRITES, &handle) == ESP_OK;
    return open;
  }
  bool write(const uint8_t* data, uint32_t length) { return open && esp_ota_write(handle, data, length) == ESP_OK; }
  bool finish() {
    if (!open) return false;
    open = false;
    // esp_ota_end checks the image's own checksum before it may be started.
    return esp_ota_end(handle) == ESP_OK && esp_ota_set_boot_partition(slot) == ESP_OK;
  }
  void abort() { if (open) esp_ota_abort(handle); open = false; }
  bool confirm() { return esp_ota_mark_app_valid_cancel_rollback() == ESP_OK; }
};
OtaFlash otaFlash;
fw::Receiver<OtaFlash> updater(fw::TARGET_GATEWAY, CCM_FW_BUILD, otaFlash);
uint32_t restartAt = 0;
// A new firmware that the dash has not confirmed within this time is given up: the
// gateway restarts into the firmware it had before.
constexpr uint32_t trialMs = 90000;

void ARDUINO_ISR_ATTR capture(volatile gateway::Pulse& p, uint32_t minimum, uint32_t timeout) {
  const uint32_t now = micros();
  portENTER_CRITICAL_ISR(&pulseLock);
  p.edge(now, minimum, timeout);
  portEXIT_CRITICAL_ISR(&pulseLock);
}
void ARDUINO_ISR_ATTR vssIsr() { capture(vss, vssMinPulseUs, vssTimeoutUs); }
void ARDUINO_ISR_ATTR tachIsr() { capture(tach, tachMinPulseUs, tachTimeoutUs); }

uint8_t readRegister(uint8_t address) {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CCM_PIN_CAN_SPI_CS, LOW);
  SPI.transfer(0x03); SPI.transfer(address); uint8_t value = SPI.transfer(0);
  digitalWrite(CCM_PIN_CAN_SPI_CS, HIGH); SPI.endTransaction(); return value;
}
void modifyRegister(uint8_t address, uint8_t mask, uint8_t value) {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CCM_PIN_CAN_SPI_CS, LOW);
  SPI.transfer(0x05); SPI.transfer(address); SPI.transfer(mask); SPI.transfer(value);
  digitalWrite(CCM_PIN_CAN_SPI_CS, HIGH); SPI.endTransaction();
}
bool startCan() {
  pinMode(CCM_PIN_CAN_SPI_RST, OUTPUT);
  digitalWrite(CCM_PIN_CAN_SPI_RST, LOW); delay(2);
  digitalWrite(CCM_PIN_CAN_SPI_RST, HIGH); delay(10);
  SPI.begin(CCM_PIN_SPI_SCK, CCM_PIN_SPI_MISO, CCM_PIN_SPI_MOSI, CCM_PIN_CAN_SPI_CS);
  if (can.reset() != MCP2515::ERROR_OK ||
      can.setBitrate(CAN_500KBPS, MCP_8MHZ) != MCP2515::ERROR_OK) return false;
  if (can.setFilterMask(MCP2515::MASK0, false, 0x7FF) != MCP2515::ERROR_OK ||
      can.setFilterMask(MCP2515::MASK1, false, 0x7FF) != MCP2515::ERROR_OK) return false;
  // Both receive buffers accept light commands and firmware-update frames, nothing else.
  for (uint8_t i = 0; i < 6; ++i)
    if (can.setFilter(static_cast<MCP2515::RXF>(i), false,
                      (i == 1 || i == 5) ? fw::ID_COMMAND : wire::LIGHT_COMMAND) != MCP2515::ERROR_OK)
      return false;
  if (can.setNormalMode() != MCP2515::ERROR_OK) return false;
  // Avoid the library's setNormalOneShotMode readback bug; see taillight transport.
  modifyRegister(0x0F, 0x08, 0x08);
  return (readRegister(0x0F) & 0xE8) == 0x08;
}
void failTx(uint32_t now) {
  modifyRegister(0x30, 0x08, 0); txPending = false;
  txBackoff = true; txFailure = now;
}
bool sendFrame(const can_protocol::CanFrame& source, uint32_t now) {
  if (txBackoff && uint32_t(now - txFailure) < 1000) return false;
  txBackoff = false;
  if (txPending) {
    const uint8_t ctrl = readRegister(0x30);
    if (ctrl & 0x08) {
      if (uint32_t(now - txStart) >= 20) failTx(now);
      return false;
    }
    txPending = false;
    // Arbitration loss alone is normal bus contention, not a disconnected bus.
    if (ctrl & 0x50) { failTx(now); return false; }
  }
  if (can.getErrorFlags() & 0x20) { failTx(now); return false; }
  can_frame frame{}; frame.can_id = source.id; frame.can_dlc = source.dlc;
  memcpy(frame.data, source.data, source.dlc);
  if (can.sendMessage(MCP2515::TXB0, &frame) != MCP2515::ERROR_OK) {
    failTx(now); return false;
  }
  txPending = true; txStart = now; return true;
}
// Answers during an update go out at once: not in the 10 ms telemetry slot, and not
// held back by its retry pause. A lost answer is repeated by the dash asking again.
void sendUrgent(const can_protocol::CanFrame& source) {
  const uint32_t start = millis();
  while (readRegister(0x30) & 0x08) {
    if (uint32_t(millis() - start) >= 25) { modifyRegister(0x30, 0x08, 0); break; }
  }
  can_frame frame{}; frame.can_id = source.id; frame.can_dlc = source.dlc;
  memcpy(frame.data, source.data, source.dlc);
  txBackoff = false;
  if (can.sendMessage(MCP2515::TXB0, &frame) == MCP2515::ERROR_OK) { txPending = true; txStart = millis(); }
}
void handleUpdate(const can_frame& input, uint32_t now) {
  if (input.can_dlc != 8) return;
  can_protocol::CanFrame frame{}; frame.id = input.can_id; frame.dlc = 8;
  memcpy(frame.data, input.data, 8);
  can_protocol::CanFrame reply{};
  fw::Action action = fw::Action::NONE;
  if (!updater.handle(frame, reply, now, action)) return;
  sendUrgent(reply);
  if (action == fw::Action::STARTED) Serial.printf("[gateway] firmware update started: %lu bytes\n",
      static_cast<unsigned long>(updater.size()));
  if (action == fw::Action::STOPPED) Serial.println("[gateway] firmware update stopped; running firmware kept");
  if (action == fw::Action::CONFIRMED) Serial.println("[gateway] new firmware confirmed by the dash");
  if (action == fw::Action::RESTART) {
    Serial.println("[gateway] firmware received and checked; restarting into it");
    restartAt = (now + 300) | 1;  // After the answer has left. Never 0, which means "not due".
  }
}
void readSensors(uint32_t now) {
  uint32_t vp, tp;
  portENTER_CRITICAL(&pulseLock);
  const uint32_t us = micros();
  vp = vss.periodAt(us, vssTimeoutUs); tp = tach.periodAt(us, tachTimeoutUs);
  portEXIT_CRITICAL(&pulseLock);
  const bool vr = vp != 0, tr = tp != 0;
  sensors.speedKph10 = vr ? gateway::speedKph10(vp, vssPulsesPerMile) : 0;
  sensors.rpm = tr ? gateway::rpm(tp, tachCalibration) : 0;
  sensors.valid = (sensors.valid & wire::FUEL_VALID) |
      (vr ? wire::VSS_VALID : 0) | (tr ? wire::RPM_VALID : 0);
  if (uint32_t(now - lastFuel) < 100) return;
  lastFuel = now;
  sensors.fuelRaw = analogRead(fuelPin);
  float resistance;
  const bool valid = gateway::fuelResistance(analogReadMilliVolts(fuelPin), fuelSupplyMv,
      fuelPullupOhms, resistance) && resistance >= min(fuelEmptyOhms, fuelFullOhms) * 0.5f &&
      resistance <= max(fuelEmptyOhms, fuelFullOhms) * 1.5f;
  if (!valid) {
    fuelFilterReady = false; sensors.fuelPercent = 255;
    sensors.valid &= ~wire::FUEL_VALID; return;
  }
  filteredFuel = fuelFilterReady ? filteredFuel + (resistance - filteredFuel) * 0.05f : resistance;
  fuelFilterReady = true;
  sensors.fuelPercent = gateway::fuelPercent(filteredFuel, fuelEmptyOhms, fuelFullOhms);
  sensors.valid |= wire::FUEL_VALID;
}
void readButtons(uint32_t now) {
  const uint32_t onMv = analogReadMilliVolts(steeringOnPin);
  onHigh = gateway::onPressed(onMv, onHigh);
  const uint8_t mask = buttons.update(onHigh, analogReadMilliVolts(steeringLadderPin), now);
  if (mask != buttonState.pressed) {
    buttonState.pressed = mask; ++buttonState.sequence; buttonsDirty = true;
  }
}
void receive(uint32_t now) {
  if (can.getErrorFlags() & 0xC0) can.clearRXnOVRFlags();
  can_frame input{};
  const uint8_t limit = updater.active() ? 32 : 4;
  for (uint8_t n = 0; n < limit && can.readMessage(&input) == MCP2515::ERROR_OK; ++n) {
    if (input.can_id == fw::ID_COMMAND) { handleUpdate(input, now); continue; }
    // Reject extended, RTR and error frames before narrowing the ID.
    if (input.can_id != wire::LIGHT_COMMAND || input.can_dlc != 6) continue;
    can_protocol::CanFrame frame{}; frame.id = input.can_id; frame.dlc = input.can_dlc;
    memcpy(frame.data, input.data, input.can_dlc);
    wire::Light command;
    if (!wire::unpackLight(frame, command)) continue;
    for (uint8_t i = 0; i < 2; ++i) {
      if (command.channel && command.channel != i + 1) continue;
      lights[i].value = command; lights[i].value.channel = i + 1;
      lights[i].lease.refresh(now); lightsDirty = true;
    }
  }
}
void render(uint32_t now) {
  for (auto& light : lights) {
    if (light.lease.expire(now, lightTimeoutMs)) {
      light.value.brightness = 0; lightsDirty = true;
    }
  }
  if (!lightsDirty || uint32_t(now - lastLed) < 20) return;
  for (uint8_t i = 0; i < 2; ++i) {
    auto& strip = i ? lower : upper;
    const auto& c = lights[i].value;
    strip.fill(strip.Color(uint16_t(c.red) * c.brightness / 255,
        uint16_t(c.green) * c.brightness / 255, uint16_t(c.blue) * c.brightness / 255));
    strip.show();
  }
  lightsDirty = false; lastLed = now;
}
can_protocol::CanFrame nextFrame() {
  if (!(txSlot & 1)) {
    return wire::packTach(sensors.rpm, tachCalibration, sensors.valid & wire::RPM_VALID);
  }
  if (txSlot == 1) return wire::packSensors(sensors);
  if (txSlot == 3) return wire::packButtons(buttonState);
  const auto& light = lights[txSlot == 5 ? 0 : 1];
  auto f = wire::packLight(light.value); f.id = wire::LIGHT_STATE;
  f.dlc = 7; f.data[6] = light.lease.active ? 1 : 0; return f;
}
} // namespace

void setup() {
  Serial.begin(115200);
  // Reuse the dashboard calibration on this board without modifying its settings.
  Preferences preferences;
  if (preferences.begin("ccm_cfg", true)) {
    const uint8_t saved = preferences.getUChar("tach_ppr10", tachPulsesPerRev10);
    if (saved != 0) tachCalibration = saved;
    preferences.end();
  }
  pinMode(steeringOnPin, INPUT); pinMode(steeringLadderPin, INPUT);
  pinMode(vssPin, INPUT_PULLUP); pinMode(tachPin, INPUT);
  analogReadResolution(12); analogSetPinAttenuation(fuelPin, ADC_11db);
  analogSetPinAttenuation(steeringLadderPin, ADC_11db);
  analogSetPinAttenuation(steeringOnPin, ADC_11db);
  attachInterrupt(digitalPinToInterrupt(vssPin), vssIsr, RISING);
  attachInterrupt(digitalPinToInterrupt(tachPin), tachIsr, RISING);
  pinMode(spareLed1Pin, OUTPUT); digitalWrite(spareLed1Pin, LOW);
  pinMode(spareLed2Pin, OUTPUT); digitalWrite(spareLed2Pin, LOW);
  upper.begin(); lower.begin(); upper.clear(); lower.clear(); upper.show(); lower.show();
  lights[0].value.channel = 1; lights[1].value.channel = 2;
  // A firmware that hangs is restarted by the watchdog; if it was on trial, the restart
  // goes back to the previous one.
  enableLoopWDT();
  esp_ota_img_states_t imageState;
  const esp_partition_t* running = esp_ota_get_running_partition();
  updater.setOnTrial(running && esp_ota_get_state_partition(running, &imageState) == ESP_OK &&
                     imageState == ESP_OTA_IMG_PENDING_VERIFY);
  canReady = startCan();
  Serial.printf("[gateway] CAN=%s, VSS=%lu pulses/mile, tach=%.1f pulses/rev\n",
      canReady ? "ready" : "init failed; check wiring and restart",
      static_cast<unsigned long>(vssPulsesPerMile), tachCalibration / 10.0f);
  Serial.printf("[gateway] firmware build %08lX%s\n", static_cast<unsigned long>(CCM_FW_BUILD),
      updater.onTrial() ? " (new: on trial until the dash confirms it)" : "");
}
void loop() {
  const uint32_t now = millis();
  if (restartAt && int32_t(now - restartAt) >= 0) ESP.restart();
  if (updater.active()) {
    // Parked and asked for by the dash: nothing but the transfer, a frame every millisecond.
    if (canReady) receive(now);
    if (updater.expired(now)) Serial.println("[gateway] firmware update abandoned: the dash went quiet");
    return;
  }
  if (updater.onTrial() && now >= trialMs) {
    Serial.println("[gateway] new firmware not confirmed by the dash; going back to the previous one");
    esp_ota_mark_app_invalid_rollback_and_reboot();
    updater.setOnTrial(false);  // Only reached if there is nothing to go back to.
  }
  readSensors(now); readButtons(now);
  if (canReady) {
    receive(now);
    if (uint32_t(now - lastTx) >= 10) {
      lastTx = now;
      // Send debounced button changes at the next available slot, plus refreshes.
      if (buttonsDirty) {
        if (sendFrame(wire::packButtons(buttonState), now)) buttonsDirty = false;
      } else if (sendFrame(nextFrame(), now)) {
        txSlot = (txSlot + 1) & 7;
      }
    }
  }
  render(now);
  delay(1);
}
