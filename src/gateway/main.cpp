#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>
#include <Adafruit_NeoPixel.h>
#include "pin_map.h"
#include "config.h"
#include "logic.h"
#include <can_contract/gateway_protocol.h>

// NeoPixel 1.12.3 uses a stack-based RMT symbol array on Arduino 3.x.
// Only one strip is transmitted at a time; retain 16 KiB for the rest of loop().
SET_LOOP_TASK_STACK_SIZE(16384 + gateway_config::ledCount * 24 * sizeof(rmt_data_t));

namespace {
using namespace gateway_config;
namespace wire = can_protocol::gateway;
constexpr bool validPins() {
  const int used[] = {vssPin, tachPin, fuelPin, buttonPins[0], buttonPins[1],
      buttonPins[2], buttonPins[3], buttonPins[4], upperLedPin, lowerLedPin,
      CCM_PIN_SPI_SCK, CCM_PIN_SPI_MOSI, CCM_PIN_SPI_MISO,
      CCM_PIN_CAN_SPI_CS, CCM_PIN_CAN_SPI_INT, CCM_PIN_CAN_SPI_RST};
  for (unsigned i = 0; i < sizeof(used) / sizeof(used[0]); ++i) {
    if (used[i] < 0 || used[i] > 48 || used[i] == 19 || used[i] == 20 ||
        (used[i] >= 22 && used[i] <= 37)) return false;
    for (unsigned j = 0; j < i; ++j) if (used[i] == used[j]) return false;
  }
  return fuelPin >= 1 && fuelPin <= 10;
}
static_assert(validPins(), "Gateway pins must be unique, USB/flash/PSRAM-safe; fuel needs ADC1");
MCP2515 can(CCM_PIN_CAN_SPI_CS);
Adafruit_NeoPixel upper(ledCount, upperLedPin, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel lower(ledCount, lowerLedPin, NEO_GRB + NEO_KHZ800);
gateway::DebouncedButton buttons[5];
wire::Buttons buttonState{0, 31, 0};
wire::Sensors sensors;
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

void ARDUINO_ISR_ATTR capture(volatile gateway::Pulse& p, uint32_t timeout) {
  const uint32_t now = micros();
  portENTER_CRITICAL_ISR(&pulseLock);
  p.edge(now, minPulseUs, timeout);
  portEXIT_CRITICAL_ISR(&pulseLock);
}
void ARDUINO_ISR_ATTR vssIsr() { capture(vss, vssTimeoutUs); }
void ARDUINO_ISR_ATTR tachIsr() { capture(tach, tachTimeoutUs); }

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
  for (uint8_t i = 0; i < 6; ++i)
    if (can.setFilter(static_cast<MCP2515::RXF>(i), false, wire::LIGHT_COMMAND) != MCP2515::ERROR_OK)
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
    if (ctrl & 0x70) { failTx(now); return false; }
  }
  if (can.getErrorFlags() & 0x20) { failTx(now); return false; }
  can_frame frame{}; frame.can_id = source.id; frame.can_dlc = source.dlc;
  memcpy(frame.data, source.data, source.dlc);
  if (can.sendMessage(MCP2515::TXB0, &frame) != MCP2515::ERROR_OK) {
    failTx(now); return false;
  }
  txPending = true; txStart = now; return true;
}
void readSensors(uint32_t now) {
  uint32_t vp, tp;
  portENTER_CRITICAL(&pulseLock);
  const uint32_t us = micros();
  vp = vss.periodAt(us, vssTimeoutUs); tp = tach.periodAt(us, tachTimeoutUs);
  portEXIT_CRITICAL(&pulseLock);
  const bool vr = vp != 0, tr = tp != 0;
  sensors.speedKph10 = vr ? gateway::speedKph10(vp, vssPulsesPerMile) : 0;
  sensors.rpm = tr ? gateway::rpm(tp, tachPulsesPerRev10) : 0;
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
  uint8_t mask = 0;
  for (uint8_t i = 0; i < 5; ++i) {
    buttons[i].update(digitalRead(buttonPins[i]) == LOW, now);
    if (buttons[i].stable) mask |= 1U << i;
  }
  if (mask != buttonState.pressed) {
    buttonState.pressed = mask; ++buttonState.sequence; buttonsDirty = true;
  }
}
void receive(uint32_t now) {
  if (can.getErrorFlags() & 0xC0) can.clearRXnOVRFlags();
  can_frame input{};
  for (uint8_t n = 0; n < 4 && can.readMessage(&input) == MCP2515::ERROR_OK; ++n) {
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
    return wire::packTach(sensors.rpm, tachPulsesPerRev10, sensors.valid & wire::RPM_VALID);
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
  for (int pin : buttonPins) pinMode(pin, INPUT_PULLUP);
  pinMode(vssPin, INPUT_PULLUP); pinMode(tachPin, INPUT_PULLUP);
  analogReadResolution(12); analogSetPinAttenuation(fuelPin, ADC_11db);
  attachInterrupt(digitalPinToInterrupt(vssPin), vssIsr, RISING);
  attachInterrupt(digitalPinToInterrupt(tachPin), tachIsr, RISING);
  upper.begin(); lower.begin(); upper.clear(); lower.clear(); upper.show(); lower.show();
  lights[0].value.channel = 1; lights[1].value.channel = 2;
  canReady = startCan();
  Serial.printf("[gateway] CAN=%s, VSS=%lu pulses/mile, tach=%.1f pulses/rev\n",
      canReady ? "ready" : "init failed; check wiring and restart",
      static_cast<unsigned long>(vssPulsesPerMile), tachPulsesPerRev10 / 10.0f);
}
void loop() {
  const uint32_t now = millis();
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
