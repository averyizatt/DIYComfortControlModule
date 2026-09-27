#pragma once
#include <cstdint>

// Gateway-only wiring; freed display/touch/SD pins are available to switches.
// Inputs require protected 0..3.3 V signals. VR/ignition signals are not GPIO-safe.
namespace gateway_config {
constexpr int vssPin = 4;
constexpr int tachPin = 2;
constexpr int fuelPin = 1; // ADC1; 3.3 V -> 100 ohm -> ADC -> sender -> GND
constexpr int buttonPins[5] = {5, 6, 7, 9, 10}; // active LOW, internal pull-ups
constexpr int upperLedPin = 38;
constexpr int lowerLedPin = 39;
constexpr unsigned ledCount = 180; // per interior output
constexpr uint32_t vssPulsesPerMile = 8000;
constexpr uint16_t tachPulsesPerRev10 = 40; // V8 ignition tach: 4 pulses/revolution
constexpr float fuelSupplyMv = 3300.0f;
constexpr float fuelPullupOhms = 100.0f;
constexpr float fuelEmptyOhms = 16.0f; // 1987+ Mustang; confirm installed sender
constexpr float fuelFullOhms = 158.0f;
constexpr uint32_t lightTimeoutMs = 5000; // refresh commands at least once/second
constexpr uint32_t tachTimeoutUs = 500000;
constexpr uint32_t vssTimeoutUs = 1500000;
constexpr uint32_t minPulseUs = 500; // reject glitches above 2 kHz
static_assert(vssPulsesPerMile > 0 && tachPulsesPerRev10 > 0 && tachPulsesPerRev10 <= 255,
              "Pulse calibration must be positive and fit the tach CAN field");
static_assert(fuelEmptyOhms != fuelFullOhms && fuelSupplyMv > 0 && fuelPullupOhms > 0,
              "Invalid fuel calibration");
} // namespace gateway_config
