#pragma once
#include <cstdint>

// Gateway-only wiring; freed display/touch/SD pins are available to switches.
// Inputs require protected 0..3.3 V signals. VR/ignition signals are not GPIO-safe.
namespace gateway_config {
constexpr int vssPin = 15;
constexpr int tachPin = 6;
// 3.3 V -> 100 ohm -> sense -> sender -> GND; sense -> 4.7 kohm -> ADC.
// ADC -> 100 nF -> GND. See docs/SENSOR_GATEWAY.md.
constexpr int fuelPin = 7; // ADC1
constexpr int steeringOnPin = 5; // Yellow wire; active HIGH, external 10k pulldown
constexpr int steeringLadderPin = 4; // Blue wire; ADC1, no internal pulls
constexpr uint32_t steeringToleranceMv = 200; // Acceptance window around each button level
constexpr int upperLedPin = 14;
constexpr int lowerLedPin = 13;
constexpr int spareLed1Pin = 12; // Reserved LED channel; held LOW until assigned
constexpr int spareLed2Pin = 11; // Reserved LED channel; held LOW until assigned
constexpr unsigned ledCount = 180; // per interior output
constexpr uint32_t vssPulsesPerMile = 8000;
constexpr uint16_t tachPulsesPerRev10 = 20; // Existing CCM default; saved tach_ppr10 takes precedence
constexpr float fuelSupplyMv = 3300.0f;
constexpr float fuelPullupOhms = 100.0f;
constexpr float fuelEmptyOhms = 158.0f; // User-specified sender endpoints
constexpr float fuelFullOhms = 16.0f;
constexpr uint32_t lightTimeoutMs = 5000; // refresh commands at least once/second
constexpr uint32_t tachTimeoutUs = 250000;
constexpr uint32_t vssTimeoutUs = 1500000;
constexpr uint32_t vssMinPulseUs = 500;
constexpr uint32_t tachMinPulseUs = 1000; // Same filtering as dashboard tach input
static_assert(vssPulsesPerMile > 0 && tachPulsesPerRev10 > 0 && tachPulsesPerRev10 <= 255,
              "Pulse calibration must be positive and fit the tach CAN field");
static_assert(fuelEmptyOhms != fuelFullOhms && fuelSupplyMv > 0 && fuelPullupOhms > 0,
              "Invalid fuel calibration");
} // namespace gateway_config
