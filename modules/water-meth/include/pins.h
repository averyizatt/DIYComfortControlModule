#pragma once

namespace pins {
// Core controls (preserve established hardware defaults)
constexpr int MAP_SENSOR_ADC = 1;       // GPIO1 / A0
constexpr int FLOAT_SENSOR_DIGITAL = 3; // GPIO3 / D3
constexpr int PUMP_PWM = 2;             // GPIO2 / D2
constexpr int WARNING_LED = 17;         // GPIO17
constexpr int KNOCK_SENSOR_ADC = 2;     // GPIO2 / A1

// MCP2515 SPI CAN module control pins (when using MCP2515 profile)
constexpr int CAN_SPI_CS = 10;  // GPIO10 / D10
constexpr int CAN_SPI_INT = 7;  // GPIO7 / D7

// TWAI CAN transceiver pins
constexpr int CAN_TX = 5; // GPIO5
constexpr int CAN_RX = 6; // GPIO6

// Thermistors (default disabled unless wired)
constexpr int IAT_THERM_PIN = 4;        // GPIO4 / D4
constexpr int DHT11_PIN = 6;            // GPIO6 / D6
constexpr int ENGINE_BAY_THERM_PIN = 8; // GPIO8
constexpr int CABIN_THERM_PIN = 9;      // GPIO9
constexpr int AMBIENT_THERM_PIN = 10;   // GPIO10

// Pressure transducers (divider-scaled to ADC)
constexpr int OIL_PRESSURE_ADC = 11;       // GPIO11 / A4
constexpr int FUEL_PRESSURE_ADC = 12;      // GPIO12 / A5
constexpr int METH_PRESSURE_ADC = 13;      // GPIO13
constexpr int BOOST_REF_PRESSURE_ADC = 14; // GPIO14
constexpr int SPARE_PRESSURE_1_ADC = 15;   // GPIO15
constexpr int SPARE_PRESSURE_2_ADC = 16;   // GPIO16
} // namespace pins
