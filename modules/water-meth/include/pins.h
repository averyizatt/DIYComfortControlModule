#pragma once

namespace pins {
// Core controls
constexpr int MAP_SENSOR_ADC = 1;       // GPIO1
constexpr int FLOAT_SENSOR_DIGITAL = 2; // GPIO2
constexpr int PUMP_PWM = 3;             // GPIO3
constexpr int WARNING_LED = 4;          // GPIO4

// TWAI CAN transceiver pins
constexpr int CAN_TX = 5; // GPIO5
constexpr int CAN_RX = 6; // GPIO6

// Thermistors (default disabled unless wired)
constexpr int IAT_THERM_PIN = 7;        // GPIO7
constexpr int ENGINE_BAY_THERM_PIN = 8; // GPIO8
constexpr int CABIN_THERM_PIN = 9;      // GPIO9
constexpr int AMBIENT_THERM_PIN = 10;   // GPIO10

// Pressure transducers (divider-scaled to ADC)
constexpr int OIL_PRESSURE_ADC = 11;       // GPIO11
constexpr int FUEL_PRESSURE_ADC = 12;      // GPIO12
constexpr int METH_PRESSURE_ADC = 13;      // GPIO13
constexpr int BOOST_REF_PRESSURE_ADC = 14; // GPIO14
constexpr int SPARE_PRESSURE_1_ADC = 15;   // GPIO15
constexpr int SPARE_PRESSURE_2_ADC = 16;   // GPIO16
} // namespace pins

