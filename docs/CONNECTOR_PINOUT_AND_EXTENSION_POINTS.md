# Connector/Pinout and Extension Points

## Core Assignments (centralized in `src/pin_map.h`)
- Shared SPI bus (LCD + SD + MCP2515): MOSI GPIO11, MISO GPIO13, SCK GPIO12
- LCD ST7796S: CS GPIO10, RST GPIO9, DC GPIO8, BL/PWM GPIO7
- SD: CS GPIO16
- Touch: SCL GPIO47, SDA GPIO48, RST GPIO14, INT GPIO15
- CAN TWAI path: TX GPIO5, RX GPIO4
- MCP2515 CAN module path: CS GPIO17, INT GPIO18, RST GPIO21 (optional)
- GPS (NEO-6M): RX GPIO42, TX GPIO41
- Tach output (LEDC): GPIO6
- Tach input capture: GPIO2
- Gyro/IMU (I2C shared): SCL GPIO47, SDA GPIO48, INT GPIO3
- LED channel data pins: GPIO38 / GPIO39 / GPIO40
- Aux output expansion: GPIO33 / GPIO34

## Power and Grounding Notes
- Keep logic ground return isolated from high-current returns until star-ground join.
- Route CAN transceiver and ESP logic from clean power branch.
- Keep flyback-protected inductive loads on dirty power branch.

## Reserved Expansion Interfaces
- Header A: analog/digital sensor expansion (future humidity, pressure, etc.)
- Header B: high-side/low-side output controls (fans, relays, lighting)
- Header C: service/debug UART + future BLE/WiFi diagnostics bridge

## Software Extension Points
- `include/can/CanProtocol.hpp`: add IDs, owners, and command/status payload contracts.
- `include/hal/*`: implement board-specific adapters without changing service logic.
- `src/core/Application.cpp`: add new FreeRTOS tasks and queue contracts.
- `include/ui/UiManager.hpp`: extend page actions and menu routes.
