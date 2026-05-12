# Connector/Pinout and Extension Points

## Core Assignments (initial)
- CAN RX: GPIO4
- CAN TX: GPIO5
- GPS RX: GPIO18
- GPS TX: GPIO17
- Touch controller: I2C + IRQ/RESET (see main firmware touch pin definitions)
- Tach output: GPIO6 (LEDC)

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
