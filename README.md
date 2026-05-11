# DIYComfortControlModule

Firmware skeleton for a Foxbody Mustang ESP32-S3 master comfort/control module.

## Project Status
This repository now includes:
- PlatformIO ESP32-S3 Arduino foundation (`platformio.ini`) with debug/release profiles
- Modular architecture folders for core/ui/can/gps/tach/sensors/hal/safety/config
- Queue-based non-blocking FreeRTOS task architecture scaffold
- CAN protocol schema with water/meth and taillight command channels
- Tach output control scaffold with configurable scaling modes
- GPS and environmental monitoring service scaffolds
- Diagnostics + safety fault evaluation hooks
- Bench validation and connector/extension-point documentation in `docs/`
- Shared CAN protocol + manager layer for Foxbody distributed modules:
  - `src/can/can_protocol.h`
  - `src/can/can_manager.h/.cpp`
  - `src/state/vehicle_state.h/.cpp`

## Shared CAN Protocol Highlights
- Preserves taillight compatibility IDs:
  - `0x100` taillight state broadcast
  - `0x101` taillight command
  - `0x102` taillight fault broadcast
- Adds cabin master frames:
  - `0x200` heartbeat
  - `0x201` master command
  - `0x202` tach state
  - `0x203` GPS state
- Adds engine/water meth frames:
  - `0x300` meth state
  - `0x301` meth command
  - `0x302` meth fault
  - `0x303` extended sensors
- Includes pack/unpack helpers, DLC checks, and timeout-based node online/offline logic.
- Includes `DEMO_MODE` CAN simulation fallback for bench UI testing when hardware CAN is unavailable.

## Build
```bash
pio run -e nano_esp32_debug
pio run -e nano_esp32_release
```

## Next Steps
Implement concrete hardware drivers for the ST7796S + capacitive touch stack, TWAI backend, and production sensor/tach input circuits.
