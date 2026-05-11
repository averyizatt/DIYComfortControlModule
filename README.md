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

## Build
```bash
pio run -e nano_esp32_debug
pio run -e nano_esp32_release
```

## Next Steps
Implement concrete hardware drivers for the ST7796S + capacitive touch stack, TWAI backend, and production sensor/tach input circuits.
