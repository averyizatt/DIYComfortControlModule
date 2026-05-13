# Comfort Module

This repository root is the current comfort module PlatformIO project.

- Build debug: `pio run -e esp32s3_devkit_debug`
- Build release: `pio run -e esp32s3_devkit_release`
- Flash release: `pio run -e esp32s3_devkit_release -t upload`

Compatibility pin: `CAN_PROTOCOL_SCHEMA_VERSION` must match the shared CAN contract.
