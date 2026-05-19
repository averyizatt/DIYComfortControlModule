# DIYComfortControlModule

Central comfort and control firmware for a Foxbody Mustang electronics retrofit built around an **ESP32-S3 DevKitC-1**. The Comfort Control Module (CCM) is the master node on the vehicle CAN network and ties together the touchscreen UI, GPS, tach output, environmental sensing, water/methanol controls, and taillight coordination.

## Table of Contents

- [Overview](#overview)
- [Project Status](#project-status)
- [What the CCM Does](#what-the-ccm-does)
- [Hardware Platform](#hardware-platform)
- [Knock Sense](#knock-sense)
- [Build Variants](#build-variants)
- [Getting Started](#getting-started)
- [Validation](#validation)
- [Pin Configuration](#pin-configuration)
- [Software Architecture](#software-architecture)
- [CAN Integration](#can-integration)
- [Repository Layout](#repository-layout)
- [Submodules](#submodules)
- [Roadmap](#roadmap)

## Overview

This repository contains the firmware for the **master comfort module** in a distributed, CAN-connected vehicle electronics system. The CCM is responsible for:

- rendering the in-car touchscreen UI with LVGL
- publishing master state and receiving commands over CAN
- collecting local inputs such as GPS, tach, buttons, and sensor data
- supervising fault handling and degraded-mode behavior
- coordinating external modules for taillights and water/methanol injection

The project is structured as a PlatformIO-based Arduino firmware application with FreeRTOS-driven tasks and a shared CAN contract that is also consumed by the linked module repositories.

## Project Status

The codebase is actively being migrated toward a production-ready ESP32-S3 DevKit-based controller. The repository already includes:

- the current PlatformIO configuration for the ESP32-S3 DevKitC-1
- a centralized board pin map in [`src/pin_map.h`](src/pin_map.h)
- a shared CAN contract integration check in [`scripts/check_can_contract.py`](scripts/check_can_contract.py)
- linked submodules for external vehicle modules

Some subsystems are still in-progress or stubbed for bench bring-up, so the README focuses on the **current architecture and source-of-truth configuration**, not older bench assumptions.

## What the CCM Does

### User-facing functions

- **Touchscreen dashboard** with multiple pages for vehicle data, diagnostics, and settings
- **GPS-backed data display** for speed and location-derived information
- **Tach output generation** for an analog-style cluster tachometer
- **Environmental monitoring** for cabin, engine bay, outside air, and intake-related values
- **Water/methanol control** through a dedicated slave module
- **Taillight mode control** through a dedicated slave module

### System-level responsibilities

- **CAN master heartbeat** and node health supervision
- **Fault propagation** across the distributed module network
- **Undervoltage handling** and degraded-mode transitions
- **Demo-mode simulation** for UI and integration work without a live CAN network

## Hardware Platform

| Item | Current target |
|---|---|
| MCU board | Espressif ESP32-S3 DevKitC-1 |
| Framework | Arduino via PlatformIO |
| Display target | ST7796S SPI display |
| Touch target | Capacitive touch controller on I2C |
| CAN paths | Native ESP32-S3 TWAI pins plus MCP2515 support pins |
| GPS | UART NMEA receiver at 9600 baud |
| Vehicle power | 12 V automotive supply with local regulated logic rail |

For connector-level notes and reserved expansion points, see [`docs/CONNECTOR_PINOUT_AND_EXTENSION_POINTS.md`](docs/CONNECTOR_PINOUT_AND_EXTENSION_POINTS.md).

## Knock Sense

The CCM includes a supplemental engine knock monitoring subsystem that is **not** an ECU knock control system and must not be relied upon as the sole means of knock protection. It is designed for warning, logging, and water-meth safety coordination only.

### Purpose

Provide real-time knock-energy monitoring via a piezo sensor front-end to:

- Detect abnormal combustion vibration signatures
- Trigger operator warnings and log events for post-session review
- Optionally coordinate with the water-meth injection module for supplemental octane/cooling response

### Circuit summary

```
Bosch piezo knock sensor (block or intake manifold mount)
  → shielded cable
  → AC coupling capacitor
  → 1.65 V bias node (voltage divider from 3.3 V)
  → op-amp gain stage (~10–11×, optional bandpass ~5–15 kHz)
  → ADC protection resistor + optional clamp diodes to 3.3 V / GND
  → CCM knock ADC input (GPIO 48 default, see src/pin_map.h)
```

### How the firmware processes knock data

1. **Sampling**: 10 ADC samples per tick, midpoint-referenced (2048 = quiescent), absolute-valued and averaged into an energy envelope using exponential smoothing.
2. **Adaptive baseline**: A slow exponential moving average tracks background noise during idle/non-event windows. Learning is gated by boost and RPM thresholds so highway noise does not corrupt the baseline.
3. **Dynamic threshold**: `threshold = baseline × multiplier + offset`. The multiplier and offset are user-configurable. Default: 2.5× + 8 units.
4. **Event detection**: When energy exceeds threshold during the enable window (boost ≥ enable kPa and RPM ≥ enable min), a knock event is registered after a cooldown gate.
5. **Warning/critical escalation**: A sliding event counter triggers warning then critical states. Both are cleared by the user or by the clear-events command.
6. **CAN publishing**: Knock state is broadcast on `0x307` at 50 ms intervals. Fault frames are sent on `0x308` when events or sensor faults occur.
7. **SD logging**: Events and fault frames are logged to `/logs/knock/` on the SD card when mounted.

### Response modes

| Value | Name | Behavior |
|---|---|---|
| 0 | LOG_ONLY | Log to SD; no output or alert |
| 1 | WARN_ONLY | Default — sets warning/critical state flags only |
| 2 | FORCE_METH_ENABLE | If critical and meth module is armed, keeps meth enabled |
| 3 | SAFETY_SHUTDOWN | If critical, disarms meth module each event; does not prevent re-arming |

> **Warning:** Modes 2 and 3 interact with the active water-meth system. Changing these modes should be done intentionally and is confirmed by the web UI.

### Touchscreen Knock Sense page

The dedicated **KNOCK** tab (accessible from the 8-tab bottom navigation) shows:

- Sensor health (OK, disconnected, clipping, noisy, baseline learning)
- Live knock energy bar (color: blue → orange → red as energy approaches threshold)
- Baseline bar (green)
- Threshold bar (orange reference line at 100%)
- Event counter with warning/critical state indicators
- Last knock event: RPM, boost kPa, IAT °C, elapsed time
- Current response mode
- SD logging status
- Controls: Enable/Disable, Reset Baseline, Clear Events, Simulate (demo mode only)

### Web dashboard Knock Sense page

The dedicated `/knock` web page provides:

- Live metric cards with progress bars for energy/baseline/threshold
- 30-second rolling history chart (energy, baseline, threshold)
- Controls: Enable/Disable, Reset Baseline, Clear Events, Simulate
- Settings panel: threshold multiplier, boost enable kPa, RPM enable, response mode
- Safety confirmation prompt for response modes that affect active systems

### API reference

| Endpoint | Method | Purpose |
|---|---|---|
| `GET /api/knock/state` | GET | Full knock state with all runtime fields |
| `POST /api/knock` | POST | Send commands or update settings |
| `GET /api/state` | GET | Includes knock summary fields |
| `GET /api/settings` | GET | Includes all knock settings |
| `POST /api/settings` | POST | Update knock settings |

`POST /api/knock` accepts an `action` key:

```json
{ "action": "enable" }
{ "action": "disable" }
{ "action": "reset_baseline" }
{ "action": "clear_events" }
{ "action": "simulate_event" }
```

Simulate is only effective in `DEMO_MODE` builds or when `knock_demo_mode_enabled` is set.

### Limitations and false positive/negative notes

- **Not ECU knock control.** The CCM has no authority over ignition timing. Knock events from this system must not be interpreted as a substitute for ECU-level protection.
- **False positives** can occur from: mechanical noise (valve clatter, injector tick, road vibration), gain too high, threshold multiplier too low, sensor mounting resonance, or a clipped/noisy ADC signal.
- **False negatives** can occur from: gain too low, threshold too high, boost/RPM gates set too high, sensor disconnected, or baseline contaminated by actual knock during learning.
- The adaptive baseline **learns knock as noise** if knock events are sustained during the baseline learning window. Reset the baseline after corrective action.
- ADC resolution (12-bit, 3.3 V rail) is the practical limit of sensitivity. Proper analog front-end design (bandpass amplifier, gain stage, bias) is essential for meaningful signal quality.
- Treat this system as a **supplemental data source** for margin assessment, not as a safety guarantee.

## Build Variants

The active PlatformIO environments are defined in [`platformio.ini`](platformio.ini):

| Environment | Purpose |
|---|---|
| `esp32s3_devkit_debug` | Debug-oriented build with `CCM_BUILD_DEBUG=1` |
| `esp32s3_devkit_release` | Default production-oriented build |
| `esp32s3_devkit_demo` | Release-style build with `DEMO_MODE=1` for simulated CAN data |

The default environment is `esp32s3_devkit_release`.

## Getting Started

### Prerequisites

- [PlatformIO Core](https://docs.platformio.org/en/latest/core/installation.html) or PlatformIO for VS Code
- Git with submodule support
- USB connection to the ESP32-S3 DevKitC-1

### Clone the repository

```bash
git clone --recurse-submodules https://github.com/averyizatt/DIYComfortControlModule.git
cd DIYComfortControlModule
```

If the repository was cloned without submodules:

```bash
git submodule update --init --recursive
```

### Build

```bash
# Debug
pio run -e esp32s3_devkit_debug

# Default release
pio run -e esp32s3_devkit_release

# Demo / simulated CAN
pio run -e esp32s3_devkit_demo
```

### Flash

```bash
pio run -e esp32s3_devkit_release --target upload
```

### Serial monitor

```bash
pio device monitor --baud 115200
```

## Validation

The most important repository-level validation for this project is the CAN contract check:

```bash
python scripts/check_can_contract.py
```

This verifies that:

- the shared CAN contract header is present
- the comfort module shim includes the shared contract
- schema version pin files across modules match
- required external module submodules are present and wired to the shared contract

For bench bring-up and manual verification, use [`docs/BENCH_VALIDATION_CHECKLIST.md`](docs/BENCH_VALIDATION_CHECKLIST.md).

## Pin Configuration

### Source of truth

The authoritative runtime pin assignments live in:

- [`src/pin_map.h`](src/pin_map.h)
- [`include/config/SystemConfig.hpp`](include/config/SystemConfig.hpp)

If the README and code ever disagree, **trust the code**.

### Current default pin map summary

| Function | GPIO |
|---|---|
| LCD CS / RST / DC / BL | 10 / 9 / 8 / 7 |
| Shared SPI MOSI / MISO / SCK | 11 / 13 / 12 |
| Touch SCL / SDA / RST / INT | 47 / 48 / 14 / 15 |
| CAN TX / RX | 5 / 4 |
| MCP2515 CS / INT / RST | 17 / 18 / 21 |
| GPS RX / TX | 41 / 42 |
| Tach out / in | 6 / 2 |
| Buttons up / down / select | 35 / 36 / 37 |
| Aux outputs | 33 / 34 |
| LED data outputs | 38 / 39 / 40 |
| Battery sense | 46 |

Most pin assignments can be overridden at build time with `CCM_PIN_*` defines. See the comments at the top of [`src/pin_map.h`](src/pin_map.h) for the override pattern.

## Software Architecture

The firmware is organized around FreeRTOS tasks and modular service layers.

### Major runtime concerns

- **CAN manager** handles frame transport, message packing/unpacking, and heartbeat traffic
- **Vehicle state** provides a thread-safe state container
- **UI manager** renders LVGL pages and dashboard updates
- **Safety manager** evaluates faults and degraded behavior
- **HAL interfaces** isolate board-specific details from higher-level logic

### Important files

| Path | Purpose |
|---|---|
| `src/main.cpp` | Entry point |
| `src/core/` | Application orchestration |
| `src/can/` | CAN manager and compatibility shim |
| `src/state/` | Shared vehicle state |
| `src/ui/` | UI implementation |
| `src/gps/` | GPS integration |
| `src/sensors/` | Sensor integration |
| `src/tach/` | Tach controller logic |
| `include/config/SystemConfig.hpp` | Compile-time system constants |
| `include/hal/` | Hardware abstraction interfaces |
| `include/can/CanProtocol.hpp` | Modern CAN types and IDs |

## CAN Integration

The CCM uses a shared CAN contract so that this repository and the linked external modules stay compatible.

Key facts:

- **Bus speed:** 500 kbps
- **Frame type:** standard 11-bit IDs
- **Schema compatibility:** pinned through `CAN_PROTOCOL_SCHEMA_VERSION`
- **Shared header:** `shared/can_contract/include/can_contract/can_protocol.h`

Compatibility-sensitive pieces in this repo:

- [`include/can/CanProtocol.hpp`](include/can/CanProtocol.hpp)
- [`src/can/can_protocol.h`](src/can/can_protocol.h)
- [`scripts/check_can_contract.py`](scripts/check_can_contract.py)

## Repository Layout

```text
DIYComfortControlModule/
├── docs/               # Bench validation, connector notes, extension guidance
├── include/            # Public headers and subsystem interfaces
├── modules/            # Linked module repositories
├── scripts/            # Validation and helper scripts
├── shared/             # Shared CAN contract
├── src/                # Firmware implementation
├── test/               # Unit/integration test area (when present)
└── platformio.ini      # Build environments and compile-time flags
```

## Submodules

| Module | Path | Repository |
|---|---|---|
| Custom taillight controller | `modules/taillights` | [averyizatt/CustomTaillights](https://github.com/averyizatt/CustomTaillights) |
| Water/methanol injection controller | `modules/water-meth` | [averyizatt/DIYWaterMethInjection](https://github.com/averyizatt/DIYWaterMethInjection) |

These modules are expected to be present when validating shared CAN compatibility.

## Roadmap

<<<<<<< HEAD
- [x] Implement ST7796S SPI display driver and LVGL display flush (Arduino GFX + ST7796, 5-tab dashboard)
- [x] Implement capacitive touch HAL (`src/touch/touch_manager`)
- [x] Implement production TWAI CAN backend (`src/can/can_manager`)
- [x] Implement production sensor and tach input circuits (`src/sensors/`, `src/tach/`)
- [x] Wire up full Application task graph in `src/core/Application.cpp`
- [x] Standalone water/meth architecture — CCM sends arm + ratio, reads back MAP/IAT/bay temp
- [ ] Add OTA update support via WiFi
- [ ] BLE/WiFi diagnostics bridge on expansion Header C
=======
- [ ] Complete production display driver and LVGL flush path
- [ ] Complete capacitive touch hardware integration
- [ ] Replace stubbed/demo transport paths with production TWAI behavior where needed
- [ ] Expand diagnostics and service tooling
- [ ] Add OTA or wireless diagnostics support
- [ ] Continue tightening integration with taillight and water/meth modules
>>>>>>> 49d0f60c0b0b3d50d39ff953deba8cda37416dba
