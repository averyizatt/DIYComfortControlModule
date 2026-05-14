# DIYComfortControlModule

Firmware for a Foxbody Mustang ESP32-S3 master comfort and control module. This module acts as the central CAN bus master on a distributed multi-module network, coordinating a dash-mounted touchscreen display, GPS, environmental sensors, tach output, water/methanol injection control, and custom taillight sequencing.

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Hardware Requirements](#hardware-requirements)
- [Pin Assignments](#pin-assignments)
- [Architecture](#architecture)
- [CAN Protocol Reference](#can-protocol-reference)
- [Getting Started](#getting-started)
- [Repository Structure](#repository-structure)
- [Configuration](#configuration)
- [Demo Mode](#demo-mode)
- [Submodules](#submodules)
- [Bench Validation](#bench-validation)
- [Roadmap](#roadmap)

---

## Overview

The DIYComfortControlModule (CCM) is the brain of a modular Foxbody Mustang electronics upgrade. It runs on an Espressif ESP32-S3 DevKitC-1 and communicates with peripheral slave modules over a 500 kbps CAN bus. A touchscreen LVGL UI surfaces real-time data across multiple pages (dashboard, environment, water/meth, lighting, diagnostics, and settings). FreeRTOS tasks keep each subsystem independent and non-blocking.

This repository contains the master module firmware. The taillight controller and water/meth injection controller are maintained as linked submodules.

---

## Features

- **CAN bus master** — 500 kbps, 11-bit standard IDs, heartbeat/timeout node management
- **Touchscreen UI** — LVGL 8.3.11 dashboard with ~30 FPS refresh and multiple pages
- **GPS integration** — Speed and position display via TinyGPSPlus
- **Environmental sensors** — Cabin, engine bay, outside, and intake air temperature monitoring
- **Tach output** — LEDC PWM output to drive an analog cluster tachometer; configurable RPM/15 and RPM/30 scaling modes, plus startup sweep
- **Water/methanol injection control** — Arm/disarm, boost and IAT thresholds, pump duty, fault reporting (via slave module)
- **Custom taillight sequencing** — Sequential, show, and demo animation modes (via slave module)
- **Safety/fault subsystem** — Undervoltage detection, fault flag propagation, and degraded-mode fallback
- **Demo mode** — Simulated CAN data for bench UI development without live hardware

---

## Hardware Requirements

| Component | Details |
|---|---|
| Microcontroller | Espressif ESP32-S3 DevKitC-1 |
| Display | ST7796S (SPI, target) |
| Touch | Capacitive touch panel |
| CAN transceiver | Any 3.3 V-compatible SN65HVD23x or similar |
| GPS module | UART NMEA GPS at 9600 baud |
| Power | 12 V vehicle supply; module logic fed from clean branch |

---

## Pin Assignments

| Signal | GPIO |
|---|---|
| CAN TX | 5 |
| CAN RX | 4 |
| GPS RX (UART1 in) | 18 |
| GPS TX (UART1 out) | 17 |
| Button — Up | 8 |
| Button — Down | 9 |
| Button — Select | 10 |
| Tach Output (LEDC ch 0) | 6 |

See [`docs/CONNECTOR_PINOUT_AND_EXTENSION_POINTS.md`](docs/CONNECTOR_PINOUT_AND_EXTENSION_POINTS.md) for power/grounding notes and reserved expansion headers.

---

## Architecture

The firmware is structured around a set of FreeRTOS tasks pinned to the two ESP32-S3 cores. Tasks communicate exclusively through FreeRTOS queues — there is no shared mutable state outside of the protected `VehicleState` object.

| Task | Core | Purpose |
|---|---|---|
| `can_task` | 0 | TWAI TX/RX, pack/unpack CAN frames |
| `sensor_task` | 1 | Poll environmental sensors |
| `gps_task` | 1 | Parse NMEA sentences |
| `tach_task` | 1 | Update LEDC frequency from RPM |
| `ui_task` | 1 | LVGL render loop |
| `diagnostics_task` | 0 | Safety evaluation, fault flag management |
| `hb_task` | 1 | Master state and input flag heartbeat |

**Key source files:**

| Path | Role |
|---|---|
| `src/main.cpp` | Entry point; initialises tasks |
| `include/core/Application.hpp` | Top-level application class |
| `include/config/SystemConfig.hpp` | Compile-time pin and timing constants |
| `src/can/can_protocol.h` | Legacy CAN ID, struct, and pack/unpack definitions |
| `include/can/CanProtocol.hpp` | Modern namespaced CAN ID and type definitions |
| `src/can/can_manager.h/.cpp` | CAN manager layer |
| `src/state/vehicle_state.h/.cpp` | Thread-safe shared vehicle state |
| `include/hal/` | Hardware abstraction layer interfaces |
| `include/safety/SafetyManager.hpp` | Fault evaluation hooks |

---

## CAN Protocol Reference

All frames use standard 11-bit IDs at **500 kbps**. IDs are divided into reserved blocks:

| Block base | Owner |
|---|---|
| `0x100` | Taillight controller (compatibility — do not change) |
| `0x200` | CCM master (cabin) |
| `0x300` | Engine/water-meth controller |
| `0x400` | GPS |
| `0x500` | Comfort expansion |
| `0x600` | Future use |

### Taillight block (0x100–0x102)

| ID | Direction | DLC | Period | Description |
|---|---|---|---|---|
| `0x100` | Slave → Master | 7 | 100 ms | Taillight state (left/right state, input flags, brightness, die temp, thermal derate, status) |
| `0x101` | Master → Slave | varies | on demand | Taillight command (set brightness, set/clear override, trigger animation) |
| `0x102` | Slave → Master | 4 | on fault | Taillight fault (code, severity, data) |

### Master block (0x200–0x203)

| ID | Direction | DLC | Period | Description |
|---|---|---|---|---|
| `0x200` | Master → All | 8 | 100 ms | Master heartbeat (master state, UI page, fault flags) |
| `0x201` | Any → Master | varies | on demand | Master command (set UI page, set brightness, trigger tach sweep, set drive mode) |
| `0x202` | Master → All | 8 | 20 ms | Tach/RPM state |
| `0x203` | Master → All | 8 | 250 ms | GPS state |

### Engine/meth block (0x300–0x303)

| ID | Direction | DLC | Period | Description |
|---|---|---|---|---|
| `0x300` | Slave → Master | 8 | 50 ms | Meth state (state, pump duty, tank %, flow status, boost kPa, IAT, bay temp, faults) |
| `0x301` | Master → Slave | varies | on demand | Meth command (arm, manual test duty, stop test, set boost/IAT threshold, clear faults) |
| `0x302` | Slave → Master | 4 | on fault | Meth fault (code, severity, data) |
| `0x303` | Slave → Master | 8 | 250 ms | Extended engine sensors |

**Encoding conventions:**
- Temperatures are `+40` offset encoded (e.g. `uint8_t = celsius + 40`), range −40 °C … +215 °C.
- Voltages are `×10` encoded (e.g. `uint8_t = volts × 10`).
- Multi-byte integers are big-endian.
- Nodes that do not transmit their expected heartbeat within `kNodeTimeoutMs` (1500 ms) are flagged offline.

---

## Getting Started

### Prerequisites

- [PlatformIO Core](https://docs.platformio.org/en/latest/core/installation.html) (CLI) or the PlatformIO IDE extension for VS Code
- USB cable to the ESP32-S3 DevKitC-1

### Clone with submodules

```bash
git clone --recurse-submodules https://github.com/averyizatt/DIYComfortControlModule.git
cd DIYComfortControlModule
```

If you already cloned without submodules:

```bash
git submodule update --init --recursive
```

### Build

```bash
# Debug build (CCM_BUILD_DEBUG=1)
pio run -e esp32s3_devkit_debug

# Release build
pio run -e esp32s3_devkit_release
```

### Flash

```bash
pio run -e esp32s3_devkit_release --target upload
```

### Serial monitor

```bash
pio device monitor --baud 115200
```

---

## Repository Structure

```
DIYComfortControlModule/
├── docs/
│   ├── BENCH_VALIDATION_CHECKLIST.md
│   └── CONNECTOR_PINOUT_AND_EXTENSION_POINTS.md
├── include/
│   ├── can/          # Modern CAN protocol types (CanProtocol.hpp)
│   ├── config/       # Compile-time system configuration
│   ├── core/         # Application, shared types, system state, task contracts
│   ├── gps/          # GPS service interface
│   ├── hal/          # Hardware abstraction layer interfaces
│   ├── safety/       # Safety manager interface
│   ├── sensors/      # Environment service interface
│   ├── tach/         # Tach controller interface
│   └── ui/           # UI manager interface
├── modules/
│   ├── tailights/    # Submodule: CustomTaillights firmware
│   └── water-meth/   # Submodule: DIYWaterMethInjection firmware
├── src/
│   ├── can/          # CAN manager + legacy protocol definitions
│   ├── core/         # Application implementation
│   ├── gps/          # GPS service implementation
│   ├── hal/          # Hardware adapter implementations
│   ├── safety/       # Safety manager implementation
│   ├── sensors/      # Environment service implementation
│   ├── state/        # Thread-safe vehicle state
│   ├── tach/         # Tach controller implementation
│   ├── ui/           # UI manager implementation
│   └── main.cpp      # Entry point
└── platformio.ini
```

---

## Configuration

Compile-time constants live in [`include/config/SystemConfig.hpp`](include/config/SystemConfig.hpp). Key values:

| Constant | Default | Description |
|---|---|---|
| `kCanBitrate` | 500 000 | CAN bus speed (bps) |
| `kCanTxPin` / `kCanRxPin` | 5 / 4 | TWAI GPIO pins |
| `kGpsBaud` | 9 600 | GPS UART baud rate |
| `kUndervoltageThreshold` | 11.6 V | Voltage below which degraded mode is entered |
| `kDashboardRefreshMs` | 33 ms | UI task target period (~30 FPS) |
| `kCanHeartbeatMs` | 250 ms | Master heartbeat transmit interval |
| `kNodeTimeoutMs` | 1 500 ms | CAN node offline detection window |

Build flags in `platformio.ini`:

| Flag | Description |
|---|---|
| `CCM_CAN_BITRATE` | CAN bitrate (matches `kCanBitrate`) |
| `CCM_CAN_PROTOCOL_VERSION` | Protocol version embedded in heartbeat |
| `CCM_BUILD_DEBUG` | 1 in debug builds, 0 in release |
| `DEMO_MODE` | 1 to enable simulated CAN data |

---

## Demo Mode

When `DEMO_MODE=1` (available in `esp32s3_devkit_demo`), the CAN manager substitutes simulated frame data instead of requiring live CAN hardware. This allows full UI development and bench testing without a wired CAN network.

To build without demo mode, override the flag in a release environment or set `DEMO_MODE=0` in your `platformio.ini`.

---

## Submodules

| Submodule | Path | Repository |
|---|---|---|
| Water/methanol injection controller | `modules/water-meth` | [averyizatt/DIYWaterMethInjection](https://github.com/averyizatt/DIYWaterMethInjection) |
| Custom taillight controller | `modules/tailights` | [averyizatt/CustomTaillights](https://github.com/averyizatt/CustomTaillights) |

---

## Bench Validation

A structured checklist covering display/UI, CAN, tach, GPS/sensors, and reliability soak testing is provided in [`docs/BENCH_VALIDATION_CHECKLIST.md`](docs/BENCH_VALIDATION_CHECKLIST.md).

---

## Roadmap

- [x] Implement ST7796S SPI display driver and LVGL display flush (Arduino GFX + ST7796, 5-tab dashboard)
- [x] Implement capacitive touch HAL (`src/touch/touch_manager`)
- [x] Implement production TWAI CAN backend (`src/can/can_manager`)
- [x] Implement production sensor and tach input circuits (`src/sensors/`, `src/tach/`)
- [x] Wire up full Application task graph in `src/core/Application.cpp`
- [x] Standalone water/meth architecture — CCM sends arm + ratio, reads back MAP/IAT/bay temp
- [ ] Add OTA update support via WiFi
- [ ] BLE/WiFi diagnostics bridge on expansion Header C
