# DIYComfortControlModule

Distributed automotive electronics and dashboard platform for a **1989 Foxbody Mustang** using multiple ESP32-based nodes over CAN bus.

---

## Project Overview

This project is a modular in-vehicle control ecosystem where each subsystem can run on its own controller while sharing state over a deterministic CAN network.

The **Cabin Master (ESP32-S3)** is the primary node and acts as:
- dashboard + control node
- touchscreen UI controller
- CAN gateway/orchestrator
- tachometer interface node
- GPS interface node
- environmental monitoring node
- ambient lighting controller
- web dashboard + configuration host

Additional ESP32 nodes can be assigned to:
- taillight control
- water/meth injection
- engine bay sensors
- additional lighting domains
- future body/comfort systems

The design goal is an extensible platform with clear module boundaries, deterministic communication, and automotive safety-first behavior.

---

## Main Features

- 4" ST7796S capacitive touch dashboard
- physical tactile button fallback navigation
- ESP32 TWAI + transceiver CAN networking
- stock tachometer integration
- GPS integration
- 3-channel addressable cabin ambient LED control
- water/meth monitoring + configuration flow
- environmental monitoring
- SD card logging + UI asset storage
- onboard WiFi web dashboard + config portal
- modular CAN architecture with versioned packets
- fail-safe design philosophy (engine bay safety autonomy)
- `DEMO_MODE` for bench testing without full hardware

---

## Hardware

### Main Controller
- **Arduino Nano ESP32** (ESP32-S3)
- USB-C
- dual-core MCU
- WiFi + Bluetooth capability

### Display Module
- **Hosyond 4.0" 320x480 ST7796S SPI TFT**
- capacitive touch controller over I2C
- onboard microSD slot sharing SPI bus

### CAN
- ESP32 TWAI peripheral
- SN65HVD230 (or compatible) transceiver
- 500 kbit/s, standard 11-bit CAN

### Lighting
- WS2812B / SK6812 compatible LEDs
- 3 independent data channels for cabin zones

### GPS
- UART GPS module

### Tach
- stock Foxbody tach frequency-driven interface
- tach input capture + tach output generation/calibration path

### Water/Meth
- distributed engine-bay controller node
- cabin sends desired config; engine node enforces local safety

---

## Display Pinout

| Module Label | Function |
|---|---|
| VCC | Module power |
| GND | Ground |
| LCD_CS | LCD SPI chip select |
| LCD_RST | LCD reset |
| LCD_RS / DC | LCD data/command select |
| SDI/MOSI | Shared SPI MOSI |
| SDO/MISO | Shared SPI MISO |
| SCK | Shared SPI clock |
| LED | Backlight control (PWM-capable preferred) |
| CTP_SCL | Touch I2C SCL |
| CTP_SDA | Touch I2C SDA |
| CTP_RST | Touch reset GPIO |
| CTP_INT | Touch interrupt GPIO |
| SD_CS | SD card SPI chip select |

### Bus Rules
- LCD and SD share SPI lines (MOSI/MISO/SCK).
- Touch controller uses I2C.
- Backlight should use PWM for brightness control where possible.
- `LCD_CS` and `SD_CS` must never be active together.
- Keep inactive CS lines high.
- SD operations must not block CAN/tach/UI-critical timing paths.

---

## System Architecture

Distributed node model over deterministic CAN message flow.

### CAN ID ranges
- `0x100–0x10F` taillight ECU
- `0x200–0x20F` cabin master
- `0x300–0x30F` engine bay / water meth
- `0x400–0x40F` GPS / logging
- `0x500–0x50F` comfort / climate
- `0x600–0x60F` future expansion

### Protocol characteristics
- standard 11-bit CAN IDs
- 500 kbit/s
- fixed-length embedded-friendly packets
- simple and deterministic scheduling

---

## Existing Taillight Compatibility

The existing taillight contract is preserved:
- `0x100` taillight state broadcast
- `0x101` taillight command frame
- `0x102` taillight fault frame

Cabin master capabilities:
- view taillight state
- set brightness
- trigger custom animations
- clear/override modes
- show taillight fault telemetry

---

## Water Meth System

### Responsibility split
- **Cabin Master** stores and broadcasts user intent/config.
- **Engine Bay Module** owns immediate pump safety and local enforcement.

### Frames
- `0x300` `ENGINE_METH_STATE`
- `0x301` `ENGINE_METH_COMMAND`
- `0x302` `ENGINE_METH_FAULT`
- `0x304` `METH_CONFIG_BROADCAST`
- `0x305` `METH_CONFIG_REQUEST`
- `0x306` `METH_CONFIG_ACK`

### Safety behavior
- boots disarmed
- manual test requires explicit confirmation and timeout
- critical faults latch and force safe output behavior
- CAN loss must not trigger spraying
- engine module can run from last known safe config or disarm policy
- engine module remains autonomous for safety-critical decisions

> ⚠️ **Mixture ratio warning:** Selected meth ratio is user-configured and **not sensor-verified** unless a real concentration sensor is installed.

---

## Tachometer System

- Supports stock Foxbody tach use-cases (frequency-driven behavior).
- RPM-to-frequency scaling path is configurable/calibratable.
- Startup sweep support for gauge verification.
- Designed with LM1819-style air-core tach assumptions in mind.

Important electrical note:
- ESP32 generates only the conditioned tach signal path.
- Tach power/ground for the gauge remain in proper vehicle electrical domains.

---

## Ambient LED System

- 3 independent addressable LED channels:
  1. driver accent
  2. passenger accent
  3. dash/console accent
- supports brightness, color, per-channel mode, and themes
- supports startup animation and special status behaviors
- includes non-blocking modes such as breathing, rainbow, RPM-reactive, warning, meth-active, CAN-fault handling
- supports night dimming behavior

All animation timing is non-blocking (`millis()` scheduling), so LED effects do not block CAN/tach/UI/GPS tasks.

---

## SD Card Logging

### Intended logs
- CAN logs
- GPS logs
- fault logs
- water/meth logs
- tach calibration/session logs

### Suggested structure

```text
/logs/
  can/
  gps/
  meth/
  faults/
  tach/
/ui/
  images/
  icons/
  themes/
  splash/
```

### Design expectations
- buffered writes
- periodic flushes
- immediate critical-fault flush attempt
- GPS timestamps when available, uptime fallback otherwise
- system must operate safely if SD is absent or fails to mount

---

## Web Dashboard

Planned/implemented LAN-focused web control portal:
- live dashboard
- settings/config page
- LED control
- water/meth control
- diagnostics
- taillight control

### API endpoints
- `GET /api/state`
- `GET /api/settings`
- `POST /api/settings`
- `POST /api/led`
- `POST /api/meth`
- `POST /api/taillights`
- `POST /api/tach`
- `GET /api/diagnostics`

Design notes:
- JSON APIs
- live updates by WebSocket/SSE
- local-network operation focus
- confirmation required for unsafe actions

---

## Software Architecture

```text
src/
  ui/
  can/
  gps/
  tach/
  led/
  web/
  storage/
  touch/
  settings/
  meth/
  state/
```

### Module roles
- `state/`: shared `VehicleState` and synchronization
- `can/`: protocol definitions + transport/scheduling
- `meth/`: meth config model/packing/validation
- `settings/`: NVS-backed persistent configuration
- `led/`: non-blocking ambient LED engine
- `web/`: HTTP/WebSocket APIs + dashboard hosting
- `touch/`: capacitive touch manager abstraction
- `storage/`: SD mount + buffered logging
- `ui/`: dashboard rendering + assets
- `gps/`: GNSS ingest and validity tracking
- `tach/`: tach input/output control paths

### VehicleState pattern
- producers (CAN/GPS/sensors/tasks) update one central `VehicleState`
- consumers (UI/web/diagnostics) read the same state object
- avoids fragmented ownership and keeps telemetry coherent

---

## Automotive Electrical Design (Safety + Protection)

Recommended protections:
- TVS diodes on automotive-exposed rails
- reverse polarity protection
- quality buck conversion for logic rails
- flyback protection on inductive loads
- star grounding strategy
- separate dirty/high-current and clean/signal grounds
- bulk capacitance near load domains
- series GPIO protection resistors where appropriate
- protected MOSFET output stages for high-current controls

### Strong warnings
- Do **not** connect raw automotive signals directly to ESP GPIO.
- Do **not** share heavy motor/pump grounds directly with sensitive signal returns.
- Do **not** run pumps/fans/solenoids without flyback/transient suppression.

---

## Recommended PCB Layout Philosophy

- split dirty/high-current and clean/logic board zones
- keep high-current loops short
- use appropriately thick power/ground copper
- route CAN cleanly (controlled path, robust connector strategy)
- place decoupling close to each IC power pin
- design for serviceability and modular replacement
- use locking automotive-capable connectors where practical

---

## DEMO_MODE

`DEMO_MODE` supports bench validation without full vehicle integration:
- fake/simulated CAN telemetry
- simulated tach behavior
- fake GPS + sensor data
- dashboard and UI workflow testing without live bus hardware

---

## Future Roadmap

Potential next features:
- launch control integration
- digital gauges and advanced pages
- shift lights
- fan/thermal control
- vehicle security/immobilizer features
- OTA updates
- Bluetooth companion app
- audio integration
- camera integration
- suspension monitoring
- richer data export workflows
- track mode + lap timing

---

## Contributing

Please keep contributions aligned with project goals:
- keep CAN protocol updates documented
- avoid blocking loops in runtime tasks
- preserve fail-safe behavior
- comment packet meanings/bitfields
- verify `DEMO_MODE` compatibility
- preserve taillight compatibility (`0x100/0x101/0x102`)

Safety-first and modularity-first changes are preferred over feature speed.

---

## License

License to be added.

