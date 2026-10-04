# Headless ESP32-S3 sensor gateway

Select **PlatformIO: Build and flash sensor gateway**, or run:

```powershell
pio run -e esp32s3_gateway
pio run -e esp32s3_gateway -t upload
```

The original dashboard remains the default `esp32s3_devkit_release` environment.
The gateway is a replacement firmware for the same ESP32-S3 + MCP2515 (8 MHz),
using standard 11-bit CAN at 500 kbit/s. It compiles only its own entry point,
MCP2515 driver, NeoPixel driver and Arduino framework. There is no display,
LVGL, touch, GPS, SD, Wi-Fi, web server, tach output, or RPM LED gauge.
It does not send water/meth arm/configuration commands or impersonate the master
heartbeat. Another project's CAN controller must provide any required master
supervision. Do not run two publishers of tach ID 0x202 on the same bus.

## Wiring and calibration

All gateway-specific settings are in `src/gateway/config.h`.
The MCP2515 uses the existing `src/pin_map.h` SPI pins at 1 MHz, except CS,
which the gateway build moves to GPIO10 (`-D CCM_PIN_CAN_SPI_CS=10`) so GPIO11
can be an LED channel.

| Connection | GPIO | Input/output |
|---|---:|---|
| Conditioned VSS | 15 | Rising-edge digital input |
| Conditioned tach | 6 | Rising-edge digital input |
| Fuel divider junction | 7 | ADC1 input |
| Steering ON (yellow) | 5 | ADC1 voltage through a 10 kohm / 3.3 kohm divider; pressed at 0.5 V or more at the pin |
| Steering ladder (blue): OFF / COAST / SET ACCEL / RESUME | 4 | ADC1 voltage ladder; no internal pull-up |
| Upper interior LEDs | 14 | WS2812/GRB, 180 slots |
| Lower interior LEDs | 13 | WS2812/GRB, 180 slots |
| Spare LED channel 1 | 12 | Reserved; held LOW |
| Spare LED channel 2 | 11 | Reserved; held LOW |
| MCP2515 SCK / MOSI / MISO | 8 / 3 / 17 | SPI |
| MCP2515 CS / INT / RESET | 10 / 18 / 21 | INT currently polled through SPI |

Steering ON (yellow) is GPIO5 and the ladder (blue) is GPIO4. These are GPIO
numbers, not board D-labels. Disconnect any old display/touch/SD wiring first. ON is approximately 0 V idle and
3.0 V pressed, with your external 10 kohm pulldown. Both inputs use `INPUT`
without internal pulls. Signals must stay within 0..3.3 V with common ground;
VSS conditioner and tach interface outputs must also be 3.3 V-safe.

| Ladder button | Measured on this wheel | Accepted millivolts | CAN bit |
|---|---:|---:|---:|
| OFF | 0.15 V | 0..270 | 1 |
| COAST | 0.40 V | 280..700 | 2 |
| SET ACCEL | 1.50 V | 1250..1750 | 3 |
| RESUME | 2.30 V | 2050..2550 | 4 |
| None | about 3.3 V | Outside button windows | None |

ON uses CAN bit 0 independently. The windows are `steeringWindows` in
`src/gateway/config.h`, one per button because OFF and COAST are only 0.25 V
apart on this wheel; re-measure at GPIO4 and adjust them for a different wheel or
pull-up. The ladder uses calibrated millivolts at 11 dB
attenuation. ESP32-S3's documented measurable range ends around 3100 mV, so
high idle readings need not reach 3300 mV to decode as released; see
[Espressif ADC documentation](https://docs.espressif.com/projects/esp-idf/en/v4.4.3/esp32s3/api-reference/peripherals/adc.html).
Readings in gaps between button windows also decode as released after debounce.
A short to ground is electrically indistinguishable from OFF. The ladder reports
one selection at a time; arbitrary simultaneous ladder presses cannot be resolved.
ON can be held together with any one ladder button.

VSS defaults to **8000 pulses/mile**, measured on rising edges after the VR
conditioner. This is the nominal Ford calibration, not an invariant of the
transmission: tire size, axle ratio, speedometer gears and conditioner edge
multiplication affect it. Set `vssPulsesPerMile` to the measured count over a known
distance. At 60 mph the nominal input is 133.333 Hz (7500 us period).
See [AccuTach Ford VSS calibration](https://accutach.com/ford-clusters) and
[calibration factors](https://accutach.com/speedcal-calculator).

Tach uses a GPIO6 rising-edge input (the dashboard used GPIO2). It reads the
existing `ccm_cfg` / `tach_ppr10` calibration from the ESP32's NVS in read-only
mode. Without a saved nonzero value it uses the same dashboard default of
**2.0 pulses per revolution** (`tachPulsesPerRev10 = 20`). A saved calibration
therefore survives switching firmware. The active value is printed at startup
and included in tach frame byte 6. No settings are erased or rewritten.

Tach rejects edges less than 1000 us apart and expires after 250 ms, matching
the dashboard's existing input filter. VSS rejects edges less than 500 us apart.
Both inputs require two fresh edges; VSS expires after
1500 ms, and clears validity on timeout. No pulses cannot distinguish a stopped
vehicle/engine from a disconnected sensor; zero with validity clear means no
fresh pulse measurement. Consumers must also expire all gateway data if frames
stop arriving (suggested timeout 500 ms).

Fuel wiring includes a series resistor and capacitor at the ADC input:

```text
3.3 V -- 100 ohm (0.25 W) -- sense junction -- 4.7 kohm -- ADC GPIO7
                                  |                         |
                              Fuel sender                 100 nF
                                  |                         |
                            Chassis ground ---------------- GND
```

Connect chassis ground, the sender return and ESP32 ground together. Use a
regulated 3.3 V supply. This is a dedicated divider, not a parallel connection
to a powered factory fuel gauge. GPIO7 is the project's configured fuel input;
use the corresponding board pin, not an assumed XIAO D-number.

The 4.7 kohm resistor and 100 nF capacitor filter the ADC signal. At steady state,
assuming negligible ADC loading, the capacitor draws no DC current and the
series resistor does not change the divider equation or reverse its direction.
The 100 ohm pullup remains the resistance used in the conversion (do not add
4700 ohms to `fuelPullupOhms`). The firmware uses calibrated ADC millivolts:

`V_adc = 3.3 * R_sender / (100 + R_sender)`

`R_sender = 100 * V_adc / (3.3 - V_adc)`

Do not interpolate percentage directly from voltage; the divider is nonlinear.

For this installation the sender reads **22 ohms empty / 145 ohms full**. With the
3.3 V / 100 ohm divider:

| Sender position | Resistance | ADC voltage | Published percent |
|---|---:|---:|---:|
| Empty | 22 ohms | 0.595 V | 0 |
| Half resistance range | 83.5 ohms | 1.502 V | 50 |
| 100 ohm bench test resistor | 100 ohms | 1.650 V | 63 |
| Full | 145 ohms | 1.953 V | 100 |

`fuelEmptyOhms` / `fuelFullOhms` can be fine-tuned to measured sender endpoints.
These are the requested installation settings, not a claim about the stock 1989
Mustang sender. Verify full/empty direction against the installed sender.
Percentage is a linear sender-travel estimate, not calibrated tank volume.
Sampling is 100 ms with resistance smoothing; open/short
or implausible readings publish 255 percent with fuel validity clear.

## CAN contract for the other project

Copy `shared/can_contract/include/can_contract/` into the receiving project's
include path. Include `<can_contract/gateway_protocol.h>`. It extends the existing
schema 2 contract without changing existing IDs or payloads. Integers are
big-endian. New gateway-specific message version is 1.

| ID | Direction | DLC | Bytes |
|---|---|---:|---|
| 0x202 | TX | 8 | Existing tach layout: RPM u16; generated tach Hz x10 u16 (zero, no output); source=1; status=1 live or 2 stale; pulses/revolution x10; reserved=0 |
| 0x500 | TX | 8 | Speed km/h x10 u16; RPM u16; raw fuel ADC u16; fuel percent 0..100 or 255 invalid; validity bits 0=VSS, 1=RPM, 2=fuel |
| 0x501 | TX | 4 | Pressed bitmask; enabled bitmask (0x1F); transition sequence; version=1 |
| 0x502 | RX | 6 | Channel (0 both, 1 upper, 2 lower); R; G; B; brightness; version=1 |
| 0x503 | TX | 7 | Channel (1 or 2); R; G; B; applied brightness; version=1; command fresh (0 or 1) |

Debounced button changes take the next available 10 ms transmit slot.
Normally tach is sent every 20 ms and each of sensors/buttons/upper-light-status/
lower-light-status every 80 ms. These are nominal schedules, subject to CAN ACK,
transport backoff, button-change priority and LED output time. One-shot TX, a 20 ms transmit timeout,
and 1 s retry backoff prevent a disconnected bus from blocking sensor sampling.
Runtime bus-off recovery is handled by the MCP2515; a failed initial setup requires
checking wiring and restarting. Four RX frames maximum are processed per loop.
Extended/RTR frames, incorrect lengths, versions and invalid channels are rejected.

Button bits are 0=ON, 1=OFF, 2=COAST, 3=SET ACCEL, 4=RESUME, with named
`BUTTON_*` constants in the shared header. ON and the ladder selection each
require 25 ms of stable input. Ladder transitions replace the previous selection
atomically. Sequence increments modulo 256 when the stable combined mask changes.
This is a repeated **current-state** protocol, not a guaranteed queue of every
press/release; use sequence gaps to detect missed changes, and clear held buttons
if messages time out. A button held during boot becomes pressed after debounce.
Assign actions to these button states in your other project.

```cpp
can_protocol::gateway::Buttons buttons;
if (can_protocol::gateway::unpackButtons(frame, buttons)) {
    bool onHeld = (buttons.pressed & can_protocol::gateway::BUTTON_ON) != 0;
    // Save reception time and expire this state after 500 ms of silence.
}
// Both strips, white at low brightness. Send again at least once per second.
auto command = can_protocol::gateway::packLight({0, 255, 255, 255, 35});
// Transmit command through the other project's CAN driver.
```

Lights start off. Refresh each controlled channel at least once per second;
a channel switches off five seconds after its last valid command. Brightness zero
turns it off immediately. Settings are live only and are not saved to flash.
The gateway does not implement taillight animations or dashboard lighting effects.
Its loop stack reserves the NeoPixel driver's per-strip RMT buffer plus 16 KiB
headroom; changing `ledCount` adjusts this reservation automatically.

## Validation

Host regression tests:

```powershell
$env:PATH = 'C:/msys64/ucrt64/bin;' + $env:PATH
g++ -std=c++17 -Wall -Wextra -Werror -Iinclude -Isrc -Ishared/can_contract/include test/test_gateway/test_gateway.cpp -o .pio/test_gateway.exe
if ($LASTEXITCODE -ne 0) { throw 'Compile failed' }
& .pio/test_gateway.exe
```

Before vehicle use, bench-check all five button levels, ON held with each ladder
button, releases, voltage gaps and transitions between ladder buttons,
a known pulse frequency on both inputs, fuel empty/full resistances and unplugged
sender, both lighting channels, command timeout, and CAN disconnect/reconnection.
Firmware compilation and host tests do not validate the electrical installation.
