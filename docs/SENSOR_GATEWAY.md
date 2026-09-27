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
The MCP2515 uses the existing `src/pin_map.h` SPI pins at 1 MHz.

| Connection | GPIO | Input/output |
|---|---:|---|
| Conditioned VSS | 4 | Rising-edge digital input |
| Conditioned tach | 2 | Rising-edge digital input |
| Fuel divider junction | 1 | ADC1 input |
| Steering buttons 1 through 5 | 5, 6, 7, 9, 10 | Individual switch to ground, internal pull-up |
| Upper interior LEDs | 38 | WS2812/GRB, 180 slots |
| Lower interior LEDs | 39 | WS2812/GRB, 180 slots |
| MCP2515 SCK / MOSI / MISO | 8 / 3 / 17 | SPI |
| MCP2515 CS / INT / RESET | 11 / 18 / 21 | INT currently polled through SPI |

These button pins reuse the removed display/touch/SD connections. Disconnect
those peripherals before wiring switches. Existing steering-wheel functions must
be isolated from these switch inputs; do not connect a powered factory circuit
to an ESP32 GPIO. VSS conditioner and tach interface outputs must be 3.3 V-safe.

VSS defaults to **8000 pulses/mile**, measured on rising edges after the VR
conditioner. This is the nominal Ford calibration, not an invariant of the
transmission: tire size, axle ratio, speedometer gears and conditioner edge
multiplication affect it. Set `vssPulsesPerMile` to the measured count over a known
distance. At 60 mph the nominal input is 133.333 Hz (7500 us period).
See [AccuTach Ford VSS calibration](https://accutach.com/ford-clusters) and
[calibration factors](https://accutach.com/speedcal-calculator).

Tach reuses the current controller's GPIO2 rising-edge input. It reads the
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

Fuel wiring is **3.3 V -> 100 ohm fixed resistor -> ADC junction -> sender ->
ground**. Use a regulated supply, common ground and at least a 0.25 W fixed
resistor. This is a dedicated divider, not a parallel connection to a powered
factory fuel gauge. The firmware uses calibrated ADC millivolts and computes
`R_sender = 100 * V_adc / (3.3 - V_adc)` before mapping resistance to percentage.
Do not interpolate percentage directly from voltage; the divider is nonlinear.

The **1989 Mustang** uses the configured sender range of **16 ohms empty /
158 ohms full**, corresponding to about 0.455 V / 2.021 V with this divider.
`fuelEmptyOhms` / `fuelFullOhms` can be fine-tuned to measured sender endpoints.
[AutoMeter's sender range guide](https://www.autometer.com/media/manual/2650-1858.pdf)
documents the 16/158 range. Percentage is a linear sender-travel estimate, not
calibrated tank volume. Sampling is 100 ms with resistance smoothing; open/short
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

Button bit 0 is button 1 through bit 4 for button 5. Multiple held buttons are
supported. Each input is debounced for 25 ms. Sequence increments modulo 256 when
the stable combined mask changes. This is a repeated **current-state** protocol,
not a guaranteed queue of every press/release; use sequence gaps to detect missed
changes, and clear held buttons if messages time out. A button held during boot
becomes pressed after debounce. Assign meanings (up/down/etc.) in your other project.

```cpp
can_protocol::gateway::Buttons buttons;
if (can_protocol::gateway::unpackButtons(frame, buttons)) {
    bool button1Held = (buttons.pressed & 1) != 0;
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

Before vehicle use, bench-check all five buttons (including simultaneous holds),
a known pulse frequency on both inputs, fuel empty/full resistances and unplugged
sender, both lighting channels, command timeout, and CAN disconnect/reconnection.
Firmware compilation and host tests do not validate the electrical installation.
