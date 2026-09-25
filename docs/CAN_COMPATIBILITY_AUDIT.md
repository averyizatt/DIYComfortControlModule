# CAN compatibility audit - schema 2

Audited the firmware in this repository: main ESP32, `modules/water-meth`
(Nano), and `modules/tailights` (ESP32). The correctly spelled
`modules/taillights` directory is a compatibility stub, not the firmware.

## Deployment

Build and update **all three devices together**. Schema 2 separates command
ACKs from configuration ACKs; mixing schema 1 and schema 2 firmware is not a
supported deployment. Schema pins are build/source checks, not automatic
runtime version negotiation. No hardware was flashed during this audit.

All integrated builds use standard 11-bit data frames, **500 kbit/s**, and
**8 MHz MCP2515 oscillator settings**. Verify the oscillator fitted to each
physical breakout, CAN H/L wiring, common ground, and bus termination on the
car; a source audit cannot verify those or the firmware already flashed.
Taillight CAN is now enabled by default. A standalone bench build can disable
it using `-DCCM_TAILLIGHT_CAN_ENABLED=0`.

## Corrected disagreements

- Three divergent protocol headers claimed schema 1. Main and Nano headers
  are now forwarding includes to the canonical shared header; real taillight
  firmware also consumes it. The old checker accepted a placeholder directory
  as proof that the real firmware used the shared contract.
- Taillight state byte 3 is passenger input flags, not brightness. Brightness,
  raw Celsius and derating are bytes 4, 5 and 6. The main decoder now agrees,
  and the taillight sender uses the shared packer. Reported brightness is the
  applied LED brightness, and reported states include output overrides.
- The main controller sent legacy custom-animation IDs 1/2/3 for SEQ/SHOW/DEMO;
  those IDs actually mean scrolling text, amber flashes and two-character text.
  Mode selection now uses a distinct command, including all 33 show options.
  Stock clears show/custom overrides and uses simple-flash turns; SEQ uses
  sequential turns. Demo cycles show options every five seconds.
- Taillight brightness commands previously lasted one loop iteration before
  saved settings took over. They now update live brightness until changed again.
- Legacy custom-animation duration was treated as scrolling speed. It now
  caps animation runtime in milliseconds; zero retains built-in completion.
- Nano pressure telemetry is PSI multiplied by two. The main decoder now
  divides by two, preserving half-PSI resolution (maximum 127.5 PSI).
- Disabled/faulted sensor channels now report invalid flags, which the main
  controller respects instead of marking every received channel valid.
- Command ACK and configuration ACK shared ID 0x306 with incompatible meanings.
  Arm ACKs could change mixture to 1%; version/ratio combinations could also
  look like knock ACKs. All command ACKs now use 0x30A; 0x306 is config-only.
  Rejected knock commands do not overwrite live settings.
- Main reception rejects extended/RTR frames instead of truncating their IDs
  into the standard-frame namespace.

## Active wire agreement

| ID | Sender -> receiver | DLC | Contents / nominal period |
|---|---|---:|---|
| 0x100 | Taillights -> main | 7 | left/right state, driver/passenger inputs, brightness, raw Celsius, derate; 100 ms |
| 0x101 | Main -> taillights | varies | brightness, override, clear, legacy custom animation, mode |
| 0x102 | Taillights -> main | 4 | fault code, severity, data0, data1; event |
| 0x200 | Main -> Nano | 8 | heartbeat; 250 ms |
| 0x202 | Main -> listeners | 8 | tach telemetry, big-endian 16-bit values; 50 ms |
| 0x203 | Main -> listeners | 8 | GPS telemetry, big-endian 16-bit values; 500 ms |
| 0x300 | Nano -> main | 8 | meth state, duty, binary tank 0/100, flow, gauge boost kPa, IAT+40, bay+40, faults; 50 ms |
| 0x301 | Main -> Nano | varies | meth commands 0x01..0x06 and knock commands 0x40..0x4A |
| 0x302 | Nano -> main | 4 | meth fault event format (receiver supported) |
| 0x303 | Nano -> main | 8 | four pressures *2, ambient+40, cabin+40, little-endian 16-bit invalid flags; 250 ms |
| 0x304 | Main -> Nano | 8 | config version, arm, ratio, absolute boost threshold, reserved IAT, max duty, protection flags, XOR; 1000 ms |
| 0x305 | Main -> Nano | 1 | request knock configuration pages; 1000 ms |
| 0x306 | Nano -> main | 4 | accepted config version, status, reason, active mixture ratio |
| 0x307 | Nano -> main | 8 | knock state; 50 ms |
| 0x308 | Nano -> main | 4 | knock fault event |
| 0x309 | Main -> Nano | 4 | little-endian RPM, absolute MAP kPa, valid flags; 50 ms |
| 0x30A | Nano -> main | 4 | command, status, applied value, schema=2 |
| 0x30B | Nano -> main | 8 | knock live diagnostics; 50 ms |
| 0x30C / 0x30D | Nano -> main | 8 each | knock configuration pages, periodically and on request |

0x201 is a reserved/limited main-controller command endpoint, not used by
these two peripherals. MicroSquirt telemetry is decoded separately; its
recommended IDs are covered by the existing protocol tests.

`0x101 [05 mode option]`: mode 0=stock, 1=sequential, 2=show, 3=demo;
option 0..32 selects the same show list displayed by the main UI. Invalid
mode/option/length combinations leave settings unchanged. These changes are
live, not NVS writes. Physical brake and reverse inputs retain priority.
Mode labels on the main UI indicate commands sent; the seven-byte taillight
state has no mode acknowledgement field.

`0x303` invalid bits: 0 oil, 1 fuel, 2 meth pressure, 3 boost reference,
4 IAT, 5 engine bay, 6 ambient, 7 cabin. Disabled channels are invalid too.

Nano injection is boost-only: command 0x05 (SET_IAT_THRESHOLD) is reserved and
returns UNSUPPORTED_COMMAND; the config IAT byte is not an injection gate.
Command 0x04 uses **gauge** kPa, whereas config byte 3 uses **absolute** kPa
(zero preserves the Nano threshold). The current main config sends 114 kPa.
Flow telemetry remains UNKNOWN because no flow sensor is configured.

The Nano expires engine RPM context after 500 ms. Manual pump tests have an
independent five-second local limit and stop after three seconds without
master traffic; normal injection's existing behavior on CAN loss is unchanged.

## Verification

Run `python scripts/check_can_contract.py --wire-tests` with g++ and the native
Unity dependency installed. This validates the actual include chain, schema
pins, integrated code paths, golden taillight bytes, half-PSI encoding, RPM
endianness, distinct ACK formats, malformed lengths, and all 33 show options
through the actual taillight command-to-settings function.

Bench checks still required after coordinated flashing: compare reported
brightness/temperature/pressure with each module, exercise stock/SEQ/show/demo
and clear, check brake/reverse priority, test CAN loss/recovery, and verify Arm,
mixture, tank bypass and timed pump stop. Do pump checks with the nozzle routed
into a suitable container.
