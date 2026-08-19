# MicroSquirt read-only CAN and resilient logging

## TunerStudio

The dashboard accepts simplified Dash Broadcast at `0x5E8` and Advanced
Realtime Broadcast at both the MegaSquirt default base `0x5F0` and the
recommended project base `0x700`.

For complete recording, enable Advanced Realtime Broadcasting and set its base
identifier to decimal `1792` (`0x700`). This keeps its 64-message range
`0x700..0x73F` away from the project's control IDs. Enable only the MS2 groups
needed by the installed MicroSquirt firmware, using faster rates for RPM/MAP/AFR
and slower rates for status and correction groups. All received groups in that
range are stored raw even if the dashboard does not display the field.

A conservative starting schedule is 20 Hz for groups 0-3, 10 Hz for group 4,
5 Hz for groups 5-11, and 2 Hz for groups 12-14. Additional groups supported by
the installed MS2/Extra firmware can start at 1-5 Hz. Check total bus load in
TunerStudio before increasing rates; the logger records every received group,
but it does not need every slowly changing field at 20 Hz.

The dashboard never transmits a MicroSquirt protocol message. CAN ACK bits are
still generated normally by the CAN controller, as required by the bus.

## Log durability

Files are stored in `/logs/ecu`:

- `.tmp` is the one active segment.
- `.ccmlog` is a clean, immutable segment.
- `.keep` is an event-protected immutable segment.
- `.recover` was interrupted; the converter reads it through the last valid CRC.

Each file has a CRC-protected 512-byte header and fixed 32-byte CRC-protected
records. The recorder rotates at 15 minutes or 16 MiB and writes at most one
512-byte sector per normal storage-service cycle. CAN capture uses a static ring
and never waits for the SD card.

Normal retention limits managed telemetry to 85% of card capacity. A failed
file open or short write also triggers deletion of one oldest unprotected
`.ccmlog` segment, covering cards that contain unrelated files. Event-protected
`.keep` files are never automatically deleted.

Convert a file on a PC:

```powershell
python tools/convert_ccmlog.py S00000001_000.ccmlog output.csv --format decoded
python tools/convert_ccmlog.py S00000001_000.ccmlog output.log --format candump
```

## Automotive power requirement

CRC recovery limits damage to the final record, but FAT cannot be guaranteed
against power loss in the middle of its own metadata write. A vehicle-ready
installation needs protected dashboard power plus an ignition/supply-fail sense
signal. Configure `CCM_PIN_IGNITION_SENSE` only after that input is electrically
conditioned for 3.3 V. Never connect a vehicle 12 V signal directly to an ESP32
pin. Hold-up time must be measured on the actual hardware and must cover the
worst observed queue drain, flush, and close time with margin.
