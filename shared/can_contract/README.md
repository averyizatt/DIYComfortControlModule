# Shared CAN Contract

Use this package as the single source of truth for cross-module CAN IDs, frame helpers, and command constants.

Header path:

- `shared/can_contract/include/can_contract/can_protocol.h`

For PlatformIO modules, add include path:

```ini
build_flags =
  -I ../../shared/can_contract/include
```

(or an equivalent relative path from each module project root).

This header is intentionally Arduino-agnostic (`<cstdint>` only) so all module repos can consume the same contract without framework coupling.

Schema 2 is the integrated protocol. See [the CAN audit](../../docs/CAN_COMPATIBILITY_AUDIT.md) for wire layouts, migration, and validation. Main local headers remain forwarding includes. The standalone ESP32 water/meth repository includes this shared contract; its obsolete copied Nano tree has been removed. Run `python scripts/check_can_contract.py --wire-tests` from the repository root.


Extension 10 adds `meth_drive::PULSE = 2` for ESP32 full-pressure MOSFET pulses.
Status carries on-time and cycle length in milliseconds. PWM remains value 1;
existing wire values and message layouts are unchanged.
