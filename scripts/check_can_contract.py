#!/usr/bin/env python3
from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SHARED_HEADER = ROOT / "shared" / "can_contract" / "include" / "can_contract" / "can_protocol.h"
COMFORT_SHIM = ROOT / "src" / "can" / "can_protocol.h"
MODULES_DIR = ROOT / "modules"

REQUIRED_SYMBOLS = {
    "CAN_PROTOCOL_SCHEMA_VERSION": r"constexpr\s+uint16_t\s+CAN_PROTOCOL_SCHEMA_VERSION\s*=\s*(\d+)\s*;",
    "ID_TAILLIGHT_STATE": r"constexpr\s+uint16_t\s+ID_TAILLIGHT_STATE\s*=\s*0x100\s*;",
    "ID_TAILLIGHT_COMMAND": r"constexpr\s+uint16_t\s+ID_TAILLIGHT_COMMAND\s*=\s*0x101\s*;",
    "ID_TAILLIGHT_FAULT": r"constexpr\s+uint16_t\s+ID_TAILLIGHT_FAULT\s*=\s*0x102\s*;",
    "ID_ENGINE_METH_STATE": r"constexpr\s+uint16_t\s+ID_ENGINE_METH_STATE\s*=\s*0x300\s*;",
    "ID_ENGINE_METH_COMMAND": r"constexpr\s+uint16_t\s+ID_ENGINE_METH_COMMAND\s*=\s*0x301\s*;",
    "ID_ENGINE_METH_FAULT": r"constexpr\s+uint16_t\s+ID_ENGINE_METH_FAULT\s*=\s*0x302\s*;",
}


def fail(message: str) -> int:
    print(f"FAIL: {message}")
    return 1


def main() -> int:
    if not SHARED_HEADER.exists():
        return fail(f"Missing shared CAN header: {SHARED_HEADER}")

    shared_text = SHARED_HEADER.read_text(encoding="utf-8")

    captures: dict[str, str] = {}
    for name, pattern in REQUIRED_SYMBOLS.items():
        match = re.search(pattern, shared_text)
        if not match:
            return fail(f"Missing or unexpected {name} in shared contract")
        if match.groups():
            captures[name] = match.group(1)

    schema_version = captures.get("CAN_PROTOCOL_SCHEMA_VERSION", "")
    if not schema_version:
        return fail("Could not determine CAN_PROTOCOL_SCHEMA_VERSION")

    if not COMFORT_SHIM.exists():
        return fail(f"Missing comfort shim header: {COMFORT_SHIM}")

    shim_text = COMFORT_SHIM.read_text(encoding="utf-8")
    if '"can_contract/can_protocol.h"' not in shim_text:
        return fail("Comfort shim must include shared can_contract/can_protocol.h")

    pin_files = sorted(MODULES_DIR.glob("*/CAN_PROTOCOL_SCHEMA_VERSION"))
    if not pin_files:
        return fail("No module schema pin files found")

    for pin in pin_files:
        pin_value = pin.read_text(encoding="utf-8").strip()
        if pin_value != schema_version:
            return fail(
                f"Schema mismatch for {pin.relative_to(ROOT)}: expected {schema_version}, found {pin_value or '<empty>'}"
            )

    print("PASS: shared CAN contract symbols and module schema pins are compatible")
    return 0


if __name__ == "__main__":
    sys.exit(main())
