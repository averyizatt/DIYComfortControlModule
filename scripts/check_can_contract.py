#!/usr/bin/env python3
from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SHARED_HEADER = ROOT / "shared" / "can_contract" / "include" / "can_contract" / "can_protocol.h"
COMFORT_SHIM = ROOT / "src" / "can" / "can_protocol.h"
MODULES_DIR = ROOT / "modules"

# Canonical module names (correctly-spelled directories that should contain real firmware).
# For taillights the firmware lives in the typo directory "tailights/" but the stub and schema
# pin live in the correctly-spelled "taillights/".  Both are checked below.
REQUIRED_MODULES = ("water-meth", "taillights")

# Firmware may also live under an alternate directory name (e.g. typo variant).
# Map canonical name → alternate scan directory if it exists.
MODULE_FIRMWARE_ALIASES: dict[str, str] = {
    "taillights": "tailights",  # actual firmware in the typo directory
}

SOURCE_GLOBS = ("*.h", "*.hpp", "*.c", "*.cpp", "*.ino")

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


def module_has_source(module_dir: Path) -> bool:
    for pattern in SOURCE_GLOBS:
        if any(module_dir.rglob(pattern)):
            return True
    return False


def module_uses_shared_contract(module_dir: Path) -> bool:
    for pattern in SOURCE_GLOBS:
        for source in module_dir.rglob(pattern):
            try:
                text = source.read_text(encoding="utf-8", errors="ignore")
            except OSError:
                continue
            if '"can_contract/can_protocol.h"' in text:
                return True

    for ini_file in module_dir.rglob("platformio.ini"):
        try:
            ini_text = ini_file.read_text(encoding="utf-8", errors="ignore")
        except OSError:
            continue
        if "shared/can_contract/include" in ini_text:
            return True

    return False


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

    for module_name in REQUIRED_MODULES:
        module_dir = MODULES_DIR / module_name
        if not module_dir.exists():
            return fail(f"Missing required module directory: {module_dir.relative_to(ROOT)}")

        # Use the firmware alias directory if one is configured and it exists.
        firmware_dir = module_dir
        alias_name = MODULE_FIRMWARE_ALIASES.get(module_name)
        if alias_name:
            alias_dir = MODULES_DIR / alias_name
            if alias_dir.exists():
                firmware_dir = alias_dir
            else:
                print(f"  NOTE: alias directory {alias_name}/ not found; falling back to {module_name}/")

        if not module_has_source(firmware_dir):
            return fail(
                f"Module firmware ({firmware_dir.relative_to(ROOT)}) has no source files; ensure external module repo is present instead of placeholder files"
            )

        if not module_uses_shared_contract(firmware_dir):
            # Also accept the canonical stub directory including the contract.
            if firmware_dir != module_dir and module_uses_shared_contract(module_dir):
                print(
                    f"  NOTE: {module_name} firmware dir ({firmware_dir.relative_to(ROOT)}) does not directly include"
                    f" shared contract; stub at {module_dir.relative_to(ROOT)} does."
                )
            else:
                return fail(
                    f"Module {firmware_dir.relative_to(ROOT)} does not appear to include shared can_contract/can_protocol.h or shared include path"
                )

    print("PASS: shared CAN contract symbols, module schema pins, and module contract usage are compatible")
    return 0


if __name__ == "__main__":
    sys.exit(main())
