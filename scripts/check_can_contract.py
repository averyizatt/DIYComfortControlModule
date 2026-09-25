#!/usr/bin/env python3
"""Check actual firmware includes; optionally compile/run cross-module wire tests."""
from __future__ import annotations

import argparse
import re
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SHARED = ROOT / "shared/can_contract/include/can_contract/can_protocol.h"


def text(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def require(condition: bool, message: str) -> None:
    if not condition:
        raise RuntimeError(message)


def check_structure() -> None:
    shared = SHARED.read_text(encoding="utf-8")
    version = re.search(r"CAN_PROTOCOL_SCHEMA_VERSION = (\d+);", shared).group(1)
    for wrapper in ("include/can_contract/can_protocol.h",
                    "modules/water-meth/include/can_contract/can_protocol.h"):
        body = text(wrapper)
        include = re.search(r'#include "([^"]+)"', body)
        require(include is not None, f"{wrapper}: missing canonical include")
        require((ROOT / wrapper).parent.joinpath(include.group(1)).resolve() == SHARED.resolve(),
                f"{wrapper}: does not resolve to canonical contract")
        require("namespace can_protocol" not in body, f"{wrapper}: duplicated protocol definitions")
    for firmware in ("modules/tailights/src/config.h", "modules/tailights/src/can_control.h"):
        require("../../../shared/can_contract/include/can_contract/can_protocol.h" in text(firmware),
                f"{firmware}: real firmware must include canonical contract (stub is insufficient)")
    require('"can_contract/can_protocol.h"' in text("src/can/can_protocol.h"), "Master shim missing")
    require('"can_contract/can_protocol.h"' in text("modules/water-meth/src/main_nano.cpp"), "Nano include missing")
    for module in ("comfort", "water-meth", "taillights", "tailights"):
        require(text(f"modules/{module}/CAN_PROTOCOL_SCHEMA_VERSION").strip() == version,
                f"{module}: schema pin differs from {version}")
    for project in ("platformio.ini", "modules/water-meth/platformio.ini"):
        versions = re.findall(r"CCM_CAN_PROTOCOL_VERSION=(\d+)", text(project))
        require(versions and all(v == version for v in versions), f"{project}: build schema mismatch")
    require("#define CCM_TAILLIGHT_CAN_ENABLED 1" in text("modules/tailights/src/config.h"),
            "Integrated taillight build must enable CAN")
    for source, token in (
        ("modules/tailights/src/canbus.cpp", "can_protocol::packTaillightState(state)"),
        ("modules/tailights/src/canbus.cpp", "applyCanMode(g_settings, command)"),
        ("src/can/can_manager.cpp", "can_protocol::ID_ENGINE_COMMAND_ACK"),
        ("modules/water-meth/src/main_nano.cpp", "can_protocol::packConfigAck(")):
        require(token in text(source), f"{source}: expected production protocol path missing: {token}")
    main_ini = text("platformio.ini")
    bitrates = re.findall(r"CCM_CAN_BITRATE=(\d+)", main_ini)
    require(bitrates and all(rate == "500000" for rate in bitrates),
            "Main CAN bitrate settings drifted")
    require("CCM_CAN_MCP_CLOCK_MHZ=8" in main_ini, "Main oscillator setting drifted")
    nano_ini = text("modules/water-meth/platformio.ini")
    require("CCM_CAN_SPEED=CAN_500KBPS" in nano_ini and "CCM_MCP2515_CLOCK=MCP_8MHZ" in nano_ini,
            "Nano bus settings drifted")
    require("setBitrate(CAN_500KBPS, MCP_8MHZ)" in text("modules/tailights/src/canbus.cpp"),
            "Taillight bus settings drifted")
    print(f"PASS: schema {version}, canonical includes, bus settings and integrated firmware paths", flush=True)


def wire_tests(compiler: str) -> None:
    require(shutil.which(compiler) is not None, f"Compiler unavailable: {compiler}")
    unity = ROOT / ".pio/libdeps/native/Unity/src"
    require((unity / "unity.c").exists(), "Install native test dependencies first (Unity missing)")
    output = ROOT / ".pio/can_contract_tests.exe"
    subprocess.run([compiler, "-std=c++17", "-I", str(ROOT / "include"),
                    "-I", str(ROOT / "src"), "-I", str(unity),
                    str(ROOT / "test/test_can_protocol/test_main.cpp"),
                    str(unity / "unity.c"), "-o", str(output)], check=True)
    subprocess.run([str(output)], check=True)
    print("PASS: cross-module wire payload and command tests")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--wire-tests", action="store_true")
    parser.add_argument("--compiler", default="g++")
    args = parser.parse_args()
    try:
        check_structure()
        if args.wire_tests:
            wire_tests(args.compiler)
        else:
            print("Wire behavior not tested; add --wire-tests. Hardware bus validation is separate.")
        return 0
    except (RuntimeError, OSError, subprocess.CalledProcessError) as error:
        print(f"FAIL: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
