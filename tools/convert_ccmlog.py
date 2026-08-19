#!/usr/bin/env python3
"""Convert CCMLOG2 MicroSquirt telemetry into CSV or candump text.

Interrupted .recover files are supported: parsing stops at the first partial or
CRC-invalid record and every earlier record remains usable.
"""

from __future__ import annotations

import argparse
import csv
import struct
import sys
import zlib
from pathlib import Path

HEADER_SIZE = 512
RECORD_SIZE = 32
HEADER_MAGIC = b"CCMLOG2\0"
RECORD_MAGIC = 0xC352
HEADER = struct.Struct("<8sHHIHHIHBBI")
RECORD = struct.Struct("<HBBIIHBB8sII")


def crc32(data: bytes) -> int:
    return zlib.crc32(data) & 0xFFFFFFFF


def signed16(data: bytes, offset: int) -> int:
    return int.from_bytes(data[offset : offset + 2], "big", signed=True)


def unsigned16(data: bytes, offset: int) -> int:
    return int.from_bytes(data[offset : offset + 2], "big", signed=False)


def decode_core(can_id: int, data: bytes, base: int, state: dict[str, float]) -> None:
    if can_id == 0x5E8:
        state.update(map_kpa=signed16(data, 0) / 10, rpm=unsigned16(data, 2),
                     clt_f=signed16(data, 4) / 10, tps_pct=signed16(data, 6) / 10)
    elif can_id == 0x5E9:
        state.update(pw1_ms=unsigned16(data, 0) / 1000, pw2_ms=unsigned16(data, 2) / 1000,
                     mat_f=signed16(data, 4) / 10, spark_deg=signed16(data, 6) / 10)
    elif can_id == 0x5EA:
        state.update(afr_target=data[0] / 10, afr1=data[1] / 10)
    elif can_id == 0x5EB:
        state.update(battery_v=signed16(data, 0) / 10)
    elif can_id == base:
        state.update(pw1_ms=unsigned16(data, 2) / 1000, pw2_ms=unsigned16(data, 4) / 1000,
                     rpm=unsigned16(data, 6))
    elif can_id == base + 1:
        state.update(spark_deg=signed16(data, 0) / 10, afr_target=data[4] / 10)
    elif can_id == base + 2:
        state.update(baro_kpa=signed16(data, 0) / 10, map_kpa=signed16(data, 2) / 10,
                     mat_f=signed16(data, 4) / 10, clt_f=signed16(data, 6) / 10)
    elif can_id == base + 3:
        state.update(tps_pct=signed16(data, 0) / 10, battery_v=signed16(data, 2) / 10,
                     afr1=signed16(data, 4) / 10, afr2=signed16(data, 6) / 10)
    if "map_kpa" in state and "baro_kpa" in state:
        state["boost_kpa"] = state["map_kpa"] - state["baro_kpa"]


def records(path: Path):
    with path.open("rb") as handle:
        header_raw = handle.read(HEADER_SIZE)
        if len(header_raw) != HEADER_SIZE or header_raw[:8] != HEADER_MAGIC:
            raise ValueError("not a CCMLOG2 file or header is incomplete")
        expected = int.from_bytes(header_raw[508:512], "little")
        if crc32(header_raw[:508]) != expected:
            raise ValueError("segment header CRC is invalid")
        fields = HEADER.unpack_from(header_raw)
        realtime_base = fields[7]
        while True:
            raw = handle.read(RECORD_SIZE)
            if not raw:
                return realtime_base
            if len(raw) != RECORD_SIZE:
                print(f"warning: ignored {len(raw)}-byte partial tail", file=sys.stderr)
                return realtime_base
            values = RECORD.unpack(raw)
            if values[0] != RECORD_MAGIC or values[1] != 2 or crc32(raw[:28]) != values[-1]:
                print("warning: stopped at first invalid record CRC", file=sys.stderr)
                return realtime_base
            yield realtime_base, values


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--format", choices=("raw", "decoded", "candump"), default="decoded")
    args = parser.parse_args()

    state: dict[str, float] = {}
    decoded_fields = ["rpm", "map_kpa", "baro_kpa", "boost_kpa", "clt_f", "mat_f",
                      "tps_pct", "battery_v", "afr1", "afr2", "afr_target",
                      "spark_deg", "pw1_ms", "pw2_ms"]
    with args.output.open("w", newline="", encoding="utf-8") as out:
        if args.format == "candump":
            writer = None
        else:
            columns = ["timestamp_ms", "sequence", "can_id", "dlc", "data", "dropped_before"]
            if args.format == "decoded":
                columns += decoded_fields
            writer = csv.DictWriter(out, fieldnames=columns)
            writer.writeheader()

        for base, values in records(args.input):
            _, _, record_type, sequence, timestamp_ms, can_id, dlc, _, payload, dropped, _ = values
            if record_type == 0xFE:
                continue
            payload = payload[:dlc]
            if args.format == "candump":
                out.write(f"({timestamp_ms / 1000:.3f}) can0 {can_id:03X}#{payload.hex().upper()}\n")
                continue
            row = {"timestamp_ms": timestamp_ms, "sequence": sequence,
                   "can_id": f"0x{can_id:03X}", "dlc": dlc,
                   "data": payload.hex().upper(), "dropped_before": dropped}
            if args.format == "decoded" and dlc == 8:
                decode_core(can_id, payload, base, state)
                row.update({key: state.get(key, "") for key in decoded_fields})
            writer.writerow(row)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
