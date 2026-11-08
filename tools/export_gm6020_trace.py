"""Read a completed identification trace through local OpenOCD, without moving/halt.

Run after identification exits, before resetting or starting another test.
The ELF must match the firmware currently loaded on the board.
"""
import argparse
import csv
from pathlib import Path
import re
import shutil
import socket
import struct
import subprocess
import time


def read_words(connection, address, count):
    connection.sendall(f"mdw 0x{address:x} {count}\n".encode("ascii"))
    data = b""
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        try:
            part = connection.recv(16384)
        except socket.timeout:
            continue
        if not part:
            raise RuntimeError("OpenOCD connection closed")
        data += part
        words = {}
        for at, values in re.findall(
            rb"(0x[0-9a-fA-F]+):\s*((?:[0-9a-fA-F]{8}\s*)+)", data
        ):
            for i, value in enumerate(values.split()):
                words[int(at, 16) + 4 * i] = int(value, 16)
        if all(address + 4 * i in words for i in range(count)):
            return [words[address + 4 * i] for i in range(count)]
    raise RuntimeError(f"Cannot read board memory at 0x{address:x}")


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--elf", type=Path, default=root / "build/Debug/h7_framework.elf")
    parser.add_argument("--out", type=Path, default=root / "build/Debug/gm6020-trace.csv")
    parser.add_argument("--port", type=int, default=4444)
    parser.add_argument("--nm", default=shutil.which("arm-none-eabi-nm") or
                        "C:/ST/STM32CubeCLT_1.18.0/GNU-tools-for-STM32/bin/arm-none-eabi-nm.exe")
    args = parser.parse_args()
    output = subprocess.check_output([args.nm, "-g", str(args.elf)], text=True)
    symbols = {name: int(address, 16) for address, name in re.findall(
        r"^([0-9a-fA-F]+)\s+\w\s+(g_gm6020_\w+)$", output, re.MULTILINE)}
    for name in ("g_gm6020_trace", "g_gm6020_trace_count", "g_gm6020_trace_version"):
        if name not in symbols:
            raise RuntimeError(f"{name} missing: use the newly built ELF")
    with socket.create_connection(("127.0.0.1", args.port), timeout=3) as connection:
        connection.settimeout(0.2)
        try:
            while connection.recv(4096):
                pass  # Drain telnet negotiation/banner before the first command.
        except socket.timeout:
            pass
        if read_words(connection, symbols["g_gm6020_trace_version"], 1)[0] != 0x474D0001:
            raise RuntimeError("Trace version mismatch: download the matching firmware first")
        count_address = symbols["g_gm6020_trace_count"]
        count = read_words(connection, count_address, 1)[0]
        if not 0 < count <= 6400:
            raise RuntimeError(f"No valid completed trace (sample count {count})")
        words = []
        for offset in range(0, count * 7, 256):
            words.extend(read_words(connection, symbols["g_gm6020_trace"] + offset * 4,
                                    min(256, count * 7 - offset)))
        if read_words(connection, count_address, 1)[0] != count:
            raise RuntimeError("Trace is still changing: wait until identification exits")
    raw = struct.pack(f"<{len(words)}I", *words)
    rows = list(struct.iter_unpack("<IIIffff", raw))
    if any(rows[i][0] <= rows[i - 1][0] for i in range(1, len(rows))):
        raise RuntimeError("Trace times are invalid: check ELF/firmware match")
    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["time_ms", "rx_count", "feedback_age_ms", "position_rad",
                         "omega_rad_s", "actual_current_a", "previous_command_current_a"])
        writer.writerows(rows)
    print(f"Exported {count} samples to {args.out}")


if __name__ == "__main__":
    main()
