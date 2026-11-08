"""Exercise the OpenOCD read/export path with a mock, without hardware."""
import csv
from pathlib import Path
import socket
import struct
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
import export_gm6020_trace as export


class OpenOcd:
    def __init__(self, version=0x474D0001, moving=False):
        self.buffer = b"Open On-Chip Debugger\r\n> "
        self.count_reads = 0
        self.moving = moving
        self.memory = {0x1000: version, 0x1004: 2}
        samples = [(0, 10, 1, -0.1, -0.2, -0.03, 0.04),
                   (4, 14, 0, 0.2, 0.3, 0.04, -0.05)]
        self.raw = b"".join(struct.pack("<IIIffff", *row) for row in samples)
        for i, (word,) in enumerate(struct.iter_unpack("<I", self.raw)):
            self.memory[0x2000 + i * 4] = word

    def __enter__(self): return self
    def __exit__(self, *args): pass
    def settimeout(self, value): pass

    def sendall(self, command):
        parts = command.decode().split()
        if parts[0] != "mdw":
            raise AssertionError("Only read-memory commands are allowed")
        address, count = int(parts[1], 16), int(parts[2])
        if address == 0x1004:
            self.count_reads += 1
            if self.moving and self.count_reads > 1:
                self.memory[address] = 3
        values = " ".join(f"{self.memory[address + i * 4]:08x}" for i in range(count))
        self.buffer += command + f"\x000x{address:08x}: {values}\r\n> ".encode()

    def recv(self, maximum):
        if not self.buffer:
            raise socket.timeout()
        # Deliberately fragment words across reads like a real TCP connection.
        part, self.buffer = self.buffer[:17], self.buffer[17:]
        return part


class TraceExportTests(unittest.TestCase):
    def run_export(self, connection, output):
        symbols = ("00001000 D g_gm6020_trace_version\n"
                   "00001004 B g_gm6020_trace_count\n"
                   "00002000 B g_gm6020_trace\n")
        with patch.object(export.socket, "create_connection", return_value=connection), \
             patch.object(export.subprocess, "check_output", return_value=symbols), \
             patch.object(sys, "argv", ["export", "--out", str(output)]):
            export.main()

    def test_signed_values_and_csv_columns(self):
        with tempfile.TemporaryDirectory(dir=ROOT / "build/tests-gm6020") as temp:
            output = Path(temp) / "trace.csv"
            self.run_export(OpenOcd(), output)
            with output.open(newline="", encoding="utf-8") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), 2)
            self.assertEqual(rows[1]["time_ms"], "4")
            self.assertAlmostEqual(float(rows[0]["actual_current_a"]), -0.03, places=6)
            self.assertAlmostEqual(float(rows[1]["previous_command_current_a"]), -0.05, places=6)

    def test_wrong_firmware_is_rejected(self):
        with tempfile.TemporaryDirectory(dir=ROOT / "build/tests-gm6020") as temp:
            output = Path(temp) / "trace.csv"
            with self.assertRaisesRegex(RuntimeError, "version mismatch"):
                self.run_export(OpenOcd(version=0), output)
            self.assertFalse(output.exists())

    def test_changing_trace_is_rejected(self):
        with tempfile.TemporaryDirectory(dir=ROOT / "build/tests-gm6020") as temp:
            output = Path(temp) / "trace.csv"
            with self.assertRaisesRegex(RuntimeError, "still changing"):
                self.run_export(OpenOcd(moving=True), output)
            self.assertFalse(output.exists())


if __name__ == "__main__":
    unittest.main()
