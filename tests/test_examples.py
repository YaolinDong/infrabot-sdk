# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Yaolin Dong
"""Run the shipped example scripts on the native loopback boards."""
from pathlib import Path
import subprocess
import sys
import tempfile
import time

from _infrabot_test_mcu import FakeMcu, FakeLogBoard

examples = Path(__file__).resolve().parents[1] / "examples/python"
scripts = sorted(examples.glob("[0-9][0-9]_*.py"))
assert [p.name[:2] for p in scripts] == [f"{n:02}" for n in range(1, 9)]
for path in scripts:
    if path.name.startswith("07"):
        board = FakeLogBoard()
        with tempfile.TemporaryDirectory() as directory:
            subprocess.run([sys.executable, path, "--ip", "127.0.0.1", "--log-port",
                            str(board.port), "--directory", directory], check=True, timeout=10)
            text, = Path(directory).glob("*.log")
            assert "loopback example" in text.read_text()
    else:
        board = FakeMcu()
        board.telemetry_v3 = True
        board.calibrate()
        subprocess.run([sys.executable, path, "--ip", "127.0.0.2", "--command-port",
                        str(board.command_port), "--telemetry-port", str(board.telemetry_port)],
                       check=True, timeout=12)
        if path.name.startswith(("01", "08")):
            assert board.enabled_mask == 0
        else:
            assert board.mode == 0 and board.enabled_mask == 127
    del board


# Isolate DMIO from DMTL. Exercise the actual example with delayed telemetry,
# then verify that a silent telemetry channel times out before enabling.
for deliver in (True, False):
    board = FakeMcu()
    board.calibrate()
    board.telemetry_v3 = True
    board.drop_telemetry = True
    command = [sys.executable, examples / "04_rt_joint_stream_1khz.py", "--ip", "127.0.0.2",
               "--command-port", str(board.command_port), "--telemetry-port", str(board.telemetry_port)]
    with subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True) as child:
        try:
            end = time.monotonic() + 3
            while board.info_count == 0 and time.monotonic() < end:
                time.sleep(0.005)
            assert board.info_count > 0, "example never queried controller info"
            if deliver:
                time.sleep(0.05)
                board.drop_telemetry = False
            _, stderr = child.communicate(timeout=5)
            if deliver:
                assert child.returncode == 0 and board.enabled_mask == 127, stderr
            else:
                assert child.returncode != 0 and "No fresh telemetry before drive setup" in stderr, stderr
                assert board.enabled_mask == 0
        finally:
            if child.poll() is None:
                child.kill()
                child.wait()
    del board
print("Delayed first telemetry accepted; missing first telemetry left drives disabled.")
