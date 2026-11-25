# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Yaolin Dong
"""Run the shipped example scripts, and the C++ example programs when their directory is given in
INFRABOT_EXAMPLE_BIN, on the native loopback boards."""
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time

from _infrabot_test_mcu import FakeMcu, FakeLogBoard

COUNT = 17
examples = Path(__file__).resolve().parents[1] / "examples/python"
scripts = sorted(examples.glob("[0-9][0-9]_*.py"))
assert [p.name[:2] for p in scripts] == [f"{n:02}" for n in range(1, COUNT + 1)]
binaries = Path(os.environ.get("INFRABOT_EXAMPLE_BIN", ""))
programs = sorted(binaries.glob("[0-9][0-9]_*")) if binaries.parts else []
programs = [p for p in programs if p.is_file() and os.access(p, os.X_OK)]
if binaries.parts:
    assert [p.name for p in programs] == [p.stem for p in scripts], programs

# Examples that leave every drive as they found it (disabled). 16 enables only the gripper.
READ_ONLY = ("01", "07", "08", "09", "10", "17")


def run_example(path, python):
    number = path.name[:2]
    if number == "07":
        board = FakeLogBoard()
        with tempfile.TemporaryDirectory() as directory:
            if python:
                command = [sys.executable, path, "--ip", "127.0.0.1", "--log-port", str(board.port),
                           "--directory", directory]
                subprocess.run(command, check=True, timeout=10)
                text, = Path(directory).glob("*.log")
                assert "loopback example" in text.read_text()
        return  # the C++ 07 uses the fixed log port of a real board
    board = FakeMcu()
    board.telemetry_v3 = True
    board.supports_cartesian = True
    board.calibrate()
    with tempfile.TemporaryDirectory() as directory:
        extra = {"17": ["--directory", directory]}.get(number, []) if python else \
                {"17": [directory, "0.5"], "10": ["0.5"]}.get(number, [])
        if python:
            command = [sys.executable, path, "--ip", "127.0.0.2", "--command-port", str(board.command_port),
                       "--telemetry-port", str(board.telemetry_port)] + extra
            env = None
        else:
            command = [path, "127.0.0.2"] + extra
            env = dict(os.environ, INFRABOT_EXAMPLE_COMMAND_PORT=str(board.command_port),
                       INFRABOT_EXAMPLE_TELEMETRY_PORT=str(board.telemetry_port))
        subprocess.run(command, check=True, timeout=15, env=env, stdout=subprocess.DEVNULL)
        if number == "17":
            assert list(Path(directory).rglob("*.csv")), "no recording written"
    if number in READ_ONLY:
        assert board.enabled_mask == 0, (path, board.enabled_mask)
        assert board.heartbeat_count == 0, (path, "a read-only example sent heartbeats")
    elif number == "16":
        assert board.enabled_mask == 0x80 and board.gripper_count == 2, (path, board.enabled_mask)
    else:
        assert board.mode == 0 and board.enabled_mask == 127, (path, board.mode, board.enabled_mask)
    if number in ("11", "12"):
        assert board.motion_count >= 3, (path, board.motion_count)  # the hold, out and back
    if number == "13":
        assert board.cartesian_count > 10, (path, board.cartesian_count)
    del board


for path in scripts:
    run_example(path, python=True)
for path in programs:
    run_example(path, python=False)
print(f"{len(scripts)} Python and {len(programs)} C++ examples ran on the loopback board.")

# Back to back on one board: an enabling example leaves the arm enabled and holding in IDLE, and the next
# one reuses it instead of failing to apply the motor profile (which needs every drive disabled).
board = FakeMcu()
board.telemetry_v3 = True
board.calibrate()
ports = ["--ip", "127.0.0.2", "--command-port", str(board.command_port), "--telemetry-port", str(board.telemetry_port)]
for name in ("02_enable_hold", "11_movej_offset", "15_stop_resume"):
    subprocess.run([sys.executable, examples / f"{name}.py"] + ports, check=True, timeout=15, stdout=subprocess.DEVNULL)
    assert board.mode == 0 and board.enabled_mask == 127, (name, board.mode, board.enabled_mask)
env = dict(os.environ, INFRABOT_EXAMPLE_COMMAND_PORT=str(board.command_port),
           INFRABOT_EXAMPLE_TELEMETRY_PORT=str(board.telemetry_port))
for name in ("11_movej_offset",) if programs else ():
    subprocess.run([binaries / name, "127.0.0.2"], check=True, timeout=15, env=env, stdout=subprocess.DEVNULL)
    assert board.mode == 0 and board.enabled_mask == 127, (name, board.mode, board.enabled_mask)
assert board.disable_count == 0
del board
print("Enabling examples run back to back reuse the enabled arm.")


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
