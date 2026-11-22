# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Yaolin Dong
"""Connection arguments and the enable/stop sequence shared by the examples."""
import argparse
import time
from contextlib import contextmanager

import infrabot


def arguments(*, log=False):
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", default="192.168.0.50")
    parser.add_argument("--command-port", type=int, default=5001)
    parser.add_argument("--telemetry-port", type=int, default=5000)
    if log:
        parser.add_argument("--directory", default="")
        parser.add_argument("--log-port", type=int, default=infrabot.McuLogOptions().port)
    return parser.parse_args()


@contextmanager
def connected(args, *, enable_arm=False):
    robot = infrabot.Robot(args.ip, args.command_port, args.telemetry_port)
    robot.connect()
    try:
        if not robot.info().received:
            raise RuntimeError("Controller did not reply")
        if enable_arm:
            # DMIO can arrive before the first DMTL. Match the SDK's 100 ms freshness check.
            deadline = time.monotonic() + 0.5
            while robot.state().telemetry_age_ms > 100:
                if time.monotonic() >= deadline:
                    raise TimeoutError("No fresh telemetry before drive setup")
                time.sleep(0.005)
            robot.set_motor_profile(infrabot.MotorProfile.Mit)
            robot.set_heartbeat(True)
            robot.servo_on(infrabot.MotorGroup.Arm)
        try:
            yield robot
        finally:
            if enable_arm:
                robot.stop()  # controlled stop to enabled hold
    finally:
        robot.disconnect()
