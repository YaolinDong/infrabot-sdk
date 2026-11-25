# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Yaolin Dong
"""Connection arguments and the enable/stop sequence shared by the examples."""
import argparse
import time
from contextlib import contextmanager

import infrabot


def arguments(*, log=False, extra=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", default="192.168.0.50")
    parser.add_argument("--command-port", type=int, default=5001)
    parser.add_argument("--telemetry-port", type=int, default=5000)
    if log:
        parser.add_argument("--directory", default="")
        parser.add_argument("--log-port", type=int, default=infrabot.McuLogOptions().port)
    if extra:
        extra(parser)
    return parser.parse_args()


@contextmanager
def connected(args, *, enable_arm=False):
    robot = infrabot.Robot(args.ip, args.command_port, args.telemetry_port)
    # A read-only session sends no heartbeat, so it never looks like the controlling PC.
    robot.set_heartbeat(enable_arm)
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
            robot.set_heartbeat(True)
            s = robot.state()
            arm_enabled = (s.enabled_mask & infrabot.ARM_MASK) == infrabot.ARM_MASK
            if s.enabled and not (arm_enabled and s.mode == infrabot.Mode.Idle and s.profile_ready
                                  and s.profile == infrabot.MotorProfile.Mit):
                raise RuntimeError("The arm is enabled outside IDLE/MIT or only partly: stop or disable it first")
            if not arm_enabled:
                # A previous example left the arm enabled and holding in IDLE: reuse it as it is.
                robot.set_motor_profile(infrabot.MotorProfile.Mit)
                robot.servo_on(infrabot.MotorGroup.Arm)
            # A mode switch needs a still arm. Right after a previous example stopped a local mode, the
            # hold is still settling for a moment.
            deadline = time.monotonic() + 3.0
            while max(abs(v) for v in robot.state().joints.velocity[:7]) > 0.05:
                if time.monotonic() >= deadline:
                    raise RuntimeError("The arm does not come to rest (above 0.05 rad/s): let go of it")
                time.sleep(0.01)
        try:
            yield robot
        finally:
            if enable_arm:
                robot.stop()  # controlled stop to enabled hold
    finally:
        robot.disconnect()
