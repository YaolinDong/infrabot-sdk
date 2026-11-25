# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Yaolin Dong
# Keep fingers and objects clear of the gripper.
# Enables only the J8 gripper group (the arm stays as it is), moves it to --position and back to where it
# started. The force is a current limit as a fraction of the drive maximum. It stays enabled and holding.
import time

import infrabot
from example_common import arguments, connected


def extra(parser):
    parser.add_argument("--position", type=float, default=1.0, help="rad, 0-5.94")
    parser.add_argument("--speed", type=float, default=1.0, help="rad/s")
    parser.add_argument("--force", type=float, default=0.3, help="0-1")


if __name__ == "__main__":
    args = arguments(extra=extra)
    with connected(args) as robot:
        robot.set_heartbeat(True)
        deadline = time.monotonic() + 0.5
        while robot.state().telemetry_age_ms > 100:
            if time.monotonic() >= deadline:
                raise TimeoutError("No fresh telemetry before drive setup")
            time.sleep(0.005)
        s = robot.state()
        if s.joints.feedback_age_ms[7] > 1000:
            raise RuntimeError("J8 sends no feedback: no gripper connected or powered, nothing enabled")
        if not s.profile_ready:
            robot.set_motor_profile(infrabot.MotorProfile.Mit)
        if not s.enabled_mask & 0x80:
            robot.servo_on(infrabot.MotorGroup.Gripper)
        start = robot.state().joints.position[7]
        here = start
        for target in (args.position, start):
            robot.gripper(target, args.speed, args.force)
            time.sleep(abs(target - here) / args.speed + 0.5)
            here = robot.state().joints.position[7]
            print(f"gripper at {here:.3f} rad (target {target:.3f})")
