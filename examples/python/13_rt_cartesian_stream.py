# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Yaolin Dong
# Before enabling the arm, clear its workspace and verify the hardware emergency stop.
# RT Cartesian stream: servep at 100 Hz (the MCU interpolates at 1 kHz and stops after a 100 ms gap).
# The target rises --dz m along z and comes back on a smooth cosine over --seconds.
import math
import time

import infrabot
from example_common import arguments, connected


def extra(parser):
    parser.add_argument("--dz", type=float, default=0.01)
    parser.add_argument("--seconds", type=float, default=4.0)


if __name__ == "__main__":
    args = arguments(extra=extra)
    with connected(args, enable_arm=True) as robot:
        robot.switch_mode(infrabot.Mode.RtCartesian)
        start = infrabot.Robot.forward(robot.hold_command()[:7])
        command = infrabot.CartesianCommand()
        command.limits.linear_velocity = 0.05
        command.limits.angular_velocity = 0.3
        pose = list(start)
        t0 = time.monotonic()
        while (t := time.monotonic() - t0) < args.seconds:
            pose[2] = start[2] + args.dz * 0.5 * (1 - math.cos(2 * math.pi * t / args.seconds))
            command.pose = pose
            robot.servep(command)
            time.sleep(0.01)
        command.pose = start
        robot.servep(command)
        print("stream ended at the start pose")
