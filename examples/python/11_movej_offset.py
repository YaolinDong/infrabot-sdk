# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Yaolin Dong
# Before enabling the arm, clear its workspace and verify the hardware emergency stop.
# NRT joint move: one joint goes by --delta rad from where the arm holds, then back. Default: J7 +0.2 rad.
# MIT holds by stiffness alone: without gravity feed-forward a loaded joint stays 10-30 mrad short of its
# target, and a light one (J7) can stall a little short on friction. The arrival tolerance below allows for
# that; the printed error is what the arm actually reached.
import infrabot
from example_common import arguments, connected


def extra(parser):
    parser.add_argument("--joint", type=int, default=7, choices=range(1, 8))
    parser.add_argument("--delta", type=float, default=0.2)
    parser.add_argument("--tolerance", type=float, default=0.04, help="rad, at most 0.05")


def check(result):
    if result.result:
        raise RuntimeError(result.message)


if __name__ == "__main__":
    args = arguments(extra=extra)
    with connected(args, enable_arm=True) as robot:
        robot.switch_mode(infrabot.Mode.NrtJoint)
        start = robot.hold_command()[:7]
        target = list(start)
        target[args.joint - 1] += args.delta
        options = infrabot.MoveOptions()  # 0.1 rad/s, 0.2 rad/s^2 per joint
        options.position_tolerance = args.tolerance
        result = robot.movej(target, options)
        check(result)
        print(f"J{args.joint} at {robot.state().joints.position[args.joint - 1]:+.3f} rad, "
              f"largest joint error {result.max_error_rad * 1000:.0f} mrad")
        result = robot.movej(start, options)
        check(result)
        print(f"back at the start, largest joint error {result.max_error_rad * 1000:.0f} mrad")
