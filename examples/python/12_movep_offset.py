# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Yaolin Dong
# Before enabling the arm, clear its workspace and verify the hardware emergency stop.
# NRT Cartesian move: the tool goes --dz m along the base z axis in a straight line, then back.
# The start pose is the forward kinematics of the held joints. Default +0.02 m at 0.02 m/s.
# MIT holds by stiffness alone: without gravity feed-forward the tool stays about 10 mm short of its
# target. The arrival tolerance below allows for that; the printed error is what the arm actually reached.
import infrabot
from example_common import arguments, connected


def extra(parser):
    parser.add_argument("--dz", type=float, default=0.02)
    parser.add_argument("--speed", type=float, default=0.02, help="m/s")
    parser.add_argument("--tolerance", type=float, default=0.03, help="m and rad, at most 0.05")


def check(result):
    if result.result:
        raise RuntimeError(result.message)


if __name__ == "__main__":
    args = arguments(extra=extra)
    with connected(args, enable_arm=True) as robot:
        robot.switch_mode(infrabot.Mode.NrtCartesian)
        start = infrabot.Robot.forward(robot.hold_command()[:7])
        target = list(start)
        target[2] += args.dz
        options = infrabot.CartesianMoveOptions()
        options.limits.linear_velocity = args.speed
        options.limits.angular_velocity = 0.2
        options.position_tolerance = args.tolerance
        result = robot.movep(target, options)
        check(result)
        print(f"tool at z = {target[2]:.3f} m, error {result.max_error_rad * 1000:.1f} mm/mrad")
        result = robot.movep(start, options)
        check(result)
        print(f"back at the start, error {result.max_error_rad * 1000:.1f} mm/mrad")
