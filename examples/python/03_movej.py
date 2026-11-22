# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Yaolin Dong
# Before enabling the arm, clear its workspace and verify the hardware emergency stop.
import infrabot
from example_common import arguments, connected


if __name__ == "__main__":
    with connected(arguments(), enable_arm=True) as robot:
        robot.switch_mode(infrabot.Mode.NrtJoint)
        # Start with the held pose. Joint positions are in radians.
        result = robot.movej(robot.hold_command()[:7])
        if result.result:
            raise RuntimeError(result.message)
