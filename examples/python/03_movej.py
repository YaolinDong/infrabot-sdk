# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Yaolin Dong
# Before enabling the arm, clear its workspace and verify the hardware emergency stop.
import infrabot
from example_common import arguments, connected


if __name__ == "__main__":
    with connected(arguments(), enable_arm=True) as robot:
        robot.switch_mode(infrabot.Mode.NrtJoint)
        # Start with the held pose. Joint positions are in radians.
        # MIT holds by stiffness alone and sags below its command by tens of mrad.
        options = infrabot.MoveOptions()
        options.position_tolerance = 0.04
        result = robot.movej(robot.hold_command()[:7], options)
        if result.result:
            raise RuntimeError(result.message)
