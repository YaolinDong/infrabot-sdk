# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Yaolin Dong
# Before enabling the arm, clear its workspace and verify the hardware emergency stop.
import time
import infrabot
from example_common import arguments, connected


if __name__ == "__main__":
    with connected(arguments(), enable_arm=True) as robot:
        robot.switch_mode(infrabot.Mode.RtJoint)
        command = infrabot.JointCommand()
        command.position = robot.hold_command()[:7]
        robot.start_joint_stream(command)
        try:
            # Native pump sends at 1 kHz. Refresh the held target at 100 Hz.
            for _ in range(100):
                robot.servej(command)
                time.sleep(0.01)
        finally:
            robot.stop_joint_stream()
