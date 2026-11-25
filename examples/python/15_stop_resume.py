# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Yaolin Dong
# Before enabling the arm, clear its workspace and verify the hardware emergency stop.
# Controlled stop and resume: STOP brakes the arm and holds it enabled in IDLE; resume() re-enters the
# mode the stop interrupted, from where the arm stopped. The arm does not move.
import infrabot
from example_common import arguments, connected


if __name__ == "__main__":
    with connected(arguments(), enable_arm=True) as robot:
        robot.switch_mode(infrabot.Mode.NrtJoint)
        robot.stop()
        s = robot.state()
        print(f"stopped: mode {s.mode.name}, reason {s.safety.stop_reason.name}, "
              f"interrupted {s.safety.stop_mode.name}, enabled 0x{s.enabled_mask:02x}")
        print(f"resumed {robot.resume().name}")
