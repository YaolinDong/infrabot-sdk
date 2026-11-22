# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Yaolin Dong
# Before enabling the arm, clear its workspace and verify the hardware emergency stop.
import time
from example_common import arguments, connected


if __name__ == "__main__":
    with connected(arguments(), enable_arm=True) as robot:
        print("Arm enabled in IDLE hold")
        time.sleep(1)
