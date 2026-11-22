# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Yaolin Dong
from example_common import arguments, connected


if __name__ == "__main__":
    with connected(arguments()) as robot:
        print(robot.info().product, robot.info().firmware)
