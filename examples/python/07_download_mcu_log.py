# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Yaolin Dong
import infrabot
from example_common import arguments

if __name__ == "__main__":
    args = arguments(log=True)
    robot = infrabot.Robot(args.ip)
    options = infrabot.McuLogOptions()
    options.port = args.log_port
    options.directory = args.directory
    result = robot.download_mcu_log(options)
    print(result.text_path, result.telemetry_path, sep="\n")
