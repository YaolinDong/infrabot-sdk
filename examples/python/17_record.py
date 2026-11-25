# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Yaolin Dong
# Read only: records the 1 kHz telemetry, the commands sent and the events as CSV for a few seconds.
# Nothing is enabled. The files go to --directory (default: the SDK log directory).
import time

import infrabot
from example_common import arguments, connected


def extra(parser):
    parser.add_argument("--directory", default=infrabot.Robot.log_directory())
    parser.add_argument("--seconds", type=float, default=1.0)


if __name__ == "__main__":
    args = arguments(extra=extra)
    with connected(args) as robot:
        options = infrabot.RecordOptions()
        options.label = "example"
        robot.start_recording(args.directory, options)
        robot.record_event("example marker")
        time.sleep(args.seconds)
        robot.stop_recording()
        stats = robot.recording()
        print(f"{stats.directory}: {stats.telemetry_rows} telemetry rows, {stats.events} events, "
              f"{stats.dropped} dropped")
        if stats.telemetry_rows == 0:
            raise RuntimeError("no telemetry was recorded")
