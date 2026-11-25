# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Yaolin Dong
# Lists the controller's tunable parameters (gravity, impedance, admittance). The arm is not enabled and
# does not move. --set NAME VALUE writes one first: RAM only, the controller slews its applied value
# toward the requested one, and every parameter returns to its default at reboot.
import argparse

from example_common import connected


def arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", default="192.168.0.50")
    parser.add_argument("--command-port", type=int, default=5001)
    parser.add_argument("--telemetry-port", type=int, default=5000)
    parser.add_argument("--set", nargs=2, action="append", default=[], metavar=("NAME", "VALUE"))
    return parser.parse_args()


if __name__ == "__main__":
    args = arguments()
    with connected(args) as robot:
        for name, value in args.set:
            written = robot.set_parameter(name, float(value))
            print(f"set {written.name}: requested {written.requested:g}, applied {written.applied:g}")
        for p in robot.list_parameters():
            print(f"{p.name:<20} {p.requested:9.3f} (applied {p.applied:9.3f}) [{p.minimum:g}..{p.maximum:g}] {p.unit}")
