# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Yaolin Dong
# Before enabling the arm, clear its workspace and verify the hardware emergency stop.
# Local compliance modes run on the MCU at 1 kHz without a PC stream. Push the arm by hand while it runs.
# STOP at the end brakes it and holds it in IDLE. Gains: see 08_parameters.
import time

import infrabot
from example_common import arguments, connected

MODES = {
    "gravity-free": infrabot.Mode.GravityFree,
    "gravity-lock": infrabot.Mode.GravityLock,
    "joint-impedance": infrabot.Mode.JointImpedance,
    "cartesian-impedance": infrabot.Mode.CartesianImpedance,
    "joint-admittance": infrabot.Mode.JointAdmittance,
    "cartesian-admittance": infrabot.Mode.CartesianAdmittance,
}


def extra(parser):
    parser.add_argument("--mode", choices=MODES, default="cartesian-impedance")
    parser.add_argument("--seconds", type=float, default=1.0)


if __name__ == "__main__":
    args = arguments(extra=extra)
    with connected(args, enable_arm=True) as robot:
        robot.switch_mode(MODES[args.mode])
        print(f"{args.mode} for {args.seconds:g} s")
        end = time.monotonic() + args.seconds
        while time.monotonic() < end:
            time.sleep(0.5)
            j = robot.state().joints
            print(" ".join(f"{q:+.2f}" for q in j.position[:7]))
