# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Yaolin Dong
# Read only: prints the controller state, every joint and the link quality for a few seconds, then the
# fault log. No heartbeat is sent and nothing is enabled. Connecting registers for telemetry, so do not run
# it on a PC where another program controls the arm (docs/examples.md).
import time

from example_common import arguments, connected


def extra(parser):
    parser.add_argument("--seconds", type=float, default=1.0)


if __name__ == "__main__":
    args = arguments(extra=extra)
    with connected(args) as robot:
        # The first telemetry packet follows the handshake.
        first = time.monotonic() + 0.5
        while robot.state().telemetry_age_ms > 100:
            if time.monotonic() >= first:
                raise TimeoutError("No telemetry")
            time.sleep(0.005)
        end = time.monotonic() + args.seconds
        while True:
            s = robot.state()
            print(f"{s.operational_status.name} mode={s.mode.name} enabled=0x{s.enabled_mask:02x} "
                  f"telemetry age {s.telemetry_age_ms} ms")
            for i in range(8):
                j = s.joints
                print(f"  J{i + 1} q={j.position[i]:+.3f} rad dq={j.velocity[i]:+.3f} rad/s "
                      f"tau={j.torque[i]:+.2f} Nm drive={j.drive_state[i]} {j.temperature[i]} C")
            if time.monotonic() >= end:
                break
            time.sleep(0.5)
        link = robot.link_stats()
        print(f"link: {link.rx_last_second} packets/s, lost {link.lost_last_second}, "
              f"longest gap {link.rx_interval_max_us} us")
        for event in robot.fault_log():
            print(f"fault {event.kind.name} J{event.joint + 1}: {event.message}")
