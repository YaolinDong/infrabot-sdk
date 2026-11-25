# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Yaolin Dong
# Prints one diagnostic snapshot: loop timing, CAN and command counters, memory, drive states. Read only:
# any PC may ask, even while another one controls the arm. The arm is not enabled and does not move.
import argparse

from example_common import connected


def arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", default="192.168.0.50")
    parser.add_argument("--command-port", type=int, default=5001)
    parser.add_argument("--telemetry-port", type=int, default=5000)
    return parser.parse_args()


if __name__ == "__main__":
    args = arguments()
    with connected(args) as robot:
        d = robot.diagnostics()
        print(f"uptime {d.uptime_ms / 1000:.1f} s, controller status {d.controller_status}")
        print(f"last second: control {d.control_hz} Hz, missed {d.control_missed}, "
              f"period max {d.control_period_max_us} us")
        print(f"  CAN tx {d.can_tx} rx {d.can_rx} tx errors {d.can_tx_errors}; "
              f"commands {d.command_rx}, invalid {d.command_invalid}; telemetry {d.telemetry_tx}")
        print(f"since boot: missed {d.control_missed_total}, CAN tx errors {d.can_tx_errors_total}, "
              f"CAN error events {d.can_error_events}, invalid commands {d.command_invalid_total} "
              f"(last reason {d.invalid_reason})")
        flags = (" error-passive" if d.can_error_passive else "") + (" BUS-OFF" if d.can_bus_off else "")
        print(f"CAN TEC {d.can_tec} REC {d.can_rec}{flags}")
        print(f"heap free {d.heap_free_bytes} B (lowest {d.heap_min_free_bytes} B); least stack free: "
              f"control {d.stack_free_control_bytes} B, status {d.stack_free_status_bytes} B, "
              f"ethernet {d.stack_free_ethernet_bytes} B")
        print("drive states:", " ".join(f"J{i + 1}={s}" for i, s in enumerate(d.drive_state)))
