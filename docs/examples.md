# Examples

Every example exists in C++ (`examples/cpp`, built with `-DINFRABOT_BUILD_EXAMPLES=ON`) and Python (`examples/python`). C++ takes the controller IP as the first argument; Python takes `--ip`. The default IP is `192.168.0.50`. `ctest` runs all of them on a loopback board (`python_examples_loopback`), except the C++ `07`, which needs the fixed log port of a real board.

Read-only examples send no heartbeat and enable nothing. Every example still registers for telemetry when it connects, and the controller sends telemetry to one receiver: on the PC that controls the arm the newest registration wins, so do not run an example on that PC while another program there controls the arm, or that program loses its feedback. From a second PC the registration is refused while the arm is controlled; `08` reads and `09` still work, since they use the command port. The others enable J1–J7, end with a controlled stop and leave the arm enabled and holding in IDLE; disable it from the touchscreen or with `servo_off` when done. Before any of them, clear the workspace, support the arm and keep the hardware emergency stop at hand.

| # | Name | What it shows | Drives |
| --- | --- | --- | --- |
| 01 | `connect_info` | Handshake, product and firmware | read only |
| 02 | `enable_hold` | Profile, self-check, enable, hold 1 s | enables the arm, does not move |
| 03 | `movej` | NRT joint move to the held pose | enables the arm, does not move |
| 04 | `rt_joint_stream_1khz` | Native 1 kHz joint stream at the held pose | enables the arm, does not move |
| 05 | `gravity_compensation` | Local GRAVITY_LOCK for 1 s | enables the arm |
| 06 | `joint_impedance` | Local joint impedance for 1 s | enables the arm |
| 07 | `download_mcu_log` | MCU run log (text and 10 s of telemetry) | read only |
| 08 | `parameters` | List, read and write controller parameters | read only; a write needs control |
| 09 | `diagnostics` | Loop timing, CAN, memory, drive states | read only |
| 10 | `monitor` | State, joints, link quality, fault log | read only |
| 11 | `movej_offset` | One joint out by a small angle and back (default J7 +0.2 rad) | **moves** |
| 12 | `movep_offset` | Tool up along z in a straight line and back (default 2 cm) | **moves** |
| 13 | `rt_cartesian_stream` | 100 Hz `servep` up and back on a cosine (default 1 cm) | **moves** |
| 14 | `compliance_modes` | Any local gravity, impedance or admittance mode | enables the arm, push it by hand |
| 15 | `stop_resume` | Controlled stop, then `resume()` | enables the arm, does not move |
| 16 | `gripper` | Enable only J8, move it and back | **moves the gripper**, leaves it enabled |
| 17 | `record` | 1 kHz telemetry, commands and events as CSV | read only |

Arguments beyond the IP (C++ positional, Python options):

| # | C++ | Python |
| --- | --- | --- |
| 08 | `[IP [NAME VALUE]]` | `--set NAME VALUE` |
| 10 | `[IP [SECONDS]]` | `--seconds` |
| 11 | `[IP [JOINT DELTA]]` | `--joint`, `--delta` |
| 12 | `[IP [DZ]]` | `--dz`, `--speed` |
| 13 | `[IP [DZ SECONDS]]` | `--dz`, `--seconds` |
| 14 | `[IP [MODE SECONDS]]` | `--mode`, `--seconds` |
| 16 | `[IP [POSITION SPEED FORCE]]` | `--position`, `--speed`, `--force` |
| 17 | `[IP [DIRECTORY SECONDS]]` | `--directory`, `--seconds` |

`14` modes: `gravity-free`, `gravity-lock`, `joint-impedance`, `cartesian-impedance` (default), `joint-admittance`, `cartesian-admittance`. The admittance modes start only with every joint at least 0.03 rad inside its limit. J2 and J4 have their lower limit at 0, so in the usual folded pose raise them first, for example with `11` (`--joint 2 --delta 0.1`).

To run the C++ examples against a loopback board on other ports, set `INFRABOT_EXAMPLE_COMMAND_PORT` and `INFRABOT_EXAMPLE_TELEMETRY_PORT`.
