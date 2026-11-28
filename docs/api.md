# Robot API

Create one `infrabot::Robot` (Python `infrabot.Robot`) per controller. Defaults are `192.168.0.50` and command/telemetry ports 5001/5000. The constructor configures the local object. `connect` performs discovery and starts the receiver and service threads. For a monitoring session call `set_heartbeat(false)` before connecting. The board routes telemetry to the registered receiver. [API reference](https://yaolindong.github.io/infrabot-sdk/) for every function and type ([build it locally](reference/README.md)) · [Public C++ declarations](../include/infrabot/robot.hpp) · [Python signatures](../python/infrabot/infrabot_native.pyi)

## Timing

| Class | Typical calls | Where to use it |
| --- | --- | --- |
| RT | `state`, `servej`, `servep`, low-level `send_command` / `send_joint_stream` / `send_cartesian_stream` | The periodic target loop. Successful calls use a snapshot, publish a target or send a datagram without waiting for a firmware reply. |
| Non-blocking | `info`, `link_stats`, `fault_log`, `hold_command`, `submit_movej`, `submit_movep`, `motion`, `joint_stream_stats` | Supervisory code. Returning without waiting for motion completion can still involve locks, copying or allocation. |
| Blocking | `connect`, `disconnect`, `set_motor_profile`, `servo_on`, `switch_mode`, `movej`, `movep`, `wait_motion`, maintenance, parameters, log download | Setup, operator actions and teardown. These can wait for a peer, disk I/O or a worker thread. |

Use only `[RT]` calls in a 1 kHz loop. These labels describe API behavior; OS scheduling and UDP delivery still affect timing. C++ reports most errors through a return value and an error string. Check both the result code and error for blocking moves. Python raises `RuntimeError` for SDK errors; a motion's `result` field reports the controller outcome.

## Read state

- `info()` contains protocol and firmware identity, capabilities and drive ranges. `state().connected` indicates an open connection. Use `telemetry_age_ms`, per-drive feedback age and sample sequence/timestamps for freshness.
- `joints.host_time_us` is the host's monotonic timestamp when the SDK received the sample. Reading it repeatedly gives the timestamp of that same packet. `joints.feedback_age_ms[i]` measures how long it has been since the individual drive's feedback counter advanced; it is `UINT32_MAX` before feedback is available. A new Ethernet packet can still carry old drive feedback.
- `telemetry_epoch` advances when the receiver resynchronizes after stale telemetry or a source restart. During a fresh sequence it discards duplicate and out-of-order samples. The native joint sender detects an epoch change and requests a controlled stop.
- The joint arrays contain J1–J7 followed by J8. Arm command arrays have seven elements. Positions are radians, velocities rad/s and torque N·m. A tool pose is base-frame XYZ in metres followed by a row-major 3×3 rotation matrix.
- `link_stats()` reports host loss/intervals and, when available, controller link and timing counters. `fault_log()` retains recent drive, stop and self-check events. [Protocol fields](protocol.md)

## Drive lifecycle

1. Wait for fresh drive telemetry after the info handshake. The enabling examples wait up to 500 ms for a sample at most 100 ms old and exit on timeout. With drives disabled in IDLE, select the motor profile and check the self-check result. Zero calibration is a separate deliberate operation.
2. Enable `MotorGroup::Arm` for J1–J7; the gripper group is independent. Enter the required control mode from IDLE with fresh feedback and a stationary arm.
3. Seed RT targets from `hold_command()`. The held command can differ from the measured position under load. In NRT, submit a goal or use a blocking move; the SDK maintains the session and reports completion. Return through IDLE before changing active control families.
4. On firmware with controlled-stop support, `stop()` brakes to enabled hold. `servo_off()` removes drive torque; support the load first. `cancel_motion()` cancels an NRT move, while the older `cancel()` API requests immediate group disable. `resume()` requires firmware support and re-enters the interrupted mode from hold.

For host-controlled RT/NRT motion, a lost required stream triggers the MCU's stop supervisor. For a local compliance mode, issue STOP while connected before ending the session. Use an independent hardware emergency stop for drive-power isolation. [Joint, Cartesian and compliance examples](../examples/cpp)

## Controller parameters

Firmware with `INFRABOT_CAP_PARAMETERS` (protocol 1.1) exposes the gravity, impedance and admittance settings that the touchscreen sliders change. `list_parameters()` returns every parameter with its name, unit, range, default, slew rate and its requested and applied value, so the SDK needs no table of its own. `get_parameter` and `set_parameter` take an id or a name. `set_parameter` stores the requested value and the controller slews its applied value toward it at the parameter rate, so a write never steps a gain. Values are RAM only and return to their defaults at reboot. Reading is open to any PC; writing needs the PC that controls the arm, or no controlling PC, and is otherwise refused.

Names start with the group: `gravity.*` (FREE damping and LOCK stiffness), `imp.*` (joint and Cartesian impedance gains and the extra joint damping) and `adm.*` (joint and Cartesian admittance). `adm.bias_learning` is a switch that takes the value 0 or 1 and acts at once.

These calls block for the reply (three tries of 200 ms at most) and do not belong in a 1 kHz loop. C++ returns `false` with an error; Python raises `RuntimeError`. [Python example](../examples/python/08_parameters.py), [C++ example](../examples/cpp/08_parameters.cpp)

## Diagnostics

Firmware with `INFRABOT_CAP_DIAGNOSTICS` answers `diagnostics()` with one snapshot of the controller: the last one-second window of the status task (control ticks, missed ticks, longest control period, CAN frames sent and received, CAN send errors, command datagrams accepted and rejected, telemetry packets), totals since boot, the CAN error counters and error-passive and bus-off flags, free FreeRTOS heap now and its lowest value, the least free stack ever of the control, status and Ethernet tasks, and the drive states, controller status and reason of the last rejected command. The query is read only and open to any PC; it never takes control of the arm. It blocks for the reply like the parameter calls. `infrabot/diagnostics.hpp` turns the drive, status and rejection codes into text. [Python example](../examples/python/09_diagnostics.py), [C++ example](../examples/cpp/09_diagnostics.cpp)

## Record and retrieve logs

| Source | API | Data |
| --- | --- | --- |
| Live UDP session | `start_recording(directory, …)` / `stop_recording()` | Timestamped telemetry, commands and events under the requested directory |
| Controller UART | `start_serial_log` / `stop_serial_log` | Timestamped text from the local serial adapter, independent of UDP |
| Board SDRAM | `download_mcu_log` | A text file and telemetry CSV from the current boot, over UDP 5003; firmware must permit the download |

Start and finish logging outside the periodic loop. For the serial and MCU logs, an explicit directory takes precedence over `INFRABOT_LOG_DIR`, then `$XDG_STATE_HOME/infrabot/log` and `~/.local/state/infrabot/log`. A board reset clears the SDRAM log. Read it before cycling power when investigating a fault. [Python download example](../examples/python/07_download_mcu_log.py)
