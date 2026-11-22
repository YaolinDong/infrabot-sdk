# Robot API

Create one `infrabot::Robot` (Python `infrabot.Robot`) per controller. Defaults are `192.168.0.50` and command/telemetry ports 5001/5000. The constructor configures the local object. `connect` performs discovery and starts the receiver and service threads. For a monitoring session call `set_heartbeat(false)` before connecting. The board routes telemetry to the registered receiver. [Public C++ declarations](../include/infrabot/robot.hpp) · [Python signatures](../python/infrabot/infrabot_native.pyi)

## Timing

| Class | Typical calls | Where to use it |
| --- | --- | --- |
| RT | `state`, `servej`, `servep`, low-level `send_command` / `send_joint_stream` / `send_cartesian_stream` | The periodic target loop. Successful calls use a snapshot, publish a target or send a datagram without waiting for a firmware reply. |
| Non-blocking | `info`, `link_stats`, `fault_log`, `hold_command`, `submit_movej`, `submit_movep`, `motion`, `joint_stream_stats` | Supervisory code. Returning without waiting for motion completion can still involve locks, copying or allocation. |
| Blocking | `connect`, `disconnect`, `set_motor_profile`, `servo_on`, `switch_mode`, `movej`, `movep`, `wait_motion`, maintenance, log download | Setup, operator actions and teardown. These can wait for a peer, disk I/O or a worker thread. |

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

## Record and retrieve logs

| Source | API | Data |
| --- | --- | --- |
| Live UDP session | `start_recording(directory, …)` / `stop_recording()` | Timestamped telemetry, commands and events under the requested directory |
| Controller UART | `start_serial_log` / `stop_serial_log` | Timestamped text from the local serial adapter, independent of UDP |
| Board SDRAM | `download_mcu_log` | A text file and telemetry CSV from the current boot, over UDP 5003; firmware must permit the download |

Start and finish logging outside the periodic loop. For the serial and MCU logs, an explicit directory takes precedence over `INFRABOT_LOG_DIR`, then `$XDG_STATE_HOME/infrabot/log` and `~/.local/state/infrabot/log`. A board reset clears the SDRAM log. Read it before cycling power when investigating a fault. [Python download example](../examples/python/07_download_mcu_log.py)
