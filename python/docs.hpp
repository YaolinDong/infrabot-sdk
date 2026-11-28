/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#pragma once
/* Python docstrings, used by bindings.cpp. They follow the C++ reference
 * (include/infrabot/robot.hpp) with the Python calling convention: a failure
 * raises RuntimeError with the reason instead of returning false.
 * tests/test_python_docs.py checks that every public name has one. */

namespace pydoc {
inline constexpr const char *module = R"doc(Python API of the InfraBot SDK.

One Robot per arm controller. Joint values are in SI units: rad, rad/s, N·m.
A call that fails raises RuntimeError with the reason, in Chinese.

Timing classes, given at the start of each docstring:

- [Blocking] waits for the controller; never call it from a 1 kHz loop.
- [Non-blocking] returns at once.
- [RT] non-blocking: safe in a 1 kHz loop.

Lifecycle: connect() → set_motor_profile() → servo_on() → switch_mode() →
commands → stop() or switch_mode(Mode.Idle) → servo_off().)doc";
inline constexpr const char *RobotInfo = R"doc(The controller's identity, limits and capabilities, from the connect() handshake.)doc";
inline constexpr const char *JointState = R"doc(Measured state of the eight drives (joint frame, SI units), and with telemetry v3 what the controller commands them.)doc";
inline constexpr const char *SafetyState = R"doc(Controlled stop and parameter self-check (telemetry v3).)doc";
inline constexpr const char *RobotState = R"doc(Controller state from the newest telemetry packet. Returned by Robot.state().)doc";
inline constexpr const char *ControllerLinkStats = R"doc(The controller's view of the link and its control loop over the last second.)doc";
inline constexpr const char *LinkStats = R"doc(Quality of the UDP link. The last-second values are refreshed once a second.)doc";
inline constexpr const char *FaultEvent = R"doc(A drive fault, a controlled stop or a self-check result, as it happened.)doc";
inline constexpr const char *JointStreamOptions = R"doc(Options of the native RT_JOINT producer (Robot.start_joint_stream()).)doc";
inline constexpr const char *JointStreamStats = R"doc(State of the native RT_JOINT producer.)doc";
inline constexpr const char *JointCommand = R"doc(An RT_JOINT command for J1–J7, sent with Robot.servej(). MIT profile: position and velocity with kp/kd plus a torque feed-forward. Position-velocity profile: position at no more than velocity_limit. Defaults: template gains and 3 rad/s.)doc";
inline constexpr const char *CartesianLimits = R"doc(Cartesian speed limits. 0 selects the controller's default, which is its maximum.)doc";
inline constexpr const char *CartesianCommand = R"doc(An RT_CARTESIAN command, sent with Robot.servep(). pose is x, y, z in metres, then the row-major 3×3 rotation.)doc";
inline constexpr const char *MoveOptions = R"doc(Options of a joint move (Robot.movej()). Defaults: 0.1 rad/s, 0.2 rad/s² and the template gains.)doc";
inline constexpr const char *CartesianMoveOptions = R"doc(Options of a Cartesian move (Robot.movep()).)doc";
inline constexpr const char *MotionResult = R"doc(Outcome of a move.)doc";
inline constexpr const char *MoveReply = R"doc(The controller's reply about a move (DMOS).)doc";
inline constexpr const char *MotionStatus = R"doc(The movej/movep session of the NRT modes. Returned by Robot.motion().)doc";
inline constexpr const char *RecordOptions = R"doc(Options of Robot.start_recording().)doc";
inline constexpr const char *RecorderStats = R"doc(Progress of a recording. Returned by Robot.recording().)doc";
inline constexpr const char *SerialLogOptions = R"doc(Options of Robot.start_serial_log().)doc";
inline constexpr const char *SerialLogStats = R"doc(Progress of the serial log. Returned by Robot.serial_log().)doc";
inline constexpr const char *McuLogOptions = R"doc(Options of Robot.download_mcu_log(). The board keeps the newest 10 minutes of text and 10 s of 1 kHz telemetry in SDRAM; they are lost on power-off or reset.)doc";
inline constexpr const char *McuLogResult = R"doc(Files written by Robot.download_mcu_log().)doc";
inline constexpr const char *ParameterInfo = R"doc(One tunable controller parameter (firmware 0.2.0, CAP_PARAMETERS).)doc";
inline constexpr const char *DiagnosticsInfo = R"doc(One diagnostic snapshot (firmware 0.2.0, CAP_DIAGNOSTICS). Window counters cover the controller's last one-second status window; _total counters count since boot.)doc";
inline constexpr const char *Mode = R"doc(Control mode of the controller. The admittance modes need every joint 0.03 rad inside its limits.)doc";
inline constexpr const char *MotorProfile = R"doc(Drive mode of J1–J7: Mit (position, velocity, kp/kd and a torque feed-forward) or PositionVelocity (position with a velocity limit). J8 is always force-position.)doc";
inline constexpr const char *MotorGroup = R"doc(Drive groups enabled and disabled together: All (J1–J8), Arm (J1–J7), Gripper (J8).)doc";
inline constexpr const char *OperationalStatus = R"doc(What the controller is doing now, from telemetry.)doc";
inline constexpr const char *StopReason = R"doc(Why the controller made a controlled stop. None_ means no stop is latched.)doc";
inline constexpr const char *StopPhase = R"doc(Phase of a controlled stop: Braking to rest, then Holding enabled until a PC is back.)doc";
inline constexpr const char *SelfCheck = R"doc(Result of the drive parameter self-check that set_motor_profile() runs.)doc";
inline constexpr const char *FaultKind = R"doc(What a FaultEvent records.)doc";
inline constexpr const char *MotionPhase = R"doc(Phase of the movej/movep session.)doc";
inline constexpr const char *Robot = R"doc(The client for one arm controller.

Robot(address="192.168.0.50", command_port=5001, telemetry_port=5000) creates the
client; connect() opens the sockets. It runs a receive thread and a 1 kHz service
thread (heartbeat, NRT session, native joint stream). Calls release the GIL.
A call that fails raises RuntimeError with the reason, in Chinese.)doc";
namespace field {
namespace RobotInfo {
inline constexpr const char *received = R"doc(A DMIO answer arrived (firmware with DMIF).)doc";
inline constexpr const char *firmware = R"doc(Firmware build string, e.g. "v0.2.0"; empty for V1-only firmware.)doc";
inline constexpr const char *product = R"doc(Product name.)doc";
inline constexpr const char *axes = R"doc(Arm joints (7).)doc";
inline constexpr const char *motor_count = R"doc(Drives including the gripper (8).)doc";
inline constexpr const char *capabilities = R"doc(CAP_* bits: test these for optional features.)doc";
inline constexpr const char *protocol_version = R"doc(DMIO envelope version.)doc";
inline constexpr const char *protocol_major = R"doc(Protocol major version; must match the SDK's.)doc";
inline constexpr const char *protocol_minor = R"doc(Protocol minor version; minors only add features.)doc";
inline constexpr const char *board = R"doc(Controller board name.)doc";
inline constexpr const char *arm_model = R"doc(Arm model name.)doc";
inline constexpr const char *torque_max = R"doc(Drive torque ranges, N·m.)doc";
inline constexpr const char *velocity_max = R"doc(Drive velocity ranges, rad/s.)doc";
inline constexpr const char *position_min = R"doc(Joint position limits, rad.)doc";
inline constexpr const char *position_max = R"doc(Joint position limits, rad.)doc";
inline constexpr const char *profile_ready = R"doc(The motor profile was applied and passed its self-check.)doc";
inline constexpr const char *enabled_mask = R"doc(Enabled drives at connect (bit i = J(i+1)).)doc";
inline constexpr const char *control_mode = R"doc(Control mode at connect (a Mode value).)doc";
inline constexpr const char *motor_profile = R"doc(Drive mode at connect (a MotorProfile value).)doc";
inline constexpr const char *controller_status = R"doc(Status at connect (an OperationalStatus value).)doc";
}
namespace JointState {
inline constexpr const char *position = R"doc(Position, rad.)doc";
inline constexpr const char *velocity = R"doc(Velocity, rad/s.)doc";
inline constexpr const char *torque = R"doc(Torque, N·m.)doc";
inline constexpr const char *temperature = R"doc(MOS temperature, °C.)doc";
inline constexpr const char *temperature_coil = R"doc(Coil temperature, °C.)doc";
inline constexpr const char *feedback_count = R"doc(Drive feedback counter; it stops when a drive goes quiet.)doc";
inline constexpr const char *feedback_age_ms = R"doc(Age of each drive's feedback, ms; 4294967295 before the first.)doc";
inline constexpr const char *drive_state = R"doc(Drive status code: 0 disabled, 1 enabled, 8 and above a fault.)doc";
inline constexpr const char *motor_flags = R"doc(Motor flag bits.)doc";
inline constexpr const char *drive_mode = R"doc(Drive mode code of each drive.)doc";
inline constexpr const char *has_command = R"doc(The command fields are valid (telemetry v3).)doc";
inline constexpr const char *command_position = R"doc(Planned command, rad.)doc";
inline constexpr const char *command_velocity = R"doc(MIT velocity feed-forward or position-velocity v_des, rad/s.)doc";
inline constexpr const char *command_torque = R"doc(MIT torque feed-forward, N·m.)doc";
inline constexpr const char *sequence = R"doc(Telemetry sequence number.)doc";
inline constexpr const char *timestamp_us = R"doc(Controller clock, µs.)doc";
inline constexpr const char *host_time_us = R"doc(CLOCK_MONOTONIC when the packet arrived, µs.)doc";
}
namespace SafetyState {
inline constexpr const char *valid = R"doc(The fields come from telemetry v3.)doc";
inline constexpr const char *stop_reason = R"doc(The latched stop; StopReason.None_ after it is acknowledged.)doc";
inline constexpr const char *stop_phase = R"doc(Braking or holding.)doc";
inline constexpr const char *stop_joint = R"doc(Joint that caused the stop, 0-based; 255 if none.)doc";
inline constexpr const char *stop_detail = R"doc(Reason-specific detail (for a drive fault, its status code).)doc";
inline constexpr const char *stop_mode = R"doc(The mode the stop interrupted; resume() re-enters it.)doc";
inline constexpr const char *selfcheck = R"doc(Last self-check result.)doc";
inline constexpr const char *selfcheck_joint = R"doc(Joint that failed the self-check, 0-based.)doc";
inline constexpr const char *selfcheck_register = R"doc(Register that failed.)doc";
inline constexpr const char *selfcheck_value = R"doc(Value read back (raw register bits).)doc";
inline constexpr const char *selfcheck_expected = R"doc(Value expected (raw register bits).)doc";
}
namespace RobotState {
inline constexpr const char *connected = R"doc(Sockets open; use telemetry_age_ms for liveness.)doc";
inline constexpr const char *enabled_mask = R"doc(Enabled drives (bit i = J(i+1)).)doc";
inline constexpr const char *enabled = R"doc(Any drive enabled.)doc";
inline constexpr const char *mode = R"doc(Current control mode.)doc";
inline constexpr const char *profile = R"doc(Current drive mode of J1–J7.)doc";
inline constexpr const char *profile_ready = R"doc(The motor profile was applied and passed its self-check.)doc";
inline constexpr const char *modes_supported = R"doc(Firmware with the control-mode state machine.)doc";
inline constexpr const char *zero_valid = R"doc(The drives' zero positions are calibrated.)doc";
inline constexpr const char *telemetry_age_ms = R"doc(Age of the newest telemetry, ms; 4294967295 before the first.)doc";
inline constexpr const char *telemetry_epoch = R"doc(Increments when telemetry resynchronizes after a stale or restarted source.)doc";
inline constexpr const char *telemetry_version = R"doc(Telemetry format, 2 or 3.)doc";
inline constexpr const char *joints = R"doc(Per-drive state (JointState).)doc";
inline constexpr const char *safety = R"doc(Controlled stop and self-check (SafetyState).)doc";
inline constexpr const char *operational_status = R"doc(What the controller is doing.)doc";
inline constexpr const char *controller_state = R"doc(Raw status code.)doc";
inline constexpr const char *command_active = R"doc(The controller applies a PC command stream.)doc";
inline constexpr const char *command_applied_sequence = R"doc(Sequence of the newest command the controller applied.)doc";
}
namespace ControllerLinkStats {
inline constexpr const char *command_rx = R"doc(Command datagrams accepted.)doc";
inline constexpr const char *command_invalid = R"doc(Command datagrams rejected.)doc";
inline constexpr const char *command_interval_max_us = R"doc(Longest gap between command datagrams, µs.)doc";
inline constexpr const char *command_jitter_max_us = R"doc(Largest command jitter, µs.)doc";
inline constexpr const char *control_missed = R"doc(Control ticks missed.)doc";
inline constexpr const char *control_period_max_us = R"doc(Longest control period, µs.)doc";
inline constexpr const char *can_tx_errors = R"doc(Drive frames that could not be queued.)doc";
inline constexpr const char *telemetry_errors = R"doc(Telemetry packets that could not be sent.)doc";
inline constexpr const char *invalid_reason = R"doc(Why the last command datagram was rejected.)doc";
}
namespace LinkStats {
inline constexpr const char *tx_sent = R"doc(Datagrams sent.)doc";
inline constexpr const char *tx_errors = R"doc(Datagrams that failed to send.)doc";
inline constexpr const char *rx_telemetry = R"doc(Telemetry packets received.)doc";
inline constexpr const char *rx_lost = R"doc(Telemetry sequence gaps.)doc";
inline constexpr const char *rx_invalid = R"doc(Datagrams not understood.)doc";
inline constexpr const char *rx_last_second = R"doc(Telemetry packets in the last second.)doc";
inline constexpr const char *lost_last_second = R"doc(Telemetry packets lost in the last second.)doc";
inline constexpr const char *loss_percent = R"doc(Loss in the last second, %.)doc";
inline constexpr const char *rx_interval_max_us = R"doc(Longest gap between telemetry packets in the last second, µs.)doc";
inline constexpr const char *mcu_valid = R"doc(`mcu` is valid (telemetry v3).)doc";
inline constexpr const char *mcu = R"doc(The controller's view of the link (ControllerLinkStats).)doc";
}
namespace FaultEvent {
inline constexpr const char *kind = R"doc(What happened (FaultKind).)doc";
inline constexpr const char *host_time_us = R"doc(PC steady clock, µs.)doc";
inline constexpr const char *mcu_time_us = R"doc(Controller clock, µs.)doc";
inline constexpr const char *joint = R"doc(Joint, 0-based; 255 if none.)doc";
inline constexpr const char *code = R"doc(Drive, stop or self-check code.)doc";
inline constexpr const char *detail = R"doc(Code-specific detail.)doc";
inline constexpr const char *message = R"doc(What happened, the probable cause and what to do, in Chinese.)doc";
}
namespace JointStreamOptions {
inline constexpr const char *transmit_hz = R"doc(Send rate.)doc";
inline constexpr const char *command_timeout_ms = R"doc(Stop when servej() has not refreshed the command for this long.)doc";
inline constexpr const char *feedback_timeout_ms = R"doc(Stop when drive feedback is older than this.)doc";
inline constexpr const char *allow_zero_kp = R"doc(Accept kp = 0; off because the controller's STOP does not restore holding gains.)doc";
}
namespace JointStreamStats {
inline constexpr const char *active = R"doc(The producer is sending.)doc";
inline constexpr const char *sent = R"doc(Commands sent.)doc";
inline constexpr const char *skipped_deadlines = R"doc(Send deadlines missed.)doc";
inline constexpr const char *fault = R"doc(Why it stopped; latched, so a failed stream needs an explicit start.)doc";
}
namespace JointCommand {
inline constexpr const char *position = R"doc(Target, rad.)doc";
inline constexpr const char *velocity = R"doc(MIT velocity feed-forward, rad/s.)doc";
inline constexpr const char *kp = R"doc(MIT stiffness, 0–500.)doc";
inline constexpr const char *kd = R"doc(MIT damping, 0–5.)doc";
inline constexpr const char *torque = R"doc(MIT torque feed-forward, N·m.)doc";
inline constexpr const char *velocity_limit = R"doc(Position-velocity v_des, rad/s.)doc";
}
namespace CartesianLimits {
inline constexpr const char *linear_velocity = R"doc(m/s, at most 0.5.)doc";
inline constexpr const char *angular_velocity = R"doc(rad/s, at most π.)doc";
inline constexpr const char *linear_acceleration = R"doc(m/s², at most 2.)doc";
inline constexpr const char *angular_acceleration = R"doc(rad/s², at most 2π.)doc";
}
namespace CartesianCommand {
inline constexpr const char *pose = R"doc(Target tool pose (12 values).)doc";
inline constexpr const char *limits = R"doc(Speed limits of the straight-line motion.)doc";
inline constexpr const char *kp = R"doc(MIT stiffness.)doc";
inline constexpr const char *kd = R"doc(MIT damping.)doc";
inline constexpr const char *torque = R"doc(MIT torque feed-forward, N·m.)doc";
inline constexpr const char *velocity_limit = R"doc(Position-velocity v_des, rad/s.)doc";
}
namespace MoveOptions {
inline constexpr const char *timeout_ms = R"doc(The move fails if it has not arrived by then.)doc";
inline constexpr const char *dwell_ms = R"doc(How long the arm must stay within tolerance to count as arrived.)doc";
inline constexpr const char *position_tolerance = R"doc(Arrival tolerance per joint, rad; 0.001–0.05.)doc";
inline constexpr const char *velocity_tolerance = R"doc(Arrival speed tolerance per joint, rad/s.)doc";
inline constexpr const char *velocity_max = R"doc(Speed cap per joint, rad/s (position-velocity: also v_des).)doc";
inline constexpr const char *acceleration_max = R"doc(Acceleration cap per joint, rad/s².)doc";
inline constexpr const char *kp = R"doc(MIT stiffness.)doc";
inline constexpr const char *kd = R"doc(MIT damping.)doc";
inline constexpr const char *torque = R"doc(MIT torque feed-forward, N·m.)doc";
}
namespace CartesianMoveOptions {
inline constexpr const char *timeout_ms = R"doc(The move fails if it has not arrived by then.)doc";
inline constexpr const char *dwell_ms = R"doc(How long the arm must stay within tolerance to count as arrived.)doc";
inline constexpr const char *position_tolerance = R"doc(Arrival tolerance of the tool, m and rad; 0.0005–0.05.)doc";
inline constexpr const char *velocity_tolerance = R"doc(Arrival speed tolerance per joint, rad/s.)doc";
inline constexpr const char *limits = R"doc(Speed limits of the straight-line motion.)doc";
inline constexpr const char *velocity_max = R"doc(Per-joint speed caps, rad/s; 0 selects the default.)doc";
inline constexpr const char *acceleration_max = R"doc(Per-joint acceleration caps, rad/s²; 0 selects 100.)doc";
inline constexpr const char *kp = R"doc(MIT stiffness.)doc";
inline constexpr const char *kd = R"doc(MIT damping.)doc";
}
namespace MotionResult {
inline constexpr const char *result = R"doc(0 means arrived; otherwise the move result code.)doc";
inline constexpr const char *state = R"doc(Move state code.)doc";
inline constexpr const char *motion_id = R"doc(The move this result belongs to.)doc";
inline constexpr const char *arrived_mask = R"doc(Joints within tolerance (bit i = J(i+1)).)doc";
inline constexpr const char *fresh_mask = R"doc(Joints with fresh feedback.)doc";
inline constexpr const char *max_error_rad = R"doc(Largest remaining error: rad for joint moves, m or rad for Cartesian moves.)doc";
inline constexpr const char *reason = R"doc(Reason code the move ended.)doc";
inline constexpr const char *message = R"doc(Why it failed, in Chinese; empty on arrival.)doc";
}
namespace MoveReply {
inline constexpr const char *result = R"doc(Result code.)doc";
inline constexpr const char *state = R"doc(Move state code.)doc";
inline constexpr const char *session = R"doc(Session identifier.)doc";
inline constexpr const char *request_id = R"doc(The move this reply answers.)doc";
inline constexpr const char *active_id = R"doc(The move the controller is executing.)doc";
inline constexpr const char *elapsed_ms = R"doc(Time since the move started, ms.)doc";
inline constexpr const char *duration_ms = R"doc(Planned duration, ms.)doc";
inline constexpr const char *arrived_mask = R"doc(Joints within tolerance.)doc";
inline constexpr const char *enabled_mask = R"doc(Enabled drives.)doc";
inline constexpr const char *fresh_mask = R"doc(Joints with fresh feedback.)doc";
inline constexpr const char *max_error_rad = R"doc(Largest remaining error.)doc";
inline constexpr const char *reason = R"doc(Reason code.)doc";
}
namespace MotionStatus {
inline constexpr const char *phase = R"doc(Session phase (MotionPhase).)doc";
inline constexpr const char *id = R"doc(The newest move; 1 is the hold made on entering the mode.)doc";
inline constexpr const char *can_execute = R"doc(A new move is accepted now.)doc";
inline constexpr const char *has_reply = R"doc(`reply` is valid.)doc";
inline constexpr const char *reply_age_us = R"doc(Age of `reply`, µs.)doc";
inline constexpr const char *error = R"doc(Why the session failed, in Chinese.)doc";
inline constexpr const char *reply = R"doc(The controller's newest reply for the move (MoveReply).)doc";
}
namespace RecordOptions {
inline constexpr const char *label = R"doc(Appended to the directory name.)doc";
inline constexpr const char *rotate_seconds = R"doc(Start new telemetry and command files this often; 0 never.)doc";
inline constexpr const char *commands = R"doc(Also log every command datagram sent.)doc";
}
namespace RecorderStats {
inline constexpr const char *active = R"doc(A recording is running.)doc";
inline constexpr const char *directory = R"doc(Directory being written.)doc";
inline constexpr const char *telemetry_rows = R"doc(Telemetry rows written.)doc";
inline constexpr const char *command_rows = R"doc(Command rows written.)doc";
inline constexpr const char *events = R"doc(Event rows written.)doc";
inline constexpr const char *dropped = R"doc(Rows lost because the disk could not keep up.)doc";
inline constexpr const char *file_index = R"doc(Index of the current telemetry/command file.)doc";
inline constexpr const char *error = R"doc(Last write error.)doc";
}
namespace SerialLogOptions {
inline constexpr const char *port = R"doc(Serial device.)doc";
inline constexpr const char *baud = R"doc(Baud rate.)doc";
inline constexpr const char *directory = R"doc(Output directory; empty selects Robot.log_directory().)doc";
}
namespace SerialLogStats {
inline constexpr const char *active = R"doc(The log is running.)doc";
inline constexpr const char *path = R"doc(File being written.)doc";
inline constexpr const char *lines = R"doc(Lines written.)doc";
inline constexpr const char *bytes = R"doc(Bytes written.)doc";
inline constexpr const char *reconnects = R"doc(Times the port was lost and reopened.)doc";
inline constexpr const char *port_open = R"doc(The port is open now.)doc";
inline constexpr const char *error = R"doc(Last port or write error.)doc";
}
namespace McuLogOptions {
inline constexpr const char *directory = R"doc(Output directory; empty selects Robot.log_directory().)doc";
inline constexpr const char *label = R"doc(Appended to the file names.)doc";
inline constexpr const char *port = R"doc(Log port of the board.)doc";
inline constexpr const char *timeout_ms = R"doc(Timeout per request.)doc";
inline constexpr const char *retries = R"doc(Retries per request.)doc";
inline constexpr const char *progress = R"doc(Called with (text_lines, text_total, telemetry_rows, telemetry_total); return False to cancel.)doc";
}
namespace McuLogResult {
inline constexpr const char *text_path = R"doc(The text log, lines "<mcu_time_s> <line>".)doc";
inline constexpr const char *telemetry_path = R"doc(The telemetry CSV.)doc";
inline constexpr const char *boot_id = R"doc(Changes on every controller boot.)doc";
inline constexpr const char *text_lines = R"doc(Text lines saved.)doc";
inline constexpr const char *telemetry_rows = R"doc(Telemetry rows saved.)doc";
inline constexpr const char *lost_bytes = R"doc(printf bytes the board could not keep.)doc";
inline constexpr const char *writer_max_us = R"doc(Longest log-task pass since the previous download, µs.)doc";
inline constexpr const char *fault_latched = R"doc(The board froze the telemetry at a fault: the CSV ends at that tick.)doc";
}
namespace ParameterInfo {
inline constexpr const char *id = R"doc(group << 8 | index.)doc";
inline constexpr const char *group = R"doc(PARAM_GROUP_GRAVITY, _IMPEDANCE or _ADMITTANCE.)doc";
inline constexpr const char *name = R"doc(E.g. "imp.joint_k".)doc";
inline constexpr const char *unit = R"doc(E.g. "Nm/rad"; empty when dimensionless.)doc";
inline constexpr const char *requested = R"doc(The last value written, or the default after a reboot.)doc";
inline constexpr const char *applied = R"doc(The value the controller uses now, slewing toward `requested`.)doc";
inline constexpr const char *minimum = R"doc(Lowest accepted value.)doc";
inline constexpr const char *maximum = R"doc(Highest accepted value.)doc";
inline constexpr const char *default_value = R"doc(Value after a reboot.)doc";
inline constexpr const char *rate = R"doc(Largest change of the applied value, units per second.)doc";
inline constexpr const char *writable = R"doc(set_parameter() accepts it.)doc";
inline constexpr const char *boolean = R"doc(The value is 0 or 1.)doc";
}
namespace DiagnosticsInfo {
inline constexpr const char *uptime_ms = R"doc(Since the application started; wraps after 49 days.)doc";
inline constexpr const char *snapshot_seconds = R"doc(Status-task second of the window.)doc";
inline constexpr const char *control_hz = R"doc(Control ticks completed in the window (1000 when healthy).)doc";
inline constexpr const char *control_missed = R"doc(Control ticks missed in the window.)doc";
inline constexpr const char *control_period_max_us = R"doc(Longest control period in the window, µs.)doc";
inline constexpr const char *can_tx = R"doc(Drive frames queued in the window.)doc";
inline constexpr const char *can_rx = R"doc(CAN frames received in the window.)doc";
inline constexpr const char *can_tx_errors = R"doc(Drive frames that could not be queued in the window.)doc";
inline constexpr const char *command_rx = R"doc(Command datagrams accepted in the window.)doc";
inline constexpr const char *command_invalid = R"doc(Command datagrams rejected in the window.)doc";
inline constexpr const char *telemetry_tx = R"doc(Telemetry packets sent in the window.)doc";
inline constexpr const char *can_error_events = R"doc(CAN error events since boot.)doc";
inline constexpr const char *control_missed_total = R"doc(Control ticks missed since boot.)doc";
inline constexpr const char *can_tx_errors_total = R"doc(Drive frames not queued since boot.)doc";
inline constexpr const char *command_invalid_total = R"doc(Command datagrams rejected since boot.)doc";
inline constexpr const char *can_tec = R"doc(CAN transmit error counter.)doc";
inline constexpr const char *can_rec = R"doc(CAN receive error counter.)doc";
inline constexpr const char *can_error_passive = R"doc(CAN controller error-passive.)doc";
inline constexpr const char *can_bus_off = R"doc(CAN controller bus-off.)doc";
inline constexpr const char *heap_free_bytes = R"doc(FreeRTOS heap free now.)doc";
inline constexpr const char *heap_min_free_bytes = R"doc(FreeRTOS heap free at its lowest since boot.)doc";
inline constexpr const char *stack_free_control_bytes = R"doc(Least free stack ever of the control task.)doc";
inline constexpr const char *stack_free_status_bytes = R"doc(Least free stack ever of the status task.)doc";
inline constexpr const char *stack_free_ethernet_bytes = R"doc(Least free stack ever of the Ethernet task.)doc";
inline constexpr const char *drive_state = R"doc(Drive status code of J1–J8, now.)doc";
inline constexpr const char *controller_status = R"doc(Controller status code, now.)doc";
inline constexpr const char *invalid_reason = R"doc(Why the last command datagram was rejected.)doc";
}
}  // namespace field
namespace robot {
inline constexpr const char *__init__ = R"doc([Non-blocking] Creates the client for the controller at `address`. connect() opens the sockets.)doc";
inline constexpr const char *connect = R"doc([Blocking, up to 0.5 s] Negotiates the protocol, then starts the heartbeat. A different protocol major version or no valid reply raises. Check state().telemetry_age_ms for live drive feedback.)doc";
inline constexpr const char *disconnect = R"doc([Blocking] Closes the sockets and sends nothing. An enabled arm notices the PC is gone after 100 ms, stops in a controlled way and holds. Call stop() first for an immediate stop.)doc";
inline constexpr const char *connected = R"doc([Non-blocking] The sockets are open.)doc";
inline constexpr const char *info = R"doc([Non-blocking] Identity, limits and capabilities from the handshake (RobotInfo).)doc";
inline constexpr const char *state = R"doc([RT] A copy of the newest telemetry (RobotState).)doc";
inline constexpr const char *link_stats = R"doc([Non-blocking] Link quality (LinkStats).)doc";
inline constexpr const char *fault_log = R"doc([Non-blocking] Faults, stops and self-checks, oldest first, at most 200 (list of FaultEvent).)doc";
inline constexpr const char *set_heartbeat = R"doc([Non-blocking] Turns the heartbeat on or off; on by default. Off: this process only watches while another program controls the arm.)doc";
inline constexpr const char *set_motor_profile = R"doc([Blocking, 3 s] Selects the drive mode of J1–J7 (MotorProfile) with every drive disabled in IDLE. The controller then checks every drive's parameters and refuses to enable on a mismatch; the error names the drive and register.)doc";
inline constexpr const char *set_zero = R"doc([Blocking, 2 s] Stores the zero of every drive at its current angle; every drive must be disabled. enable_after=True enables both groups afterwards.)doc";
inline constexpr const char *servo_on = R"doc([Blocking, 3 s] Enables a drive group (default MotorGroup.All) in IDLE and waits for telemetry to show it. On timeout that group is disabled again.)doc";
inline constexpr const char *servo_off = R"doc([Blocking, 2 s] Disables a group (default all) at once, in any mode. No controlled stop: the arm falls. wait=False sends the request twice and returns.)doc";
inline constexpr const char *stop = R"doc([Blocking, 2 s] Controlled stop in any mode: the controller brakes the arm to rest and holds it enabled in IDLE. wait=False sends the request and returns. resume() continues the interrupted mode.)doc";
inline constexpr const char *resume = R"doc([Blocking, 3 s] Re-enters the mode a controlled stop interrupted, from where the arm stopped, and returns that Mode. A stop that happened in IDLE is only acknowledged. Refuses a disabled or faulted arm drive.)doc";
inline constexpr const char *clear_fault = R"doc([Blocking, timeout_ms] Clears the disabled drives that report a fault and waits until none does. An enabled faulted drive needs servo_off() first.)doc";
inline constexpr const char *switch_mode = R"doc([Blocking, 2 s] Switches the control mode. A mode other than IDLE is entered from IDLE with the arm enabled and still. RT modes: then stream servej()/servep() starting from hold_command(). NRT modes: the SDK starts the movej/movep session. Retries for up to 1 s while the controller is still handing control over.)doc";
inline constexpr const char *hold_command = R"doc([Non-blocking] The hold command read when the mode was entered (joint frame, rad, 8 values). RT streams should start here.)doc";
inline constexpr const char *read_hold_command = R"doc([Blocking, 0.5 s] Reads the hold command of the enabled drives now.)doc";
inline constexpr const char *servej = R"doc([RT] Sends one RT_JOINT command (JointCommand). While the native joint stream runs, publishes the command to it instead.)doc";
inline constexpr const char *servep = R"doc([RT] Sends one RT_CARTESIAN command (CartesianCommand).)doc";
inline constexpr const char *start_joint_stream = R"doc([Non-blocking] Starts the native 1 kHz RT_JOINT producer from `initial`; later servej() calls replace the command. Needs RT_JOINT with the MIT profile and fresh feedback. A producer timeout, stale feedback or a controller fault requests a controlled stop.)doc";
inline constexpr const char *stop_joint_stream = R"doc([Non-blocking] Ends the native producer; sends nothing. Use stop() to stop the arm.)doc";
inline constexpr const char *joint_stream_stats = R"doc([Non-blocking] State of the native producer (JointStreamStats).)doc";
inline constexpr const char *movej = R"doc([Blocking] Moves J1–J7 to `position` (rad) in NRT_JOINT and waits until arrival, failure or options.timeout_ms + 3 s. Returns a MotionResult whose result is 0 on arrival.)doc";
inline constexpr const char *movep = R"doc([Blocking] Moves the tool to `pose` along a straight line in NRT_CARTESIAN. Returns a MotionResult whose result is 0 on arrival.)doc";
inline constexpr const char *submit_movej = R"doc([Non-blocking] Starts a joint move and returns; follow it with motion() or wait_motion(). The previous move must be done.)doc";
inline constexpr const char *submit_movep = R"doc([Non-blocking] Starts a Cartesian move and returns. See submit_movej().)doc";
inline constexpr const char *motion = R"doc([Non-blocking] State of the movej/movep session (MotionStatus).)doc";
inline constexpr const char *wait_motion = R"doc([Blocking, timeout_ms] Waits until the newest move has arrived and returns its MotionResult; raises if it failed.)doc";
inline constexpr const char *cancel_motion = R"doc([Non-blocking] Ends the session's move with a controlled stop; the arm holds in IDLE.)doc";
inline constexpr const char *set_default_move_options = R"doc([Non-blocking] Options of the hold move that opens an NRT_JOINT session.)doc";
inline constexpr const char *set_default_cartesian_options = R"doc([Non-blocking] Options of the hold move that opens an NRT_CARTESIAN session.)doc";
inline constexpr const char *gripper = R"doc([Blocking, 1 s] gripper(position_rad, speed_rad_s, force_ratio) moves the gripper to 0–5.94 rad. force_ratio is the current limit as a fraction of the drive maximum (0–1). Fails while the gripper group is disabled.)doc";
inline constexpr const char *start_recording = R"doc([Blocking, file setup] Records every telemetry packet (1 kHz), every command sent and the events as CSV under directory/<time>[-label]/. A background thread writes.)doc";
inline constexpr const char *stop_recording = R"doc([Blocking] Finishes the writer and closes the files.)doc";
inline constexpr const char *recording = R"doc([Non-blocking] Progress of the recording (RecorderStats).)doc";
inline constexpr const char *record_event = R"doc([Non-blocking] Adds a marker to events.csv.)doc";
inline constexpr const char *start_serial_log = R"doc([Blocking, port/file setup] Saves the controller's UART debug output and returns the file path. Fails if another program has the port open.)doc";
inline constexpr const char *stop_serial_log = R"doc([Blocking] Joins the serial worker.)doc";
inline constexpr const char *serial_log = R"doc([Non-blocking] Progress of the serial log (SerialLogStats).)doc";
inline constexpr const char *log_directory = R"doc([Non-blocking] Default SDK log directory: $INFRABOT_LOG_DIR, else $XDG_STATE_HOME/infrabot/log, else ~/.local/state/infrabot/log.)doc";
inline constexpr const char *download_mcu_log = R"doc([Blocking, a few s] Downloads the controller's run log into two new files and returns a McuLogResult. Needs no connect(). The board refuses while an RT stream runs.)doc";
inline constexpr const char *forward = R"doc([Non-blocking] Forward kinematics of the H7C arm: J1–J7 in rad to a tool pose (12 values).)doc";
inline constexpr const char *request_feedback = R"doc([Blocking, 1.5 s] Maintenance: asks drive `motor` (0–7) for a feedback frame.)doc";
inline constexpr const char *get_state = R"doc([Blocking, 1.5 s] Maintenance: checks a drive's feedback freshness.)doc";
inline constexpr const char *get_register = R"doc([Blocking, 1.5 s] Maintenance: get_register(motor, register_id, value_type) returns the raw register bits. value_type is REGISTER_U32 or REGISTER_F32 and must match the register.)doc";
inline constexpr const char *write_register = R"doc([Blocking, 1.5 s] Maintenance: write_register(motor, register_id, value, value_type) writes a register within its allowed range and reads it back.)doc";
inline constexpr const char *enable = R"doc([Blocking, 1.5 s] Maintenance: enables one drive directly.)doc";
inline constexpr const char *disable = R"doc([Blocking, 1.5 s] Maintenance: disables one drive.)doc";
inline constexpr const char *clear_error = R"doc([Blocking, 1.5 s] Maintenance: clears one drive's fault.)doc";
inline constexpr const char *set_zero_position = R"doc([Blocking, 1.5 s] Maintenance: stores one drive's zero at its current angle.)doc";
inline constexpr const char *ensure_mode = R"doc([Blocking, 1.5 s] Maintenance: switches one drive to a drive mode (1–4) if needed.)doc";
inline constexpr const char *send_mit = R"doc([Blocking, 1.5 s] Maintenance: send_mit(motor, position, velocity, kp, kd, torque) sends one MIT frame.)doc";
inline constexpr const char *send_pos_vel = R"doc([Blocking, 1.5 s] Maintenance: send_pos_vel(motor, position, velocity_limit) sends one position-velocity frame.)doc";
inline constexpr const char *send_vel = R"doc([Blocking, 1.5 s] Maintenance: send_vel(motor, velocity) sends one velocity frame.)doc";
inline constexpr const char *send_force_pos = R"doc([Blocking, 1.5 s] Maintenance: send_force_pos(motor, position, velocity_limit, torque_ratio) sends one force-position frame.)doc";
inline constexpr const char *set_can_timeout_ms = R"doc([Blocking, 1.5 s] Maintenance: sets one drive's CAN timeout.)doc";
inline constexpr const char *store_parameters = R"doc([Blocking, 1.5 s] Maintenance: saves one drive's registers to its flash.)doc";
inline constexpr const char *list_parameters = R"doc([Blocking, about 10 ms per parameter] Lists every controller parameter (list of ParameterInfo). Firmware 0.2.0 (CAP_PARAMETERS).)doc";
inline constexpr const char *get_parameter_id = R"doc([Blocking] Reads one parameter by id (group << 8 | index).)doc";
inline constexpr const char *get_parameter_name = R"doc([Blocking] Reads one parameter by name, e.g. "imp.joint_k".)doc";
inline constexpr const char *set_parameter_id = R"doc([Blocking] Requests a new value for a parameter by id and returns the ParameterInfo after the write. The controller slews the applied value toward it. RAM only; needs the PC that controls the arm, or no controlling PC.)doc";
inline constexpr const char *set_parameter_name = R"doc([Blocking] Requests a new value for a parameter by name and returns the ParameterInfo after the write. See the id overload.)doc";
inline constexpr const char *diagnostics = R"doc([Blocking, up to 600 ms] One diagnostic snapshot (DiagnosticsInfo). Firmware 0.2.0 (CAP_DIAGNOSTICS); read only and open to any PC.)doc";
}  // namespace robot
inline constexpr const char *update_firmware = R"doc([Blocking] Installs a signed IFW package after approval on the touchscreen and returns True, or False without sending anything when the board already runs an image from the package. Uses its own maintenance socket; needs no connected Robot. Support the arm, disable the drives and disconnect motor power first. Raises RuntimeError on refusal, timeout, an invalid package or a verification failure.)doc";
inline constexpr const char *firmware_status = R"doc([Blocking] Reports the running slot: A or B, confirmed or on trial, and its image ID, e.g. "confirmed A 3f9c…"; "recovery-loader" while the board waits in its loader.)doc";
inline constexpr const char *rollback_firmware = R"doc([Blocking] Makes the board boot the other A/B slot.)doc";
}  // namespace pydoc
