# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Yaolin Dong
"""
Python API of the InfraBot SDK.

One Robot per arm controller. Joint values are in SI units: rad, rad/s, N·m.
A call that fails raises RuntimeError with the reason, in Chinese.

Timing classes, given at the start of each docstring:

- [Blocking] waits for the controller; never call it from a 1 kHz loop.
- [Non-blocking] returns at once.
- [RT] non-blocking: safe in a 1 kHz loop.

Lifecycle: connect() → set_motor_profile() → servo_on() → switch_mode() →
commands → stop() or switch_mode(Mode.Idle) → servo_off().
"""
from __future__ import annotations
import collections.abc
import typing
__all__: list[str] = ['ARM_MASK', 'ARM_MAX_JOINT_VELOCITY', 'CAP_ADMITTANCE', 'CAP_CONTROLLED_STOP', 'CAP_DIAGNOSTICS', 'CAP_GROUPS', 'CAP_JOINT_RT', 'CAP_LOCAL_GRAVITY', 'CAP_PARAMETERS', 'CAP_TELEMETRY_V3', 'CartesianCommand', 'CartesianLimits', 'CartesianMoveOptions', 'ControllerLinkStats', 'DiagnosticsInfo', 'FaultEvent', 'FaultKind', 'JointCommand', 'JointState', 'JointStreamOptions', 'JointStreamStats', 'LinkStats', 'MOTOR_FEEDBACK_STALE', 'McuLogOptions', 'McuLogResult', 'Mode', 'MotionPhase', 'MotionResult', 'MotionStatus', 'MotorGroup', 'MotorProfile', 'MoveOptions', 'MoveReply', 'OperationalStatus', 'PARAM_GROUP_ADMITTANCE', 'PARAM_GROUP_GRAVITY', 'PARAM_GROUP_IMPEDANCE', 'ParameterInfo', 'REGISTER_F32', 'REGISTER_U32', 'RecordOptions', 'RecorderStats', 'Robot', 'RobotInfo', 'RobotState', 'SafetyState', 'SelfCheck', 'SerialLogOptions', 'SerialLogStats', 'StopPhase', 'StopReason', 'firmware_status', 'rollback_firmware', 'update_firmware']
class CartesianCommand:
    """
    An RT_CARTESIAN command, sent with Robot.servep(). pose is x, y, z in metres, then the row-major 3×3 rotation.
    """
    def __init__(self) -> None:
        ...
    @property
    def kd(self) -> typing.Annotated[list[float], "FixedSize(7)"]:
        """
        MIT damping.
        """
    @kd.setter
    def kd(self, arg0: typing.Annotated[collections.abc.Sequence[typing.SupportsFloat | typing.SupportsIndex], "FixedSize(7)"]) -> None:
        ...
    @property
    def kp(self) -> typing.Annotated[list[float], "FixedSize(7)"]:
        """
        MIT stiffness.
        """
    @kp.setter
    def kp(self, arg0: typing.Annotated[collections.abc.Sequence[typing.SupportsFloat | typing.SupportsIndex], "FixedSize(7)"]) -> None:
        ...
    @property
    def limits(self) -> CartesianLimits:
        """
        Speed limits of the straight-line motion.
        """
    @limits.setter
    def limits(self, arg0: CartesianLimits) -> None:
        ...
    @property
    def pose(self) -> typing.Annotated[list[float], "FixedSize(12)"]:
        """
        Target tool pose (12 values).
        """
    @pose.setter
    def pose(self, arg0: typing.Annotated[collections.abc.Sequence[typing.SupportsFloat | typing.SupportsIndex], "FixedSize(12)"]) -> None:
        ...
    @property
    def torque(self) -> typing.Annotated[list[float], "FixedSize(7)"]:
        """
        MIT torque feed-forward, N·m.
        """
    @torque.setter
    def torque(self, arg0: typing.Annotated[collections.abc.Sequence[typing.SupportsFloat | typing.SupportsIndex], "FixedSize(7)"]) -> None:
        ...
    @property
    def velocity_limit(self) -> typing.Annotated[list[float], "FixedSize(7)"]:
        """
        Position-velocity v_des, rad/s.
        """
    @velocity_limit.setter
    def velocity_limit(self, arg0: typing.Annotated[collections.abc.Sequence[typing.SupportsFloat | typing.SupportsIndex], "FixedSize(7)"]) -> None:
        ...
class CartesianLimits:
    """
    Cartesian speed limits. 0 selects the controller's default, which is its maximum.
    """
    def __init__(self) -> None:
        ...
    @property
    def angular_acceleration(self) -> float:
        """
        rad/s², at most 2π.
        """
    @angular_acceleration.setter
    def angular_acceleration(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...
    @property
    def angular_velocity(self) -> float:
        """
        rad/s, at most π.
        """
    @angular_velocity.setter
    def angular_velocity(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...
    @property
    def linear_acceleration(self) -> float:
        """
        m/s², at most 2.
        """
    @linear_acceleration.setter
    def linear_acceleration(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...
    @property
    def linear_velocity(self) -> float:
        """
        m/s, at most 0.5.
        """
    @linear_velocity.setter
    def linear_velocity(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...
class CartesianMoveOptions:
    """
    Options of a Cartesian move (Robot.movep()).
    """
    def __init__(self) -> None:
        ...
    @property
    def acceleration_max(self) -> typing.Annotated[list[float], "FixedSize(7)"]:
        """
        Per-joint acceleration caps, rad/s²; 0 selects 100.
        """
    @acceleration_max.setter
    def acceleration_max(self, arg0: typing.Annotated[collections.abc.Sequence[typing.SupportsFloat | typing.SupportsIndex], "FixedSize(7)"]) -> None:
        ...
    @property
    def dwell_ms(self) -> int:
        """
        How long the arm must stay within tolerance to count as arrived.
        """
    @dwell_ms.setter
    def dwell_ms(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    @property
    def kd(self) -> typing.Annotated[list[float], "FixedSize(7)"]:
        """
        MIT damping.
        """
    @kd.setter
    def kd(self, arg0: typing.Annotated[collections.abc.Sequence[typing.SupportsFloat | typing.SupportsIndex], "FixedSize(7)"]) -> None:
        ...
    @property
    def kp(self) -> typing.Annotated[list[float], "FixedSize(7)"]:
        """
        MIT stiffness.
        """
    @kp.setter
    def kp(self, arg0: typing.Annotated[collections.abc.Sequence[typing.SupportsFloat | typing.SupportsIndex], "FixedSize(7)"]) -> None:
        ...
    @property
    def limits(self) -> CartesianLimits:
        """
        Speed limits of the straight-line motion.
        """
    @limits.setter
    def limits(self, arg0: CartesianLimits) -> None:
        ...
    @property
    def position_tolerance(self) -> float:
        """
        Arrival tolerance of the tool, m and rad; 0.0005–0.05.
        """
    @position_tolerance.setter
    def position_tolerance(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...
    @property
    def timeout_ms(self) -> int:
        """
        The move fails if it has not arrived by then.
        """
    @timeout_ms.setter
    def timeout_ms(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    @property
    def velocity_max(self) -> typing.Annotated[list[float], "FixedSize(7)"]:
        """
        Per-joint speed caps, rad/s; 0 selects the default.
        """
    @velocity_max.setter
    def velocity_max(self, arg0: typing.Annotated[collections.abc.Sequence[typing.SupportsFloat | typing.SupportsIndex], "FixedSize(7)"]) -> None:
        ...
    @property
    def velocity_tolerance(self) -> float:
        """
        Arrival speed tolerance per joint, rad/s.
        """
    @velocity_tolerance.setter
    def velocity_tolerance(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...
class ControllerLinkStats:
    """
    The controller's view of the link and its control loop over the last second.
    """
    @property
    def can_tx_errors(self) -> int:
        """
        Drive frames that could not be queued.
        """
    @property
    def command_interval_max_us(self) -> int:
        """
        Longest gap between command datagrams, µs.
        """
    @property
    def command_invalid(self) -> int:
        """
        Command datagrams rejected.
        """
    @property
    def command_jitter_max_us(self) -> int:
        """
        Largest command jitter, µs.
        """
    @property
    def command_rx(self) -> int:
        """
        Command datagrams accepted.
        """
    @property
    def control_missed(self) -> int:
        """
        Control ticks missed.
        """
    @property
    def control_period_max_us(self) -> int:
        """
        Longest control period, µs.
        """
    @property
    def invalid_reason(self) -> int:
        """
        Why the last command datagram was rejected.
        """
    @property
    def telemetry_errors(self) -> int:
        """
        Telemetry packets that could not be sent.
        """
class DiagnosticsInfo:
    """
    One diagnostic snapshot (firmware 0.2.0, CAP_DIAGNOSTICS). Window counters cover the controller's last one-second status window; _total counters count since boot.
    """
    @property
    def can_bus_off(self) -> bool:
        """
        CAN controller bus-off.
        """
    @property
    def can_error_events(self) -> int:
        """
        CAN error events since boot.
        """
    @property
    def can_error_passive(self) -> bool:
        """
        CAN controller error-passive.
        """
    @property
    def can_rec(self) -> int:
        """
        CAN receive error counter.
        """
    @property
    def can_rx(self) -> int:
        """
        CAN frames received in the window.
        """
    @property
    def can_tec(self) -> int:
        """
        CAN transmit error counter.
        """
    @property
    def can_tx(self) -> int:
        """
        Drive frames queued in the window.
        """
    @property
    def can_tx_errors(self) -> int:
        """
        Drive frames that could not be queued in the window.
        """
    @property
    def can_tx_errors_total(self) -> int:
        """
        Drive frames not queued since boot.
        """
    @property
    def command_invalid(self) -> int:
        """
        Command datagrams rejected in the window.
        """
    @property
    def command_invalid_total(self) -> int:
        """
        Command datagrams rejected since boot.
        """
    @property
    def command_rx(self) -> int:
        """
        Command datagrams accepted in the window.
        """
    @property
    def control_hz(self) -> int:
        """
        Control ticks completed in the window (1000 when healthy).
        """
    @property
    def control_missed(self) -> int:
        """
        Control ticks missed in the window.
        """
    @property
    def control_missed_total(self) -> int:
        """
        Control ticks missed since boot.
        """
    @property
    def control_period_max_us(self) -> int:
        """
        Longest control period in the window, µs.
        """
    @property
    def controller_status(self) -> int:
        """
        Controller status code, now.
        """
    @property
    def drive_state(self) -> typing.Annotated[list[int], "FixedSize(8)"]:
        """
        Drive status code of J1–J8, now.
        """
    @property
    def heap_free_bytes(self) -> int:
        """
        FreeRTOS heap free now.
        """
    @property
    def heap_min_free_bytes(self) -> int:
        """
        FreeRTOS heap free at its lowest since boot.
        """
    @property
    def invalid_reason(self) -> int:
        """
        Why the last command datagram was rejected.
        """
    @property
    def snapshot_seconds(self) -> int:
        """
        Status-task second of the window.
        """
    @property
    def stack_free_control_bytes(self) -> int:
        """
        Least free stack ever of the control task.
        """
    @property
    def stack_free_ethernet_bytes(self) -> int:
        """
        Least free stack ever of the Ethernet task.
        """
    @property
    def stack_free_status_bytes(self) -> int:
        """
        Least free stack ever of the status task.
        """
    @property
    def telemetry_tx(self) -> int:
        """
        Telemetry packets sent in the window.
        """
    @property
    def uptime_ms(self) -> int:
        """
        Since the application started; wraps after 49 days.
        """
class FaultEvent:
    """
    A drive fault, a controlled stop or a self-check result, as it happened.
    """
    @property
    def code(self) -> int:
        """
        Drive, stop or self-check code.
        """
    @property
    def detail(self) -> int:
        """
        Code-specific detail.
        """
    @property
    def host_time_us(self) -> int:
        """
        PC steady clock, µs.
        """
    @property
    def joint(self) -> int:
        """
        Joint, 0-based; 255 if none.
        """
    @property
    def kind(self) -> FaultKind:
        """
        What happened (FaultKind).
        """
    @property
    def mcu_time_us(self) -> int:
        """
        Controller clock, µs.
        """
    @property
    def message(self) -> str:
        """
        What happened, the probable cause and what to do, in Chinese.
        """
class FaultKind:
    """
    What a FaultEvent records.

    Members:

      DriveFault

      DriveCleared

      Stop

      SelfCheck

      Link
    """
    DriveCleared: typing.ClassVar[FaultKind]  # value = <FaultKind.DriveCleared: 1>
    DriveFault: typing.ClassVar[FaultKind]  # value = <FaultKind.DriveFault: 0>
    Link: typing.ClassVar[FaultKind]  # value = <FaultKind.Link: 4>
    SelfCheck: typing.ClassVar[FaultKind]  # value = <FaultKind.SelfCheck: 3>
    Stop: typing.ClassVar[FaultKind]  # value = <FaultKind.Stop: 2>
    __members__: typing.ClassVar[dict[str, FaultKind]]  # value = {'DriveFault': <FaultKind.DriveFault: 0>, 'DriveCleared': <FaultKind.DriveCleared: 1>, 'Stop': <FaultKind.Stop: 2>, 'SelfCheck': <FaultKind.SelfCheck: 3>, 'Link': <FaultKind.Link: 4>}
    @typing.overload
    def __eq__(self, other: FaultKind) -> bool:
        ...
    @typing.overload
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    def __int__(self) -> int:
        ...
    @typing.overload
    def __ne__(self, other: FaultKind) -> bool:
        ...
    @typing.overload
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    def __str__(self) -> str:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class JointCommand:
    """
    An RT_JOINT command for J1–J7, sent with Robot.servej(). MIT profile: position and velocity with kp/kd plus a torque feed-forward. Position-velocity profile: position at no more than velocity_limit. Defaults: template gains and 3 rad/s.
    """
    def __init__(self) -> None:
        ...
    @property
    def kd(self) -> typing.Annotated[list[float], "FixedSize(7)"]:
        """
        MIT damping, 0–5.
        """
    @kd.setter
    def kd(self, arg0: typing.Annotated[collections.abc.Sequence[typing.SupportsFloat | typing.SupportsIndex], "FixedSize(7)"]) -> None:
        ...
    @property
    def kp(self) -> typing.Annotated[list[float], "FixedSize(7)"]:
        """
        MIT stiffness, 0–500.
        """
    @kp.setter
    def kp(self, arg0: typing.Annotated[collections.abc.Sequence[typing.SupportsFloat | typing.SupportsIndex], "FixedSize(7)"]) -> None:
        ...
    @property
    def position(self) -> typing.Annotated[list[float], "FixedSize(7)"]:
        """
        Target, rad.
        """
    @position.setter
    def position(self, arg0: typing.Annotated[collections.abc.Sequence[typing.SupportsFloat | typing.SupportsIndex], "FixedSize(7)"]) -> None:
        ...
    @property
    def torque(self) -> typing.Annotated[list[float], "FixedSize(7)"]:
        """
        MIT torque feed-forward, N·m.
        """
    @torque.setter
    def torque(self, arg0: typing.Annotated[collections.abc.Sequence[typing.SupportsFloat | typing.SupportsIndex], "FixedSize(7)"]) -> None:
        ...
    @property
    def velocity(self) -> typing.Annotated[list[float], "FixedSize(7)"]:
        """
        MIT velocity feed-forward, rad/s.
        """
    @velocity.setter
    def velocity(self, arg0: typing.Annotated[collections.abc.Sequence[typing.SupportsFloat | typing.SupportsIndex], "FixedSize(7)"]) -> None:
        ...
    @property
    def velocity_limit(self) -> typing.Annotated[list[float], "FixedSize(7)"]:
        """
        Position-velocity v_des, rad/s.
        """
    @velocity_limit.setter
    def velocity_limit(self, arg0: typing.Annotated[collections.abc.Sequence[typing.SupportsFloat | typing.SupportsIndex], "FixedSize(7)"]) -> None:
        ...
class JointState:
    """
    Measured state of the eight drives (joint frame, SI units), and with telemetry v3 what the controller commands them.
    """
    @property
    def command_position(self) -> typing.Annotated[list[float], "FixedSize(8)"]:
        """
        Planned command, rad.
        """
    @property
    def command_torque(self) -> typing.Annotated[list[float], "FixedSize(8)"]:
        """
        MIT torque feed-forward, N·m.
        """
    @property
    def command_velocity(self) -> typing.Annotated[list[float], "FixedSize(8)"]:
        """
        MIT velocity feed-forward or position-velocity v_des, rad/s.
        """
    @property
    def drive_mode(self) -> typing.Annotated[list[int], "FixedSize(8)"]:
        """
        Drive mode code of each drive.
        """
    @property
    def drive_state(self) -> typing.Annotated[list[int], "FixedSize(8)"]:
        """
        Drive status code: 0 disabled, 1 enabled, 8 and above a fault.
        """
    @property
    def feedback_age_ms(self) -> typing.Annotated[list[int], "FixedSize(8)"]:
        """
        Age of each drive's feedback, ms; 4294967295 before the first.
        """
    @property
    def feedback_count(self) -> typing.Annotated[list[int], "FixedSize(8)"]:
        """
        Drive feedback counter; it stops when a drive goes quiet.
        """
    @property
    def has_command(self) -> bool:
        """
        The command fields are valid (telemetry v3).
        """
    @property
    def host_time_us(self) -> int:
        """
        CLOCK_MONOTONIC when the packet arrived, µs.
        """
    @property
    def motor_flags(self) -> typing.Annotated[list[int], "FixedSize(8)"]:
        """
        Motor flag bits.
        """
    @property
    def position(self) -> typing.Annotated[list[float], "FixedSize(8)"]:
        """
        Position, rad.
        """
    @property
    def sequence(self) -> int:
        """
        Telemetry sequence number.
        """
    @property
    def temperature(self) -> typing.Annotated[list[int], "FixedSize(8)"]:
        """
        MOS temperature, °C.
        """
    @property
    def temperature_coil(self) -> typing.Annotated[list[int], "FixedSize(8)"]:
        """
        Coil temperature, °C.
        """
    @property
    def timestamp_us(self) -> int:
        """
        Controller clock, µs.
        """
    @property
    def torque(self) -> typing.Annotated[list[float], "FixedSize(8)"]:
        """
        Torque, N·m.
        """
    @property
    def velocity(self) -> typing.Annotated[list[float], "FixedSize(8)"]:
        """
        Velocity, rad/s.
        """
class JointStreamOptions:
    """
    Options of the native RT_JOINT producer (Robot.start_joint_stream()).
    """
    def __init__(self) -> None:
        ...
    @property
    def allow_zero_kp(self) -> bool:
        """
        Accept kp = 0; off because the controller's STOP does not restore holding gains.
        """
    @allow_zero_kp.setter
    def allow_zero_kp(self, arg0: bool) -> None:
        ...
    @property
    def command_timeout_ms(self) -> int:
        """
        Stop when servej() has not refreshed the command for this long.
        """
    @command_timeout_ms.setter
    def command_timeout_ms(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    @property
    def feedback_timeout_ms(self) -> int:
        """
        Stop when drive feedback is older than this.
        """
    @feedback_timeout_ms.setter
    def feedback_timeout_ms(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    @property
    def transmit_hz(self) -> int:
        """
        Send rate.
        """
    @transmit_hz.setter
    def transmit_hz(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
class JointStreamStats:
    """
    State of the native RT_JOINT producer.
    """
    @property
    def active(self) -> bool:
        """
        The producer is sending.
        """
    @property
    def fault(self) -> str:
        """
        Why it stopped; latched, so a failed stream needs an explicit start.
        """
    @property
    def sent(self) -> int:
        """
        Commands sent.
        """
    @property
    def skipped_deadlines(self) -> int:
        """
        Send deadlines missed.
        """
class LinkStats:
    """
    Quality of the UDP link. The last-second values are refreshed once a second.
    """
    @property
    def loss_percent(self) -> float:
        """
        Loss in the last second, %.
        """
    @property
    def lost_last_second(self) -> int:
        """
        Telemetry packets lost in the last second.
        """
    @property
    def mcu(self) -> ControllerLinkStats:
        """
        The controller's view of the link (ControllerLinkStats).
        """
    @property
    def mcu_valid(self) -> bool:
        """
        `mcu` is valid (telemetry v3).
        """
    @property
    def rx_interval_max_us(self) -> int:
        """
        Longest gap between telemetry packets in the last second, µs.
        """
    @property
    def rx_invalid(self) -> int:
        """
        Datagrams not understood.
        """
    @property
    def rx_last_second(self) -> int:
        """
        Telemetry packets in the last second.
        """
    @property
    def rx_lost(self) -> int:
        """
        Telemetry sequence gaps.
        """
    @property
    def rx_telemetry(self) -> int:
        """
        Telemetry packets received.
        """
    @property
    def tx_errors(self) -> int:
        """
        Datagrams that failed to send.
        """
    @property
    def tx_sent(self) -> int:
        """
        Datagrams sent.
        """
class McuLogOptions:
    """
    Options of Robot.download_mcu_log(). The board keeps the newest 10 minutes of text and 10 s of 1 kHz telemetry in SDRAM; they are lost on power-off or reset.
    """
    def __init__(self) -> None:
        ...
    @property
    def directory(self) -> str:
        """
        Output directory; empty selects Robot.log_directory().
        """
    @directory.setter
    def directory(self, arg0: str) -> None:
        ...
    @property
    def label(self) -> str:
        """
        Appended to the file names.
        """
    @label.setter
    def label(self, arg0: str) -> None:
        ...
    @property
    def port(self) -> int:
        """
        Log port of the board.
        """
    @port.setter
    def port(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    @property
    def progress(self) -> collections.abc.Callable[[typing.SupportsInt | typing.SupportsIndex, typing.SupportsInt | typing.SupportsIndex, typing.SupportsInt | typing.SupportsIndex, typing.SupportsInt | typing.SupportsIndex], bool]:
        """
        Called with (text_lines, text_total, telemetry_rows, telemetry_total); return False to cancel.
        """
    @progress.setter
    def progress(self, arg0: collections.abc.Callable[[int, int, int, int], bool]) -> None:
        ...
    @property
    def retries(self) -> int:
        """
        Retries per request.
        """
    @retries.setter
    def retries(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    @property
    def timeout_ms(self) -> int:
        """
        Timeout per request.
        """
    @timeout_ms.setter
    def timeout_ms(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
class McuLogResult:
    """
    Files written by Robot.download_mcu_log().
    """
    @property
    def boot_id(self) -> int:
        """
        Changes on every controller boot.
        """
    @property
    def fault_latched(self) -> bool:
        """
        The board froze the telemetry at a fault: the CSV ends at that tick.
        """
    @property
    def lost_bytes(self) -> int:
        """
        printf bytes the board could not keep.
        """
    @property
    def telemetry_path(self) -> str:
        """
        The telemetry CSV.
        """
    @property
    def telemetry_rows(self) -> int:
        """
        Telemetry rows saved.
        """
    @property
    def text_lines(self) -> int:
        """
        Text lines saved.
        """
    @property
    def text_path(self) -> str:
        """
        The text log, lines "<mcu_time_s> <line>".
        """
    @property
    def writer_max_us(self) -> int:
        """
        Longest log-task pass since the previous download, µs.
        """
class Mode:
    """
    Control mode of the controller. The admittance modes need every joint 0.03 rad inside its limits.

    Members:

      Idle

      RtJoint

      RtCartesian

      NrtJoint

      NrtCartesian

      GravityFree

      GravityLock

      JointImpedance

      CartesianImpedance

      JointAdmittance

      CartesianAdmittance

      Unknown
    """
    CartesianAdmittance: typing.ClassVar[Mode]  # value = <Mode.CartesianAdmittance: 10>
    CartesianImpedance: typing.ClassVar[Mode]  # value = <Mode.CartesianImpedance: 8>
    GravityFree: typing.ClassVar[Mode]  # value = <Mode.GravityFree: 5>
    GravityLock: typing.ClassVar[Mode]  # value = <Mode.GravityLock: 6>
    Idle: typing.ClassVar[Mode]  # value = <Mode.Idle: 0>
    JointAdmittance: typing.ClassVar[Mode]  # value = <Mode.JointAdmittance: 9>
    JointImpedance: typing.ClassVar[Mode]  # value = <Mode.JointImpedance: 7>
    NrtCartesian: typing.ClassVar[Mode]  # value = <Mode.NrtCartesian: 4>
    NrtJoint: typing.ClassVar[Mode]  # value = <Mode.NrtJoint: 3>
    RtCartesian: typing.ClassVar[Mode]  # value = <Mode.RtCartesian: 2>
    RtJoint: typing.ClassVar[Mode]  # value = <Mode.RtJoint: 1>
    Unknown: typing.ClassVar[Mode]  # value = <Mode.Unknown: 255>
    __members__: typing.ClassVar[dict[str, Mode]]  # value = {'Idle': <Mode.Idle: 0>, 'RtJoint': <Mode.RtJoint: 1>, 'RtCartesian': <Mode.RtCartesian: 2>, 'NrtJoint': <Mode.NrtJoint: 3>, 'NrtCartesian': <Mode.NrtCartesian: 4>, 'GravityFree': <Mode.GravityFree: 5>, 'GravityLock': <Mode.GravityLock: 6>, 'JointImpedance': <Mode.JointImpedance: 7>, 'CartesianImpedance': <Mode.CartesianImpedance: 8>, 'JointAdmittance': <Mode.JointAdmittance: 9>, 'CartesianAdmittance': <Mode.CartesianAdmittance: 10>, 'Unknown': <Mode.Unknown: 255>}
    @typing.overload
    def __eq__(self, other: Mode) -> bool:
        ...
    @typing.overload
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    def __int__(self) -> int:
        ...
    @typing.overload
    def __ne__(self, other: Mode) -> bool:
        ...
    @typing.overload
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    def __str__(self) -> str:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class MotionPhase:
    """
    Phase of the movej/movep session.

    Members:

      Off

      Preparing

      Waiting

      Running

      Settling

      Ready

      Done

      Cancelling

      Cancelled

      Error
    """
    Cancelled: typing.ClassVar[MotionPhase]  # value = <MotionPhase.Cancelled: 8>
    Cancelling: typing.ClassVar[MotionPhase]  # value = <MotionPhase.Cancelling: 7>
    Done: typing.ClassVar[MotionPhase]  # value = <MotionPhase.Done: 6>
    Error: typing.ClassVar[MotionPhase]  # value = <MotionPhase.Error: 9>
    Off: typing.ClassVar[MotionPhase]  # value = <MotionPhase.Off: 0>
    Preparing: typing.ClassVar[MotionPhase]  # value = <MotionPhase.Preparing: 1>
    Ready: typing.ClassVar[MotionPhase]  # value = <MotionPhase.Ready: 5>
    Running: typing.ClassVar[MotionPhase]  # value = <MotionPhase.Running: 3>
    Settling: typing.ClassVar[MotionPhase]  # value = <MotionPhase.Settling: 4>
    Waiting: typing.ClassVar[MotionPhase]  # value = <MotionPhase.Waiting: 2>
    __members__: typing.ClassVar[dict[str, MotionPhase]]  # value = {'Off': <MotionPhase.Off: 0>, 'Preparing': <MotionPhase.Preparing: 1>, 'Waiting': <MotionPhase.Waiting: 2>, 'Running': <MotionPhase.Running: 3>, 'Settling': <MotionPhase.Settling: 4>, 'Ready': <MotionPhase.Ready: 5>, 'Done': <MotionPhase.Done: 6>, 'Cancelling': <MotionPhase.Cancelling: 7>, 'Cancelled': <MotionPhase.Cancelled: 8>, 'Error': <MotionPhase.Error: 9>}
    @typing.overload
    def __eq__(self, other: MotionPhase) -> bool:
        ...
    @typing.overload
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    def __int__(self) -> int:
        ...
    @typing.overload
    def __ne__(self, other: MotionPhase) -> bool:
        ...
    @typing.overload
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    def __str__(self) -> str:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class MotionResult:
    """
    Outcome of a move.
    """
    @property
    def arrived_mask(self) -> int:
        """
        Joints within tolerance (bit i = J(i+1)).
        """
    @property
    def fresh_mask(self) -> int:
        """
        Joints with fresh feedback.
        """
    @property
    def max_error_rad(self) -> float:
        """
        Largest remaining error: rad for joint moves, m or rad for Cartesian moves.
        """
    @property
    def message(self) -> str:
        """
        Why it failed, in Chinese; empty on arrival.
        """
    @property
    def motion_id(self) -> int:
        """
        The move this result belongs to.
        """
    @property
    def reason(self) -> int:
        """
        Reason code the move ended.
        """
    @property
    def result(self) -> int:
        """
        0 means arrived; otherwise the move result code.
        """
    @property
    def state(self) -> int:
        """
        Move state code.
        """
class MotionStatus:
    """
    The movej/movep session of the NRT modes. Returned by Robot.motion().
    """
    @property
    def can_execute(self) -> bool:
        """
        A new move is accepted now.
        """
    @property
    def error(self) -> str:
        """
        Why the session failed, in Chinese.
        """
    @property
    def has_reply(self) -> bool:
        """
        `reply` is valid.
        """
    @property
    def id(self) -> int:
        """
        The newest move; 1 is the hold made on entering the mode.
        """
    @property
    def phase(self) -> MotionPhase:
        """
        Session phase (MotionPhase).
        """
    @property
    def reply(self) -> MoveReply:
        """
        The controller's newest reply for the move (MoveReply).
        """
    @property
    def reply_age_us(self) -> int:
        """
        Age of `reply`, µs.
        """
class MotorGroup:
    """
    Drive groups enabled and disabled together: All (J1–J8), Arm (J1–J7), Gripper (J8).

    Members:

      All

      Arm

      Gripper
    """
    All: typing.ClassVar[MotorGroup]  # value = <MotorGroup.All: 255>
    Arm: typing.ClassVar[MotorGroup]  # value = <MotorGroup.Arm: 127>
    Gripper: typing.ClassVar[MotorGroup]  # value = <MotorGroup.Gripper: 128>
    __members__: typing.ClassVar[dict[str, MotorGroup]]  # value = {'All': <MotorGroup.All: 255>, 'Arm': <MotorGroup.Arm: 127>, 'Gripper': <MotorGroup.Gripper: 128>}
    @typing.overload
    def __eq__(self, other: MotorGroup) -> bool:
        ...
    @typing.overload
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    def __int__(self) -> int:
        ...
    @typing.overload
    def __ne__(self, other: MotorGroup) -> bool:
        ...
    @typing.overload
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    def __str__(self) -> str:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class MotorProfile:
    """
    Drive mode of J1–J7: Mit (position, velocity, kp/kd and a torque feed-forward) or PositionVelocity (position with a velocity limit). J8 is always force-position.

    Members:

      Mit

      PositionVelocity
    """
    Mit: typing.ClassVar[MotorProfile]  # value = <MotorProfile.Mit: 1>
    PositionVelocity: typing.ClassVar[MotorProfile]  # value = <MotorProfile.PositionVelocity: 2>
    __members__: typing.ClassVar[dict[str, MotorProfile]]  # value = {'Mit': <MotorProfile.Mit: 1>, 'PositionVelocity': <MotorProfile.PositionVelocity: 2>}
    @typing.overload
    def __eq__(self, other: MotorProfile) -> bool:
        ...
    @typing.overload
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    def __int__(self) -> int:
        ...
    @typing.overload
    def __ne__(self, other: MotorProfile) -> bool:
        ...
    @typing.overload
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    def __str__(self) -> str:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class MoveOptions:
    """
    Options of a joint move (Robot.movej()). Defaults: 0.1 rad/s, 0.2 rad/s² and the template gains.
    """
    def __init__(self) -> None:
        ...
    @property
    def acceleration_max(self) -> typing.Annotated[list[float], "FixedSize(7)"]:
        """
        Acceleration cap per joint, rad/s².
        """
    @acceleration_max.setter
    def acceleration_max(self, arg0: typing.Annotated[collections.abc.Sequence[typing.SupportsFloat | typing.SupportsIndex], "FixedSize(7)"]) -> None:
        ...
    @property
    def dwell_ms(self) -> int:
        """
        How long the arm must stay within tolerance to count as arrived.
        """
    @dwell_ms.setter
    def dwell_ms(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    @property
    def kd(self) -> typing.Annotated[list[float], "FixedSize(7)"]:
        """
        MIT damping.
        """
    @kd.setter
    def kd(self, arg0: typing.Annotated[collections.abc.Sequence[typing.SupportsFloat | typing.SupportsIndex], "FixedSize(7)"]) -> None:
        ...
    @property
    def kp(self) -> typing.Annotated[list[float], "FixedSize(7)"]:
        """
        MIT stiffness.
        """
    @kp.setter
    def kp(self, arg0: typing.Annotated[collections.abc.Sequence[typing.SupportsFloat | typing.SupportsIndex], "FixedSize(7)"]) -> None:
        ...
    @property
    def position_tolerance(self) -> float:
        """
        Arrival tolerance per joint, rad; 0.001–0.05.
        """
    @position_tolerance.setter
    def position_tolerance(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...
    @property
    def timeout_ms(self) -> int:
        """
        The move fails if it has not arrived by then.
        """
    @timeout_ms.setter
    def timeout_ms(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    @property
    def torque(self) -> typing.Annotated[list[float], "FixedSize(7)"]:
        """
        MIT torque feed-forward, N·m.
        """
    @torque.setter
    def torque(self, arg0: typing.Annotated[collections.abc.Sequence[typing.SupportsFloat | typing.SupportsIndex], "FixedSize(7)"]) -> None:
        ...
    @property
    def velocity_max(self) -> typing.Annotated[list[float], "FixedSize(7)"]:
        """
        Speed cap per joint, rad/s (position-velocity: also v_des).
        """
    @velocity_max.setter
    def velocity_max(self, arg0: typing.Annotated[collections.abc.Sequence[typing.SupportsFloat | typing.SupportsIndex], "FixedSize(7)"]) -> None:
        ...
    @property
    def velocity_tolerance(self) -> float:
        """
        Arrival speed tolerance per joint, rad/s.
        """
    @velocity_tolerance.setter
    def velocity_tolerance(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...
class MoveReply:
    """
    The controller's reply about a move (DMOS).
    """
    @property
    def active_id(self) -> int:
        """
        The move the controller is executing.
        """
    @property
    def arrived_mask(self) -> int:
        """
        Joints within tolerance.
        """
    @property
    def duration_ms(self) -> int:
        """
        Planned duration, ms.
        """
    @property
    def elapsed_ms(self) -> int:
        """
        Time since the move started, ms.
        """
    @property
    def enabled_mask(self) -> int:
        """
        Enabled drives.
        """
    @property
    def fresh_mask(self) -> int:
        """
        Joints with fresh feedback.
        """
    @property
    def max_error_rad(self) -> float:
        """
        Largest remaining error.
        """
    @property
    def reason(self) -> int:
        """
        Reason code.
        """
    @property
    def request_id(self) -> int:
        """
        The move this reply answers.
        """
    @property
    def result(self) -> int:
        """
        Result code.
        """
    @property
    def session(self) -> int:
        """
        Session identifier.
        """
    @property
    def state(self) -> int:
        """
        Move state code.
        """
class OperationalStatus:
    """
    What the controller is doing now, from telemetry.

    Members:

      Booting

      Precheck

      ProfileRequired

      Disabled

      Enabling

      IdleHold

      RtJoint

      RtCartesian

      NrtJoint

      NrtCartesian

      GravityFree

      GravityLock

      JointImpedance

      CartesianImpedance

      JointAdmittance

      CartesianAdmittance

      Stopping

      Fault

      Recovery

      Estop

      Unknown
    """
    Booting: typing.ClassVar[OperationalStatus]  # value = <OperationalStatus.Booting: 0>
    CartesianAdmittance: typing.ClassVar[OperationalStatus]  # value = <OperationalStatus.CartesianAdmittance: 20>
    CartesianImpedance: typing.ClassVar[OperationalStatus]  # value = <OperationalStatus.CartesianImpedance: 18>
    Disabled: typing.ClassVar[OperationalStatus]  # value = <OperationalStatus.Disabled: 3>
    Enabling: typing.ClassVar[OperationalStatus]  # value = <OperationalStatus.Enabling: 4>
    Estop: typing.ClassVar[OperationalStatus]  # value = <OperationalStatus.Estop: 13>
    Fault: typing.ClassVar[OperationalStatus]  # value = <OperationalStatus.Fault: 11>
    GravityFree: typing.ClassVar[OperationalStatus]  # value = <OperationalStatus.GravityFree: 15>
    GravityLock: typing.ClassVar[OperationalStatus]  # value = <OperationalStatus.GravityLock: 16>
    IdleHold: typing.ClassVar[OperationalStatus]  # value = <OperationalStatus.IdleHold: 5>
    JointAdmittance: typing.ClassVar[OperationalStatus]  # value = <OperationalStatus.JointAdmittance: 19>
    JointImpedance: typing.ClassVar[OperationalStatus]  # value = <OperationalStatus.JointImpedance: 17>
    NrtCartesian: typing.ClassVar[OperationalStatus]  # value = <OperationalStatus.NrtCartesian: 9>
    NrtJoint: typing.ClassVar[OperationalStatus]  # value = <OperationalStatus.NrtJoint: 8>
    Precheck: typing.ClassVar[OperationalStatus]  # value = <OperationalStatus.Precheck: 1>
    ProfileRequired: typing.ClassVar[OperationalStatus]  # value = <OperationalStatus.ProfileRequired: 2>
    Recovery: typing.ClassVar[OperationalStatus]  # value = <OperationalStatus.Recovery: 12>
    RtCartesian: typing.ClassVar[OperationalStatus]  # value = <OperationalStatus.RtCartesian: 7>
    RtJoint: typing.ClassVar[OperationalStatus]  # value = <OperationalStatus.RtJoint: 6>
    Stopping: typing.ClassVar[OperationalStatus]  # value = <OperationalStatus.Stopping: 10>
    Unknown: typing.ClassVar[OperationalStatus]  # value = <OperationalStatus.Unknown: 14>
    __members__: typing.ClassVar[dict[str, OperationalStatus]]  # value = {'Booting': <OperationalStatus.Booting: 0>, 'Precheck': <OperationalStatus.Precheck: 1>, 'ProfileRequired': <OperationalStatus.ProfileRequired: 2>, 'Disabled': <OperationalStatus.Disabled: 3>, 'Enabling': <OperationalStatus.Enabling: 4>, 'IdleHold': <OperationalStatus.IdleHold: 5>, 'RtJoint': <OperationalStatus.RtJoint: 6>, 'RtCartesian': <OperationalStatus.RtCartesian: 7>, 'NrtJoint': <OperationalStatus.NrtJoint: 8>, 'NrtCartesian': <OperationalStatus.NrtCartesian: 9>, 'GravityFree': <OperationalStatus.GravityFree: 15>, 'GravityLock': <OperationalStatus.GravityLock: 16>, 'JointImpedance': <OperationalStatus.JointImpedance: 17>, 'CartesianImpedance': <OperationalStatus.CartesianImpedance: 18>, 'JointAdmittance': <OperationalStatus.JointAdmittance: 19>, 'CartesianAdmittance': <OperationalStatus.CartesianAdmittance: 20>, 'Stopping': <OperationalStatus.Stopping: 10>, 'Fault': <OperationalStatus.Fault: 11>, 'Recovery': <OperationalStatus.Recovery: 12>, 'Estop': <OperationalStatus.Estop: 13>, 'Unknown': <OperationalStatus.Unknown: 14>}
    @typing.overload
    def __eq__(self, other: OperationalStatus) -> bool:
        ...
    @typing.overload
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    def __int__(self) -> int:
        ...
    @typing.overload
    def __ne__(self, other: OperationalStatus) -> bool:
        ...
    @typing.overload
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    def __str__(self) -> str:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class ParameterInfo:
    """
    One tunable controller parameter (firmware 0.2.0, CAP_PARAMETERS).
    """
    @property
    def applied(self) -> float:
        """
        The value the controller uses now, slewing toward `requested`.
        """
    @property
    def boolean(self) -> bool:
        """
        The value is 0 or 1.
        """
    @property
    def default_value(self) -> float:
        """
        Value after a reboot.
        """
    @property
    def group(self) -> int:
        """
        PARAM_GROUP_GRAVITY, _IMPEDANCE or _ADMITTANCE.
        """
    @property
    def id(self) -> int:
        """
        group << 8 | index.
        """
    @property
    def maximum(self) -> float:
        """
        Highest accepted value.
        """
    @property
    def minimum(self) -> float:
        """
        Lowest accepted value.
        """
    @property
    def name(self) -> str:
        """
        E.g. "imp.joint_k".
        """
    @property
    def rate(self) -> float:
        """
        Largest change of the applied value, units per second.
        """
    @property
    def requested(self) -> float:
        """
        The last value written, or the default after a reboot.
        """
    @property
    def unit(self) -> str:
        """
        E.g. "Nm/rad"; empty when dimensionless.
        """
    @property
    def writable(self) -> bool:
        """
        set_parameter() accepts it.
        """
class RecordOptions:
    """
    Options of Robot.start_recording().
    """
    def __init__(self) -> None:
        ...
    @property
    def commands(self) -> bool:
        """
        Also log every command datagram sent.
        """
    @commands.setter
    def commands(self, arg0: bool) -> None:
        ...
    @property
    def label(self) -> str:
        """
        Appended to the directory name.
        """
    @label.setter
    def label(self, arg0: str) -> None:
        ...
    @property
    def rotate_seconds(self) -> int:
        """
        Start new telemetry and command files this often; 0 never.
        """
    @rotate_seconds.setter
    def rotate_seconds(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
class RecorderStats:
    """
    Progress of a recording. Returned by Robot.recording().
    """
    @property
    def active(self) -> bool:
        """
        A recording is running.
        """
    @property
    def command_rows(self) -> int:
        """
        Command rows written.
        """
    @property
    def directory(self) -> str:
        """
        Directory being written.
        """
    @property
    def dropped(self) -> int:
        """
        Rows lost because the disk could not keep up.
        """
    @property
    def error(self) -> str:
        """
        Last write error.
        """
    @property
    def events(self) -> int:
        """
        Event rows written.
        """
    @property
    def file_index(self) -> int:
        """
        Index of the current telemetry/command file.
        """
    @property
    def telemetry_rows(self) -> int:
        """
        Telemetry rows written.
        """
class Robot:
    """
    The client for one arm controller.

    Robot(address="192.168.0.50", command_port=5001, telemetry_port=5000) creates the
    client; connect() opens the sockets. It runs a receive thread and a 1 kHz service
    thread (heartbeat, NRT session, native joint stream). Calls release the GIL.
    A call that fails raises RuntimeError with the reason, in Chinese.
    """
    @staticmethod
    def forward(joints: typing.Annotated[collections.abc.Sequence[typing.SupportsFloat | typing.SupportsIndex], "FixedSize(7)"]) -> typing.Annotated[list[float], "FixedSize(12)"]:
        """
        [Non-blocking] Forward kinematics of the H7C arm: J1–J7 in rad to a tool pose (12 values).
        """
    @staticmethod
    def log_directory() -> str:
        """
        [Non-blocking] Default SDK log directory: $INFRABOT_LOG_DIR, else $XDG_STATE_HOME/infrabot/log, else ~/.local/state/infrabot/log.
        """
    def __init__(self, address: str = '192.168.0.50', command_port: typing.SupportsInt | typing.SupportsIndex = 5001, telemetry_port: typing.SupportsInt | typing.SupportsIndex = 5000) -> None:
        """
        [Non-blocking] Creates the client for the controller at `address`. connect() opens the sockets.
        """
    def cancel_motion(self) -> None:
        """
        [Non-blocking] Ends the session's move with a controlled stop; the arm holds in IDLE.
        """
    def clear_error(self, motor: typing.SupportsInt | typing.SupportsIndex) -> None:
        """
        [Blocking, 1.5 s] Maintenance: clears one drive's fault.
        """
    def clear_fault(self, timeout_ms: typing.SupportsInt | typing.SupportsIndex = 1000) -> None:
        """
        [Blocking, timeout_ms] Clears the disabled drives that report a fault and waits until none does. An enabled faulted drive needs servo_off() first.
        """
    def connect(self) -> None:
        """
        [Blocking, up to 0.5 s] Negotiates the protocol, then starts the heartbeat. A different protocol major version or no valid reply raises. Check state().telemetry_age_ms for live drive feedback.
        """
    def connected(self) -> bool:
        """
        [Non-blocking] The sockets are open.
        """
    def diagnostics(self) -> DiagnosticsInfo:
        """
        [Blocking, up to 600 ms] One diagnostic snapshot (DiagnosticsInfo). Firmware 0.2.0 (CAP_DIAGNOSTICS); read only and open to any PC.
        """
    def disable(self, motor: typing.SupportsInt | typing.SupportsIndex) -> None:
        """
        [Blocking, 1.5 s] Maintenance: disables one drive.
        """
    def disconnect(self) -> None:
        """
        [Blocking] Closes the sockets and sends nothing. An enabled arm notices the PC is gone after 100 ms, stops in a controlled way and holds. Call stop() first for an immediate stop.
        """
    def download_mcu_log(self, options: McuLogOptions = ...) -> McuLogResult:
        """
        [Blocking, a few s] Downloads the controller's run log into two new files and returns a McuLogResult. Needs no connect(). The board refuses while an RT stream runs.
        """
    def enable(self, motor: typing.SupportsInt | typing.SupportsIndex) -> None:
        """
        [Blocking, 1.5 s] Maintenance: enables one drive directly.
        """
    def ensure_mode(self, motor: typing.SupportsInt | typing.SupportsIndex, mode: typing.SupportsInt | typing.SupportsIndex) -> None:
        """
        [Blocking, 1.5 s] Maintenance: switches one drive to a drive mode (1–4) if needed.
        """
    def fault_log(self) -> list[FaultEvent]:
        """
        [Non-blocking] Faults, stops and self-checks, oldest first, at most 200 (list of FaultEvent).
        """
    @typing.overload
    def get_parameter(self, id: typing.SupportsInt | typing.SupportsIndex) -> ParameterInfo:
        """
        [Blocking] Reads one parameter by id (group << 8 | index).
        """
    @typing.overload
    def get_parameter(self, name: str) -> ParameterInfo:
        """
        [Blocking] Reads one parameter by name, e.g. "imp.joint_k".
        """
    def get_register(self, motor: typing.SupportsInt | typing.SupportsIndex, register_id: typing.SupportsInt | typing.SupportsIndex, value_type: typing.SupportsInt | typing.SupportsIndex) -> int:
        """
        [Blocking, 1.5 s] Maintenance: get_register(motor, register_id, value_type) returns the raw register bits. value_type is REGISTER_U32 or REGISTER_F32 and must match the register.
        """
    def get_state(self, motor: typing.SupportsInt | typing.SupportsIndex) -> None:
        """
        [Blocking, 1.5 s] Maintenance: checks a drive's feedback freshness.
        """
    def gripper(self, position_rad: typing.SupportsFloat | typing.SupportsIndex, speed_rad_s: typing.SupportsFloat | typing.SupportsIndex, force_ratio: typing.SupportsFloat | typing.SupportsIndex) -> None:
        """
        [Blocking, 1 s] gripper(position_rad, speed_rad_s, force_ratio) moves the gripper to 0–5.94 rad. force_ratio is the current limit as a fraction of the drive maximum (0–1). Fails while the gripper group is disabled.
        """
    def hold_command(self) -> typing.Annotated[list[float], "FixedSize(8)"]:
        """
        [Non-blocking] The hold command read when the mode was entered (joint frame, rad, 8 values). RT streams should start here.
        """
    def info(self) -> RobotInfo:
        """
        [Non-blocking] Identity, limits and capabilities from the handshake (RobotInfo).
        """
    def joint_stream_stats(self) -> JointStreamStats:
        """
        [Non-blocking] State of the native producer (JointStreamStats).
        """
    def link_stats(self) -> LinkStats:
        """
        [Non-blocking] Link quality (LinkStats).
        """
    def list_parameters(self) -> list[ParameterInfo]:
        """
        [Blocking, about 10 ms per parameter] Lists every controller parameter (list of ParameterInfo). Firmware 0.2.0 (CAP_PARAMETERS).
        """
    def motion(self) -> MotionStatus:
        """
        [Non-blocking] State of the movej/movep session (MotionStatus).
        """
    def movej(self, position: typing.Annotated[collections.abc.Sequence[typing.SupportsFloat | typing.SupportsIndex], "FixedSize(7)"], options: MoveOptions = ...) -> MotionResult:
        """
        [Blocking] Moves J1–J7 to `position` (rad) in NRT_JOINT and waits until arrival, failure or options.timeout_ms + 3 s. Returns a MotionResult whose result is 0 on arrival.
        """
    def movep(self, pose: typing.Annotated[collections.abc.Sequence[typing.SupportsFloat | typing.SupportsIndex], "FixedSize(12)"], options: CartesianMoveOptions = ...) -> MotionResult:
        """
        [Blocking] Moves the tool to `pose` along a straight line in NRT_CARTESIAN. Returns a MotionResult whose result is 0 on arrival.
        """
    def read_hold_command(self) -> typing.Annotated[list[float], "FixedSize(8)"]:
        """
        [Blocking, 0.5 s] Reads the hold command of the enabled drives now.
        """
    def record_event(self, text: str) -> None:
        """
        [Non-blocking] Adds a marker to events.csv.
        """
    def recording(self) -> RecorderStats:
        """
        [Non-blocking] Progress of the recording (RecorderStats).
        """
    def request_feedback(self, motor: typing.SupportsInt | typing.SupportsIndex) -> None:
        """
        [Blocking, 1.5 s] Maintenance: asks drive `motor` (0–7) for a feedback frame.
        """
    def resume(self) -> Mode:
        """
        [Blocking, 3 s] Re-enters the mode a controlled stop interrupted, from where the arm stopped, and returns that Mode. A stop that happened in IDLE is only acknowledged. Refuses a disabled or faulted arm drive.
        """
    def send_force_pos(self, motor: typing.SupportsInt | typing.SupportsIndex, position_rad: typing.SupportsFloat | typing.SupportsIndex, velocity_limit_rad_s: typing.SupportsFloat | typing.SupportsIndex, torque_ratio: typing.SupportsFloat | typing.SupportsIndex) -> None:
        """
        [Blocking, 1.5 s] Maintenance: send_force_pos(motor, position, velocity_limit, torque_ratio) sends one force-position frame.
        """
    def send_mit(self, motor: typing.SupportsInt | typing.SupportsIndex, position_rad: typing.SupportsFloat | typing.SupportsIndex, velocity_rad_s: typing.SupportsFloat | typing.SupportsIndex, kp: typing.SupportsFloat | typing.SupportsIndex, kd: typing.SupportsFloat | typing.SupportsIndex, torque_nm: typing.SupportsFloat | typing.SupportsIndex) -> None:
        """
        [Blocking, 1.5 s] Maintenance: send_mit(motor, position, velocity, kp, kd, torque) sends one MIT frame.
        """
    def send_pos_vel(self, motor: typing.SupportsInt | typing.SupportsIndex, position_rad: typing.SupportsFloat | typing.SupportsIndex, velocity_limit_rad_s: typing.SupportsFloat | typing.SupportsIndex) -> None:
        """
        [Blocking, 1.5 s] Maintenance: send_pos_vel(motor, position, velocity_limit) sends one position-velocity frame.
        """
    def send_vel(self, motor: typing.SupportsInt | typing.SupportsIndex, velocity_rad_s: typing.SupportsFloat | typing.SupportsIndex) -> None:
        """
        [Blocking, 1.5 s] Maintenance: send_vel(motor, velocity) sends one velocity frame.
        """
    def serial_log(self) -> SerialLogStats:
        """
        [Non-blocking] Progress of the serial log (SerialLogStats).
        """
    def servej(self, command: JointCommand) -> None:
        """
        [RT] Sends one RT_JOINT command (JointCommand). While the native joint stream runs, publishes the command to it instead.
        """
    def servep(self, command: CartesianCommand) -> None:
        """
        [RT] Sends one RT_CARTESIAN command (CartesianCommand).
        """
    def servo_off(self, group: MotorGroup = ..., wait: bool = True) -> None:
        """
        [Blocking, 2 s] Disables a group (default all) at once, in any mode. No controlled stop: the arm falls. wait=False sends the request twice and returns.
        """
    def servo_on(self, group: MotorGroup = ...) -> None:
        """
        [Blocking, 3 s] Enables a drive group (default MotorGroup.All) in IDLE and waits for telemetry to show it. On timeout that group is disabled again.
        """
    def set_can_timeout_ms(self, motor: typing.SupportsInt | typing.SupportsIndex, timeout_ms: typing.SupportsInt | typing.SupportsIndex) -> None:
        """
        [Blocking, 1.5 s] Maintenance: sets one drive's CAN timeout.
        """
    def set_default_cartesian_options(self, options: CartesianMoveOptions) -> None:
        """
        [Non-blocking] Options of the hold move that opens an NRT_CARTESIAN session.
        """
    def set_default_move_options(self, options: MoveOptions) -> None:
        """
        [Non-blocking] Options of the hold move that opens an NRT_JOINT session.
        """
    def set_heartbeat(self, enabled: bool) -> None:
        """
        [Non-blocking] Turns the heartbeat on or off; on by default. Off: this process only watches while another program controls the arm.
        """
    def set_motor_profile(self, profile: MotorProfile) -> None:
        """
        [Blocking, 3 s] Selects the drive mode of J1–J7 (MotorProfile) with every drive disabled in IDLE. The controller then checks every drive's parameters and refuses to enable on a mismatch; the error names the drive and register.
        """
    @typing.overload
    def set_parameter(self, id: typing.SupportsInt | typing.SupportsIndex, value: typing.SupportsFloat | typing.SupportsIndex) -> ParameterInfo:
        """
        [Blocking] Requests a new value for a parameter by id and returns the ParameterInfo after the write. The controller slews the applied value toward it. RAM only; needs the PC that controls the arm, or no controlling PC.
        """
    @typing.overload
    def set_parameter(self, name: str, value: typing.SupportsFloat | typing.SupportsIndex) -> ParameterInfo:
        """
        [Blocking] Requests a new value for a parameter by name and returns the ParameterInfo after the write. See the id overload.
        """
    def set_zero(self, enable_after: bool = False) -> None:
        """
        [Blocking, 2 s] Stores the zero of every drive at its current angle; every drive must be disabled. enable_after=True enables both groups afterwards.
        """
    def set_zero_position(self, motor: typing.SupportsInt | typing.SupportsIndex) -> None:
        """
        [Blocking, 1.5 s] Maintenance: stores one drive's zero at its current angle.
        """
    def start_joint_stream(self, initial: JointCommand, options: JointStreamOptions = ...) -> None:
        """
        [Non-blocking] Starts the native 1 kHz RT_JOINT producer from `initial`; later servej() calls replace the command. Needs RT_JOINT with the MIT profile and fresh feedback. A producer timeout, stale feedback or a controller fault requests a controlled stop.
        """
    def start_recording(self, directory: str, options: RecordOptions = ...) -> None:
        """
        [Blocking, file setup] Records every telemetry packet (1 kHz), every command sent and the events as CSV under directory/<time>[-label]/. A background thread writes.
        """
    def start_serial_log(self, options: SerialLogOptions = ...) -> str:
        """
        [Blocking, port/file setup] Saves the controller's UART debug output and returns the file path. Fails if another program has the port open.
        """
    def state(self) -> RobotState:
        """
        [RT] A copy of the newest telemetry (RobotState).
        """
    def stop(self, wait: bool = True) -> None:
        """
        [Blocking, 2 s] Controlled stop in any mode: the controller brakes the arm to rest and holds it enabled in IDLE. wait=False sends the request and returns. resume() continues the interrupted mode.
        """
    def stop_joint_stream(self) -> None:
        """
        [Non-blocking] Ends the native producer; sends nothing. Use stop() to stop the arm.
        """
    def stop_recording(self) -> None:
        """
        [Blocking] Finishes the writer and closes the files.
        """
    def stop_serial_log(self) -> None:
        """
        [Blocking] Joins the serial worker.
        """
    def store_parameters(self, motor: typing.SupportsInt | typing.SupportsIndex) -> None:
        """
        [Blocking, 1.5 s] Maintenance: saves one drive's registers to its flash.
        """
    def submit_movej(self, position: typing.Annotated[collections.abc.Sequence[typing.SupportsFloat | typing.SupportsIndex], "FixedSize(7)"], options: MoveOptions = ...) -> None:
        """
        [Non-blocking] Starts a joint move and returns; follow it with motion() or wait_motion(). The previous move must be done.
        """
    def submit_movep(self, pose: typing.Annotated[collections.abc.Sequence[typing.SupportsFloat | typing.SupportsIndex], "FixedSize(12)"], options: CartesianMoveOptions = ...) -> None:
        """
        [Non-blocking] Starts a Cartesian move and returns. See submit_movej().
        """
    def switch_mode(self, mode: Mode) -> None:
        """
        [Blocking, 2 s] Switches the control mode. A mode other than IDLE is entered from IDLE with the arm enabled and still. RT modes: then stream servej()/servep() starting from hold_command(). NRT modes: the SDK starts the movej/movep session. Retries for up to 1 s while the controller is still handing control over.
        """
    def wait_motion(self, timeout_ms: typing.SupportsInt | typing.SupportsIndex) -> MotionResult:
        """
        [Blocking, timeout_ms] Waits until the newest move has arrived and returns its MotionResult; raises if it failed.
        """
    def write_register(self, motor: typing.SupportsInt | typing.SupportsIndex, register_id: typing.SupportsInt | typing.SupportsIndex, value: typing.SupportsInt | typing.SupportsIndex, value_type: typing.SupportsInt | typing.SupportsIndex) -> None:
        """
        [Blocking, 1.5 s] Maintenance: write_register(motor, register_id, value, value_type) writes a register within its allowed range and reads it back.
        """
class RobotInfo:
    """
    The controller's identity, limits and capabilities, from the connect() handshake.
    """
    @property
    def arm_model(self) -> str:
        """
        Arm model name.
        """
    @property
    def axes(self) -> int:
        """
        Arm joints (7).
        """
    @property
    def board(self) -> str:
        """
        Controller board name.
        """
    @property
    def capabilities(self) -> int:
        """
        CAP_* bits: test these for optional features.
        """
    @property
    def control_mode(self) -> int:
        """
        Control mode at connect (a Mode value).
        """
    @property
    def controller_status(self) -> int:
        """
        Status at connect (an OperationalStatus value).
        """
    @property
    def enabled_mask(self) -> int:
        """
        Enabled drives at connect (bit i = J(i+1)).
        """
    @property
    def firmware(self) -> str:
        """
        Firmware build string, e.g. "v0.2.0"; empty for V1-only firmware.
        """
    @property
    def motor_count(self) -> int:
        """
        Drives including the gripper (8).
        """
    @property
    def motor_profile(self) -> int:
        """
        Drive mode at connect (a MotorProfile value).
        """
    @property
    def position_max(self) -> typing.Annotated[list[float], "FixedSize(8)"]:
        """
        Joint position limits, rad.
        """
    @property
    def position_min(self) -> typing.Annotated[list[float], "FixedSize(8)"]:
        """
        Joint position limits, rad.
        """
    @property
    def product(self) -> str:
        """
        Product name.
        """
    @property
    def profile_ready(self) -> bool:
        """
        The motor profile was applied and passed its self-check.
        """
    @property
    def protocol_major(self) -> int:
        """
        Protocol major version; must match the SDK's.
        """
    @property
    def protocol_minor(self) -> int:
        """
        Protocol minor version; minors only add features.
        """
    @property
    def protocol_version(self) -> int:
        """
        DMIO envelope version.
        """
    @property
    def received(self) -> bool:
        """
        A DMIO answer arrived (firmware with DMIF).
        """
    @property
    def torque_max(self) -> typing.Annotated[list[float], "FixedSize(8)"]:
        """
        Drive torque ranges, N·m.
        """
    @property
    def velocity_max(self) -> typing.Annotated[list[float], "FixedSize(8)"]:
        """
        Drive velocity ranges, rad/s.
        """
class RobotState:
    """
    Controller state from the newest telemetry packet. Returned by Robot.state().
    """
    @property
    def command_active(self) -> bool:
        """
        The controller applies a PC command stream.
        """
    @property
    def command_applied_sequence(self) -> int:
        """
        Sequence of the newest command the controller applied.
        """
    @property
    def connected(self) -> bool:
        """
        Sockets open; use telemetry_age_ms for liveness.
        """
    @property
    def controller_state(self) -> int:
        """
        Raw status code.
        """
    @property
    def enabled(self) -> bool:
        """
        Any drive enabled.
        """
    @property
    def enabled_mask(self) -> int:
        """
        Enabled drives (bit i = J(i+1)).
        """
    @property
    def joints(self) -> JointState:
        """
        Per-drive state (JointState).
        """
    @property
    def mode(self) -> Mode:
        """
        Current control mode.
        """
    @property
    def modes_supported(self) -> bool:
        """
        Firmware with the control-mode state machine.
        """
    @property
    def operational_status(self) -> OperationalStatus:
        """
        What the controller is doing.
        """
    @property
    def profile(self) -> MotorProfile:
        """
        Current drive mode of J1–J7.
        """
    @property
    def profile_ready(self) -> bool:
        """
        The motor profile was applied and passed its self-check.
        """
    @property
    def safety(self) -> SafetyState:
        """
        Controlled stop and self-check (SafetyState).
        """
    @property
    def telemetry_age_ms(self) -> int:
        """
        Age of the newest telemetry, ms; 4294967295 before the first.
        """
    @property
    def telemetry_epoch(self) -> int:
        """
        Increments when telemetry resynchronizes after a stale or restarted source.
        """
    @property
    def telemetry_version(self) -> int:
        """
        Telemetry format, 2 or 3.
        """
    @property
    def zero_valid(self) -> bool:
        """
        The drives' zero positions are calibrated.
        """
class SafetyState:
    """
    Controlled stop and parameter self-check (telemetry v3).
    """
    @property
    def selfcheck(self) -> SelfCheck:
        """
        Last self-check result.
        """
    @property
    def selfcheck_expected(self) -> int:
        """
        Value expected (raw register bits).
        """
    @property
    def selfcheck_joint(self) -> int:
        """
        Joint that failed the self-check, 0-based.
        """
    @property
    def selfcheck_register(self) -> int:
        """
        Register that failed.
        """
    @property
    def selfcheck_value(self) -> int:
        """
        Value read back (raw register bits).
        """
    @property
    def stop_detail(self) -> int:
        """
        Reason-specific detail (for a drive fault, its status code).
        """
    @property
    def stop_joint(self) -> int:
        """
        Joint that caused the stop, 0-based; 255 if none.
        """
    @property
    def stop_mode(self) -> Mode:
        """
        The mode the stop interrupted; resume() re-enters it.
        """
    @property
    def stop_phase(self) -> StopPhase:
        """
        Braking or holding.
        """
    @property
    def stop_reason(self) -> StopReason:
        """
        The latched stop; StopReason.None_ after it is acknowledged.
        """
    @property
    def valid(self) -> bool:
        """
        The fields come from telemetry v3.
        """
class SelfCheck:
    """
    Result of the drive parameter self-check that set_motor_profile() runs.

    Members:

      NotRun

      Ok

      Mismatch

      NoReply
    """
    Mismatch: typing.ClassVar[SelfCheck]  # value = <SelfCheck.Mismatch: 2>
    NoReply: typing.ClassVar[SelfCheck]  # value = <SelfCheck.NoReply: 3>
    NotRun: typing.ClassVar[SelfCheck]  # value = <SelfCheck.NotRun: 0>
    Ok: typing.ClassVar[SelfCheck]  # value = <SelfCheck.Ok: 1>
    __members__: typing.ClassVar[dict[str, SelfCheck]]  # value = {'NotRun': <SelfCheck.NotRun: 0>, 'Ok': <SelfCheck.Ok: 1>, 'Mismatch': <SelfCheck.Mismatch: 2>, 'NoReply': <SelfCheck.NoReply: 3>}
    @typing.overload
    def __eq__(self, other: SelfCheck) -> bool:
        ...
    @typing.overload
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    def __int__(self) -> int:
        ...
    @typing.overload
    def __ne__(self, other: SelfCheck) -> bool:
        ...
    @typing.overload
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    def __str__(self) -> str:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class SerialLogOptions:
    """
    Options of Robot.start_serial_log().
    """
    def __init__(self) -> None:
        ...
    @property
    def baud(self) -> int:
        """
        Baud rate.
        """
    @baud.setter
    def baud(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    @property
    def directory(self) -> str:
        """
        Output directory; empty selects Robot.log_directory().
        """
    @directory.setter
    def directory(self, arg0: str) -> None:
        ...
    @property
    def port(self) -> str:
        """
        Serial device.
        """
    @port.setter
    def port(self, arg0: str) -> None:
        ...
class SerialLogStats:
    """
    Progress of the serial log. Returned by Robot.serial_log().
    """
    @property
    def active(self) -> bool:
        """
        The log is running.
        """
    @property
    def bytes(self) -> int:
        """
        Bytes written.
        """
    @property
    def error(self) -> str:
        """
        Last port or write error.
        """
    @property
    def lines(self) -> int:
        """
        Lines written.
        """
    @property
    def path(self) -> str:
        """
        File being written.
        """
    @property
    def port_open(self) -> bool:
        """
        The port is open now.
        """
    @property
    def reconnects(self) -> int:
        """
        Times the port was lost and reopened.
        """
class StopPhase:
    """
    Phase of a controlled stop: Braking to rest, then Holding enabled until a PC is back.

    Members:

      None_

      Braking

      Holding
    """
    Braking: typing.ClassVar[StopPhase]  # value = <StopPhase.Braking: 1>
    Holding: typing.ClassVar[StopPhase]  # value = <StopPhase.Holding: 2>
    None_: typing.ClassVar[StopPhase]  # value = <StopPhase.None_: 0>
    __members__: typing.ClassVar[dict[str, StopPhase]]  # value = {'None_': <StopPhase.None_: 0>, 'Braking': <StopPhase.Braking: 1>, 'Holding': <StopPhase.Holding: 2>}
    @typing.overload
    def __eq__(self, other: StopPhase) -> bool:
        ...
    @typing.overload
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    def __int__(self) -> int:
        ...
    @typing.overload
    def __ne__(self, other: StopPhase) -> bool:
        ...
    @typing.overload
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    def __str__(self) -> str:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class StopReason:
    """
    Why the controller made a controlled stop. None_ means no stop is latched.

    Members:

      None_

      StreamLost

      MoveLost

      MoveFailed

      Requested

      NotFollowing

      InvalidTarget

      FeedbackLost

      DriveFault

      TorqueLimit
    """
    DriveFault: typing.ClassVar[StopReason]  # value = <StopReason.DriveFault: 8>
    FeedbackLost: typing.ClassVar[StopReason]  # value = <StopReason.FeedbackLost: 7>
    InvalidTarget: typing.ClassVar[StopReason]  # value = <StopReason.InvalidTarget: 6>
    MoveFailed: typing.ClassVar[StopReason]  # value = <StopReason.MoveFailed: 3>
    MoveLost: typing.ClassVar[StopReason]  # value = <StopReason.MoveLost: 2>
    None_: typing.ClassVar[StopReason]  # value = <StopReason.None_: 0>
    NotFollowing: typing.ClassVar[StopReason]  # value = <StopReason.NotFollowing: 5>
    Requested: typing.ClassVar[StopReason]  # value = <StopReason.Requested: 4>
    StreamLost: typing.ClassVar[StopReason]  # value = <StopReason.StreamLost: 1>
    TorqueLimit: typing.ClassVar[StopReason]  # value = <StopReason.TorqueLimit: 9>
    __members__: typing.ClassVar[dict[str, StopReason]]  # value = {'None_': <StopReason.None_: 0>, 'StreamLost': <StopReason.StreamLost: 1>, 'MoveLost': <StopReason.MoveLost: 2>, 'MoveFailed': <StopReason.MoveFailed: 3>, 'Requested': <StopReason.Requested: 4>, 'NotFollowing': <StopReason.NotFollowing: 5>, 'InvalidTarget': <StopReason.InvalidTarget: 6>, 'FeedbackLost': <StopReason.FeedbackLost: 7>, 'DriveFault': <StopReason.DriveFault: 8>, 'TorqueLimit': <StopReason.TorqueLimit: 9>}
    @typing.overload
    def __eq__(self, other: StopReason) -> bool:
        ...
    @typing.overload
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    def __int__(self) -> int:
        ...
    @typing.overload
    def __ne__(self, other: StopReason) -> bool:
        ...
    @typing.overload
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    def __str__(self) -> str:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
def firmware_status(ip: str, public_key_file: str = '') -> str:
    """
    [Blocking] Reports the board's A/B slots: which is confirmed and their image IDs.
    """
def rollback_firmware(ip: str, public_key_file: str = '') -> None:
    """
    [Blocking] Makes the board boot the other A/B slot.
    """
def update_firmware(ip: str, package: str, public_key_file: str = '') -> bool:
    """
    [Blocking] Installs a signed IFW package after approval on the touchscreen and returns True, or False without sending anything when the board already runs an image from the package. Uses its own maintenance socket; needs no connected Robot. Support the arm, disable the drives and disconnect motor power first. Raises RuntimeError on refusal, timeout, an invalid package or a verification failure.
    """
ARM_MASK: int = 127
ARM_MAX_JOINT_VELOCITY: float = 3.140000104904175
CAP_ADMITTANCE: int = 4096
CAP_CONTROLLED_STOP: int = 256
CAP_DIAGNOSTICS: int = 32768
CAP_GROUPS: int = 64
CAP_JOINT_RT: int = 1
CAP_LOCAL_GRAVITY: int = 2048
CAP_PARAMETERS: int = 16384
CAP_TELEMETRY_V3: int = 128
MOTOR_FEEDBACK_STALE: int = 16
PARAM_GROUP_ADMITTANCE: int = 3
PARAM_GROUP_GRAVITY: int = 1
PARAM_GROUP_IMPEDANCE: int = 2
REGISTER_F32: int = 1
REGISTER_U32: int = 0
