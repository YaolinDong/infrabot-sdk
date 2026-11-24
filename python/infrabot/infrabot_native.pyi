# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Yaolin Dong
"""
Bindings for the existing infrabot::Robot C++ SDK (joint SI units)
"""
from __future__ import annotations
import collections.abc
import typing
__all__: list[str] = ['ARM_MASK', 'ARM_MAX_JOINT_VELOCITY', 'CAP_ADMITTANCE', 'CAP_CONTROLLED_STOP', 'CAP_GROUPS', 'CAP_JOINT_RT', 'CAP_LOCAL_GRAVITY', 'CAP_TELEMETRY_V3', 'CartesianCommand', 'CartesianLimits', 'CartesianMoveOptions', 'ControllerLinkStats', 'FaultEvent', 'FaultKind', 'JointCommand', 'JointState', 'JointStreamOptions', 'JointStreamStats', 'LinkStats', 'MOTOR_FEEDBACK_STALE', 'McuLogOptions', 'McuLogResult', 'Mode', 'MotionPhase', 'MotionResult', 'MotionStatus', 'MotorGroup', 'MotorProfile', 'MoveOptions', 'MoveReply', 'OperationalStatus', 'REGISTER_F32', 'REGISTER_U32', 'RecordOptions', 'RecorderStats', 'Robot', 'RobotInfo', 'RobotState', 'SafetyState', 'SelfCheck', 'SerialLogOptions', 'SerialLogStats', 'StopPhase', 'StopReason', 'firmware_status', 'rollback_firmware', 'update_firmware']
class Mode:
    """
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
    CartesianAdmittance: typing.ClassVar[Mode]
    CartesianImpedance: typing.ClassVar[Mode]
    GravityFree: typing.ClassVar[Mode]
    GravityLock: typing.ClassVar[Mode]
    Idle: typing.ClassVar[Mode]
    JointAdmittance: typing.ClassVar[Mode]
    JointImpedance: typing.ClassVar[Mode]
    NrtCartesian: typing.ClassVar[Mode]
    NrtJoint: typing.ClassVar[Mode]
    RtCartesian: typing.ClassVar[Mode]
    RtJoint: typing.ClassVar[Mode]
    Unknown: typing.ClassVar[Mode]
    __members__: typing.ClassVar[dict[str, Mode]]
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
class MotorProfile:
    """
    Members:

      Mit

      PositionVelocity
    """
    Mit: typing.ClassVar[MotorProfile]
    PositionVelocity: typing.ClassVar[MotorProfile]
    __members__: typing.ClassVar[dict[str, MotorProfile]]
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
class MotorGroup:
    """
    Members:

      All

      Arm

      Gripper
    """
    All: typing.ClassVar[MotorGroup]
    Arm: typing.ClassVar[MotorGroup]
    Gripper: typing.ClassVar[MotorGroup]
    __members__: typing.ClassVar[dict[str, MotorGroup]]
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
class OperationalStatus:
    """
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
    Booting: typing.ClassVar[OperationalStatus]
    CartesianAdmittance: typing.ClassVar[OperationalStatus]
    CartesianImpedance: typing.ClassVar[OperationalStatus]
    Disabled: typing.ClassVar[OperationalStatus]
    Enabling: typing.ClassVar[OperationalStatus]
    Estop: typing.ClassVar[OperationalStatus]
    Fault: typing.ClassVar[OperationalStatus]
    GravityFree: typing.ClassVar[OperationalStatus]
    GravityLock: typing.ClassVar[OperationalStatus]
    IdleHold: typing.ClassVar[OperationalStatus]
    JointAdmittance: typing.ClassVar[OperationalStatus]
    JointImpedance: typing.ClassVar[OperationalStatus]
    NrtCartesian: typing.ClassVar[OperationalStatus]
    NrtJoint: typing.ClassVar[OperationalStatus]
    Precheck: typing.ClassVar[OperationalStatus]
    ProfileRequired: typing.ClassVar[OperationalStatus]
    Recovery: typing.ClassVar[OperationalStatus]
    RtCartesian: typing.ClassVar[OperationalStatus]
    RtJoint: typing.ClassVar[OperationalStatus]
    Stopping: typing.ClassVar[OperationalStatus]
    Unknown: typing.ClassVar[OperationalStatus]
    __members__: typing.ClassVar[dict[str, OperationalStatus]]
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
class StopReason:
    """
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
    DriveFault: typing.ClassVar[StopReason]
    FeedbackLost: typing.ClassVar[StopReason]
    InvalidTarget: typing.ClassVar[StopReason]
    MoveFailed: typing.ClassVar[StopReason]
    MoveLost: typing.ClassVar[StopReason]
    None_: typing.ClassVar[StopReason]
    NotFollowing: typing.ClassVar[StopReason]
    Requested: typing.ClassVar[StopReason]
    StreamLost: typing.ClassVar[StopReason]
    TorqueLimit: typing.ClassVar[StopReason]
    __members__: typing.ClassVar[dict[str, StopReason]]
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
class StopPhase:
    """
    Members:

      None_

      Braking

      Holding
    """
    Braking: typing.ClassVar[StopPhase]
    Holding: typing.ClassVar[StopPhase]
    None_: typing.ClassVar[StopPhase]
    __members__: typing.ClassVar[dict[str, StopPhase]]
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
class SelfCheck:
    """
    Members:

      NotRun

      Ok

      Mismatch

      NoReply
    """
    Mismatch: typing.ClassVar[SelfCheck]
    NoReply: typing.ClassVar[SelfCheck]
    NotRun: typing.ClassVar[SelfCheck]
    Ok: typing.ClassVar[SelfCheck]
    __members__: typing.ClassVar[dict[str, SelfCheck]]
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
class JointStreamOptions:
    allow_zero_kp: bool
    def __init__(self) -> None:
        ...
    @property
    def command_timeout_ms(self) -> int:
        ...
    @command_timeout_ms.setter
    def command_timeout_ms(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    @property
    def feedback_timeout_ms(self) -> int:
        ...
    @feedback_timeout_ms.setter
    def feedback_timeout_ms(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    @property
    def transmit_hz(self) -> int:
        ...
    @transmit_hz.setter
    def transmit_hz(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
class JointStreamStats:
    @property
    def active(self) -> bool:
        ...
    @property
    def fault(self) -> str:
        ...
    @property
    def sent(self) -> int:
        ...
    @property
    def skipped_deadlines(self) -> int:
        ...
class JointCommand:
    def __init__(self) -> None:
        ...
    @property
    def kd(self) -> typing.Annotated[list[float], "FixedSize(7)"]:
        ...
    @kd.setter
    def kd(self, arg0: typing.Annotated[collections.abc.Sequence[typing.SupportsFloat | typing.SupportsIndex], "FixedSize(7)"]) -> None:
        ...
    @property
    def kp(self) -> typing.Annotated[list[float], "FixedSize(7)"]:
        ...
    @kp.setter
    def kp(self, arg0: typing.Annotated[collections.abc.Sequence[typing.SupportsFloat | typing.SupportsIndex], "FixedSize(7)"]) -> None:
        ...
    @property
    def position(self) -> typing.Annotated[list[float], "FixedSize(7)"]:
        ...
    @position.setter
    def position(self, arg0: typing.Annotated[collections.abc.Sequence[typing.SupportsFloat | typing.SupportsIndex], "FixedSize(7)"]) -> None:
        ...
    @property
    def torque(self) -> typing.Annotated[list[float], "FixedSize(7)"]:
        ...
    @torque.setter
    def torque(self, arg0: typing.Annotated[collections.abc.Sequence[typing.SupportsFloat | typing.SupportsIndex], "FixedSize(7)"]) -> None:
        ...
    @property
    def velocity(self) -> typing.Annotated[list[float], "FixedSize(7)"]:
        ...
    @velocity.setter
    def velocity(self, arg0: typing.Annotated[collections.abc.Sequence[typing.SupportsFloat | typing.SupportsIndex], "FixedSize(7)"]) -> None:
        ...
    @property
    def velocity_limit(self) -> typing.Annotated[list[float], "FixedSize(7)"]:
        ...
    @velocity_limit.setter
    def velocity_limit(self, arg0: typing.Annotated[collections.abc.Sequence[typing.SupportsFloat | typing.SupportsIndex], "FixedSize(7)"]) -> None:
        ...
class JointState:
    @property
    def command_position(self) -> typing.Annotated[list[float], "FixedSize(8)"]:
        ...
    @property
    def command_torque(self) -> typing.Annotated[list[float], "FixedSize(8)"]:
        ...
    @property
    def command_velocity(self) -> typing.Annotated[list[float], "FixedSize(8)"]:
        ...
    @property
    def drive_mode(self) -> typing.Annotated[list[int], "FixedSize(8)"]:
        ...
    @property
    def drive_state(self) -> typing.Annotated[list[int], "FixedSize(8)"]:
        ...
    @property
    def feedback_age_ms(self) -> typing.Annotated[list[int], "FixedSize(8)"]:
        ...
    @property
    def feedback_count(self) -> typing.Annotated[list[int], "FixedSize(8)"]:
        ...
    @property
    def has_command(self) -> bool:
        ...
    @property
    def host_time_us(self) -> int:
        ...
    @property
    def motor_flags(self) -> typing.Annotated[list[int], "FixedSize(8)"]:
        ...
    @property
    def position(self) -> typing.Annotated[list[float], "FixedSize(8)"]:
        ...
    @property
    def sequence(self) -> int:
        ...
    @property
    def temperature(self) -> typing.Annotated[list[int], "FixedSize(8)"]:
        ...
    @property
    def temperature_coil(self) -> typing.Annotated[list[int], "FixedSize(8)"]:
        ...
    @property
    def timestamp_us(self) -> int:
        ...
    @property
    def torque(self) -> typing.Annotated[list[float], "FixedSize(8)"]:
        ...
    @property
    def velocity(self) -> typing.Annotated[list[float], "FixedSize(8)"]:
        ...
class SafetyState:
    @property
    def selfcheck(self) -> SelfCheck:
        ...
    @property
    def selfcheck_expected(self) -> int:
        ...
    @property
    def selfcheck_joint(self) -> int:
        ...
    @property
    def selfcheck_register(self) -> int:
        ...
    @property
    def selfcheck_value(self) -> int:
        ...
    @property
    def stop_detail(self) -> int:
        ...
    @property
    def stop_joint(self) -> int:
        ...
    @property
    def stop_mode(self) -> Mode:
        ...
    @property
    def stop_phase(self) -> StopPhase:
        ...
    @property
    def stop_reason(self) -> StopReason:
        ...
    @property
    def valid(self) -> bool:
        ...
class RobotState:
    @property
    def command_active(self) -> bool:
        ...
    @property
    def command_applied_sequence(self) -> int:
        ...
    @property
    def connected(self) -> bool:
        ...
    @property
    def controller_state(self) -> int:
        ...
    @property
    def enabled(self) -> bool:
        ...
    @property
    def enabled_mask(self) -> int:
        ...
    @property
    def joints(self) -> JointState:
        ...
    @property
    def mode(self) -> Mode:
        ...
    @property
    def modes_supported(self) -> bool:
        ...
    @property
    def operational_status(self) -> OperationalStatus:
        ...
    @property
    def profile(self) -> MotorProfile:
        ...
    @property
    def profile_ready(self) -> bool:
        ...
    @property
    def safety(self) -> SafetyState:
        ...
    @property
    def telemetry_age_ms(self) -> int:
        ...
    @property
    def telemetry_epoch(self) -> int:
        ...
    @property
    def telemetry_version(self) -> int:
        ...
    @property
    def zero_valid(self) -> bool:
        ...
class RobotInfo:
    @property
    def arm_model(self) -> str:
        ...
    @property
    def axes(self) -> int:
        ...
    @property
    def board(self) -> str:
        ...
    @property
    def capabilities(self) -> int:
        ...
    @property
    def control_mode(self) -> int:
        ...
    @property
    def controller_status(self) -> int:
        ...
    @property
    def enabled_mask(self) -> int:
        ...
    @property
    def firmware(self) -> str:
        ...
    @property
    def motor_count(self) -> int:
        ...
    @property
    def motor_profile(self) -> int:
        ...
    @property
    def position_max(self) -> typing.Annotated[list[float], "FixedSize(8)"]:
        ...
    @property
    def position_min(self) -> typing.Annotated[list[float], "FixedSize(8)"]:
        ...
    @property
    def product(self) -> str:
        ...
    @property
    def profile_ready(self) -> bool:
        ...
    @property
    def protocol_major(self) -> int:
        ...
    @property
    def protocol_minor(self) -> int:
        ...
    @property
    def protocol_version(self) -> int:
        ...
    @property
    def received(self) -> bool:
        ...
    @property
    def torque_max(self) -> typing.Annotated[list[float], "FixedSize(8)"]:
        ...
    @property
    def velocity_max(self) -> typing.Annotated[list[float], "FixedSize(8)"]:
        ...
class ControllerLinkStats:
    @property
    def can_tx_errors(self) -> int:
        ...
    @property
    def command_interval_max_us(self) -> int:
        ...
    @property
    def command_invalid(self) -> int:
        ...
    @property
    def command_jitter_max_us(self) -> int:
        ...
    @property
    def command_rx(self) -> int:
        ...
    @property
    def control_missed(self) -> int:
        ...
    @property
    def control_period_max_us(self) -> int:
        ...
    @property
    def invalid_reason(self) -> int:
        ...
    @property
    def telemetry_errors(self) -> int:
        ...
class LinkStats:
    @property
    def loss_percent(self) -> float:
        ...
    @property
    def lost_last_second(self) -> int:
        ...
    @property
    def mcu(self) -> ControllerLinkStats:
        ...
    @property
    def mcu_valid(self) -> bool:
        ...
    @property
    def rx_interval_max_us(self) -> int:
        ...
    @property
    def rx_invalid(self) -> int:
        ...
    @property
    def rx_last_second(self) -> int:
        ...
    @property
    def rx_lost(self) -> int:
        ...
    @property
    def rx_telemetry(self) -> int:
        ...
    @property
    def tx_errors(self) -> int:
        ...
    @property
    def tx_sent(self) -> int:
        ...
class FaultKind:
    """
    Members:

      DriveFault

      DriveCleared

      Stop

      SelfCheck

      Link
    """
    DriveCleared: typing.ClassVar[FaultKind]
    DriveFault: typing.ClassVar[FaultKind]
    Link: typing.ClassVar[FaultKind]
    SelfCheck: typing.ClassVar[FaultKind]
    Stop: typing.ClassVar[FaultKind]
    __members__: typing.ClassVar[dict[str, FaultKind]]
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
class FaultEvent:
    @property
    def code(self) -> int:
        ...
    @property
    def detail(self) -> int:
        ...
    @property
    def host_time_us(self) -> int:
        ...
    @property
    def joint(self) -> int:
        ...
    @property
    def kind(self) -> FaultKind:
        ...
    @property
    def mcu_time_us(self) -> int:
        ...
    @property
    def message(self) -> str:
        ...
class RecordOptions:
    commands: bool
    label: str
    def __init__(self) -> None:
        ...
    @property
    def rotate_seconds(self) -> int:
        ...
    @rotate_seconds.setter
    def rotate_seconds(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
class RecorderStats:
    @property
    def active(self) -> bool:
        ...
    @property
    def command_rows(self) -> int:
        ...
    @property
    def directory(self) -> str:
        ...
    @property
    def dropped(self) -> int:
        ...
    @property
    def error(self) -> str:
        ...
    @property
    def events(self) -> int:
        ...
    @property
    def file_index(self) -> int:
        ...
    @property
    def telemetry_rows(self) -> int:
        ...
class SerialLogOptions:
    directory: str
    port: str
    def __init__(self) -> None:
        ...
    @property
    def baud(self) -> int:
        ...
    @baud.setter
    def baud(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
class SerialLogStats:
    @property
    def active(self) -> bool:
        ...
    @property
    def bytes(self) -> int:
        ...
    @property
    def error(self) -> str:
        ...
    @property
    def lines(self) -> int:
        ...
    @property
    def path(self) -> str:
        ...
    @property
    def port_open(self) -> bool:
        ...
    @property
    def reconnects(self) -> int:
        ...
class McuLogOptions:
    directory: str
    label: str
    def __init__(self) -> None:
        ...
    @property
    def port(self) -> int:
        ...
    @port.setter
    def port(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    @property
    def progress(self) -> collections.abc.Callable[[typing.SupportsInt | typing.SupportsIndex, typing.SupportsInt | typing.SupportsIndex, typing.SupportsInt | typing.SupportsIndex, typing.SupportsInt | typing.SupportsIndex], bool]:
        ...
    @progress.setter
    def progress(self, arg0: collections.abc.Callable[[int, int, int, int], bool]) -> None:
        ...
    @property
    def retries(self) -> int:
        ...
    @retries.setter
    def retries(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    @property
    def timeout_ms(self) -> int:
        ...
    @timeout_ms.setter
    def timeout_ms(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
class McuLogResult:
    @property
    def boot_id(self) -> int:
        ...
    @property
    def fault_latched(self) -> bool:
        ...
    @property
    def lost_bytes(self) -> int:
        ...
    @property
    def telemetry_path(self) -> str:
        ...
    @property
    def telemetry_rows(self) -> int:
        ...
    @property
    def text_lines(self) -> int:
        ...
    @property
    def text_path(self) -> str:
        ...
    @property
    def writer_max_us(self) -> int:
        ...
class CartesianLimits:
    def __init__(self) -> None:
        ...
    @property
    def angular_acceleration(self) -> float:
        ...
    @angular_acceleration.setter
    def angular_acceleration(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...
    @property
    def angular_velocity(self) -> float:
        ...
    @angular_velocity.setter
    def angular_velocity(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...
    @property
    def linear_acceleration(self) -> float:
        ...
    @linear_acceleration.setter
    def linear_acceleration(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...
    @property
    def linear_velocity(self) -> float:
        ...
    @linear_velocity.setter
    def linear_velocity(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...
class CartesianCommand:
    limits: CartesianLimits
    def __init__(self) -> None:
        ...
    @property
    def kd(self) -> typing.Annotated[list[float], "FixedSize(7)"]:
        ...
    @kd.setter
    def kd(self, arg0: typing.Annotated[collections.abc.Sequence[typing.SupportsFloat | typing.SupportsIndex], "FixedSize(7)"]) -> None:
        ...
    @property
    def kp(self) -> typing.Annotated[list[float], "FixedSize(7)"]:
        ...
    @kp.setter
    def kp(self, arg0: typing.Annotated[collections.abc.Sequence[typing.SupportsFloat | typing.SupportsIndex], "FixedSize(7)"]) -> None:
        ...
    @property
    def pose(self) -> typing.Annotated[list[float], "FixedSize(12)"]:
        ...
    @pose.setter
    def pose(self, arg0: typing.Annotated[collections.abc.Sequence[typing.SupportsFloat | typing.SupportsIndex], "FixedSize(12)"]) -> None:
        ...
    @property
    def torque(self) -> typing.Annotated[list[float], "FixedSize(7)"]:
        ...
    @torque.setter
    def torque(self, arg0: typing.Annotated[collections.abc.Sequence[typing.SupportsFloat | typing.SupportsIndex], "FixedSize(7)"]) -> None:
        ...
    @property
    def velocity_limit(self) -> typing.Annotated[list[float], "FixedSize(7)"]:
        ...
    @velocity_limit.setter
    def velocity_limit(self, arg0: typing.Annotated[collections.abc.Sequence[typing.SupportsFloat | typing.SupportsIndex], "FixedSize(7)"]) -> None:
        ...
class MoveOptions:
    def __init__(self) -> None:
        ...
    @property
    def acceleration_max(self) -> typing.Annotated[list[float], "FixedSize(7)"]:
        ...
    @acceleration_max.setter
    def acceleration_max(self, arg0: typing.Annotated[collections.abc.Sequence[typing.SupportsFloat | typing.SupportsIndex], "FixedSize(7)"]) -> None:
        ...
    @property
    def dwell_ms(self) -> int:
        ...
    @dwell_ms.setter
    def dwell_ms(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    @property
    def kd(self) -> typing.Annotated[list[float], "FixedSize(7)"]:
        ...
    @kd.setter
    def kd(self, arg0: typing.Annotated[collections.abc.Sequence[typing.SupportsFloat | typing.SupportsIndex], "FixedSize(7)"]) -> None:
        ...
    @property
    def kp(self) -> typing.Annotated[list[float], "FixedSize(7)"]:
        ...
    @kp.setter
    def kp(self, arg0: typing.Annotated[collections.abc.Sequence[typing.SupportsFloat | typing.SupportsIndex], "FixedSize(7)"]) -> None:
        ...
    @property
    def position_tolerance(self) -> float:
        ...
    @position_tolerance.setter
    def position_tolerance(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...
    @property
    def timeout_ms(self) -> int:
        ...
    @timeout_ms.setter
    def timeout_ms(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    @property
    def torque(self) -> typing.Annotated[list[float], "FixedSize(7)"]:
        ...
    @torque.setter
    def torque(self, arg0: typing.Annotated[collections.abc.Sequence[typing.SupportsFloat | typing.SupportsIndex], "FixedSize(7)"]) -> None:
        ...
    @property
    def velocity_max(self) -> typing.Annotated[list[float], "FixedSize(7)"]:
        ...
    @velocity_max.setter
    def velocity_max(self, arg0: typing.Annotated[collections.abc.Sequence[typing.SupportsFloat | typing.SupportsIndex], "FixedSize(7)"]) -> None:
        ...
    @property
    def velocity_tolerance(self) -> float:
        ...
    @velocity_tolerance.setter
    def velocity_tolerance(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...
class CartesianMoveOptions:
    limits: CartesianLimits
    def __init__(self) -> None:
        ...
    @property
    def acceleration_max(self) -> typing.Annotated[list[float], "FixedSize(7)"]:
        ...
    @acceleration_max.setter
    def acceleration_max(self, arg0: typing.Annotated[collections.abc.Sequence[typing.SupportsFloat | typing.SupportsIndex], "FixedSize(7)"]) -> None:
        ...
    @property
    def dwell_ms(self) -> int:
        ...
    @dwell_ms.setter
    def dwell_ms(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    @property
    def kd(self) -> typing.Annotated[list[float], "FixedSize(7)"]:
        ...
    @kd.setter
    def kd(self, arg0: typing.Annotated[collections.abc.Sequence[typing.SupportsFloat | typing.SupportsIndex], "FixedSize(7)"]) -> None:
        ...
    @property
    def kp(self) -> typing.Annotated[list[float], "FixedSize(7)"]:
        ...
    @kp.setter
    def kp(self, arg0: typing.Annotated[collections.abc.Sequence[typing.SupportsFloat | typing.SupportsIndex], "FixedSize(7)"]) -> None:
        ...
    @property
    def position_tolerance(self) -> float:
        ...
    @position_tolerance.setter
    def position_tolerance(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...
    @property
    def timeout_ms(self) -> int:
        ...
    @timeout_ms.setter
    def timeout_ms(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    @property
    def velocity_max(self) -> typing.Annotated[list[float], "FixedSize(7)"]:
        ...
    @velocity_max.setter
    def velocity_max(self, arg0: typing.Annotated[collections.abc.Sequence[typing.SupportsFloat | typing.SupportsIndex], "FixedSize(7)"]) -> None:
        ...
    @property
    def velocity_tolerance(self) -> float:
        ...
    @velocity_tolerance.setter
    def velocity_tolerance(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...
class MotionResult:
    @property
    def arrived_mask(self) -> int:
        ...
    @property
    def fresh_mask(self) -> int:
        ...
    @property
    def max_error_rad(self) -> float:
        ...
    @property
    def message(self) -> str:
        ...
    @property
    def motion_id(self) -> int:
        ...
    @property
    def reason(self) -> int:
        ...
    @property
    def result(self) -> int:
        ...
    @property
    def state(self) -> int:
        ...
class MotionPhase:
    """
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
    Cancelled: typing.ClassVar[MotionPhase]
    Cancelling: typing.ClassVar[MotionPhase]
    Done: typing.ClassVar[MotionPhase]
    Error: typing.ClassVar[MotionPhase]
    Off: typing.ClassVar[MotionPhase]
    Preparing: typing.ClassVar[MotionPhase]
    Ready: typing.ClassVar[MotionPhase]
    Running: typing.ClassVar[MotionPhase]
    Settling: typing.ClassVar[MotionPhase]
    Waiting: typing.ClassVar[MotionPhase]
    __members__: typing.ClassVar[dict[str, MotionPhase]]
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
class MoveReply:
    @property
    def active_id(self) -> int:
        ...
    @property
    def arrived_mask(self) -> int:
        ...
    @property
    def duration_ms(self) -> int:
        ...
    @property
    def elapsed_ms(self) -> int:
        ...
    @property
    def enabled_mask(self) -> int:
        ...
    @property
    def fresh_mask(self) -> int:
        ...
    @property
    def max_error_rad(self) -> float:
        ...
    @property
    def reason(self) -> int:
        ...
    @property
    def request_id(self) -> int:
        ...
    @property
    def result(self) -> int:
        ...
    @property
    def session(self) -> int:
        ...
    @property
    def state(self) -> int:
        ...
class MotionStatus:
    @property
    def can_execute(self) -> bool:
        ...
    @property
    def error(self) -> str:
        ...
    @property
    def has_reply(self) -> bool:
        ...
    @property
    def id(self) -> int:
        ...
    @property
    def phase(self) -> MotionPhase:
        ...
    @property
    def reply(self) -> MoveReply:
        ...
    @property
    def reply_age_us(self) -> int:
        ...
class Robot:
    @staticmethod
    def forward(arg0: typing.Annotated[collections.abc.Sequence[typing.SupportsFloat | typing.SupportsIndex], "FixedSize(7)"]) -> typing.Annotated[list[float], "FixedSize(12)"]:
        ...
    @staticmethod
    def log_directory() -> str:
        ...
    def __init__(self, address: str = '192.168.0.50', command_port: typing.SupportsInt | typing.SupportsIndex = 5001, telemetry_port: typing.SupportsInt | typing.SupportsIndex = 5000) -> None:
        ...
    def cancel_motion(self) -> None:
        ...
    def clear_error(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    def clear_fault(self, timeout_ms: typing.SupportsInt | typing.SupportsIndex = 1000) -> None:
        ...
    def connect(self) -> None:
        ...
    def connected(self) -> bool:
        ...
    def disable(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    def disconnect(self) -> None:
        ...
    def download_mcu_log(self, options: McuLogOptions = ...) -> McuLogResult:
        ...
    def enable(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    def ensure_mode(self, arg0: typing.SupportsInt | typing.SupportsIndex, arg1: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    def fault_log(self) -> list[FaultEvent]:
        ...
    def get_register(self, arg0: typing.SupportsInt | typing.SupportsIndex, arg1: typing.SupportsInt | typing.SupportsIndex, arg2: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...
    def get_state(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    def gripper(self, arg0: typing.SupportsFloat | typing.SupportsIndex, arg1: typing.SupportsFloat | typing.SupportsIndex, arg2: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...
    def hold_command(self) -> typing.Annotated[list[float], "FixedSize(8)"]:
        ...
    def info(self) -> RobotInfo:
        ...
    def joint_stream_stats(self) -> JointStreamStats:
        ...
    def link_stats(self) -> LinkStats:
        ...
    def motion(self) -> MotionStatus:
        ...
    def movej(self, position: typing.Annotated[collections.abc.Sequence[typing.SupportsFloat | typing.SupportsIndex], "FixedSize(7)"], options: MoveOptions = ...) -> MotionResult:
        ...
    def movep(self, pose: typing.Annotated[collections.abc.Sequence[typing.SupportsFloat | typing.SupportsIndex], "FixedSize(12)"], options: CartesianMoveOptions = ...) -> MotionResult:
        ...
    def read_hold_command(self) -> typing.Annotated[list[float], "FixedSize(8)"]:
        ...
    def record_event(self, arg0: str) -> None:
        ...
    def recording(self) -> RecorderStats:
        ...
    def request_feedback(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    def resume(self) -> Mode:
        ...
    def send_force_pos(self, arg0: typing.SupportsInt | typing.SupportsIndex, arg1: typing.SupportsFloat | typing.SupportsIndex, arg2: typing.SupportsFloat | typing.SupportsIndex, arg3: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...
    def send_mit(self, arg0: typing.SupportsInt | typing.SupportsIndex, arg1: typing.SupportsFloat | typing.SupportsIndex, arg2: typing.SupportsFloat | typing.SupportsIndex, arg3: typing.SupportsFloat | typing.SupportsIndex, arg4: typing.SupportsFloat | typing.SupportsIndex, arg5: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...
    def send_pos_vel(self, arg0: typing.SupportsInt | typing.SupportsIndex, arg1: typing.SupportsFloat | typing.SupportsIndex, arg2: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...
    def send_vel(self, arg0: typing.SupportsInt | typing.SupportsIndex, arg1: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...
    def serial_log(self) -> SerialLogStats:
        ...
    def servej(self, arg0: JointCommand) -> None:
        ...
    def servep(self, arg0: CartesianCommand) -> None:
        ...
    def servo_off(self, group: MotorGroup = ..., wait: bool = True) -> None:
        ...
    def servo_on(self, group: MotorGroup = ...) -> None:
        ...
    def set_can_timeout_ms(self, arg0: typing.SupportsInt | typing.SupportsIndex, arg1: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    def set_default_cartesian_options(self, arg0: CartesianMoveOptions) -> None:
        ...
    def set_default_move_options(self, arg0: MoveOptions) -> None:
        ...
    def set_heartbeat(self, arg0: bool) -> None:
        ...
    def set_motor_profile(self, arg0: MotorProfile) -> None:
        ...
    def set_zero(self, enable_after: bool = False) -> None:
        ...
    def set_zero_position(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    def start_joint_stream(self, initial: JointCommand, options: JointStreamOptions = ...) -> None:
        ...
    def start_recording(self, directory: str, options: RecordOptions = ...) -> None:
        ...
    def start_serial_log(self, options: SerialLogOptions = ...) -> str:
        ...
    def state(self) -> RobotState:
        ...
    def stop(self, wait: bool = True) -> None:
        ...
    def stop_joint_stream(self) -> None:
        ...
    def stop_recording(self) -> None:
        ...
    def stop_serial_log(self) -> None:
        ...
    def store_parameters(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    def submit_movej(self, position: typing.Annotated[collections.abc.Sequence[typing.SupportsFloat | typing.SupportsIndex], "FixedSize(7)"], options: MoveOptions = ...) -> None:
        ...
    def submit_movep(self, pose: typing.Annotated[collections.abc.Sequence[typing.SupportsFloat | typing.SupportsIndex], "FixedSize(12)"], options: CartesianMoveOptions = ...) -> None:
        ...
    def switch_mode(self, arg0: Mode) -> None:
        ...
    def wait_motion(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> MotionResult:
        ...
    def write_register(self, arg0: typing.SupportsInt | typing.SupportsIndex, arg1: typing.SupportsInt | typing.SupportsIndex, arg2: typing.SupportsInt | typing.SupportsIndex, arg3: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
def firmware_status(ip: str, public_key_file: str = '') -> str:
    ...
def rollback_firmware(ip: str, public_key_file: str = '') -> None:
    ...
def update_firmware(ip: str, package: str, public_key_file: str = '') -> bool:
    ...
ARM_MASK: int = 127
ARM_MAX_JOINT_VELOCITY: float = 3.140000104904175
CAP_ADMITTANCE: int = 4096
CAP_CONTROLLED_STOP: int = 256
CAP_GROUPS: int = 64
CAP_JOINT_RT: int = 1
CAP_LOCAL_GRAVITY: int = 2048
CAP_TELEMETRY_V3: int = 128
MOTOR_FEEDBACK_STALE: int = 16
REGISTER_F32: int = 1
REGISTER_U32: int = 0
