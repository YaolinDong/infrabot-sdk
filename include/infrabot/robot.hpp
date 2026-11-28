/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#pragma once

/**
 * @file robot.hpp
 * @brief The C++ API of the InfraBot SDK: one Robot per arm controller.
 *
 * The SDK is the PC side of the controller's UDP protocol (infrabot/protocol.h).
 *
 * @par Timing classes
 * Every function is tagged with how it behaves:
 * - **[Blocking]** waits for the controller (a reply or telemetry), at most the
 *   time given. Never call it from a 1 kHz loop.
 * - **[Non-blocking]** returns at once: it sends one datagram or copies state.
 * - **[RT]** non-blocking and allocation-free on success: safe in a 1 kHz loop.
 *
 * A function that fails returns false (or a result code) and says why in `error`.
 * The messages are in Chinese.
 *
 * @par Lifecycle
 * connect() → set_motor_profile() (drives disabled, IDLE; runs the controller's
 * parameter self-check) → servo_on() → switch_mode() → commands →
 * stop() or switch_mode(Mode::Idle) → servo_off().
 *
 * While connected the SDK keeps the controller alive in IDLE and the NRT modes
 * (a 100 Hz heartbeat and the movej/movep session). In the RT modes servej() or
 * servep() must keep coming (at least 10 Hz, normally 1 kHz), or opt into
 * start_joint_stream() and refresh servej() within its producer timeout.
 *
 * A controller whose PC goes quiet (program closed, cable pulled) stops the arm
 * in a controlled way and holds it enabled. It never disables the drives by
 * itself (firmware with INFRABOT_CAP_CONTROLLED_STOP; older firmware disables).
 */

#include "infrabot/protocol.h"

#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

/// The InfraBot SDK.
namespace infrabot {

/// Control mode of the controller (INFRABOT_CONTROL_*).
enum class Mode : uint8_t {
    Idle = INFRABOT_CONTROL_IDLE,                    ///< Drives hold their command; no PC stream needed.
    RtJoint = INFRABOT_CONTROL_RT_JOINT,             ///< The PC streams joint targets with servej().
    RtCartesian = INFRABOT_CONTROL_RT_CARTESIAN,     ///< The PC streams tool poses with servep(); the controller solves IK at 1 kHz.
    NrtJoint = INFRABOT_CONTROL_NRT_JOINT,           ///< Point-to-point joint moves with movej().
    NrtCartesian = INFRABOT_CONTROL_NRT_CARTESIAN,   ///< Straight-line tool moves with movep().
    GravityFree = INFRABOT_CONTROL_GRAVITY_FREE,     ///< Local gravity compensation: the arm floats and can be moved by hand.
    GravityLock = INFRABOT_CONTROL_GRAVITY_LOCK,     ///< Local gravity compensation with a soft hold at the current pose.
    JointImpedance=INFRABOT_CONTROL_JOINT_IMPEDANCE, ///< Local joint impedance around the pose at entry.
    CartesianImpedance=INFRABOT_CONTROL_CARTESIAN_IMPEDANCE, ///< Local Cartesian impedance around the tool pose at entry.
    JointAdmittance=INFRABOT_CONTROL_JOINT_ADMITTANCE,       ///< Local joint admittance; every joint needs 0.03 rad of margin to its limits.
    CartesianAdmittance=INFRABOT_CONTROL_CARTESIAN_ADMITTANCE, ///< Local Cartesian admittance; same joint-limit margin.
    Unknown = 0xff,                                  ///< Not known yet (before the first telemetry).
};

/// Drive mode of J1–J7. J8 (the gripper) is always force-position.
enum class MotorProfile : uint8_t {
    Mit = INFRABOT_DRIVE_MODE_MIT,                   ///< MIT: position and velocity with kp/kd plus a torque feed-forward.
    PositionVelocity = INFRABOT_DRIVE_MODE_POS_VEL,  ///< Position with a velocity limit (the drive's v_des).
};

/**
 * @brief Drive groups enabled and disabled together.
 *
 * The values are the enabled-mask bits. Every control mode needs only the arm;
 * the gripper may be disabled or absent (another tool, identification runs).
 */
enum class MotorGroup : uint8_t {
    All = INFRABOT_ALL_MASK,          ///< J1–J8.
    Arm = INFRABOT_ARM_MASK,          ///< J1–J7.
    Gripper = INFRABOT_GRIPPER_MASK,  ///< J8.
};

/// What the controller is doing now (INFRABOT_STATUS_*), from telemetry.
enum class OperationalStatus : uint8_t {
    Booting = INFRABOT_STATUS_BOOTING, Precheck, ProfileRequired, Disabled, Enabling, IdleHold,
    RtJoint, RtCartesian, NrtJoint, NrtCartesian, Stopping, Fault, Recovery, Estop,
    Unknown = INFRABOT_STATUS_UNKNOWN, GravityFree = INFRABOT_STATUS_GRAVITY_FREE,
    GravityLock = INFRABOT_STATUS_GRAVITY_LOCK,
    JointImpedance=INFRABOT_STATUS_JOINT_IMPEDANCE,CartesianImpedance=INFRABOT_STATUS_CARTESIAN_IMPEDANCE,
    JointAdmittance=INFRABOT_STATUS_JOINT_ADMITTANCE,CartesianAdmittance=INFRABOT_STATUS_CARTESIAN_ADMITTANCE
};

/// Why the controller made a controlled stop (INFRABOT_STOP_*). describe_stop() explains it.
enum class StopReason : uint8_t {
    None = INFRABOT_STOP_NONE, StreamLost, MoveLost, MoveFailed, Requested, NotFollowing,
    InvalidTarget, FeedbackLost, DriveFault, TorqueLimit
};
/// Phase of a controlled stop: braking to rest, then holding enabled until a PC is back.
enum class StopPhase : uint8_t { None = INFRABOT_STOP_PHASE_NONE, Braking, Holding };
/// Result of the drive parameter self-check that set_motor_profile() runs.
enum class SelfCheck : uint8_t { NotRun = INFRABOT_SELFCHECK_NOT_RUN, Ok, Mismatch, NoReply };

/// The controller's identity, limits and capabilities, from the connect() handshake.
struct RobotInfo {
    std::string product = "infrabot-damiao";  ///< Product name.
    std::string firmware;                     ///< Firmware build string, e.g. "v0.2.0"; empty for V1-only firmware.
    uint16_t protocol_version = 0;  ///< DMIO envelope version, once connected.
    uint16_t protocol_major = 0;    ///< Protocol major version; must match the SDK's.
    uint16_t protocol_minor = 0;    ///< Protocol minor version; minors only add features.
    std::string board;              ///< Controller board name.
    std::string arm_model;          ///< Arm model name.
    uint8_t axes = INFRABOT_ARM_AXES;         ///< Arm joints (7).
    uint8_t motor_count = INFRABOT_MOTOR_COUNT;  ///< Drives including the gripper (8).
    bool received = false;     ///< A DMIO answer arrived (firmware with DMIF).
    uint32_t capabilities = 0; ///< INFRABOT_CAP_* bits: test these for optional features.
    uint8_t controller_status = static_cast<uint8_t>(OperationalStatus::Unknown);  ///< INFRABOT_STATUS_* at connect.
    uint8_t control_mode = static_cast<uint8_t>(Mode::Unknown);  ///< INFRABOT_CONTROL_* at connect.
    uint8_t motor_profile = static_cast<uint8_t>(MotorProfile::Mit);  ///< INFRABOT_DRIVE_MODE_* at connect.
    bool profile_ready = false;     ///< The motor profile was applied and passed its self-check.
    uint8_t enabled_mask = 0;       ///< Enabled drives at connect (bit i = J(i+1)).
    std::array<float, 8> position_min{};  ///< Joint position limits, rad.
    std::array<float, 8> position_max{};  ///< Joint position limits, rad.
    std::array<float, 8> velocity_max{};  ///< Drive velocity ranges, rad/s.
    std::array<float, 8> torque_max{};    ///< Drive torque ranges, N·m.
};

/**
 * @brief Measured state of the eight drives (joint frame, SI units).
 *
 * With telemetry v3 it also carries what the controller commands them.
 */
struct JointState {
    std::array<float, 8> position{};          ///< Position, rad.
    std::array<float, 8> velocity{};          ///< Velocity, rad/s.
    std::array<float, 8> torque{};            ///< Torque, N·m.
    std::array<uint8_t, 8> drive_state{};     ///< INFRABOT_DRIVE_*; drive_state_info() explains a code.
    std::array<uint8_t, 8> temperature{};     ///< MOS temperature, °C.
    std::array<uint16_t, 8> feedback_count{}; ///< Drive feedback counter; it stops when a drive goes quiet.
    bool has_command = false;                 ///< The command fields below are valid (telemetry v3).
    std::array<float, 8> command_position{};  ///< Planned command, rad.
    std::array<float, 8> command_velocity{};  ///< MIT velocity feed-forward or position-velocity v_des, rad/s.
    std::array<float, 8> command_torque{};    ///< MIT torque feed-forward, N·m.
    std::array<uint8_t, 8> temperature_coil{};///< Coil temperature, °C.
    std::array<uint8_t, 8> motor_flags{};     ///< INFRABOT_MOTOR_* flags.
    std::array<uint8_t, 8> drive_mode{};      ///< INFRABOT_DRIVE_MODE_* of each drive.
    uint32_t sequence = 0;                    ///< Telemetry sequence number.
    uint64_t timestamp_us = 0;                ///< Controller clock, µs.
    uint64_t host_time_us = 0;                ///< CLOCK_MONOTONIC when the packet arrived (not when state() was called).
    std::array<uint32_t, 8> feedback_age_ms{}; ///< Age of each drive's feedback, ms; UINT32_MAX before the first.
};

/// Controlled stop and parameter self-check (telemetry v3).
struct SafetyState {
    bool valid = false;                     ///< The fields below come from telemetry v3.
    StopReason stop_reason = StopReason::None;  ///< The latched stop, None after it is acknowledged.
    StopPhase stop_phase = StopPhase::None;     ///< Braking or holding.
    uint8_t stop_joint = INFRABOT_NO_JOINT; ///< Joint that caused the stop, 0-based; INFRABOT_NO_JOINT if none.
    uint8_t stop_detail = 0;                ///< Reason-specific detail (for a drive fault, its INFRABOT_DRIVE_* code).
    Mode stop_mode = Mode::Idle;            ///< The mode the stop interrupted; resume() re-enters it.
    SelfCheck selfcheck = SelfCheck::NotRun;    ///< Last self-check result.
    uint8_t selfcheck_joint = INFRABOT_NO_JOINT;  ///< Joint that failed the self-check, 0-based.
    uint8_t selfcheck_register = 0;         ///< Register that failed (INFRABOT_RID_*).
    uint32_t selfcheck_value = 0;           ///< Value read back (raw register bits).
    uint32_t selfcheck_expected = 0;        ///< Value expected (raw register bits).
};

/**
 * @brief Quality of the UDP link.
 *
 * The "last second" values are refreshed once a second by the receive thread.
 */
struct LinkStats {
    uint64_t tx_sent = 0;                         ///< Datagrams sent.
    uint64_t tx_errors = 0;                       ///< Datagrams that failed to send.
    uint64_t rx_telemetry = 0;                    ///< Telemetry packets received.
    uint64_t rx_lost = 0;                         ///< Telemetry sequence gaps.
    uint64_t rx_invalid = 0;                      ///< Datagrams not understood.
    uint32_t rx_last_second = 0;                  ///< Telemetry packets in the last second.
    uint32_t lost_last_second = 0;                ///< Telemetry packets lost in the last second.
    float loss_percent = 0.0f;                    ///< Loss in the last second, %.
    uint32_t rx_interval_max_us = 0;              ///< Longest gap between telemetry packets in the last second, µs.
    bool mcu_valid = false;                       ///< `mcu` is valid (telemetry v3).
    InfrabotLinkStats mcu{};                      ///< The controller's view of the link and its control loop.
};

/// Controller state from the newest telemetry packet. Returned by Robot::state().
struct RobotState {
    OperationalStatus operational_status = OperationalStatus::Unknown;  ///< What the controller is doing.
    Mode mode = Mode::Unknown;       ///< Current control mode.
    MotorProfile profile = MotorProfile::Mit;  ///< Current drive mode of J1–J7.
    bool connected = false;          ///< Sockets open; use telemetry_age_ms for liveness.
    bool modes_supported = false;    ///< Firmware with the control-mode state machine.
    bool profile_ready = false;      ///< The motor profile was applied and passed its self-check.
    bool enabled = false;            ///< Any drive enabled.
    bool zero_valid = false;         ///< The drives' zero positions are calibrated.
    uint8_t enabled_mask = 0;        ///< Enabled drives (bit i = J(i+1)).
    uint8_t controller_state = 0;    ///< Raw INFRABOT_STATUS_*.
    uint8_t telemetry_version = 0;   ///< Telemetry format, 2 or 3.
    bool command_active = false;     ///< The controller applies a PC command stream.
    uint32_t command_applied_sequence = 0;  ///< Sequence of the newest command the controller applied.
    uint32_t telemetry_age_ms = UINT32_MAX; ///< Age of the newest telemetry, ms; UINT32_MAX before the first.
    uint32_t telemetry_epoch = 0; ///< Increments when telemetry resynchronizes after a stale or restarted source.
    JointState joints{};             ///< Per-drive state.
    SafetyState safety{};            ///< Controlled stop and self-check.
};

/// A drive fault, a controlled stop or a self-check result, as it happened. See Robot::fault_log().
struct FaultEvent {
    /// What happened.
    enum class Kind : uint8_t { DriveFault, DriveCleared, Stop, SelfCheck, Link };
    Kind kind = Kind::DriveFault;   ///< What happened.
    uint64_t host_time_us = 0;      ///< PC steady clock, µs.
    uint64_t mcu_time_us = 0;       ///< Controller clock, µs.
    uint8_t joint = INFRABOT_NO_JOINT; ///< Joint, 0-based; INFRABOT_NO_JOINT if none.
    uint8_t code = 0;               ///< INFRABOT_DRIVE_*, INFRABOT_STOP_* or INFRABOT_SELFCHECK_*.
    uint8_t detail = 0;             ///< Code-specific detail.
    std::string message;            ///< Chinese: what happened, the probable cause and what to do.
};

/**
 * @brief An RT_JOINT command for J1–J7. Sent with Robot::servej().
 *
 * MIT profile: the drives track position and velocity with kp/kd plus the
 * torque feed-forward. Position-velocity profile: the drives move to position
 * at no more than velocity_limit (their v_des; 0 would stop them); velocity, kp,
 * kd and torque are not used.
 */
struct JointCommand {
    std::array<float, 7> position{};       ///< Target, rad.
    std::array<float, 7> velocity{};       ///< MIT velocity feed-forward, rad/s.
    std::array<float, 7> torque{};         ///< MIT torque feed-forward, N·m.
    std::array<float, 7> kp{};             ///< MIT stiffness, 0–500.
    std::array<float, 7> kd{};             ///< MIT damping, 0–5.
    std::array<float, 7> velocity_limit{}; ///< Position-velocity v_des, rad/s.
    JointCommand();                        ///< Template gains and a 3 rad/s velocity limit.
};

/// Cartesian speed limits. 0 selects the controller's default, which is its maximum.
struct CartesianLimits {
    float linear_velocity = 0.0f;      ///< m/s, at most 0.5.
    float angular_velocity = 0.0f;     ///< rad/s, at most π.
    float linear_acceleration = 0.0f;  ///< m/s², at most 2.
    float angular_acceleration = 0.0f; ///< rad/s², at most 2π.
};

/// Tool pose in the base frame: x, y, z in metres, then the row-major 3×3 rotation.
using Pose = std::array<float, 12>;

/**
 * @brief An RT_CARTESIAN command. Sent with Robot::servep().
 *
 * The controller moves the tool to `pose` along a straight line within `limits`,
 * solving IK at 1 kHz. Per joint, the output gains work as in JointCommand.
 */
struct CartesianCommand {
    Pose pose{};                            ///< Target tool pose.
    CartesianLimits limits{};               ///< Speed limits of the straight-line motion.
    std::array<float, 7> torque{};          ///< MIT torque feed-forward, N·m.
    std::array<float, 7> kp{};              ///< MIT stiffness.
    std::array<float, 7> kd{};              ///< MIT damping.
    std::array<float, 7> velocity_limit{};  ///< Position-velocity v_des, rad/s.
    CartesianCommand();                     ///< Identity rotation, template gains, 3 rad/s velocity limit.
};

/// Options of a joint move. Used by Robot::movej() and Robot::submit_movej().
struct MoveOptions {
    uint32_t timeout_ms = 30000;       ///< The move fails if it has not arrived by then.
    uint32_t dwell_ms = 100;           ///< How long the arm must stay within tolerance to count as arrived.
    float position_tolerance = 0.01f;  ///< Arrival tolerance per joint, rad; 0.001–0.05.
    float velocity_tolerance = 0.05f;  ///< Arrival speed tolerance per joint, rad/s.
    std::array<float, 7> velocity_max{};      ///< Speed cap per joint, rad/s (position-velocity: also v_des).
    std::array<float, 7> acceleration_max{};  ///< Acceleration cap per joint, rad/s².
    std::array<float, 7> kp{};                ///< MIT stiffness.
    std::array<float, 7> kd{};                ///< MIT damping.
    std::array<float, 7> torque{};            ///< MIT torque feed-forward, N·m.
    MoveOptions();                            ///< 0.1 rad/s, 0.2 rad/s² and the template gains.
};

/// Options of a Cartesian move. Used by Robot::movep() and Robot::submit_movep().
struct CartesianMoveOptions {
    uint32_t timeout_ms = 30000;       ///< The move fails if it has not arrived by then.
    uint32_t dwell_ms = 100;           ///< How long the arm must stay within tolerance to count as arrived.
    float position_tolerance = 0.01f;  ///< Arrival tolerance of the tool, m and rad; 0.0005–0.05.
    float velocity_tolerance = 0.05f;  ///< Arrival speed tolerance per joint, rad/s.
    CartesianLimits limits{};          ///< Speed limits of the straight-line motion.
    /** Per-joint speed caps, rad/s. 0 selects the application maximum 3.14 (MIT) or the
     * default v_des (position-velocity, where velocity_max is the drives' v_des). */
    std::array<float, 7> velocity_max{};
    std::array<float, 7> acceleration_max{}; ///< Per-joint acceleration caps, rad/s²; 0 selects 100.
    std::array<float, 7> kp{};               ///< MIT stiffness.
    std::array<float, 7> kd{};               ///< MIT damping.
    CartesianMoveOptions();                  ///< Template gains.
};

/// Outcome of a movej(), movep() or wait_motion().
struct MotionResult {
    uint8_t result = 0;     ///< INFRABOT_MOVE_*: 0 means arrived.
    uint8_t state = 0;      ///< INFRABOT_MOVE_STATE_*.
    uint32_t motion_id = 0; ///< The move this result belongs to.
    uint32_t arrived_mask = 0;  ///< Joints within tolerance (bit i = J(i+1)).
    uint32_t fresh_mask = 0;    ///< Joints with fresh feedback.
    float max_error_rad = 0;    ///< Largest remaining error: rad for joint moves, m or rad for Cartesian moves.
    uint32_t reason = 0;        ///< INFRABOT_MOVE_* reason the move ended; move_reason_text() explains it.
    std::string message;        ///< Why it failed, in Chinese; empty on arrival.
};

/// The movej/movep session of the NRT modes. Returned by Robot::motion().
struct MotionStatus {
    /// Session phase.
    enum class Phase : uint8_t { Off, Preparing, Waiting, Running, Settling, Ready, Done, Cancelling, Cancelled, Error };
    Phase phase = Phase::Off;     ///< Session phase.
    uint32_t id = 0;              ///< The newest move; 1 is the hold made on entering the mode.
    bool can_execute = false;     ///< A new move is accepted now.
    bool has_reply = false;       ///< `reply` is valid.
    InfrabotMoveReply reply{};    ///< The controller's newest DMOS reply for the move.
    uint64_t reply_age_us = UINT64_MAX;  ///< Age of `reply`, µs.
    std::string error;            ///< Why the session failed, in Chinese.
};

/// Per-joint stream values in wire units (DMCP v5/v7). Used by the protocol-level functions.
struct StreamProfile {
    std::array<int32_t, 8> velocity_mrad_s{};       ///< MIT velocity feed-forward, mrad/s.
    std::array<int32_t, 8> kp_milli{};              ///< MIT stiffness × 1000.
    std::array<int32_t, 8> kd_milli{};              ///< MIT damping × 1000.
    std::array<int32_t, 8> torque_mNm{};            ///< MIT torque feed-forward, mN·m.
    std::array<int32_t, 8> velocity_limit_mrad_s{}; ///< Position-velocity v_des, mrad/s.
    std::array<int32_t, 8> torque_ratio_milli{};    ///< Force-position current limit, ‰ of the drive maximum.
};

/// One DMCP v4 maintenance operation, in wire units. Used by Robot::send_maintenance().
struct MaintenanceRequest {
    uint8_t command = 0;                 ///< INFRABOT_CMD_*.
    uint8_t motor = INFRABOT_MOTOR_ALL;  ///< Drive 0–7, or INFRABOT_MOTOR_ALL.
    uint8_t mode = 0;                    ///< INFRABOT_DRIVE_MODE_* or INFRABOT_CONTROL_*, by command.
    uint8_t value_type = 0;              ///< INFRABOT_REGISTER_U32 or _F32.
    uint8_t register_id = 0;             ///< INFRABOT_RID_*.
    uint32_t value = 0;                  ///< Register value or command argument.
    int32_t position_mrad = 0;           ///< Position, mrad (target_position_mrad[motor] on the wire).
    int32_t velocity_mrad_s = 0;         ///< Velocity, mrad/s.
    int32_t kp_milli = 0;                ///< Stiffness × 1000.
    int32_t kd_milli = 0;                ///< Damping × 1000.
    int32_t torque_mNm = 0;              ///< Torque, mN·m.
    int32_t velocity_limit_mrad_s = 0;   ///< Velocity limit, mrad/s.
    int32_t torque_ratio_milli = 0;      ///< Current limit, ‰ of the drive maximum.
};

/// Options of Robot::start_recording().
struct RecordOptions {
    std::string label;             ///< Appended to the directory name.
    uint32_t rotate_seconds = 600; ///< Start new telemetry and command files this often; 0 never.
    bool commands = true;          ///< Also log every command datagram sent.
};

/// Progress of a recording. Returned by Robot::recording().
struct RecorderStats {
    bool active = false;           ///< A recording is running.
    std::string directory;         ///< Directory being written.
    uint64_t telemetry_rows = 0;   ///< Telemetry rows written.
    uint64_t command_rows = 0;     ///< Command rows written.
    uint64_t events = 0;           ///< Event rows written.
    uint64_t dropped = 0;          ///< Rows lost because the disk could not keep up.
    uint32_t file_index = 0;       ///< Index of the current telemetry/command file.
    std::string error;             ///< Last write error.
};

/// Options of Robot::start_serial_log(): saving the controller's UART debug output on the PC.
struct SerialLogOptions {
    std::string port = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0";  ///< Serial device.
    uint32_t baud = 115200;        ///< Baud rate.
    std::string directory;         ///< Output directory; empty selects Robot::log_directory().
};

/// Progress of the serial log. Returned by Robot::serial_log().
struct SerialLogStats {
    bool active = false;           ///< The log is running.
    std::string path;              ///< File being written.
    uint64_t lines = 0;            ///< Lines written.
    uint64_t bytes = 0;            ///< Bytes written.
    uint32_t reconnects = 0;       ///< Times the port was lost (unplug, reset) and reopened.
    bool port_open = false;        ///< The port is open now.
    std::string error;             ///< Last port or write error.
};

/**
 * @brief Options of Robot::download_mcu_log().
 *
 * The board keeps its own run log in SDRAM (firmware with INFRABOT_CAP_RUN_LOG):
 * the text it printed (newest 10 minutes) and its 1 kHz telemetry (newest 10 s).
 * It is lost on power-off or reset.
 */
struct McuLogOptions {
    std::string directory;         ///< Output directory; empty selects Robot::log_directory().
    std::string label;             ///< Appended to the file names.
    uint16_t port = INFRABOT_LOG_PORT;  ///< Log port of the board.
    uint32_t timeout_ms = 300;     ///< Timeout per request.
    uint32_t retries = 3;          ///< Retries per request.
    /** Called after each reply with the text lines and telemetry rows received so far and
     * the totals the board reported. Return false to cancel. */
    std::function<bool(uint32_t text_lines, uint32_t text_total, uint32_t telemetry_rows,
                       uint32_t telemetry_total)> progress;
};

/// Files written by Robot::download_mcu_log().
struct McuLogResult {
    std::string text_path;         ///< `mcu_YYYYMMDDHHMMSS[_label].log`, one `<mcu_time_s> <line>` per line.
    std::string telemetry_path;    ///< mcu_..._telemetry.csv, the columns of start_recording()'s telemetry with host_time_s = 0.
    uint32_t boot_id = 0;          ///< Changes on every controller boot.
    uint32_t text_lines = 0;       ///< Text lines saved.
    uint32_t telemetry_rows = 0;   ///< Telemetry rows saved.
    uint32_t lost_bytes = 0;       ///< printf bytes the board could not keep (log task starved).
    uint32_t writer_max_us = 0;    ///< Longest log-task pass since the previous download, µs.
    bool fault_latched = false;    ///< The board froze the telemetry at a fault: the CSV ends at that tick.
};

/// One tunable controller parameter (firmware with INFRABOT_CAP_PARAMETERS, protocol 1.1).
struct ParameterInfo {
    uint16_t id = 0;        ///< group << 8 | index.
    uint8_t group = 0;      ///< INFRABOT_PARAM_GROUP_*: gravity, impedance or admittance.
    std::string name;       ///< E.g. "imp.joint_k".
    std::string unit;       ///< E.g. "Nm/rad"; empty when dimensionless.
    float requested = 0.f;  ///< The last value written, or the default after a reboot.
    float applied = 0.f;    ///< The value the controller uses now, slewing toward `requested`.
    float minimum = 0.f;    ///< Lowest accepted value.
    float maximum = 0.f;    ///< Highest accepted value.
    float default_value = 0.f;  ///< Value after a reboot.
    float rate = 0.f;       ///< Largest change of the applied value, units per second.
    bool writable = false;  ///< set_parameter() accepts it.
    bool boolean = false;   ///< The value is 0 or 1.
};

/**
 * @brief One diagnostic snapshot (firmware with INFRABOT_CAP_DIAGNOSTICS).
 *
 * The window counters cover the controller's last one-second status window; the
 * `_total` counters count since boot.
 */
struct DiagnosticsInfo {
    uint32_t uptime_ms = 0;              ///< Since the application started; wraps after 49 days.
    uint32_t snapshot_seconds = 0;       ///< Status-task second of the window.
    uint32_t control_hz = 0;             ///< Control ticks completed in the window (1000 when healthy).
    uint32_t control_missed = 0;         ///< Control ticks missed in the window.
    uint32_t control_period_max_us = 0;  ///< Longest control period in the window, µs.
    uint32_t can_tx = 0;                 ///< Drive frames queued in the window.
    uint32_t can_rx = 0;                 ///< CAN frames received in the window.
    uint32_t can_tx_errors = 0;          ///< Drive frames that could not be queued in the window.
    uint32_t command_rx = 0;             ///< Command datagrams accepted in the window.
    uint32_t command_invalid = 0;        ///< Command datagrams rejected in the window.
    uint32_t telemetry_tx = 0;           ///< Telemetry packets sent in the window.
    uint32_t can_error_events = 0;       ///< CAN error events since boot.
    uint32_t control_missed_total = 0;   ///< Control ticks missed since boot.
    uint32_t can_tx_errors_total = 0;    ///< Drive frames not queued since boot.
    uint32_t command_invalid_total = 0;  ///< Command datagrams rejected since boot.
    uint8_t can_tec = 0;                 ///< CAN transmit error counter at the end of the window.
    uint8_t can_rec = 0;                 ///< CAN receive error counter at the end of the window.
    bool can_error_passive = false;      ///< CAN controller error-passive.
    bool can_bus_off = false;            ///< CAN controller bus-off.
    uint32_t heap_free_bytes = 0;        ///< FreeRTOS heap free now.
    uint32_t heap_min_free_bytes = 0;    ///< FreeRTOS heap free at its lowest since boot.
    uint32_t stack_free_control_bytes = 0;   ///< Least free stack ever of the control task.
    uint32_t stack_free_status_bytes = 0;    ///< Least free stack ever of the status task.
    uint32_t stack_free_ethernet_bytes = 0;  ///< Least free stack ever of the Ethernet task.
    std::array<uint8_t, 8> drive_state{};  ///< INFRABOT_DRIVE_* of J1–J8, now.
    uint8_t controller_status = 0;         ///< INFRABOT_STATUS_*, now.
    uint8_t invalid_reason = 0;            ///< INFRABOT_INVALID_* of the last rejected command datagram.
};

/**
 * @brief Options of the native RT_JOINT producer (Robot::start_joint_stream()).
 *
 * The producer runs on Robot's own thread. Starting it never configures or
 * enables drives and never changes modes.
 */
struct JointStreamOptions {
    uint32_t transmit_hz = 1000;        ///< Send rate.
    uint32_t command_timeout_ms = 100;  ///< Stop when servej() has not refreshed the command for this long.
    uint32_t feedback_timeout_ms = 100; ///< Stop when drive feedback is older than this.
    bool allow_zero_kp = false; ///< Accept kp = 0; off because the controller's STOP does not restore holding gains.
};
/// State of the native RT_JOINT producer. Returned by Robot::joint_stream_stats().
struct JointStreamStats {
    bool active = false;          ///< The producer is sending.
    uint64_t sent = 0;            ///< Commands sent.
    uint64_t skipped_deadlines = 0;  ///< Send deadlines missed.
    std::string fault; ///< Why it stopped; latched, so a failed stream needs an explicit start.
};

/**
 * @brief The client for one arm controller.
 *
 * One Robot per controller. It owns two UDP sockets and two threads: a receive
 * thread that parses telemetry and replies, and a 1 kHz service thread that
 * sends the heartbeat, keeps the NRT session alive and runs the native joint
 * stream. Functions are thread-safe unless noted.
 *
 * A program follows the groups below in order: connect(), wait for fresh
 * telemetry, set_motor_profile(), servo_on(), switch_mode() into the mode it
 * needs, commands or moves, then stop() (the arm holds, enabled) or
 * servo_off(), and disconnect().
 */
class Robot {
public:
    static constexpr uint8_t kAllMotors = INFRABOT_MOTOR_ALL;  ///< Motor index addressing every drive.
    /**
     * @brief [Non-blocking] Creates the client; connect() opens the sockets.
     *
     * Nothing is sent yet. The address and ports are fixed for the life of the
     * object: create one Robot per controller and keep it for the whole
     * session. Destroying it disconnects.
     * @param address Controller IPv4 address.
     * @param command_port Controller command port.
     * @param telemetry_port Telemetry port, the same on the PC and the controller.
     */
    Robot(std::string address = "192.168.0.50", uint16_t command_port = INFRABOT_COMMAND_PORT,
          uint16_t telemetry_port = INFRABOT_TELEMETRY_PORT);
    ~Robot();  ///< [Blocking] Disconnects and joins the worker threads.
    Robot(const Robot&) = delete;
    Robot& operator=(const Robot&) = delete;

    /**
     * @name Connection
     * Open and close the link, and read what the controller reports.
     */
    ///@{
    /**
     * @brief [Blocking, up to 0.5 s] Opens the link: negotiates the protocol, then starts the heartbeat.
     *
     * Call it before anything that talks to the controller (start_serial_log(),
     * download_mcu_log() and the firmware update functions work without it). It
     * binds `telemetry_port` on this PC, asks the controller for its identity
     * and capabilities (then in info()), registers for telemetry and starts the
     * receive and service threads. Calling it again reconnects from scratch.
     *
     * Success means the controller answered the handshake; telemetry follows a
     * few milliseconds later. The setup and power calls refuse telemetry older
     * than 100 ms, so wait until `state().telemetry_age_ms` is below 100 before
     * calling them.
     *
     * A V1-only controller is treated as protocol 1.0 with unknown firmware. A
     * different major version or no valid reply fails.
     * @post The receive and service threads run and telemetry starts to arrive.
     * @see info(), state(), disconnect()
     * @param[out] error Why it failed.
     * @return true when connected.
     */
    bool connect(std::string& error);
    /**
     * @brief [Blocking] Closes the link: stops the threads and closes the sockets.
     *
     * Call it when the program is done with the controller; the destructor
     * calls it too, and a second call does nothing. A native joint stream ends
     * with it. state() keeps the last values, with `connected` false.
     *
     * Nothing is sent: the controller is not told.
     * @warning In IDLE, RT and NRT modes the controller misses the PC after
     * 100 ms, makes a controlled stop and holds the arm enabled; a local mode
     * (gravity compensation, impedance, admittance) keeps running without a
     * PC. Call stop() first to leave the arm still, or servo_off() to power it
     * down.
     * @see stop(), servo_off()
     */
    void disconnect();
    /**
     * @brief [Non-blocking] Whether connect() succeeded and disconnect() has not been called since.
     *
     * It describes this side only and stays true when the controller is
     * switched off or the cable is pulled. Whether the controller is alive
     * shows in `state().telemetry_age_ms`: the SDK treats telemetry older than
     * 100 ms as stale.
     * @note A live controller: connected() and telemetry under 100 ms old.
     */
    bool connected() const;
    /**
     * @brief [Non-blocking] Identity, limits and capabilities, read once by connect().
     *
     * Firmware and protocol version, joint limits and the INFRABOT_CAP_* bits.
     * Test a bit before using an optional feature, e.g. `info().capabilities &
     * INFRABOT_CAP_PARAMETERS`. The status fields are a snapshot of connect
     * time; state() has the current status.
     */
    RobotInfo info() const;
    /**
     * @brief [RT] A copy of the newest telemetry: joints, mode, enabled drives, faults and stops.
     *
     * The controller sends telemetry at 1 kHz and the receive thread keeps the
     * newest packet; state() copies it without waiting, so it may be called as
     * often as needed, also from a 1 kHz loop. Check `telemetry_age_ms` first: a
     * large value means the values are old (link lost, controller off). To
     * handle every packet instead of polling, use set_telemetry_handler().
     */
    RobotState state() const;
    /**
     * @brief [Non-blocking] Registers a callback for every telemetry packet.
     *
     * Use it to react to each packet (1 kHz) instead of polling state(), e.g.
     * to feed a filter or a plot. Called by the receive thread after state()
     * reflects the packet. Keep it short, and never call a blocking Robot
     * function from it.
     * @warning Never call a blocking Robot function from the handler.
     * @param handler The callback; an empty function removes it.
     */
    void set_telemetry_handler(std::function<void(const RobotState&)> handler);
    /**
     * @brief [Non-blocking] Link quality: datagrams sent and received, telemetry loss and gaps.
     *
     * Use it to judge the network. Over the last second, a `loss_percent` above
     * 1 or an `rx_interval_max_us` above 50000 (50 ms) points to the cable, a
     * switch or a busy PC; fault_log() then also records a link warning. With
     * telemetry v3, `mcu` adds the controller's own view: commands received
     * and its control loop timing.
     */
    LinkStats link_stats() const;
    /**
     * @brief [Non-blocking] Faults, stops, self-checks and link warnings, oldest first, at most 200.
     *
     * The receive thread adds an event when a drive fault appears or clears, a
     * controlled stop is latched, a self-check result arrives or the link is
     * poor. Each FaultEvent has a Chinese message with the probable cause and
     * what to do. Read it after a failure to see what happened, even when the
     * state has changed since. The log lives as long as the Robot.
     */
    std::vector<FaultEvent> fault_log() const;
    /**
     * @brief [Non-blocking] Turns the heartbeat on or off. On by default.
     *
     * The heartbeat (100 Hz in IDLE and the NRT modes) tells the controller
     * that this PC is present and in control. A program that only watches (a
     * monitor or a logger) while another program controls the arm turns it off
     * before connect(), so it never looks like the controlling PC. The program
     * that controls the arm keeps it on.
     */
    void set_heartbeat(bool enabled);
    ///@}

    /**
     * @name Setup (IDLE, every drive disabled)
     * Done once after connect(), before the drives are enabled.
     */
    ///@{
    /**
     * @brief [Blocking, 3 s] Selects the drive mode of J1–J7 (J8 stays force-position) and checks the drives.
     *
     * The step between connect() and servo_on(): servo_on() refuses until a
     * profile is applied (RobotState::profile_ready). Call it in IDLE with every
     * drive disabled. MIT is the usual choice, and the native joint stream
     * needs it; position-velocity lets each drive's own position loop move to a
     * target at a speed limit.
     *
     * The controller then reads back every drive's IDs and PMAX/VMAX/TMAX and
     * refuses to enable on a mismatch (INFRABOT_CAP_SELFCHECK); `error` then names
     * the drive and register.
     * @pre IDLE, every drive disabled; servo_on() refuses until a profile is applied.
     * @see servo_on(), set_zero()
     * @param profile MIT or position-velocity.
     * @param[out] error Why it failed.
     */
    bool set_motor_profile(MotorProfile profile, std::string& error);
    /**
     * @brief [Blocking, 2 s] Stores the zero of every drive at its current angle.
     *
     * Calibration only, not part of a normal start.
     * @warning Every joint angle, limit and pose is measured from this zero, so
     * a zero taken in the wrong pose shifts all of them.
     * @pre The arm in its zero pose, every drive disabled.
     * @param enable_after Enable both groups afterwards; needs an applied motor profile.
     * @param[out] error Why it failed.
     */
    bool set_zero(bool enable_after, std::string& error);
    ///@}

    /**
     * @name Power
     * Enable and disable the drives, stop and resume, clear faults.
     */
    ///@{
    /**
     * @brief [Blocking, 3 s] Enables a drive group in IDLE and waits for telemetry to show it.
     *
     * Enabled drives produce torque: the arm holds its current pose in IDLE
     * until a mode is selected. Call it after set_motor_profile(). The arm
     * group (J1–J7) is enough for every control mode; enable the gripper only
     * when it is fitted. A group that is already enabled is refused.
     *
     * The controller brings the drives up one at a time. A lost request is
     * retried every 250 ms. On timeout that group is disabled again; the other
     * group keeps holding.
     * @pre IDLE, a motor profile applied, and the group not already enabled.
     * @post The group holds its pose, producing torque.
     * @see servo_off(), stop(), set_motor_profile()
     * @param group Arm, gripper or both.
     * @param[out] error Why it failed.
     */
    bool servo_on(MotorGroup group, std::string& error);
    /// [Blocking, 3 s] servo_on() for both groups.
    bool servo_on(std::string& error);
    /**
     * @brief [Blocking, 2 s] Disables a group at once, in any mode.
     *
     * Use it to power down at the end. To end motion and keep the arm held,
     * use stop() instead: disabling the arm also ends its move session and
     * native joint stream. The other group keeps running.
     * @warning Disabled drives produce no torque: an arm that is not supported falls.
     * @see stop(), servo_on()
     * @param group Arm, gripper or both.
     * @param[out] error Why it failed.
     * @param wait false: [Non-blocking], sends the request twice and returns.
     */
    bool servo_off(MotorGroup group, std::string& error, bool wait = true);
    /// [Blocking, 2 s] servo_off() for both groups.
    bool servo_off(std::string& error);
    /**
     * @brief [Blocking, 2 s] Controlled stop, in any mode: brakes to rest, then holds enabled in IDLE.
     *
     * The normal way to end any motion: a stream, a move or a local mode. The
     * controller brakes the joints to rest and holds them, and the call returns
     * once the arm is still. Call it before disconnect() when the arm should
     * stay where it is. A disabled arm returns at once.
     *
     * The stop stays latched in RobotState::safety: resume() goes back to the
     * interrupted mode, switch_mode() enters another one. Firmware without
     * INFRABOT_CAP_CONTROLLED_STOP is refused; use servo_off() there.
     * @post The arm is still and enabled in IDLE, and the stop stays latched.
     * @see resume(), switch_mode(), cancel_motion()
     * @param[out] error Why it failed.
     * @param wait false: [Non-blocking], sends the request twice and returns.
     */
    bool stop(std::string& error, bool wait = true);
    /**
     * @brief [Blocking, 3 s] Re-enters the mode a controlled stop interrupted.
     *
     * Use it to continue after a stop, whatever caused it: stop(), a lost
     * stream once the link is back, a failed move or a supervisor. The mode is
     * SafetyState::stop_mode, entered from where the arm stopped as
     * switch_mode() does: RT modes stream from hold_command(), NRT modes get a
     * new movej/movep session holding there. A stop that happened in IDLE is
     * only acknowledged.
     *
     * It waits for the brake to end and the arm to settle, and refuses a
     * disabled or faulted arm drive. Needs INFRABOT_CAP_RESUME.
     * @pre A latched stop (state().safety.stop_reason) and an enabled arm.
     * @see stop(), switch_mode()
     * @param[out] error Why it failed.
     * @param[out] resumed The mode entered, if not null.
     */
    bool resume(std::string& error, Mode* resumed = nullptr);
    /**
     * @brief [Blocking, timeout] Clears the drives that report a fault.
     *
     * A faulted drive (over-voltage, over-temperature, lost CAN and so on) no
     * longer follows commands. `RobotState::joints.drive_state` shows the code,
     * and fault_log() says why and what to do. Remove the cause, then
     * servo_off() the group if it is still enabled, clear_fault() and
     * servo_on() again.
     *
     * Only disabled drives are cleared; an enabled faulted drive is refused.
     * The call waits until no cleared drive reports a fault.
     * @pre The cause is gone and the faulted drives are disabled.
     * @see drive_state_info(), describe_drive_fault(), fault_log()
     * @param[out] error Why it failed.
     * @param timeout_ms How long to wait for the faults to clear.
     * @return true when no drive is left faulted.
     */
    bool clear_fault(std::string& error, uint32_t timeout_ms = 1000);
    /// [Non-blocking] Old name of `servo_off(MotorGroup::All, error, false)`; new code calls servo_off().
    bool cancel(std::string& error);
    ///@}

    /**
     * @name Control mode
     * Choose who moves the arm: the PC (RT), the SDK's moves (NRT) or the controller alone (local modes).
     */
    ///@{
    /**
     * @brief [Blocking, 2 s] Switches the controller's control mode.
     *
     * RT modes follow a stream from the PC (servej(), servep()). NRT modes run
     * point-to-point moves (movej(), movep()). The local modes (gravity
     * compensation, impedance, admittance) run on the controller by themselves.
     * A mode other than IDLE is entered from IDLE with the arm enabled and
     * still; to change modes, go back through IDLE with stop() or
     * `switch_mode(Mode::Idle)`.
     *
     * RT modes: the caller then streams servej() or servep() starting from
     * hold_command(). NRT modes: the SDK starts the movej/movep session, holding
     * the current command, and keeps it alive until the mode changes.
     *
     * While another program on this PC stopped sending less than 0.5 s ago, or a
     * stopped local mode is still handing its gains back, the controller answers
     * busy; the call retries for up to 1 s. If the controller does not answer at
     * all (outcome unknown), the SDK asks for a controlled stop.
     * @pre IDLE, the arm enabled, every joint below 0.05 rad/s.
     * @post hold_command() holds what the arm holds now.
     * @see stop(), resume(), hold_command()
     * @param mode The mode to enter.
     * @param[out] error Why it failed.
     */
    bool switch_mode(Mode mode, std::string& error);
    /**
     * @brief [Non-blocking] The hold command read when the mode was entered.
     *
     * Joint frame, rad; J8 only when enabled. Under gravity the measured
     * position sits below this command (MIT holds by stiffness), so start an RT
     * stream or a first move here, not at `state().joints.position`, or the arm
     * drops by that sag.
     * @see switch_mode(), read_hold_command()
     */
    std::array<float, 8> hold_command() const;
    /**
     * @brief [Blocking, 0.5 s] Reads the hold command of the enabled drives now.
     *
     * switch_mode() calls it to fill hold_command(); call it to see what the
     * controller holds at another time, e.g. in IDLE after a stop.
     * @param[out] position Joint frame, rad.
     * @param[out] error Why it failed.
     */
    bool read_hold_command(std::array<float, 8>& position, std::string& error);
    ///@}

    /**
     * @name RT modes (RT_JOINT, RT_CARTESIAN)
     * The PC sends a target every period, normally every millisecond, and a gap
     * of 100 ms makes the controller stop the arm.
     */
    ///@{
    /**
     * @brief [RT] Sends one RT_JOINT command.
     *
     * Call it in RT_JOINT at a steady rate, normally 1 kHz; after 100 ms
     * without a command the controller makes a controlled stop. Begin at
     * hold_command() and move the targets smoothly: each command is a new
     * target for the drives. Or let start_joint_stream() keep the rate and
     * call servej() only when the target changes.
     *
     * While the native joint stream is active it publishes the newest command
     * to the stream and refreshes its producer timeout instead. It never starts
     * or restarts a stream.
     * @pre RT_JOINT with the arm enabled; the native stream, if any, is active.
     * @warning 100 ms without a command: the controller stops the arm.
     * @see servep(), start_joint_stream()
     * @param command Targets and gains of J1–J7.
     * @param[out] error Why it failed.
     */
    bool servej(const JointCommand& command, std::string& error);
    /**
     * @brief [RT] servej() with MIT gains; torque 0, v_des 3 rad/s.
     * @param position Targets, rad.
     * @param velocity Velocity feed-forward, rad/s.
     * @param kp MIT stiffness.
     * @param kd MIT damping.
     * @param[out] error Why it failed.
     */
    bool servej(const std::array<float, 7>& position, const std::array<float, 7>& velocity,
                const std::array<float, 7>& kp, const std::array<float, 7>& kd,
                std::string& error);
    /**
     * @brief [RT] Sends one RT_CARTESIAN command.
     *
     * Call it in RT_CARTESIAN at a steady rate, 100 Hz to 1 kHz; after 100 ms
     * without a command the controller makes a controlled stop. The controller
     * moves the tool toward the pose along a straight line within the limits
     * and solves the IK at 1 kHz, so the targets may be sparser than in
     * RT_JOINT. Begin at the pose forward() gives for hold_command().
     * @pre RT_CARTESIAN with the arm enabled.
     * @warning 100 ms without a command: the controller stops the arm.
     * @see servej(), forward()
     * @param command Target pose, limits and gains.
     * @param[out] error Why it failed.
     */
    bool servep(const CartesianCommand& command, std::string& error);

    /**
     * @brief [Non-blocking] Starts the native RT_JOINT producer.
     *
     * The SDK's own thread then sends the newest command at a steady rate, so
     * the caller's loop may be slower and irregular (Python, for example): each
     * servej() replaces the target and must come within
     * `options.command_timeout_ms`.
     *
     * Needs RT_JOINT with the MIT profile and fresh feedback from all seven arm
     * drives. Repeats `initial` at `options.transmit_hz`; later servej() calls
     * replace it. A producer timeout, stale feedback or a controller fault
     * requests a controlled stop. stop(), servo_off() of the arm, a mode change
     * and disconnect() end it. Mode, setup and enable sequencing stay the
     * caller's job.
     * @pre RT_JOINT with the MIT profile, fresh feedback from all seven arm drives.
     * @see stop_joint_stream(), joint_stream_stats(), servej()
     * @param initial The first command.
     * @param options Rate and timeouts.
     * @param[out] error Why it failed.
     */
    bool start_joint_stream(const JointCommand& initial, const JointStreamOptions& options,
                            std::string& error);
    /**
     * @brief [Non-blocking] Ends the native producer; servej() sends single packets again.
     *
     * Sends nothing: in RT_JOINT the controller stops the arm 100 ms later
     * unless servej() keeps coming. Use stop() to stop the arm at once.
     */
    void stop_joint_stream();
    /**
     * @brief [Non-blocking] State of the native producer.
     *
     * Check it in the control loop: `active` false with a `fault` means the
     * producer stopped and asked for a controlled stop. Read the fault, then
     * start it again explicitly.
     */
    JointStreamStats joint_stream_stats() const;
    ///@}

    /**
     * @name NRT modes (NRT_JOINT, NRT_CARTESIAN)
     * The controller plans and runs one move at a time; the PC only names the goal.
     */
    ///@{
    /**
     * @brief [Blocking] Moves J1–J7 to `position` in NRT_JOINT.
     *
     * The simplest way to move the arm: one call per goal. The controller plans
     * a joint-space move within the speed and acceleration caps, and the call
     * returns when the arm has arrived or the move failed. Call it after
     * `switch_mode(Mode::NrtJoint)`. A move ends where it arrived: the next one
     * starts there.
     *
     * Waits for the session to be ready, then until arrival, failure or
     * `options.timeout_ms` + 3 s.
     * @pre NRT_JOINT with the arm enabled and still.
     * @see submit_movej(), motion(), wait_motion(), cancel_motion()
     * @param position Targets, rad.
     * @param options Speed, tolerance and gains.
     * @param[out] error Why it failed.
     * @return The outcome; MotionResult::result is 0 on arrival.
     */
    MotionResult movej(const std::array<float, 7>& position, const MoveOptions& options,
                       std::string& error);
    /**
     * @brief [Blocking] Moves the tool to `pose` along a straight line in NRT_CARTESIAN.
     *
     * Like movej() but in Cartesian space: the tool follows a straight line
     * within the Cartesian limits. Call it after `switch_mode(Mode::NrtCartesian)`.
     * forward() gives the current pose to start from.
     * @pre NRT_CARTESIAN with the arm enabled and still.
     * @see movej(), submit_movep(), forward()
     * @param pose Target tool pose.
     * @param options Limits, tolerance and gains.
     * @param[out] error Why it failed.
     * @return The outcome; MotionResult::result is 0 on arrival.
     */
    MotionResult movep(const Pose& pose, const CartesianMoveOptions& options, std::string& error);
    /**
     * @brief [Non-blocking] Starts a joint move and returns.
     *
     * Use it instead of movej() when the program must keep working during the
     * move (read state, update a display, decide to cancel). Follow it with
     * motion() or wait_motion(). A move does not replace another: the previous
     * one must be done.
     * @param position Targets, rad.
     * @param options Speed, tolerance and gains.
     * @param[out] error Why it was refused.
     */
    bool submit_movej(const std::array<float, 7>& position, const MoveOptions& options, std::string& error);
    /**
     * @brief [Non-blocking] Starts a Cartesian move and returns. See submit_movej().
     * @param pose Target tool pose.
     * @param options Limits, tolerance and gains.
     * @param[out] error Why it was refused.
     */
    bool submit_movep(const Pose& pose, const CartesianMoveOptions& options, std::string& error);
    /**
     * @brief [Non-blocking] State of the movej/movep session.
     *
     * Poll it after submit_movej() or submit_movep(): `phase` shows whether the
     * move is running, settling, done or failed, and `can_execute` whether a
     * new move is accepted now.
     */
    MotionStatus motion() const;
    /**
     * @brief [Blocking, timeout] Waits until the newest move has arrived or failed.
     *
     * The second half of movej(), for a move started with submit_movej() or
     * submit_movep().
     * @param timeout_ms How long to wait.
     * @param[out] result The outcome.
     * @param[out] error Why it failed.
     * @return true on arrival.
     */
    bool wait_motion(uint32_t timeout_ms, MotionResult& result, std::string& error);
    /**
     * @brief [Non-blocking] Ends the session's move with a controlled stop.
     *
     * The arm brakes and holds in IDLE, as after stop(). Use it to abort a move
     * started with submit_movej() or submit_movep().
     * @param[out] error Why it failed.
     */
    bool cancel_motion(std::string& error);
    /**
     * @brief [Non-blocking] Options of the hold move that opens an NRT_JOINT session.
     *
     * `switch_mode(Mode::NrtJoint)` first holds the current pose with a move of
     * its own; these are its gains and tolerances. Set them before the switch,
     * e.g. to hold with the same gains as the moves that follow.
     */
    void set_default_move_options(const MoveOptions& options);
    /// [Non-blocking] Options of the hold move that opens an NRT_CARTESIAN session. See set_default_move_options().
    void set_default_cartesian_options(const CartesianMoveOptions& options);
    ///@}

    /** @name Gripper (J8) */
    ///@{
    /**
     * @brief [Blocking, 1 s] Moves the gripper, in any control mode (also during movej/movep).
     *
     * The gripper moves to the position at the speed limit and stops pushing at
     * the force limit, so closing on an object grips it with that force. The
     * call returns once the controller has accepted the target, not when the
     * gripper arrives; watch RobotState::joints for that. Needs the gripper
     * group enabled (`servo_on(MotorGroup::Gripper)`).
     * @pre The gripper group is enabled: servo_on(MotorGroup::Gripper).
     * @param position_rad Target, 0–5.94 rad.
     * @param speed_rad_s Speed limit, rad/s.
     * @param force_ratio Current limit as a fraction of the drive's maximum phase
     *        current, 0–1: stalled on an object the gripper pushes with at most that.
     * @param[out] error Why it failed.
     */
    bool gripper(float position_rad, float speed_rad_s, float force_ratio, std::string& error);
    ///@}

    /**
     * @name Recording and logs
     * Three sources: the live session (start_recording()), the controller's
     * UART (start_serial_log()) and the board's own memory (download_mcu_log()).
     */
    ///@{
    /**
     * @brief [Blocking, file setup] Records telemetry, commands and events as CSV.
     *
     * Use it to analyse a run afterwards: every telemetry packet (1 kHz), every
     * command sent and the events (faults, stops, replies, markers) go under
     * `directory/<time>[-label]/`. A background thread writes; nothing in the
     * 1 kHz paths touches the disk. Recording runs until stop_recording(),
     * across mode changes.
     * @note A background thread writes; nothing in the 1 kHz paths touches the disk.
     * @param directory Parent directory.
     * @param[out] error Why it failed.
     * @param options Label, file rotation and command logging.
     */
    bool start_recording(const std::string& directory, std::string& error,
                         const RecordOptions& options = RecordOptions{});
    void stop_recording();  ///< [Blocking] Finishes the writer and closes the files.
    /**
     * @brief [Non-blocking] Progress of the recording.
     *
     * `dropped` above 0 means the disk could not keep up and rows were lost.
     */
    RecorderStats recording() const;
    /**
     * @brief [Non-blocking] Adds a marker to events.csv.
     *
     * Mark the moments that matter ("grasp", "trial 3 start") to find them in
     * the telemetry later.
     */
    void record_event(const std::string& text);

    /**
     * @brief [Blocking, port/file setup] Saves the controller's UART debug output.
     *
     * Writes `<directory>/serial_YYYYMMDDHHMMSS.log` (the call time), one line per
     * controller line: `<host_time_s> <YYYY-MM-DD HH:MM:SS> <text>`. host_time_s
     * is CLOCK_MONOTONIC, the clock of start_recording()'s host_time_s. The port
     * is only read, independently of connect() and the UDP threads. Fails if
     * another program has the port open. A running log is stopped first; an
     * unplug or reset reopens the port.
     * @param[out] error Why it failed.
     * @param options Port, baud rate and directory.
     */
    bool start_serial_log(std::string& error, const SerialLogOptions& options = SerialLogOptions{});
    void stop_serial_log();  ///< [Blocking] Joins the serial worker.
    SerialLogStats serial_log() const;  ///< [Non-blocking] Progress of the serial log.
    /**
     * @brief [Non-blocking] Default SDK log directory.
     *
     * $INFRABOT_LOG_DIR, else $XDG_STATE_HOME/infrabot/log, else
     * ~/.local/state/infrabot/log. Created on first use by start_serial_log().
     */
    static std::string log_directory();
    /**
     * @brief [Blocking, a few s] Downloads the controller's run log into two new files.
     *
     * Use it after a fault, before the controller is reset or powered off: the
     * board keeps the text it printed (newest 10 minutes) and its 1 kHz
     * telemetry (newest 10 s) in memory, frozen at a fault.
     *
     * Never overwrites. Needs no connect(): it uses its own socket on
     * INFRABOT_LOG_PORT. The board refuses while an RT stream runs. Telemetry is
     * frozen during the download, so the CSV holds the 10 s before the call.
     * Never call it from the 1 kHz loop.
     * @param[out] result The files written.
     * @param[out] error Why it failed.
     * @param options Directory, label, port and retries.
     */
    bool download_mcu_log(McuLogResult& result, std::string& error,
                          const McuLogOptions& options = McuLogOptions{});
    ///@}

    /** @name Kinematics */
    ///@{
    /**
     * @brief [Non-blocking] Forward kinematics of the H7C arm, computed locally.
     *
     * Turns joint angles into the tool pose, the same model the controller
     * uses. Use it to get the current pose for movep() or servep() from
     * hold_command() or `state().joints.position`. Needs no connection.
     * @param joints J1–J7, rad.
     * @return The tool pose.
     */
    static Pose forward(const std::array<float, 7>& joints);
    ///@}

    /**
     * @name Single-drive maintenance
     * For commissioning, in IDLE, mostly with the drives disabled. Values are in
     * rad, rad/s, N·m and ratios 0–1. Each call waits for the controller's
     * acknowledgement: [Blocking, 1.5 s]; never call them from the 1 kHz loop.
     * `motor` is 0–7 or kAllMotors where noted. Normal programs use the group
     * calls (servo_on(), stop(), clear_fault()) instead.
     */
    ///@{
    bool enable(uint8_t motor, std::string& error);    ///< Enables one drive directly, without the group sequence.
    bool disable(uint8_t motor, std::string& error);   ///< Disables one drive: it produces no torque.
    bool stop(uint8_t motor, std::string& error);      ///< Holds one drive at its current angle.
    bool clear_error(uint8_t motor, std::string& error);  ///< Clears one drive's fault; the drive must be disabled.
    bool set_zero_position(uint8_t motor, std::string& error);  ///< Stores one drive's zero at its current angle.
    /// Switches one drive to an INFRABOT_DRIVE_MODE_* (1–4) if it is not already in it.
    bool ensure_mode(uint8_t motor, uint8_t mode, std::string& error);
    /// Sends one MIT frame to a drive, e.g. to test a single joint.
    bool send_mit(uint8_t motor, float position_rad, float velocity_rad_s,
                  float kp, float kd, float torque_nm, std::string& error);
    /// Sends one position-velocity frame to a drive.
    bool send_pos_vel(uint8_t motor, float position_rad, float velocity_limit_rad_s,
                      std::string& error);
    /// Sends one velocity frame to a drive.
    bool send_vel(uint8_t motor, float velocity_rad_s, std::string& error);
    /// Sends one force-position frame to a drive.
    bool send_force_pos(uint8_t motor, float position_rad, float velocity_limit_rad_s,
                        float torque_ratio, std::string& error);
    bool request_feedback(uint8_t motor, std::string& error);  ///< Asks a drive for a feedback frame.
    bool get_state(uint8_t motor, std::string& error);  ///< Checks feedback freshness; the state comes in the next telemetry.
    bool set_can_timeout_ms(uint8_t motor, uint32_t timeout_ms, std::string& error);  ///< Sets a drive's CAN timeout.
    bool store_parameters(uint8_t motor, std::string& error);  ///< Saves a drive's registers to its flash.
    /**
     * @brief Reads a drive register.
     *
     * register_value_text() and register_summary() turn the raw bits into text.
     * @param motor Drive 0–7.
     * @param register_id INFRABOT_RID_*.
     * @param value_type INFRABOT_REGISTER_U32 or _F32; must match the register.
     * @param[out] value Raw register bits.
     * @param[out] error Why it failed.
     */
    bool get_register(uint8_t motor, uint8_t register_id, uint8_t value_type,
                      uint32_t& value, std::string& error);
    /**
     * @brief Writes a drive register within its allowed range and reads it back.
     *
     * The value lives in the drive's RAM until store_parameters().
     * @param motor Drive 0–7.
     * @param register_id INFRABOT_RID_*.
     * @param value Raw register bits.
     * @param value_type INFRABOT_REGISTER_U32 or _F32; must match the register.
     * @param[out] error Why it failed.
     */
    bool write_register(uint8_t motor, uint8_t register_id, uint32_t value,
                        uint8_t value_type, std::string& error);
    ///@}

    /**
     * @name Protocol level
     * For tools such as udp_control_gui. Wire units (mrad, mrad/s, ×1000, mN·m),
     * one datagram each, thread-safe. The caller owns timing and the controller's
     * state rules. Applications use the functions above instead.
     */
    ///@{
    /// [RT] Sends one DMCP v3 command (INFRABOT_CMD_*).
    bool send_command(uint8_t command, const std::array<int32_t, 8>& target_mrad,
                      std::string& error);
    /// [RT] Sends one DMCP v5 joint stream packet.
    bool send_joint_stream(const std::array<int32_t, 8>& target_mrad, const StreamProfile& profile,
                           std::string& error);
    /// [RT] Sends one DMCP v7 Cartesian stream packet; `limits` in wire units.
    bool send_cartesian_stream(const Pose& pose, const StreamProfile& profile,
                               const std::array<uint16_t, 4>& limits, std::string& error);
    /**
     * @brief [Non-blocking] Sends one DMCP v4 maintenance operation.
     *
     * Refused while another is unanswered (1.5 s), except DISABLE.
     * @param request The operation.
     * @param[out] sequence Identifies its DMAR reply.
     * @param[out] error Why it was refused.
     */
    bool send_maintenance(const MaintenanceRequest& request, uint32_t& sequence, std::string& error);
    /// [Blocking, timeout] Waits for the DMAR reply of `sequence`.
    bool wait_maintenance(uint32_t sequence, InfrabotMaintenanceReply& reply, uint32_t timeout_ms,
                          std::string& error);
    /// [Blocking] send_maintenance() followed by wait_maintenance().
    bool maintenance(const MaintenanceRequest& request, InfrabotMaintenanceReply& reply,
                     uint32_t timeout_ms, std::string& error);
    bool maintenance_pending() const;  ///< [Non-blocking] A maintenance operation is unanswered.
    /// [Non-blocking] The newest DMAR reply to any request; false before the first.
    bool last_maintenance_reply(InfrabotMaintenanceReply& reply) const;
    /**
     * @brief [Blocking, 3 s] Stores the runtime control profile in the controller's flash.
     *
     * DMCP v5 STORE_CONTROL_PROFILE; every drive must be disabled.
     * @param profile The profile.
     * @param[out] status The controller's DMAR status.
     * @param[out] error Why it failed.
     */
    bool store_control_profile(const StreamProfile& profile, uint8_t& status, std::string& error);
    ///@}

    /**
     * @name Controller parameters
     * [Blocking, about 10 ms per parameter] Firmware with INFRABOT_CAP_PARAMETERS
     * (protocol 1.1): the gravity, impedance and admittance settings that the
     * touchscreen sliders change. Every parameter describes itself (name, unit,
     * range, default, slew rate), so no table is compiled into the SDK.
     * set_parameter() stores the requested value and the controller slews its
     * applied value toward it, so a write never steps a gain. Values live in RAM
     * and return to their defaults at reboot. Reading is open to any PC; writing
     * needs the PC that controls the arm, or no controlling PC.
     */
    ///@{
    /**
     * @brief Lists every parameter with its value, range and unit.
     *
     * Start here to see what can be tuned on this firmware. Also caches the
     * name → id table used by the name overloads.
     */
    bool list_parameters(std::vector<ParameterInfo>& parameters, std::string& error);
    /// Reads one parameter by id (group << 8 | index).
    bool get_parameter(uint16_t id, ParameterInfo& parameter, std::string& error);
    /// Reads one parameter by name, e.g. "imp.joint_k".
    bool get_parameter(const std::string& name, ParameterInfo& parameter, std::string& error);
    /**
     * @brief Requests a new value by id; `result` shows the state after the write.
     *
     * Changes the behaviour of the local modes at once, also while one runs:
     * the applied value moves to the requested one at the parameter's rate.
     * Write the same value again after a reboot.
     * @note RAM only: the value returns to its default at reboot.
     * @see list_parameters(), get_parameter()
     */
    bool set_parameter(uint16_t id, float value, ParameterInfo& result, std::string& error);
    /// Requests a new value by name; `result` shows the state after the write.
    bool set_parameter(const std::string& name, float value, ParameterInfo& result, std::string& error);
    ///@}

    /** @name Diagnostics */
    ///@{
    /**
     * @brief [Blocking, up to 600 ms] One diagnostic snapshot of the controller.
     *
     * Use it when something misbehaves without a clear fault: missed control
     * ticks, CAN errors, rejected commands, low memory. Firmware with
     * INFRABOT_CAP_DIAGNOSTICS: loop timing, CAN and command counters, heap and
     * least free stacks, drive states. Read only and open to any PC; it never
     * takes control of the arm.
     * @note Read only, open to any PC; it never takes control of the arm.
     * @param[out] diagnostics The snapshot.
     * @param[out] error Why it failed.
     */
    bool diagnostics(DiagnosticsInfo& diagnostics, std::string& error);
    ///@}

private:
    /* switch_mode(); reply_status: the MCU's DMAR status, or
     * INFRABOT_REPLY_BUSY when the arm is not still yet (try again). */
    bool enter_mode(Mode mode, std::string& error, uint8_t* reply_status);
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

/// [Non-blocking] Name of a mode for display, e.g. "RT_JOINT".
const char* mode_name(Mode mode);
/// [Non-blocking] Name of an operational status for display.
const char* status_name(OperationalStatus status);

} // namespace infrabot
