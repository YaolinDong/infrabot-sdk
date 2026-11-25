/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#pragma once

/* infrabot SDK: the PC side of the arm controller's UDP protocol
 * (infrabot/protocol.h).  One Robot per controller.
 *
 * Every function is tagged with how it behaves:
 *   [Blocking]      waits for the MCU (a reply or the telemetry), at most the
 *                   time given; never call it from a 1 kHz loop.
 *   [Non-blocking]  returns at once (sends one datagram or copies state).
 *   [RT]            non-blocking and allocation-free on success: for a 1 kHz loop.
 * and with the control modes it applies to.  A function that fails returns
 * false (or a result code) and says why in `error`.
 *
 * Lifecycle (like Flexiv RDK): connect -> set_motor_profile (disabled, IDLE;
 * runs the controller's parameter self-check) -> servo_on -> switch_mode ->
 * commands -> stop / switch_mode(Idle) -> servo_off.  While connected the SDK
 * keeps the controller alive in IDLE and NRT (a 100 Hz heartbeat and the
 * movej/movep session); in RT modes servej/servep must keep coming (>= 10 Hz,
 * normally 1 kHz), or explicitly opt into start_joint_stream() and refresh
 * servej() within its producer timeout. A controller whose PC goes quiet (program closed, cable
 * pulled) stops the arm in a controlled way and holds it enabled; it never
 * disables it by itself (firmware with INFRABOT_CAP_CONTROLLED_STOP; older
 * firmware disables). */

#include "infrabot/protocol.h"

#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace infrabot {

enum class Mode : uint8_t {
    Idle = INFRABOT_CONTROL_IDLE,
    RtJoint = INFRABOT_CONTROL_RT_JOINT,
    RtCartesian = INFRABOT_CONTROL_RT_CARTESIAN,
    NrtJoint = INFRABOT_CONTROL_NRT_JOINT,
    NrtCartesian = INFRABOT_CONTROL_NRT_CARTESIAN,
    GravityFree = INFRABOT_CONTROL_GRAVITY_FREE,
    GravityLock = INFRABOT_CONTROL_GRAVITY_LOCK,
    JointImpedance=INFRABOT_CONTROL_JOINT_IMPEDANCE, CartesianImpedance=INFRABOT_CONTROL_CARTESIAN_IMPEDANCE,
    JointAdmittance=INFRABOT_CONTROL_JOINT_ADMITTANCE, CartesianAdmittance=INFRABOT_CONTROL_CARTESIAN_ADMITTANCE,
    Unknown = 0xff,
};

/* Drive mode of J1-J7 (J8 is always force-position). */
enum class MotorProfile : uint8_t {
    Mit = INFRABOT_DRIVE_MODE_MIT,
    PositionVelocity = INFRABOT_DRIVE_MODE_POS_VEL,
};

/* Drive groups enabled and disabled together (values: the enabled-mask
 * bits).  Every control mode needs the arm only; the gripper may be disabled
 * or absent (another tool, tool/payload/dynamics identification). */
enum class MotorGroup : uint8_t {
    All = INFRABOT_ALL_MASK,
    Arm = INFRABOT_ARM_MASK,
    Gripper = INFRABOT_GRIPPER_MASK,
};

enum class OperationalStatus : uint8_t {
    Booting = INFRABOT_STATUS_BOOTING, Precheck, ProfileRequired, Disabled, Enabling, IdleHold,
    RtJoint, RtCartesian, NrtJoint, NrtCartesian, Stopping, Fault, Recovery, Estop,
    Unknown = INFRABOT_STATUS_UNKNOWN, GravityFree = INFRABOT_STATUS_GRAVITY_FREE,
    GravityLock = INFRABOT_STATUS_GRAVITY_LOCK,
    JointImpedance=INFRABOT_STATUS_JOINT_IMPEDANCE,CartesianImpedance=INFRABOT_STATUS_CARTESIAN_IMPEDANCE,
    JointAdmittance=INFRABOT_STATUS_JOINT_ADMITTANCE,CartesianAdmittance=INFRABOT_STATUS_CARTESIAN_ADMITTANCE
};

enum class StopReason : uint8_t {
    None = INFRABOT_STOP_NONE, StreamLost, MoveLost, MoveFailed, Requested, NotFollowing,
    InvalidTarget, FeedbackLost, DriveFault, TorqueLimit
};
enum class StopPhase : uint8_t { None = INFRABOT_STOP_PHASE_NONE, Braking, Holding };
enum class SelfCheck : uint8_t { NotRun = INFRABOT_SELFCHECK_NOT_RUN, Ok, Mismatch, NoReply };

struct RobotInfo {
    std::string product = "infrabot-damiao";
    std::string firmware;
    uint16_t protocol_version = 0;  // DMIO envelope version, once connected
    uint16_t protocol_major = 0, protocol_minor = 0;
    std::string board, arm_model;
    uint8_t axes = INFRABOT_ARM_AXES;
    uint8_t motor_count = INFRABOT_MOTOR_COUNT;
    bool received = false;     // a DMIO answer arrived (firmware with DMIF)
    uint32_t capabilities = 0; // INFRABOT_CAP_*
    uint8_t controller_status = static_cast<uint8_t>(OperationalStatus::Unknown);
    uint8_t control_mode = static_cast<uint8_t>(Mode::Unknown);
    uint8_t motor_profile = static_cast<uint8_t>(MotorProfile::Mit);
    bool profile_ready = false;
    uint8_t enabled_mask = 0;
    std::array<float, 8> position_min{};
    std::array<float, 8> position_max{};
    std::array<float, 8> velocity_max{};
    std::array<float, 8> torque_max{};
};

/* Measured state of the eight drives (joint frame, SI units), and with
 * telemetry v3 what the MCU commands them. */
struct JointState {
    std::array<float, 8> position{};          // rad
    std::array<float, 8> velocity{};          // rad/s
    std::array<float, 8> torque{};            // N·m
    std::array<uint8_t, 8> drive_state{};     // INFRABOT_DRIVE_*
    std::array<uint8_t, 8> temperature{};     // MOS, °C
    std::array<uint16_t, 8> feedback_count{};
    bool has_command = false;                 // the fields below (telemetry v3)
    std::array<float, 8> command_position{};  // rad, the planned command
    std::array<float, 8> command_velocity{};  // rad/s, MIT feed-forward / v_des
    std::array<float, 8> command_torque{};    // N·m, MIT feed-forward
    std::array<uint8_t, 8> temperature_coil{};// °C
    std::array<uint8_t, 8> motor_flags{};     // INFRABOT_MOTOR_*
    std::array<uint8_t, 8> drive_mode{};      // INFRABOT_DRIVE_MODE_*
    uint32_t sequence = 0;                    // telemetry sequence
    uint64_t timestamp_us = 0;                // MCU clock
    uint64_t host_time_us = 0;                // CLOCK_MONOTONIC at receipt, not state() time
    std::array<uint32_t, 8> feedback_age_ms{}; // per-drive counter age; UINT32_MAX before feedback
};

/* Controlled stop and parameter self-check (telemetry v3). */
struct SafetyState {
    bool valid = false;
    StopReason stop_reason = StopReason::None;
    StopPhase stop_phase = StopPhase::None;
    uint8_t stop_joint = INFRABOT_NO_JOINT; // 0-based
    uint8_t stop_detail = 0;
    Mode stop_mode = Mode::Idle;            // the mode the stop interrupted (resume() re-enters it)
    SelfCheck selfcheck = SelfCheck::NotRun;
    uint8_t selfcheck_joint = INFRABOT_NO_JOINT;
    uint8_t selfcheck_register = 0;
    uint32_t selfcheck_value = 0;
    uint32_t selfcheck_expected = 0;
};

/* Quality of the UDP link.  "last second" values are refreshed once a
 * second by the receive thread. */
struct LinkStats {
    uint64_t tx_sent = 0, tx_errors = 0;          // datagrams sent / failed
    uint64_t rx_telemetry = 0;                    // telemetry packets received
    uint64_t rx_lost = 0;                         // telemetry sequence gaps
    uint64_t rx_invalid = 0;                      // datagrams not understood
    uint32_t rx_last_second = 0;                  // telemetry packets in the last second
    uint32_t lost_last_second = 0;
    float loss_percent = 0.0f;                    // last second
    uint32_t rx_interval_max_us = 0;              // longest gap between telemetry packets, last second
    bool mcu_valid = false;                       // telemetry v3: the MCU's view below
    InfrabotLinkStats mcu{};
};

struct RobotState {
    OperationalStatus operational_status = OperationalStatus::Unknown;
    Mode mode = Mode::Unknown;
    MotorProfile profile = MotorProfile::Mit;
    bool connected = false;          // socket open; see telemetry_age_ms for liveness
    bool modes_supported = false;    // firmware with control modes
    bool profile_ready = false;
    bool enabled = false;            // any drive enabled
    bool zero_valid = false;
    uint8_t enabled_mask = 0;
    uint8_t controller_state = 0;    // raw INFRABOT_STATUS_*
    uint8_t telemetry_version = 0;   // 2 or 3
    bool command_active = false;
    uint32_t command_applied_sequence = 0;
    uint32_t telemetry_age_ms = UINT32_MAX;
    uint32_t telemetry_epoch = 0; // increments when telemetry resynchronizes after a stale/restarted source
    JointState joints{};
    SafetyState safety{};
};

/* A drive fault, a controlled stop or a self-check result, as it happened. */
struct FaultEvent {
    enum class Kind : uint8_t { DriveFault, DriveCleared, Stop, SelfCheck, Link };
    Kind kind = Kind::DriveFault;
    uint64_t host_time_us = 0;      // steady clock
    uint64_t mcu_time_us = 0;
    uint8_t joint = INFRABOT_NO_JOINT; // 0-based
    uint8_t code = 0;               // INFRABOT_DRIVE_* / INFRABOT_STOP_* / INFRABOT_SELFCHECK_*
    uint8_t detail = 0;
    std::string message;            // Chinese: what, probable cause, what to do
};

/* RT_JOINT command for J1-J7.  MIT profile: the drives track position and
 * velocity with kp/kd plus the torque feed-forward.  Position-velocity
 * profile: the drives move to position at no more than velocity_limit (their
 * v_des; 0 would stop them); velocity, kp, kd and torque are not used. */
struct JointCommand {
    std::array<float, 7> position{};       // rad
    std::array<float, 7> velocity{};       // rad/s, MIT feed-forward
    std::array<float, 7> torque{};         // N·m, MIT feed-forward
    std::array<float, 7> kp{};             // MIT, 0-500
    std::array<float, 7> kd{};             // MIT, 0-5
    std::array<float, 7> velocity_limit{}; // rad/s, position-velocity v_des
    JointCommand();                        // template gains, 3 rad/s
};

/* Cartesian speed limits; 0 = the controller's default (its maximum). */
struct CartesianLimits {
    float linear_velocity = 0.0f;      // m/s, at most 0.5
    float angular_velocity = 0.0f;     // rad/s, at most pi
    float linear_acceleration = 0.0f;  // m/s^2, at most 2
    float angular_acceleration = 0.0f; // rad/s^2, at most 2 pi
};

/* Tool pose in the base frame: x, y, z (m), then the row-major 3x3 rotation. */
using Pose = std::array<float, 12>;

/* RT_CARTESIAN command: the MCU moves the tool to pose along a straight
 * line within limits (its own IK at 1 kHz).  Per joint the output gains
 * (MIT kp/kd plus torque feed-forward, position-velocity v_des) as above. */
struct CartesianCommand {
    Pose pose{};
    CartesianLimits limits{};
    std::array<float, 7> torque{};
    std::array<float, 7> kp{};
    std::array<float, 7> kd{};
    std::array<float, 7> velocity_limit{};
    CartesianCommand();
};

struct MoveOptions {
    uint32_t timeout_ms = 30000;
    uint32_t dwell_ms = 100;
    float position_tolerance = 0.01f;  // rad
    float velocity_tolerance = 0.05f;  // rad/s
    std::array<float, 7> velocity_max{};      // rad/s (position-velocity: also v_des)
    std::array<float, 7> acceleration_max{};  // rad/s^2
    std::array<float, 7> kp{};
    std::array<float, 7> kd{};
    std::array<float, 7> torque{};            // N·m, MIT feed-forward
    MoveOptions();
};

struct CartesianMoveOptions {
    uint32_t timeout_ms = 30000;
    uint32_t dwell_ms = 100;
    float position_tolerance = 0.01f;  // rad, per joint at the end
    float velocity_tolerance = 0.05f;  // rad/s
    CartesianLimits limits{};
    /* Per-joint caps; 0 = application maximum 3.14 (MIT) / the default v_des (position-velocity,
     * where velocity_max is the drives' v_des). */
    std::array<float, 7> velocity_max{};
    std::array<float, 7> acceleration_max{}; // rad/s^2, 0 = 100
    std::array<float, 7> kp{};
    std::array<float, 7> kd{};
    CartesianMoveOptions();
};

struct MotionResult {
    uint8_t result = 0;     // INFRABOT_MOVE_*: 0 = arrived
    uint8_t state = 0;      // INFRABOT_MOVE_STATE_*
    uint32_t motion_id = 0;
    uint32_t arrived_mask = 0;
    uint32_t fresh_mask = 0;
    float max_error_rad = 0;
    uint32_t reason = 0;
    std::string message;
};

/* The movej/movep session of the NRT modes (motion_session.hpp phases). */
struct MotionStatus {
    enum class Phase : uint8_t { Off, Preparing, Waiting, Running, Settling, Ready, Done, Cancelling, Cancelled, Error };
    Phase phase = Phase::Off;
    uint32_t id = 0;              // the newest move (1 = the hold on entering the mode)
    bool can_execute = false;     // a new move is accepted now
    bool has_reply = false;
    InfrabotMoveReply reply{};    // the newest DMOS for it
    uint64_t reply_age_us = UINT64_MAX;
    std::string error;
};

/* Per-joint stream values in wire units (DMCP v5/v7). */
struct StreamProfile {
    std::array<int32_t, 8> velocity_mrad_s{};
    std::array<int32_t, 8> kp_milli{};
    std::array<int32_t, 8> kd_milli{};
    std::array<int32_t, 8> torque_mNm{};
    std::array<int32_t, 8> velocity_limit_mrad_s{};
    std::array<int32_t, 8> torque_ratio_milli{};
};

/* One DMCP v4 maintenance operation (wire units). */
struct MaintenanceRequest {
    uint8_t command = 0;                 // INFRABOT_CMD_*
    uint8_t motor = INFRABOT_MOTOR_ALL;  // 0-7 or INFRABOT_MOTOR_ALL
    uint8_t mode = 0;
    uint8_t value_type = 0;
    uint8_t register_id = 0;
    uint32_t value = 0;
    int32_t position_mrad = 0;           // target_position_mrad[motor]
    int32_t velocity_mrad_s = 0;
    int32_t kp_milli = 0;
    int32_t kd_milli = 0;
    int32_t torque_mNm = 0;
    int32_t velocity_limit_mrad_s = 0;
    int32_t torque_ratio_milli = 0;
};

struct RecordOptions {
    std::string label;             // appended to the directory name
    uint32_t rotate_seconds = 600; // a new telemetry/commands file every so often (0 = never)
    bool commands = true;          // also log every command datagram sent
};

struct RecorderStats {
    bool active = false;
    std::string directory;
    uint64_t telemetry_rows = 0, command_rows = 0, events = 0;
    uint64_t dropped = 0;          // rows lost because the disk could not keep up
    uint32_t file_index = 0;
    std::string error;             // last write error
};

/* MCU UART debug output (STAT/MOT/GRAV/ADM/... lines) saved on the PC. */
struct SerialLogOptions {
    std::string port = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0";
    uint32_t baud = 115200;
    std::string directory;         // empty = Robot::log_directory()
};

struct SerialLogStats {
    bool active = false;
    std::string path;              // file being written
    uint64_t lines = 0, bytes = 0;
    uint32_t reconnects = 0;       // port lost (unplug/reset) and reopened
    bool port_open = false;
    std::string error;             // last port/write error
};

/* The board's own run log (firmware with INFRABOT_CAP_RUN_LOG), kept in its
 * SDRAM: the text it printed (newest 10 minutes) and its 1 kHz telemetry
 * (newest 10 s).  Lost on power-off/reset. */
struct McuLogOptions {
    std::string directory;         // empty = Robot::log_directory()
    std::string label;             // appended to the file names
    uint16_t port = INFRABOT_LOG_PORT;
    uint32_t timeout_ms = 300;     // per request
    uint32_t retries = 3;
    /* After each reply: text lines / telemetry rows received so far and the
     * totals the board reported.  Return false to cancel. */
    std::function<bool(uint32_t text_lines, uint32_t text_total, uint32_t telemetry_rows,
                       uint32_t telemetry_total)> progress;
};

struct McuLogResult {
    std::string text_path;         // mcu_YYYYMMDDHHMMSS[_label].log: "<mcu_time_s> <line>"
    std::string telemetry_path;    // mcu_..._telemetry.csv: start_recording()'s telemetry columns, host_time_s = 0
    uint32_t boot_id = 0;          // changes on every MCU boot
    uint32_t text_lines = 0, telemetry_rows = 0;
    uint32_t lost_bytes = 0;       // printf bytes the board could not keep (log task starved)
    uint32_t writer_max_us = 0;    // longest log-task pass since the previous download
    bool fault_latched = false;    // the board froze the telemetry at a fault: the CSV ends at that tick
};

/* One tunable controller parameter (firmware with INFRABOT_CAP_PARAMETERS, protocol 1.1). */
struct ParameterInfo {
    uint16_t id = 0;        // group << 8 | index
    uint8_t group = 0;      // INFRABOT_PARAM_GROUP_*
    std::string name;       // e.g. "imp.joint_k"
    std::string unit;       // e.g. "Nm/rad"; empty when dimensionless
    float requested = 0.f;  // the last value written, or the default after a reboot
    float applied = 0.f;    // the value the controller uses now, slewing toward requested
    float minimum = 0.f, maximum = 0.f, default_value = 0.f;
    float rate = 0.f;       // largest change of the applied value, units per second
    bool writable = false;
    bool boolean = false;   // the value is 0 or 1
};

/* One diagnostic snapshot (firmware with INFRABOT_CAP_DIAGNOSTICS, DMDR/DMDA). The "window" counters are
 * the status task's last one-second window, the "total" ones count since boot. */
struct DiagnosticsInfo {
    uint32_t uptime_ms = 0;              // since the application started; wraps after 49 days
    uint32_t snapshot_seconds = 0;       // status-task second of the window
    uint32_t control_hz = 0;             // 1 kHz control ticks completed in the window
    uint32_t control_missed = 0;
    uint32_t control_period_max_us = 0;
    uint32_t can_tx = 0, can_rx = 0, can_tx_errors = 0;
    uint32_t command_rx = 0, command_invalid = 0, telemetry_tx = 0;
    uint32_t can_error_events = 0;       // since boot
    uint32_t control_missed_total = 0, can_tx_errors_total = 0, command_invalid_total = 0;
    uint8_t can_tec = 0, can_rec = 0;    // CAN error counters at the end of the window
    bool can_error_passive = false, can_bus_off = false;
    uint32_t heap_free_bytes = 0, heap_min_free_bytes = 0;  // FreeRTOS heap, now and lowest since boot
    uint32_t stack_free_control_bytes = 0, stack_free_status_bytes = 0, stack_free_ethernet_bytes = 0;  // least ever
    std::array<uint8_t, 8> drive_state{};  // INFRABOT_DRIVE_* of J1..J8, now
    uint8_t controller_status = 0;         // INFRABOT_STATUS_*, now
    uint8_t invalid_reason = 0;            // INFRABOT_INVALID_* of the last rejected command datagram
};

/* Optional native RT Joint/MIT producer, serviced by Robot's existing C++
 * thread. Starting it never configures/enables drives or changes modes. */
struct JointStreamOptions {
    uint32_t transmit_hz = 1000;
    uint32_t command_timeout_ms = 100;
    uint32_t feedback_timeout_ms = 100;
    bool allow_zero_kp = false; // current MCU STOP does not restore holding gains
};
struct JointStreamStats {
    bool active = false;
    uint64_t sent = 0, skipped_deadlines = 0;
    std::string fault; // latched; a failed stream requires an explicit start
};

class Robot {
public:
    static constexpr uint8_t kAllMotors = INFRABOT_MOTOR_ALL;
    /* [Non-blocking] Construct the local client; connect opens the sockets. */
    Robot(std::string address = "192.168.0.50", uint16_t command_port = INFRABOT_COMMAND_PORT,
          uint16_t telemetry_port = INFRABOT_TELEMETRY_PORT);
    ~Robot();  // [Blocking] disconnects and joins workers
    Robot(const Robot&) = delete;
    Robot& operator=(const Robot&) = delete;

    /* ---- Connection ----------------------------------------------------- */
    /* [Blocking, up to 0.5 s for discovery] Negotiate the protocol before
     * starting the heartbeat. V1-only controllers are treated as 1.0 with
     * unknown firmware. A major mismatch or missing valid reply fails.
     * Check telemetry_age_ms separately for live drive feedback. */
    bool connect(std::string& error);
    /* [Blocking] Closes the sockets and sends nothing: an enabled arm notices the PC is
     * gone after 100 ms, stops in a controlled way and holds (it is not
     * disabled).  Call stop() first for an immediate stop. */
    void disconnect();
    bool connected() const;  // [Non-blocking]
    RobotInfo info() const;                 // [Non-blocking]
    RobotState state() const;               // [RT] copy of the newest telemetry
    LinkStats link_stats() const;           // [Non-blocking]
    std::vector<FaultEvent> fault_log() const; // [Non-blocking] oldest first, at most 200
    /* [Non-blocking] Off: this process only watches (no heartbeat; another
     * program controls the arm).  On by default. */
    void set_heartbeat(bool enabled);

    /* ---- Setup (IDLE, every drive disabled) --------------------------------- */
    /* [Blocking, 3 s] Selects the drive mode of J1-J7 (J8 force-position).
     * The controller then reads back every drive's IDs and PMAX/VMAX/TMAX and
     * refuses to enable on a mismatch (INFRABOT_CAP_SELFCHECK): error then
     * says which drive and register. */
    bool set_motor_profile(MotorProfile profile, std::string& error);
    /* [Blocking, 2 s] Persists the zero of every drive at its current angle
     * (all disabled); enable_after then enables both groups. */
    bool set_zero(bool enable_after, std::string& error);

    /* ---- Power ------------------------------------------------------------ */
    /* [Blocking, 3 s] IDLE: enables a group (staged in the MCU) and waits for
     * the telemetry to show it; retries a lost request every 250 ms.  On
     * timeout that group is disabled again (the other keeps holding). */
    bool servo_on(MotorGroup group, std::string& error);
    bool servo_on(std::string& error);  // both groups
    /* [Blocking, 2 s] Any mode: disables a group at once (no controlled stop;
     * the arm falls); the other keeps running.  wait=false: [Non-blocking],
     * sends the request twice without waiting. */
    bool servo_off(MotorGroup group, std::string& error, bool wait = true);
    bool servo_off(std::string& error);
    /* [Blocking, 2 s] Any mode: controlled stop -- the controller brakes the
     * arm to rest and holds it in IDLE (enabled).  Waits until it is still;
     * wait=false: [Non-blocking], sends the request twice and returns.
     * resume() continues the interrupted mode. */
    bool stop(std::string& error, bool wait = true);
    /* [Blocking, 3 s] After a controlled stop (any cause: stop(), a lost
     * stream, a failed move, a supervisor): re-enters the control mode the
     * stop interrupted (state().safety.stop_mode), from where the arm
     * stopped, as switch_mode() does: RT modes stream from hold_command(), NRT
     * modes get a new movej/movep session holding there.  A stop that
     * happened in IDLE is acknowledged.  Waits for the brake to end and the
     * arm to settle (position-velocity drives trail their stopped command);
     * refuses a disabled or faulted arm drive.  resumed: the mode entered.
     * Needs INFRABOT_CAP_RESUME. */
    bool resume(std::string& error, Mode* resumed = nullptr);
    /* [Blocking, timeout] Clears the drives that report a fault (disabled
     * ones; an enabled faulted drive needs servo_off first) and waits until
     * their state is no longer a fault.  True when nothing is left faulted. */
    bool clear_fault(std::string& error, uint32_t timeout_ms = 1000);
    /* [Non-blocking] Old name of servo_off(MotorGroup::All, error, false). */
    bool cancel(std::string& error);

    /* ---- Control mode --------------------------------------------------- */
    /* [Blocking, 2 s] Switches the control layer; a non-IDLE mode is entered
     * from IDLE with the arm enabled and still.  RT modes: the caller streams
     * servej/servep from the hold command (hold_command()).  NRT modes: the
     * SDK starts the movej/movep session, holding the current command, and
     * keeps it alive until the mode changes.  If the MCU does not answer
     * (outcome unknown) the SDK asks for a controlled stop. */
    bool switch_mode(Mode mode, std::string& error);
    /* [Non-blocking] The hold command read when the mode was entered (joint
     * frame, rad; J8 only when enabled): where a stream should start. */
    std::array<float, 8> hold_command() const;
    /* [Blocking, 0.5 s] Reads the hold command of the enabled drives now. */
    bool read_hold_command(std::array<float, 8>& position, std::string& error);

    /* ---- RT (RT_JOINT / RT_CARTESIAN) --------------------------------------- */
    /* [RT] RT_JOINT: by default sends one packet, as before. While the native
     * joint stream is active, atomically publishes the latest command and
     * refreshes its producer watchdog instead. Never starts/restarts a stream. */
    bool servej(const JointCommand& command, std::string& error);
    /* [RT] RT_JOINT, MIT gains given; torque 0, v_des 3 rad/s. */
    bool servej(const std::array<float, 7>& position, const std::array<float, 7>& velocity,
                const std::array<float, 7>& kp, const std::array<float, 7>& kd,
                std::string& error);
    bool servep(const CartesianCommand& command, std::string& error); // [RT] RT_CARTESIAN

    /* [Non-blocking] Already in RT_JOINT/MIT with fresh seven-axis feedback:
     * repeat initial at options.transmit_hz; subsequent servej() calls replace
     * it. A producer timeout, stale feedback or MCU fault requests STOP.
     * stop(), arm servo_off(), mode changes and disconnect() cancel the pump.
     * Explicit mode/setup/enable sequencing remains the caller's job. */
    bool start_joint_stream(const JointCommand& initial, const JointStreamOptions& options,
                            std::string& error);
    /* [Non-blocking] Ends native sending and restores one-packet servej().
     * Sends nothing; use stop() to stop the arm as well. */
    void stop_joint_stream();
    JointStreamStats joint_stream_stats() const;  // [Non-blocking]

    /* ---- NRT (NRT_JOINT / NRT_CARTESIAN) ------------------------------------ */
    /* [Blocking] Moves J1-J7 to position (NRT_JOINT); waits for the session to
     * be ready, then until arrival, failure or timeout_ms + 3 s. */
    MotionResult movej(const std::array<float, 7>& position, const MoveOptions& options,
                       std::string& error);
    /* [Blocking] Moves the tool to pose along a straight line (NRT_CARTESIAN). */
    MotionResult movep(const Pose& pose, const CartesianMoveOptions& options, std::string& error);
    /* [Non-blocking] Starts a move and returns; follow it with motion() or
     * wait_motion().  A move replaces nothing: the previous one must be done. */
    bool submit_movej(const std::array<float, 7>& position, const MoveOptions& options, std::string& error);
    bool submit_movep(const Pose& pose, const CartesianMoveOptions& options, std::string& error);
    MotionStatus motion() const;            // [Non-blocking]
    /* [Blocking, timeout] Until the newest move has arrived (true) or failed. */
    bool wait_motion(uint32_t timeout_ms, MotionResult& result, std::string& error);
    /* [Non-blocking] Ends the session's move with a controlled stop (DMOV
     * cancel); the arm holds in IDLE. */
    bool cancel_motion(std::string& error);
    /* [Non-blocking] Options of the hold move that starts an NRT session (gains, template). */
    void set_default_move_options(const MoveOptions& options);
    void set_default_cartesian_options(const CartesianMoveOptions& options);

    /* ---- Gripper J8 ------------------------------------------------------- */
    /* [Blocking, 1 s] Any control mode (also during movej/movep): position
     * (rad, 0-5.94), speed limit (rad/s), force = current limit as a fraction
     * of the drive's maximum phase current (0-1): stalled on an object it
     * pushes with at most that.  Fails while the gripper is disabled. */
    bool gripper(float position_rad, float speed_rad_s, float force_ratio, std::string& error);

    /* ---- Recording -------------------------------------------------------- */
    /* [Blocking, file setup] Records every telemetry packet (1 kHz), every command sent
     * and the events (faults, stops, replies, markers) as CSV under
     * directory/<time>[-label]/.  A background thread writes; nothing in the
     * 1 kHz paths touches the disk. */
    bool start_recording(const std::string& directory, std::string& error,
                         const RecordOptions& options = RecordOptions{});
    void stop_recording();  // [Blocking] finishes the writer and closes files
    RecorderStats recording() const;  // [Non-blocking]
    void record_event(const std::string& text);  // [Non-blocking] a marker in events.csv

    /* [Blocking, port/file setup] Saves the MCU's UART debug output to
     * <directory>/serial_YYYYMMDDHHMMSS.log (the call time), one line per
     * MCU line: "<host_time_s> <YYYY-MM-DD HH:MM:SS> <text>".  host_time_s is
     * CLOCK_MONOTONIC, the clock of start_recording()'s host_time_s.
     * Read-only on the port; independent of connect() and the UDP threads.
     * Fails if another program (e.g. cutecom) has the port open.  An already
     * running log is stopped first; unplug/reset reopens the port. */
    bool start_serial_log(std::string& error, const SerialLogOptions& options = SerialLogOptions{});
    void stop_serial_log();  // [Blocking] joins the serial worker
    SerialLogStats serial_log() const;  // [Non-blocking]
    /* [Non-blocking] Default SDK log directory (created on first use by start_serial_log). */
    static std::string log_directory();
    /* [Blocking, a few s] Downloads the MCU run log (McuLogOptions) into two
     * new files; never overwrites.  Needs no connect() (own socket, port
     * INFRABOT_LOG_PORT).  The board refuses while an RT Joint/Cartesian
     * stream runs; telemetry is frozen during the download, so the CSV is the
     * 10 s right before the call.  Never from the 1 kHz loop. */
    bool download_mcu_log(McuLogResult& result, std::string& error,
                          const McuLogOptions& options = McuLogOptions{});

    /* ---- Kinematics (infrabot_ik) ----------------------------------------- */
    static Pose forward(const std::array<float, 7>& joints);  // [Non-blocking] local FK

    /* ---- Single motor maintenance (IDLE, mostly disabled) -------------------
     * Values in rad, rad/s, N·m and ratio 0..1; acknowledged by the MCU,
     * [Blocking, 1.5 s]; never from the 1 kHz loop. */
    bool enable(uint8_t motor, std::string& error);
    bool disable(uint8_t motor, std::string& error);
    bool stop(uint8_t motor, std::string& error);
    bool clear_error(uint8_t motor, std::string& error);
    bool set_zero_position(uint8_t motor, std::string& error);
    bool ensure_mode(uint8_t motor, uint8_t mode, std::string& error);
    bool send_mit(uint8_t motor, float position_rad, float velocity_rad_s,
                  float kp, float kd, float torque_nm, std::string& error);
    bool send_pos_vel(uint8_t motor, float position_rad, float velocity_limit_rad_s,
                      std::string& error);
    bool send_vel(uint8_t motor, float velocity_rad_s, std::string& error);
    bool send_force_pos(uint8_t motor, float position_rad, float velocity_limit_rad_s,
                        float torque_ratio, std::string& error);
    bool request_feedback(uint8_t motor, std::string& error);
    bool get_state(uint8_t motor, std::string& error);
    bool set_can_timeout_ms(uint8_t motor, uint32_t timeout_ms, std::string& error);
    bool store_parameters(uint8_t motor, std::string& error);
    /* value_type INFRABOT_REGISTER_U32/_F32 must match the register (protocol.h). */
    bool get_register(uint8_t motor, uint8_t register_id, uint8_t value_type,
                      uint32_t& value, std::string& error);
    bool write_register(uint8_t motor, uint8_t register_id, uint32_t value,
                        uint8_t value_type, std::string& error);

    /* ---- Protocol level (udp_control_gui, tools) -----------------------------
     * Wire units (mrad, mrad/s, milli, mN·m), one datagram each, thread-safe.
     * The caller owns timing and the controller's state rules. */
    bool send_command(uint8_t command, const std::array<int32_t, 8>& target_mrad,
                      std::string& error);                                 // [RT] DMCP v3
    bool send_joint_stream(const std::array<int32_t, 8>& target_mrad, const StreamProfile& profile,
                           std::string& error);                            // [RT] DMCP v5
    bool send_cartesian_stream(const Pose& pose, const StreamProfile& profile,
                               const std::array<uint16_t, 4>& limits, std::string& error); // [RT] v7
    /* [Non-blocking] One v4 operation; refused while another is unanswered
     * (1.5 s), DISABLE excepted.  sequence identifies its DMAR. */
    bool send_maintenance(const MaintenanceRequest& request, uint32_t& sequence, std::string& error);
    /* [Blocking, timeout] The DMAR of sequence. */
    bool wait_maintenance(uint32_t sequence, InfrabotMaintenanceReply& reply, uint32_t timeout_ms,
                          std::string& error);
    /* [Blocking] send_maintenance + wait_maintenance. */
    bool maintenance(const MaintenanceRequest& request, InfrabotMaintenanceReply& reply,
                     uint32_t timeout_ms, std::string& error);
    bool maintenance_pending() const;  // [Non-blocking]
    /* [Non-blocking] The newest DMAR (any request), false before the first. */
    bool last_maintenance_reply(InfrabotMaintenanceReply& reply) const;
    /* [Blocking, 3 s] Stores the runtime control profile in the MCU Flash
     * (DMCP v5 STORE_CONTROL_PROFILE; all disabled). */
    bool store_control_profile(const StreamProfile& profile, uint8_t& status, std::string& error);
    /* [Blocking, about 10 ms per parameter] Parameter service (firmware with INFRABOT_CAP_PARAMETERS,
     * protocol 1.1): the gravity, impedance and admittance settings that the touchscreen sliders change.
     * Every parameter describes itself (name, unit, range, default, slew rate), so the list needs no table
     * compiled into the SDK. set_parameter stores the requested value and the controller slews its applied
     * value toward it, so a write never steps a gain. Values are RAM only and return to their defaults at
     * reboot. Reading is open; writing needs the PC that controls the arm (or no controlling PC). */
    bool list_parameters(std::vector<ParameterInfo>& parameters, std::string& error);
    bool get_parameter(uint16_t id, ParameterInfo& parameter, std::string& error);
    bool get_parameter(const std::string& name, ParameterInfo& parameter, std::string& error);
    bool set_parameter(uint16_t id, float value, ParameterInfo& result, std::string& error);
    bool set_parameter(const std::string& name, float value, ParameterInfo& result, std::string& error);
    /* [Blocking, up to 600 ms] One diagnostic snapshot (firmware with INFRABOT_CAP_DIAGNOSTICS): loop timing,
     * CAN and command counters, heap and least stack ever, drive states. Read only, open to any PC; it
     * never takes control of the arm. */
    bool diagnostics(DiagnosticsInfo& diagnostics, std::string& error);

    /* [Non-blocking registration] Called by the receive thread for every telemetry packet, after state()
     * reflects it.  Keep it short; it must not call back into a blocking
     * Robot function. */
    void set_telemetry_handler(std::function<void(const RobotState&)> handler);

private:
    /* switch_mode(); reply_status: the MCU's DMAR status, or
     * INFRABOT_REPLY_BUSY when the arm is not still yet (try again). */
    bool enter_mode(Mode mode, std::string& error, uint8_t* reply_status);
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

/* [Non-blocking] Enum names for display. */
const char* mode_name(Mode mode);
const char* status_name(OperationalStatus status);

} // namespace infrabot
