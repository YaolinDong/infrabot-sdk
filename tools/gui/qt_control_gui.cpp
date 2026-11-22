/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <sched.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <memory>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <random>
#include <unistd.h>
#include "motion_session.hpp"
#include "infrabot/diagnostics.hpp"
#include "infrabot/robot.hpp"

#include <QApplication>
#include <QCloseEvent>
#include <QComboBox>
#include <QDir>
#include <QScrollArea>
#include <QDebug>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QProgressBar>
#include <QPushButton>
#include <QSlider>
#include <QSpinBox>
#include <QStringList>
#include <QTimer>
#include <QTabWidget>
#include <QVBoxLayout>
#include <QWidget>
#include <QGroupBox>
#include <QHeaderView>
#include <QPlainTextEdit>
#include <QTableWidget>
#include <QSignalBlocker>
#include <QPainter>
#include <QScreen>
#include <QScrollBar>
#include <QSplitter>
#include <QDateTime>
#ifdef INFRABOT_HAS_ARM_VIEW
#include "urdf_viewer.hpp"
#endif

#include "infrabot_ik/generated_model.hpp"
#include "infrabot_ik/ik_solver.hpp"

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr uint16_t kMotorCount = INFRABOT_MOTOR_COUNT;
/* Joint limits from the H7C bare-flange URDF, in mrad.
 * J8 is not part of the seven-axis URDF: its range is the gripper's travel,
 * 0..5.94 rad (the MCU's DM_GRIPPER_POSITION_MAX).  These limits are the GUI
 * safety/input limits, while the MCU remains the final authority for drive
 * and mechanical protection. */
constexpr std::array<int, kMotorCount> kJointPositionMinMrad =
    {-2800, 0, -2800, 0, -2800, -2000, -1800, 0};
constexpr std::array<int, kMotorCount> kJointPositionMaxMrad =
    {2800, 3140, 2800, 3140, 2800, 2000, 1800, 5940};
/* Torque bars (N·m), from the Damiao manuals: rated (continuous) torque, and
 * full scale = peak torque, at most the drive's torque feedback range TMAX
 * (infrabot.tsv).  J1/J2 DM-J8009P 20/40 (TMAX 54); J3/J4 DM-J4340P-2EC V1.1
 * 12/40 (TMAX 28); J5-J8 DM-J4310-2EC 3.5/12.5 (TMAX 10). */
constexpr std::array<double, 8> kRatedTorqueNm = {20, 20, 12, 12, 3.5, 3.5, 3.5, 3.5};
constexpr std::array<double, 8> kTorqueScaleNm = {40, 40, 28, 28, 10, 10, 10, 10};
/* Protocol codes: include/infrabot/protocol.h is their one definition,
 * shared with the MCU firmware; these are the GUI's short names. */
constexpr uint8_t kMit = INFRABOT_CMD_MIT;
constexpr uint8_t kEnable = INFRABOT_CMD_ENABLE;
constexpr uint8_t kDisable = INFRABOT_CMD_DISABLE;
constexpr uint8_t kSetZero = INFRABOT_CMD_SET_ZERO;
constexpr uint8_t kSetZeroEnable = INFRABOT_CMD_SET_ZERO_ENABLE;
constexpr uint8_t kStop = INFRABOT_CMD_STOP;
constexpr uint8_t kClearError = INFRABOT_CMD_CLEAR_ERROR;
constexpr uint8_t kRequestFeedback = INFRABOT_CMD_REQUEST_FEEDBACK;
constexpr uint8_t kEnsureMode = INFRABOT_CMD_ENSURE_MODE;
constexpr uint8_t kSendMit = INFRABOT_CMD_SEND_MIT;
constexpr uint8_t kSendPosVel = INFRABOT_CMD_SEND_POS_VEL;
constexpr uint8_t kSendVel = INFRABOT_CMD_SEND_VEL;
constexpr uint8_t kSendForcePos = INFRABOT_CMD_SEND_FORCE_POS;
constexpr uint8_t kSetCanTimeout = INFRABOT_CMD_SET_CAN_TIMEOUT;
constexpr uint8_t kStoreParameters = INFRABOT_CMD_STORE_PARAMETERS;
constexpr uint8_t kGetRegister = INFRABOT_CMD_GET_REGISTER;
constexpr uint8_t kWriteRegister = INFRABOT_CMD_WRITE_REGISTER;
constexpr uint8_t kGetState = INFRABOT_CMD_GET_STATE;
constexpr uint8_t kSetArmMode = INFRABOT_CMD_SET_ARM_MODE;
constexpr uint8_t kSetControlMode = INFRABOT_CMD_SET_CONTROL_MODE;
constexpr uint8_t kGetHoldPosition = INFRABOT_CMD_GET_HOLD_POSITION;
constexpr uint8_t kEnableArm = INFRABOT_CMD_ENABLE_ARM;
constexpr uint8_t kEnableGripper = INFRABOT_CMD_ENABLE_GRIPPER;
constexpr uint8_t kDisableArm = INFRABOT_CMD_DISABLE_ARM;
constexpr uint8_t kDisableGripper = INFRABOT_CMD_DISABLE_GRIPPER;
constexpr uint8_t kGripperCommand = INFRABOT_CMD_GRIPPER;
constexpr uint8_t kArmMask = INFRABOT_ARM_MASK;         // J1-J7: every control mode needs exactly these
constexpr uint8_t kGripperMask = INFRABOT_GRIPPER_MASK; // J8
constexpr uint8_t kAllMask = INFRABOT_ALL_MASK;
constexpr uint32_t kCapabilityGroups = INFRABOT_CAP_GROUPS; // DMIO: arm and gripper enabled separately
constexpr uint8_t kGripperIndex = INFRABOT_GRIPPER_INDEX;
constexpr uint8_t kMotorAll = INFRABOT_MOTOR_ALL;
constexpr uint8_t kModeMit = INFRABOT_DRIVE_MODE_MIT;
constexpr uint8_t kModePosVel = INFRABOT_DRIVE_MODE_POS_VEL;
constexpr uint8_t kModeVel = INFRABOT_DRIVE_MODE_VEL;
constexpr uint8_t kModeForcePos = INFRABOT_DRIVE_MODE_FORCE_POS;
constexpr uint8_t kControlIdle = INFRABOT_CONTROL_IDLE;
constexpr uint8_t kControlRtJoint = INFRABOT_CONTROL_RT_JOINT;
constexpr uint8_t kControlRtCartesian = INFRABOT_CONTROL_RT_CARTESIAN;
constexpr uint8_t kControlNrtJoint = INFRABOT_CONTROL_NRT_JOINT;
constexpr uint8_t kControlNrtCartesian = INFRABOT_CONTROL_NRT_CARTESIAN;
constexpr uint8_t kStatusUnknown = INFRABOT_STATUS_UNKNOWN;
const char *controller_status_name(uint8_t status) {
    return infrabot::status_name(static_cast<infrabot::OperationalStatus>(std::min<uint8_t>(status, kStatusUnknown)));
}
const char *control_mode_name(uint8_t mode) { return infrabot::mode_name(static_cast<infrabot::Mode>(mode)); }
constexpr int64_t kPeriodNs = 1'000'000LL;

/* The bundled control_template.tsv is also the explicit startup
 * profile for the arm page.  Keep these wire-scaled values in one place so
 * movej and the 1 kHz arm stream do not silently fall back to the old
 * low-gain 4/3/2 defaults after the GUI is restarted. */
constexpr std::array<int32_t, kMotorCount> kBuiltinArmKpMilli =
    {300000, 300000, 150000, 150000, 40000, 40000, 30000, 3000};
constexpr std::array<int32_t, kMotorCount> kBuiltinArmKdMilli =
    {2500, 2500, 2500, 2500, 800, 800, 800, 800};
/* v_des of position-velocity drives; the gripper keeps 1 rad/s. */
constexpr std::array<int32_t, kMotorCount> kBuiltinArmVelocityLimitMrad =
    {3000, 3000, 3000, 3000, 3000, 3000, 3000, 1000};

struct CartesianUiSpec {
    const char *name;
    const char *unit;
    double lower;
    double upper;
    double step;
};
constexpr CartesianUiSpec kCartesianUiSpecs[6] = {
    {"X", "mm", -500.0, 500.0, 1.0},
    {"Y", "mm", -500.0, 500.0, 1.0},
    {"Z", "mm", -100.0, 800.0, 1.0},
    {"ΔRx（基座 X）", "°", -180.0, 180.0, 0.1},
    {"ΔRy（基座 Y）", "°", -180.0, 180.0, 0.1},
    {"ΔRz（基座 Z）", "°", -180.0, 180.0, 0.1},
};

/* Cartesian speed limits, shown in mm and degrees; on the wire (RT packet
 * and movep) mm/s, mrad/s, mm/s^2, mrad/s^2.  The upper bounds (tuned on the
 * arm, also the defaults) equal the MCU's ARM_CARTESIAN_MAX_*_WIRE after
 * rounding: 180 deg/s = 3142 mrad/s, 360 deg/s^2 = 6283 mrad/s^2. */
struct CartesianLimitSpec {
    const char *name;
    const char *unit;
    double lower, upper, initial, to_wire;
};
constexpr CartesianLimitSpec kCartesianLimitSpecs[4] = {
    {"线速度", "mm/s", 5.0, 500.0, 500.0, 1.0},
    {"角速度", "°/s", 1.0, 180.0, 180.0, 1000.0 * kPi / 180.0},
    {"线加速度", "mm/s²", 10.0, 2000.0, 2000.0, 1.0},
    {"角加速度", "°/s²", 5.0, 360.0, 360.0, 1000.0 * kPi / 180.0},
};
uint16_t cartesian_limit_to_wire(size_t index, double value) {
    const auto &spec = kCartesianLimitSpecs[index];
    return static_cast<uint16_t>(std::lround(std::clamp(value, spec.lower, spec.upper) * spec.to_wire));
}
/* movep per-joint caps in MIT: the configured application speed ceiling
 * and 100 rad/s^2. The MCU derives joint caps from
 * the Cartesian limits and caps each joint at its motor's rated speed itself.
 * In position-velocity mode movep's velocity_max is the drives' v_des, so the
 * runtime template's velocity_limit is sent instead (as the RT stream does). */
constexpr std::array<float, motion::axes> kCartesianJointVelocityCap = {
    INFRABOT_ARM_MAX_JOINT_VELOCITY, INFRABOT_ARM_MAX_JOINT_VELOCITY, INFRABOT_ARM_MAX_JOINT_VELOCITY,
    INFRABOT_ARM_MAX_JOINT_VELOCITY, INFRABOT_ARM_MAX_JOINT_VELOCITY, INFRABOT_ARM_MAX_JOINT_VELOCITY,
    INFRABOT_ARM_MAX_JOINT_VELOCITY};
constexpr float kCartesianJointAccelerationCap = 100.0f;

int normalized_cartesian_progress(double value, double lower, double upper) {
    if (!(upper > lower)) return 0;
    return std::clamp(static_cast<int>(std::lround((value - lower) * 1000.0 / (upper - lower))), 0, 1000);
}

class CartesianOrientationTarget {
public:
    const infrabot_ik::Mat3 &rotation() const { return rotation_; }
    void set_rotation(const infrabot_ik::Mat3 &r) { rotation_ = r; }
    void rotate_base(int axis, double radians) {
        if (axis < 0 || axis > 2 || !std::isfinite(radians) || radians == 0.0) return;
        const infrabot_ik::Mat3 delta = axis == 0 ? infrabot_ik::rot_x(radians) :
            axis == 1 ? infrabot_ik::rot_y(radians) : infrabot_ik::rot_z(radians);
        rotation_ = delta * rotation_;
        const infrabot_ik::Vec3 x = infrabot_ik::normalized({rotation_.m[0][0], rotation_.m[1][0], rotation_.m[2][0]});
        const infrabot_ik::Vec3 yc{rotation_.m[0][1], rotation_.m[1][1], rotation_.m[2][1]};
        const infrabot_ik::Vec3 y = infrabot_ik::normalized(yc - x * infrabot_ik::dot(x, yc));
        const infrabot_ik::Vec3 z = infrabot_ik::cross(x, y);
        rotation_ = {{{x.x, y.x, z.x}, {x.y, y.y, z.y}, {x.z, y.z, z.z}}};
    }
private:
    infrabot_ik::Mat3 rotation_{infrabot_ik::Mat3::identity()};
};

/* DMTL positions and MCU hold commands are already joint-frame (the MCU
 * applies J2/J4 direction), matching the frame the MCU IK uses; only convert
 * mrad -> rad here. */
infrabot_ik::JointVector joints_from_mrad(const std::array<int32_t, kMotorCount> &position_mrad) {
    infrabot_ik::JointVector q{};
    for (size_t i = 0; i < 7; ++i) q[i] = position_mrad[i] / 1000.0;
    return q;
}

/* IK seeds must be inside the URDF limits; J2/J4 (lower limit 0) read a hair
 * below it at home. */
infrabot_ik::JointVector clamped_to_limits(infrabot_ik::JointVector q) {
    for (size_t i = 0; i < 7; ++i)
        q[i] = std::clamp(q[i], infrabot_ik::kJointSpecs[i].lower, infrabot_ik::kJointSpecs[i].upper);
    return q;
}

infrabot_ik::Pose forward_from_mrad(const std::array<int32_t, kMotorCount> &position_mrad) {
    static const infrabot_ik::Kinematics kinematics;
    return kinematics.forward(joints_from_mrad(position_mrad));
}

std::array<float, 12> pose_to_wire(const infrabot_ik::Pose &pose) {
    std::array<float, 12> wire{};
    wire[0] = static_cast<float>(pose.position.x);
    wire[1] = static_cast<float>(pose.position.y);
    wire[2] = static_cast<float>(pose.position.z);
    for (int row = 0; row < 3; ++row)
        for (int col = 0; col < 3; ++col)
            wire[3 + row * 3 + col] = static_cast<float>(pose.rotation.m[row][col]);
    return wire;
}

int velocity_rad_s_to_mrad_s(double value) {
    return static_cast<int>(std::lround(value * 1000.0));
}

int position_rad_to_mrad(double value) {
    return static_cast<int>(std::lround(value * 1000.0));
}

int torque_nm_to_mnm(double value) {
    return static_cast<int>(std::lround(value * 1000.0));
}

int ratio_to_milli(double value) {
    return static_cast<int>(std::lround(value * 1000.0));
}

std::mutex g_log_mutex;
std::ofstream g_log;
std::filesystem::path g_log_dir;
std::filesystem::path g_current_log_path;

static std::filesystem::path executable_path() {
    char buffer[4096]{};
    const ssize_t length = ::readlink("/proc/self/exe", buffer, sizeof(buffer) - 1);
    if (length > 0) return std::filesystem::path(std::string(buffer, static_cast<size_t>(length)));
    return std::filesystem::current_path() / "udp_control_gui";
}

static std::string log_timestamp() {
    const std::time_t now = std::time(nullptr);
    std::tm local{};
    localtime_r(&now, &local);
    char text[32]{};
    std::strftime(text, sizeof(text), "%Y%m%d-%H%M%S", &local);
    return text;
}

void init_log() {
    std::lock_guard<std::mutex> lock(g_log_mutex);
    try {
        g_log_dir = infrabot::Robot::log_directory();
        std::filesystem::create_directories(g_log_dir);
        g_current_log_path = g_log_dir / "udp_control_gui.log";
        if (std::filesystem::exists(g_current_log_path) &&
            std::filesystem::file_size(g_current_log_path) != 0U) {
            auto archived = g_log_dir / ("udp_control_gui-" + log_timestamp() + ".log");
            unsigned suffix = 1U;
            while (std::filesystem::exists(archived)) {
                archived = g_log_dir / ("udp_control_gui-" + log_timestamp() + "-" +
                                          std::to_string(suffix++) + ".log");
            }
            std::filesystem::rename(g_current_log_path, archived);
        }
        g_log.open(g_current_log_path, std::ios::out | std::ios::trunc);
    } catch (const std::exception &) {
        /* Logging must never prevent the controller from starting. */
        g_log.open("udp_control_gui.log", std::ios::out | std::ios::trunc);
    }
    if (g_log.is_open()) {
        g_log << "--- udp_control_gui start ---\n" << std::flush;
    }
}

void log_line(const std::string &line) {
    std::lock_guard<std::mutex> lock(g_log_mutex);
    if (!g_log.is_open()) return;
    timespec ts{};
    clock_gettime(CLOCK_REALTIME, &ts);
    g_log << ts.tv_sec << '.' << std::setfill('0') << std::setw(9) << ts.tv_nsec
          << " [tid " << std::this_thread::get_id() << "] " << line << '\n' << std::flush;
}

void qt_message_handler(QtMsgType type, const QMessageLogContext &, const QString &message) {
    const char *kind = "debug";
    if (type == QtInfoMsg) kind = "info";
    else if (type == QtWarningMsg) kind = "warning";
    else if (type == QtCriticalMsg) kind = "critical";
    else if (type == QtFatalMsg) kind = "fatal";
    log_line(std::string("Qt ") + kind + ": " + message.toStdString());
}


int64_t monotonic_ns() {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<int64_t>(ts.tv_sec) * 1'000'000'000LL + ts.tv_nsec;
}

uint64_t monotonic_us() {
    return static_cast<uint64_t>(monotonic_ns() / 1000LL);
}

void sleep_until_ns(int64_t deadline_ns) {
    timespec deadline{};
    deadline.tv_sec = deadline_ns / 1'000'000'000LL;
    deadline.tv_nsec = deadline_ns % 1'000'000'000LL;
    while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &deadline, nullptr) == EINTR) {
    }
}

/* The GUI's controller.  All UDP traffic goes through the SDK
 * (infrabot::Robot): sockets, telemetry, DMAR/DMOS, the IDLE/NRT heartbeat
 * and the movej/movep session.  This class keeps what the GUI owns: the
 * slider targets, the runtime stream profile (control template) and the
 * 1 kHz RT stream thread, which hands each sample to the SDK. */
class UdpController {
public:
    struct MotorView {
        int32_t position_mrad = 0, velocity_mrad_s = 0, torque_mNm = 0;
        uint8_t state = 0, temperature_mos = 0, temperature_coil = 0, flags = 0;
        uint16_t feedback_count_lsb = 0;
    };
    struct Snapshot {
        std::array<int32_t, kMotorCount> position{};
        std::array<MotorView, kMotorCount> motors{};
        uint64_t feedback_age_ms = UINT64_MAX;
        std::string maintenance_message;
        uint64_t tx_sent = 0;
        uint64_t tx_errors = 0;
        uint64_t tx_overruns = 0;
        uint64_t rx_received = 0;
        uint64_t rx_invalid = 0;
        uint64_t rx_lost = 0;
        uint32_t last_ack = 0;
        bool command_active = false;
        bool enabled = false;
        uint8_t enabled_mask = 0;
        bool zero_valid = false;
        bool connected = false;
        bool streaming = false;
        uint8_t control_mode = kControlIdle;
        uint8_t controller_status = kStatusUnknown;
        uint8_t motor_profile = 0;
        bool mode_supported = false, profile_ready = false;
        bool non_realtime = false;
        uint8_t telemetry_version = 0;
        infrabot::SafetyState safety{};
        infrabot::LinkStats link{};
        motion::Snapshot motion;
    };

    UdpController() {
        for (auto &value : targets_) value = 0;
        cartesian_pose_.fill(0.0f);
        cartesian_pose_[3] = cartesian_pose_[7] = cartesian_pose_[11] = 1.0f;
        for (size_t i = 0; i < cartesian_limits_.size(); ++i)
            cartesian_limits_[i] = cartesian_limit_to_wire(i, kCartesianLimitSpecs[i].initial);
        arm_velocity_mrad_s_.fill(0);
        arm_kp_milli_ = kBuiltinArmKpMilli;
        arm_kd_milli_ = kBuiltinArmKdMilli;
        arm_torque_mNm_.fill(0);
        arm_velocity_limit_mrad_s_ = kBuiltinArmVelocityLimitMrad;
        arm_torque_ratio_milli_.fill(1000);
    }

    ~UdpController() { stop(); }

    bool start(const std::string &ip, int command_port, int telemetry_port,
               std::string &error) {
        log_line("start requested ip=" + ip + " command_port=" + std::to_string(command_port) +
                 " telemetry_port=" + std::to_string(telemetry_port));
        stop();
        non_realtime_.store(false);
        auto robot = std::make_unique<infrabot::Robot>(ip, static_cast<uint16_t>(command_port),
                                                       static_cast<uint16_t>(telemetry_port));
        // Per packet (SDK receive thread): sums for the torque bars' mean.
        robot->set_telemetry_handler([this](const infrabot::RobotState &state) {
            std::lock_guard<std::mutex> lock(torque_mutex_);
            for (size_t i = 0; i < kMotorCount; ++i) torque_sum_[i] += state.joints.torque[i];
            ++torque_samples_;
        });
        if (!robot->connect(error)) {
            log_line("start failed: " + error);
            return false;
        }
        const auto info = robot->info();
        log_line(std::string("SDK connected; DMIO ") + (info.received ? "received" : "missing") +
                 " caps=0x" + [&] { std::ostringstream s; s << std::hex << info.capabilities; return s.str(); }());
        robot_ = std::move(robot);
        running_.store(true, std::memory_order_release);
        tx_thread_ = std::thread(&UdpController::tx_loop, this);
        log_line("start successful; SDK and RT stream worker running");
        return true;
    }

    /* Closing the connection (window closed, reconnect) never removes power:
     * an enabled arm stops in a controlled way and holds.  Firmware without
     * the controlled stop only knows DISABLE (which it would also do by itself
     * once the stream stops). */
    void stop() {
        const bool was_running = running_.load(std::memory_order_acquire);
        if (was_running) release();
        streaming_.store(false, std::memory_order_release);
        running_.store(false, std::memory_order_release);
        if (tx_thread_.joinable()) tx_thread_.join();
        if (robot_) robot_->disconnect();
        if (was_running) log_line("stopped; sockets closed");
    }

    infrabot::RobotInfo info() const { return robot_ ? robot_->info() : infrabot::RobotInfo{}; }
    /* The connected firmware brakes and holds on a stop (DMIO capability, or
     * telemetry v3 of the same release). */
    bool controlled_stop_supported() const {
        if (!robot_) return false;
        const auto info = robot_->info();
        return (info.received && (info.capabilities & INFRABOT_CAP_CONTROLLED_STOP)) ||
               robot_->state().telemetry_version >= INFRABOT_TELEMETRY_V3;
    }
    infrabot::Robot *robot() { return robot_.get(); }
    std::vector<infrabot::FaultEvent> fault_log() const {
        return robot_ ? robot_->fault_log() : std::vector<infrabot::FaultEvent>{};
    }

    bool enable() { return enable_group(kAllMask); }
    /* Enables a drive group in IDLE: kArmMask (J1-J7, all the control modes
     * need), kGripperMask (J8) or kAllMask.  Enable ONLY: the MCU holds in
     * IDLE, and an enabled other group keeps holding. */
    bool enable_group(uint8_t group) {
        if (group != kArmMask && group != kGripperMask && group != kAllMask) return false;
        const auto state = snapshot();
        if (!robot_ || !state.mode_supported || !state.profile_ready || state.control_mode != kControlIdle ||
            !state.connected || state.feedback_age_ms > 100 || (state.enabled_mask & group) == group) return false;
        {
            std::lock_guard<std::mutex> lock(target_mutex_);
            for (size_t i = 0; i < kMotorCount; ++i)
                if (group & (1U << i)) targets_[i] = state.position[i];
        }
        home_slow_active_.store(false, std::memory_order_release);
        streaming_.store(false, std::memory_order_release);
        std::string error;
        log_line("enable group=0x" + hex(group));
        const bool ok = robot_->servo_on(static_cast<infrabot::MotorGroup>(group), error);
        log_line(ok ? "enable confirmed mask=0x" + hex(snapshot().enabled_mask) : "enable failed: " + error);
        return ok;
    }
    /* Disables one group; the other keeps running (kAllMask: disable()).
     * Without the arm the control modes end and the MCU returns to IDLE. */
    bool disable_group(uint8_t group) {
        if (group == kAllMask) return disable();
        if (group != kArmMask && group != kGripperMask) return false;
        if (group == kArmMask) {
            non_realtime_.store(false);
            home_slow_active_.store(false, std::memory_order_release);
            streaming_.store(false, std::memory_order_release);
        }
        if (!robot_) return false;
        std::string error;
        log_line("disable group=0x" + hex(group));
        // The SDK drops a move session with the arm (a cancel would ask for
        // nothing more: the MCU ends the move with it).
        return robot_->servo_off(static_cast<infrabot::MotorGroup>(group), error, false);
    }
    bool set_zero() { return zero(false); }
    bool set_zero_and_enable() { return zero(true); }

    bool enable_motor(uint8_t motor) {
        const auto state = snapshot();
        if (!state.mode_supported || !state.profile_ready || state.control_mode != kControlIdle ||
            state.feedback_age_ms > 100) return false;
        { std::lock_guard<std::mutex> lock(target_mutex_); targets_ = state.position; }
        return send_maintenance(kEnable, motor);
    }
    bool disable_motor(uint8_t motor) { return send_maintenance(kDisable, motor); }
    bool stop_motor(uint8_t motor) { return send_maintenance(kStop, motor); }
    bool clear_error(uint8_t motor) { return send_maintenance(kClearError, motor); }
    bool set_zero_motor(uint8_t motor) { return send_maintenance(kSetZero, motor); }
    bool ensure_mode(uint8_t motor, uint8_t mode) {
        return send_maintenance(kEnsureMode, motor, mode);
    }
    bool set_arm_mode(uint8_t mode) { std::string error; return set_arm_mode(mode, error); }
    /* MIT or position-velocity for J1-J7 (J8 force-position); the MCU then
     * checks every drive's parameters against its table (self-check). */
    bool set_arm_mode(uint8_t mode, std::string &error) {
        if (mode != kModeMit && mode != kModePosVel) { error = "电机模式无效"; return false; }
        const auto state = snapshot();
        if (!robot_ || !state.connected || !state.mode_supported || state.enabled_mask ||
            state.control_mode != kControlIdle || state.streaming) {
            error = "请先全部失能并回到 IDLE，确认 MCU 为新固件";
            return false;
        }
        const bool ok = robot_->set_motor_profile(static_cast<infrabot::MotorProfile>(mode), error);
        log_line(ok ? "motor profile applied mode=" + std::to_string(mode) : "motor profile failed: " + error);
        return ok;
    }
    /* Controlled stop in any mode: the MCU brakes the arm to rest and holds
     * it enabled in IDLE (firmware with INFRABOT_CAP_CONTROLLED_STOP). */
    bool controlled_stop(std::string &error) {
        if (!robot_) { error = "未连接"; return false; }
        streaming_.store(false, std::memory_order_release);
        home_slow_active_.store(false, std::memory_order_release);
        non_realtime_.store(false);
        log_line("controlled stop requested");
        return robot_->stop(error);
    }
    /* Ends the movej/movep session with a controlled stop (DMOV cancel): the
     * arm holds in IDLE, it is not disabled. */
    bool cancel_motion() {
        if (!robot_) return false;
        home_slow_active_.store(false, std::memory_order_release);
        std::string error;
        log_line("move cancel requested (controlled stop)");
        return robot_->cancel_motion(error);
    }
    /* Clears the drives that report a fault (they must be disabled). */
    bool clear_faults(std::string &error) {
        if (!robot_) { error = "未连接"; return false; }
        return robot_->clear_fault(error);
    }
    /* Gripper J8, the same in every control mode (also during a move):
     * position (joint frame mrad), speed limit (mrad/s), force = current
     * limit (per mille of the drive's maximum phase current).  Returns the
     * DMAR status (0 applied, 3 out of range, 4 busy, 7 not enabled), -1 when
     * not sent (another maintenance request pending) or not answered. */
    int gripper(int position_mrad, int velocity_mrad_s, int force_milli) {
        if (!robot_) return -1;
        infrabot::MaintenanceRequest request;
        request.command = kGripperCommand;
        request.motor = kGripperIndex;
        request.mode = kModeForcePos;
        request.position_mrad = position_mrad;
        request.velocity_limit_mrad_s = velocity_mrad_s;
        request.torque_ratio_milli = force_milli;
        uint32_t sequence = 0;
        std::string error;
        if (!robot_->send_maintenance(request, sequence, error)) return -1;
        note_sent(sequence);
        InfrabotMaintenanceReply reply{};
        if (!robot_->wait_maintenance(sequence, reply, 900, error)) return -1;
        if (reply.status == 0) { std::lock_guard<std::mutex> lock(target_mutex_); targets_[kGripperIndex] = position_mrad; }
        return reply.status;
    }
    /* The template's gripper speed limit and force (mrad/s, per mille). */
    std::pair<int32_t, int32_t> gripper_defaults() const {
        std::lock_guard<std::mutex> lock(target_mutex_);
        return {arm_velocity_limit_mrad_s_[kGripperIndex], arm_torque_ratio_milli_[kGripperIndex]};
    }
    void set_cartesian_target(const std::array<float, 12> &pose) {
        std::lock_guard<std::mutex> lock(target_mutex_);
        cartesian_pose_ = pose;
    }
    std::array<float, 12> cartesian_target() const {
        std::lock_guard<std::mutex> lock(target_mutex_);
        return cartesian_pose_;
    }
    // Wire units (see kCartesianLimitSpecs); the RT stream carries them in
    // every packet, so a change applies on the next 1 ms tick.
    void set_cartesian_limits(const std::array<uint16_t, 4> &limits) {
        std::lock_guard<std::mutex> lock(target_mutex_);
        cartesian_limits_ = limits;
    }
    std::array<uint16_t, 4> cartesian_limits() const {
        std::lock_guard<std::mutex> lock(target_mutex_);
        return cartesian_limits_;
    }
    bool set_control_mode(uint8_t mode, std::string &error) {
        if (mode > INFRABOT_CONTROL_CARTESIAN_ADMITTANCE) { error = "控制模式无效"; return false; }
        const auto state = snapshot();
        if (!robot_ || !state.connected || !state.mode_supported || state.feedback_age_ms > 100) {
            error = "请连接支持模式状态机的新固件，并确认遥测新鲜"; return false;
        }
        if (mode == state.control_mode) return true;
        if (mode != kControlIdle && (!state.profile_ready || (state.enabled_mask & kArmMask) != kArmMask ||
            state.control_mode != kControlIdle)) {
            error = "先应用 MIT/位置速度电机模式并使能机械臂 J1–J7；切换 RT/NRT 前必须回到 IDLE"; return false;
        }
        if (state.motion.phase != motion::Phase::Off && state.motion.phase != motion::Phase::Cancelled &&
            state.motion.phase != motion::Phase::Cancelling &&
            state.motion.phase != motion::Phase::Error && !state.motion.can_execute) {
            error = "当前 move 尚未完成，请等待到位；紧急情况使用全部失能"; return false;
        }
        for (size_t i = 0; i < motion::axes; ++i) // the arm; a moving gripper does not block
            if ((state.enabled_mask & (1U << i)) && std::abs(state.motors[i].velocity_mrad_s) > 50) {
                error = "切换前机械臂必须静止（速度 ≤ 0.05 rad/s）；未自动失能"; return false;
            }
        // Stop the RT stream while switching.
        struct TransitionGuard {
            std::atomic<bool> &flag;
            explicit TransitionGuard(std::atomic<bool> &f) : flag(f) { flag.store(true); }
            ~TransitionGuard() { flag.store(false); }
        } transition(mode_transition_);
        streaming_.store(false);
        home_slow_active_.store(false);
        // The hold move that starts an NRT session uses the template gains.
        robot_->set_default_move_options(to_move_options(move_defaults()));
        robot_->set_default_cartesian_options(to_cartesian_options(cartesian_defaults()));
        if (!robot_->switch_mode(static_cast<infrabot::Mode>(mode), error)) {
            log_line("control mode failed: " + error);
            return false;
        }
        entered(mode, false);
        log_line(std::string("control mode confirmed: ") + control_mode_name(mode));
        return true;
    }
    /* Firmware that reports the mode a controlled stop interrupted. */
    bool resume_supported() const {
        if (!robot_) return false;
        const auto info = robot_->info();
        return info.received && (info.capabilities & INFRABOT_CAP_RESUME);
    }
    /* After a controlled stop: back to the mode it interrupted, from where
     * the arm stopped (Robot::resume: it waits for the arm to settle).  RT
     * modes stream from the hold command; a resumed NRT session holds there
     * and the interrupted move's target stays the draft.  A stop in IDLE is
     * acknowledged.  resumed: the mode entered (kControl*). */
    bool resume(std::string &error, uint8_t &resumed) {
        if (!robot_) { error = "未连接"; return false; }
        struct TransitionGuard {
            std::atomic<bool> &flag;
            explicit TransitionGuard(std::atomic<bool> &f) : flag(f) { flag.store(true); }
            ~TransitionGuard() { flag.store(false); }
        } transition(mode_transition_);
        streaming_.store(false);
        home_slow_active_.store(false);
        robot_->set_default_move_options(to_move_options(move_defaults()));
        robot_->set_default_cartesian_options(to_cartesian_options(cartesian_defaults()));
        log_line("resume requested");
        infrabot::Mode mode = infrabot::Mode::Unknown;
        if (!robot_->resume(error, &mode)) {
            log_line("resume failed: " + error);
            return false;
        }
        resumed = static_cast<uint8_t>(mode) <= kControlNrtCartesian ? static_cast<uint8_t>(mode) : kControlIdle;
        entered(resumed, true);
        log_line(std::string("resumed: ") + control_mode_name(resumed));
        return true;
    }
    std::array<int32_t, kMotorCount> targets() const {
        std::lock_guard<std::mutex> lock(target_mutex_);
        return targets_;
    }
    bool apply_joint_parameters(uint8_t motor, int velocity_mrad_s, int kp_milli,
                                int kd_milli, int torque_mNm,
                                int velocity_limit_mrad_s, int torque_ratio_milli) {
        if (motor >= kMotorCount) return false;
        std::lock_guard<std::mutex> lock(target_mutex_);
        arm_velocity_mrad_s_[motor] = velocity_mrad_s;
        arm_kp_milli_[motor] = kp_milli;
        arm_kd_milli_[motor] = kd_milli;
        arm_torque_mNm_[motor] = torque_mNm;
        arm_velocity_limit_mrad_s_[motor] = velocity_limit_mrad_s;
        arm_torque_ratio_milli_[motor] = torque_ratio_milli;
        log_line("arm stream parameters applied motor=" + std::to_string(motor) +
                 " vel=" + std::to_string(velocity_mrad_s) +
                 " kp=" + std::to_string(kp_milli) +
                 " kd=" + std::to_string(kd_milli) +
                 " torque=" + std::to_string(torque_mNm) +
                 " vlim=" + std::to_string(velocity_limit_mrad_s) +
                 " ratio=" + std::to_string(torque_ratio_milli));
        return true;
    }
    bool scan_all_joints() {
        log_line("arm scan all joints requested");
        return request_feedback(kMotorAll);
    }
    void apply_default_arm_parameters() {
        std::lock_guard<std::mutex> lock(target_mutex_);
        arm_velocity_mrad_s_.fill(0);
        arm_kp_milli_ = kBuiltinArmKpMilli;
        arm_kd_milli_ = kBuiltinArmKdMilli;
        arm_torque_mNm_.fill(0);
        arm_velocity_limit_mrad_s_ = kBuiltinArmVelocityLimitMrad;
        arm_torque_ratio_milli_.fill(1000);
        log_line("arm default stream parameters applied");
    }
    bool export_arm_parameters(const std::string &path, std::string &error) const {
        std::ofstream out(path);
        if (!out.is_open()) { error = "无法写入参数文件"; return false; }
        std::lock_guard<std::mutex> lock(target_mutex_);
        out << "# Runtime control profile (not motor Flash registers)\n"
            << "# units: velocity=rad/s, kp/kd=unitless, torque=Nm, ratio=0..1\n"
            << "motor\tvelocity_rad_s\tkp\tkd\ttorque_Nm\tvelocity_limit_rad_s\ttorque_ratio\n";
        out << std::fixed << std::setprecision(6);
        for (size_t i = 0; i < kMotorCount; ++i) {
            out << (i + 1) << '\t' << arm_velocity_mrad_s_[i] / 1000.0 << '\t'
                << arm_kp_milli_[i] / 1000.0 << '\t' << arm_kd_milli_[i] / 1000.0 << '\t'
                << arm_torque_mNm_[i] / 1000.0 << '\t' << arm_velocity_limit_mrad_s_[i] / 1000.0
                << '\t' << arm_torque_ratio_milli_[i] / 1000.0 << '\n';
        }
        if (!out.good()) { error = "参数文件写入失败"; return false; }
        log_line("arm parameters exported path=" + path);
        return true;
    }
    bool import_arm_parameters(const std::string &path, std::string &error) {
        std::ifstream in(path);
        if (!in.is_open()) { error = "无法打开参数文件"; return false; }
        std::array<int32_t, kMotorCount> velocity{}, kp{}, kd{}, torque{}, limit{}, ratio{};
        std::array<bool, kMotorCount> seen{};
        std::string line;
        bool legacy_units = false;
        while (std::getline(in, line)) {
            if (line.empty() || line[0] == '#') continue;
            legacy_units = line.find("velocity_mrad_s") != std::string::npos;
            break;
        }
        size_t rows = 0;
        try {
            while (std::getline(in, line)) {
                if (line.empty() || line[0] == '#') continue;
                std::istringstream ss(line);
                std::array<std::string, 7> fields{};
                for (size_t i = 0; i < fields.size(); ++i) {
                    if (!std::getline(ss, fields[i], '\t')) {
                        error = "参数文件列数错误"; return false;
                    }
                }
                const int motor = std::stoi(fields[0]) - 1;
                if (motor < 0 || motor >= static_cast<int>(kMotorCount) || seen[motor]) {
                    error = "参数文件电机编号错误"; return false;
                }
                seen[motor] = true;
                const double velocity_value = std::stod(fields[1]);
                const double kp_value = std::stod(fields[2]);
                const double kd_value = std::stod(fields[3]);
                const double torque_value = std::stod(fields[4]);
                const double limit_value = std::stod(fields[5]);
                const double ratio_value = std::stod(fields[6]);
                velocity[motor] = legacy_units ? static_cast<int32_t>(std::lround(velocity_value))
                                               : velocity_rad_s_to_mrad_s(velocity_value);
                kp[motor] = legacy_units ? static_cast<int32_t>(std::lround(kp_value))
                                          : static_cast<int32_t>(std::lround(kp_value * 1000.0));
                kd[motor] = legacy_units ? static_cast<int32_t>(std::lround(kd_value))
                                          : static_cast<int32_t>(std::lround(kd_value * 1000.0));
                torque[motor] = legacy_units ? static_cast<int32_t>(std::lround(torque_value))
                                              : torque_nm_to_mnm(torque_value);
                limit[motor] = legacy_units ? static_cast<int32_t>(std::lround(limit_value))
                                             : velocity_rad_s_to_mrad_s(limit_value);
                ratio[motor] = legacy_units ? static_cast<int32_t>(std::lround(ratio_value))
                                             : ratio_to_milli(ratio_value);
                if (++rows > kMotorCount) { error = "参数文件行数过多"; return false; }
            }
        } catch (...) {
            error = "参数文件包含无效数字"; return false;
        }
        if (rows != kMotorCount) { error = "参数文件必须包含 8 个电机"; return false; }
        std::lock_guard<std::mutex> lock(target_mutex_);
        arm_velocity_mrad_s_ = velocity;
        arm_kp_milli_ = kp;
        arm_kd_milli_ = kd;
        arm_torque_mNm_ = torque;
        arm_velocity_limit_mrad_s_ = limit;
        arm_torque_ratio_milli_ = ratio;
        log_line("arm parameters imported path=" + path);
        return true;
    }
    bool send_mit_once(uint8_t motor, int position_mrad, int velocity_mrad_s,
                       int kp_milli, int kd_milli, int torque_mNm) {
        return send_maintenance(kSendMit, motor, kModeMit, 0, 0, 0,
                                 position_mrad, velocity_mrad_s, kp_milli,
                                 kd_milli, torque_mNm);
    }
    bool send_pos_vel(uint8_t motor, int position_mrad, int velocity_limit_mrad_s) {
        return send_maintenance(kSendPosVel, motor, kModePosVel, 0, 0, 0,
                                 position_mrad, 0, 0, 0, 0, velocity_limit_mrad_s);
    }
    bool send_vel(uint8_t motor, int velocity_mrad_s) {
        return send_maintenance(kSendVel, motor, kModeVel, 0, 0, 0,
                                 0, velocity_mrad_s);
    }
    bool send_force_pos(uint8_t motor, int position_mrad, int velocity_limit_mrad_s,
                        int torque_ratio_milli) {
        log_line("force-pos target motor=" + std::to_string(motor) +
                 " position_mrad=" + std::to_string(position_mrad) +
                 " velocity_limit_mrad_s=" + std::to_string(velocity_limit_mrad_s) +
                 " torque_ratio=" + std::to_string(torque_ratio_milli) + "/1000");
        return send_maintenance(kSendForcePos, motor, kModeForcePos, 0, 0, 0,
                                 position_mrad, 0, 0, 0, 0, velocity_limit_mrad_s,
                                 torque_ratio_milli);
    }
    bool request_feedback(uint8_t motor) { return send_maintenance(kRequestFeedback, motor); }
    bool get_state(uint8_t motor) { return send_maintenance(kGetState, motor); }
    bool set_can_timeout_ms(uint8_t motor, uint32_t timeout_ms) {
        return send_maintenance(kSetCanTimeout, motor, 0, 0, 0, timeout_ms);
    }
    bool store_parameters(uint8_t motor) { return send_maintenance(kStoreParameters, motor); }
    bool get_register(uint8_t motor, uint8_t register_id, uint8_t value_type) {
        return send_maintenance(kGetRegister, motor, 0, value_type, register_id);
    }
    bool write_register(uint8_t motor, uint8_t register_id, uint32_t value,
                        uint8_t value_type) {
        return send_maintenance(kWriteRegister, motor, 0, value_type, register_id, value);
    }

    /* A template import must not advance until the previous CAN transaction
     * has completed.  The MCU accepts only one maintenance request at a time;
     * these helpers wait for each DMAR (SDK). */
    bool write_register_confirmed(uint8_t motor, uint8_t register_id,
                                  uint32_t value, uint8_t value_type,
                                  uint8_t &status, uint32_t &readback) {
        return maintenance_wait(kWriteRegister, motor, 0, value_type, register_id,
                                value, status, readback);
    }

    bool store_parameters_confirmed(uint8_t motor, uint8_t &status) {
        uint32_t readback = 0;
        return maintenance_wait(kStoreParameters, motor, 0, 0, 0, 0,
                                status, readback);
    }

    bool read_register_confirmed(uint8_t motor, uint8_t register_id,
                                 uint8_t value_type, uint8_t &status,
                                 uint32_t &value) {
        return maintenance_wait(kGetRegister, motor, 0, value_type, register_id,
                                0, status, value);
    }

    bool disable_all_confirmed(uint8_t &status) {
        uint32_t readback = 0;
        streaming_.store(false, std::memory_order_release);
        return maintenance_wait(kDisable, kMotorAll, 0, 0, 0, 0,
                                status, readback);
    }

    /* Persist the current complete arm control profile in the MCU.  This is
     * intentionally a single v5 transaction so the MCU can validate all
     * joints and atomically commit one Flash record (including existing zero
     * calibration) instead of leaving a half-written profile. */
    bool store_control_profile_confirmed(uint8_t &status, std::string &error) {
        streaming_.store(false, std::memory_order_release);
        if (!robot_) { error = "未连接"; return false; }
        const bool ok = robot_->store_control_profile(stream_profile(), status, error);
        log_line(ok ? std::string("control profile stored") : "control profile store failed: " + error);
        return ok;
    }

    bool import_motorbridge_parameters(const std::string &path, std::string &error) {
        struct Row {
            uint8_t rid = 0;
            uint8_t value_type = 0;
            std::string name;
            std::array<std::string, kMotorCount> values{};
        };

        std::ifstream in(path);
        if (!in.is_open()) { error = "无法打开 Damiao 参数模板"; return false; }

        auto split_tsv = [](const std::string &line) {
            std::vector<std::string> fields;
            std::string field;
            std::istringstream stream(line);
            while (std::getline(stream, field, '\t')) fields.push_back(field);
            if (!line.empty() && line.back() == '\t') fields.emplace_back();
            return fields;
        };

        std::vector<Row> rows;
        std::array<bool, 256> seen_rid{};
        std::string line;
        std::vector<std::string> header;
        size_t first_joint = std::string::npos;
        while (std::getline(in, line)) {
            if (line.empty() || line[0] == '#') continue;
            const auto fields = split_tsv(line);
            if (header.empty() && !fields.empty() && fields[0] == "rid") {
                header = fields;
                for (size_t i = 0; i < header.size(); ++i)
                    if (header[i] == "J1") { first_joint = i; break; }
                if (first_joint == std::string::npos || header.size() < first_joint + kMotorCount) {
                    error = "模板缺少 J1-J8 参数列"; return false;
                }
                continue;
            }
            if (header.empty() || first_joint == std::string::npos) {
                error = "模板缺少 rid/type 表头"; return false;
            }
            if (fields.size() < first_joint + kMotorCount) {
                error = "模板参数行列数不足"; return false;
            }
            try {
                const int rid = std::stoi(fields[0], nullptr, 0);
                if (rid < 0 || rid > 255 || seen_rid[static_cast<size_t>(rid)]) {
                    error = "模板 RID 重复或越界"; return false;
                }
                if (fields[1] != "f32" && fields[1] != "u32") {
                    error = "模板只支持 f32/u32 RID"; return false;
                }
                Row row;
                row.rid = static_cast<uint8_t>(rid);
                row.value_type = fields[1] == "f32" ? 1U : 0U;
                row.name = fields.size() > 2 ? fields[2] : std::string();
                for (size_t motor = 0; motor < kMotorCount; ++motor) {
                    const size_t column = first_joint + motor;
                    if (column < fields.size()) row.values[motor] = fields[column];
                }
                for (size_t motor = 0; motor < kMotorCount; ++motor) {
                    if (row.values[motor].empty() || row.values[motor] == "ERR" ||
                        row.values[motor] == "offline") {
                        error = "模板包含空值/ERR/offline：RID " + std::to_string(rid) +
                                " J" + std::to_string(motor + 1);
                        return false;
                    }
                }
                seen_rid[static_cast<size_t>(rid)] = true;
                rows.push_back(std::move(row));
            } catch (...) {
                error = "模板包含无效 RID"; return false;
            }
        }
        if (rows.empty() || rows.size() > 64U) {
            error = "模板没有有效参数行"; return false;
        }

        // Parse and check every value against the register table
        // (protocol.h) before the first drive is touched: a template with a
        // typo or an out-of-range value writes nothing.
        std::vector<std::array<uint32_t, kMotorCount>> raws(rows.size());
        for (size_t r = 0; r < rows.size(); ++r) {
            const Row &row = rows[r];
            const auto *info = infrabot_register_info(row.rid);
            if (!info || info->type != row.value_type || info->access != INFRABOT_REGISTER_WRITABLE) {
                error = "模板 " + infrabot::register_name(row.rid) + (!info ? std::string(" 不是已知寄存器") :
                        info->type != row.value_type
                            ? std::string(" 的类型应为 ") + (info->type == INFRABOT_REGISTER_F32 ? "f32" : "u32")
                            : std::string(" 不允许写入（只读或受保护）"));
                return false;
            }
            for (size_t motor = 0; motor < kMotorCount; ++motor) {
                uint32_t raw = 0;
                try {
                    if (row.value_type == 0U) {
                        const unsigned long parsed = std::stoul(row.values[motor], nullptr, 0);
                        if (parsed > 0xFFFFFFFFUL) throw std::out_of_range("u32");
                        raw = static_cast<uint32_t>(parsed);
                    } else {
                        const float value = std::stof(row.values[motor]);
                        if (!std::isfinite(value)) throw std::out_of_range("f32");
                        std::memcpy(&raw, &value, sizeof(raw));
                    }
                } catch (...) {
                    error = "模板数值无效：J" + std::to_string(motor + 1) +
                            " RID " + std::to_string(row.rid);
                    return false;
                }
                if (!infrabot_register_write_allowed(row.rid, raw)) {
                    error = "模板数值越界：J" + std::to_string(motor + 1) + " " +
                            infrabot::register_name(row.rid) + " = " + row.values[motor] +
                            "（" + infrabot_register_info(row.rid)->description + "）";
                    return false;
                }
                raws[r][motor] = raw;
            }
        }

        const auto state = snapshot();
        if (state.feedback_age_ms > 500U) {
            error = "MCU 遥测未连接或反馈已过期"; return false;
        }
        if (!disable_all_confirmed_state(error)) return false;

        log_line("motor parameter import started path=" + path +
                 " rows=" + std::to_string(rows.size()));
        for (size_t motor = 0; motor < kMotorCount; ++motor) {
            for (size_t r = 0; r < rows.size(); ++r) {
                const Row &row = rows[r];
                const uint32_t raw = raws[r][motor];
                uint8_t status = 0;
                uint32_t readback = 0;
                if (!write_register_confirmed(static_cast<uint8_t>(motor), row.rid,
                                              raw, row.value_type, status, readback) ||
                    status != 0U || readback != raw) {
                    error = "写入失败：J" + std::to_string(motor + 1) +
                            " RID " + std::to_string(row.rid) + " (" + row.name +
                            ") status=" + std::to_string(status) +
                            " readback=0x" + [&] {
                                std::ostringstream text;
                                text << std::hex << readback;
                                return text.str();
                            }();
                    log_line("motor parameter import failed motor=" + std::to_string(motor + 1) +
                             " rid=" + std::to_string(row.rid) +
                             " status=" + std::to_string(status));
                    return false;
                }
            }
            uint8_t status = 0;
            if (!store_parameters_confirmed(static_cast<uint8_t>(motor), status) || status > 1U) {
                error = "保存失败：J" + std::to_string(motor + 1) +
                        " status=" + std::to_string(status);
                return false;
            }
            log_line("motor parameter import stored motor=" + std::to_string(motor + 1));
        }
        log_line("motor parameter import completed");
        return true;
    }

    bool export_motorbridge_parameters(const std::string &template_path,
                                       const std::string &output_path,
                                       std::string &error) {
        struct Row {
            uint8_t rid = 0;
            uint8_t value_type = 0;
            std::vector<std::string> fields;
            std::array<std::string, kMotorCount> values{};
        };

        std::ifstream in(template_path);
        if (!in.is_open()) { error = "无法打开电机 Flash 参数模板"; return false; }

        auto split_tsv = [](const std::string &line) {
            std::vector<std::string> fields;
            std::string field;
            std::istringstream stream(line);
            while (std::getline(stream, field, '\t')) fields.push_back(field);
            if (!line.empty() && line.back() == '\t') fields.emplace_back();
            return fields;
        };

        std::vector<std::string> preamble;
        std::string header_line;
        size_t first_joint = std::string::npos;
        std::vector<Row> rows;
        std::array<bool, 256> seen_rid{};
        std::string line;
        while (std::getline(in, line)) {
            if (line.empty() || line[0] == '#') {
                if (header_line.empty()) preamble.push_back(line);
                continue;
            }
            const auto fields = split_tsv(line);
            if (header_line.empty()) {
                if (fields.empty() || fields[0] != "rid") {
                    error = "模板缺少 rid/type 表头"; return false;
                }
                for (size_t i = 0; i < fields.size(); ++i)
                    if (fields[i] == "J1") { first_joint = i; break; }
                if (first_joint == std::string::npos ||
                    fields.size() < first_joint + kMotorCount) {
                    error = "模板缺少 J1-J8 参数列"; return false;
                }
                header_line = line;
                continue;
            }
            if (fields.size() < first_joint + kMotorCount || fields.size() < 2U) {
                error = "模板参数行列数不足"; return false;
            }
            try {
                const int rid = std::stoi(fields[0], nullptr, 0);
                if (rid < 0 || rid > 255 || seen_rid[static_cast<size_t>(rid)]) {
                    error = "模板 RID 重复或越界"; return false;
                }
                if (fields[1] != "f32" && fields[1] != "u32") {
                    error = "模板只支持 f32/u32 RID"; return false;
                }
                Row row;
                row.rid = static_cast<uint8_t>(rid);
                row.value_type = fields[1] == "f32" ? 1U : 0U;
                row.fields = fields;
                seen_rid[static_cast<size_t>(rid)] = true;
                rows.push_back(std::move(row));
            } catch (...) {
                error = "模板包含无效 RID"; return false;
            }
        }
        if (header_line.empty() || rows.empty() || rows.size() > 64U) {
            error = "模板没有有效参数行"; return false;
        }

        if (!disable_all_confirmed_state(error)) return false;
        log_line("motor Flash export started template=" + template_path +
                 " rows=" + std::to_string(rows.size()));

        for (size_t motor = 0; motor < kMotorCount; ++motor) {
            for (Row &row : rows) {
                uint8_t status = 0;
                uint32_t raw = 0;
                if (!read_register_confirmed(static_cast<uint8_t>(motor), row.rid,
                                             row.value_type, status, raw) || status != 0U) {
                    error = "读取失败：J" + std::to_string(motor + 1) +
                            " RID " + std::to_string(row.rid) +
                            " status=" + std::to_string(status);
                    log_line("motor Flash export failed motor=" + std::to_string(motor + 1) +
                             " rid=" + std::to_string(row.rid) +
                             " status=" + std::to_string(status));
                    return false;
                }
                if (row.value_type == 0U) {
                    row.values[motor] = std::to_string(raw);
                } else {
                    float value = 0.0f;
                    std::memcpy(&value, &raw, sizeof(value));
                    if (!std::isfinite(value)) {
                        error = "读取到无效浮点值：J" + std::to_string(motor + 1) +
                                " RID " + std::to_string(row.rid);
                        return false;
                    }
                    std::ostringstream text;
                    text << std::setprecision(9) << value;
                    row.values[motor] = text.str();
                }
            }
            log_line("motor Flash export read motor=" + std::to_string(motor + 1));
        }

        std::ofstream out(output_path);
        if (!out.is_open()) { error = "无法写入电机 Flash 参数文件"; return false; }
        for (const std::string &comment : preamble) out << comment << '\n';
        out << header_line << '\n';
        for (const Row &row : rows) {
            for (size_t column = 0; column < row.fields.size(); ++column) {
                if (column != 0U) out << '\t';
                if (column >= first_joint && column < first_joint + kMotorCount)
                    out << row.values[column - first_joint];
                else
                    out << row.fields[column];
            }
            out << '\n';
        }
        if (!out.good()) { error = "电机 Flash 参数文件写入失败"; return false; }
        log_line("motor Flash export completed path=" + output_path);
        return true;
    }

    /* Leaving: a controlled stop of an enabled arm (no power removed). */
    void release() {
        home_slow_active_.store(false, std::memory_order_release);
        streaming_.store(false, std::memory_order_release);
        non_realtime_.store(false);
        if (!robot_ || !robot_->state().enabled) return;
        std::string error;
        if (robot_->stop(error, false)) {
            log_line("leaving: controlled stop requested, the arm holds");
            return;
        }
        log_line("leaving: " + error + "; disable (firmware without controlled stop)");
        (void)robot_->servo_off(infrabot::MotorGroup::All, error, false);
    }

    bool disable() {
        home_slow_active_.store(false, std::memory_order_release);
        streaming_.store(false, std::memory_order_release);
        non_realtime_.store(false);
        if (!robot_ || !running_.load()) return false;
        std::string error;
        log_line("disable all");
        // Twice, without waiting: the SDK cancels the move session too.
        return robot_->servo_off(infrabot::MotorGroup::All, error, false);
    }

    motion::Parameters move_defaults() const {
        motion::Parameters p;
        std::lock_guard<std::mutex> lock(target_mutex_);
        for (size_t i = 0; i < motion::axes; ++i) {
            p.kp[i] = arm_kp_milli_[i] / 1000.0f;
            p.kd[i] = arm_kd_milli_[i] / 1000.0f;
            p.torque[i] = arm_torque_mNm_[i] / 1000.0f;
            p.velocity_max[i] = 0.1f;
            p.acceleration_max[i] = 0.2f;
        }
        return p;
    }

    motion::CartesianParameters cartesian_defaults() const {
        motion::CartesianParameters p;
        const bool position_velocity = snapshot().motor_profile == kModePosVel;
        {
            std::lock_guard<std::mutex> lock(target_mutex_);
            p.pose = cartesian_pose_;
            p.cartesian_limits = cartesian_limits_;
            for (size_t i = 0; i < motion::axes; ++i) {
                p.kp[i] = arm_kp_milli_[i] / 1000.0f;
                p.kd[i] = arm_kd_milli_[i] / 1000.0f;
                p.velocity_max[i] = position_velocity
                    ? std::clamp(arm_velocity_limit_mrad_s_[i] / 1000.0f, 0.001f, kCartesianJointVelocityCap[i])
                    : kCartesianJointVelocityCap[i];
                p.acceleration_max[i] = kCartesianJointAccelerationCap;
            }
        }
        return p;
    }

    bool movej(const motion::Parameters &parameters, std::string &error) {
        const auto state = snapshot();
        if (!robot_ || state.control_mode != kControlNrtJoint || !non_realtime_.load() || !state.connected ||
            state.feedback_age_ms > 100 || (state.enabled_mask & 0x7f) != 0x7f) {
            error = "请先在 move 模式使能，并确认七轴在线"; return false;
        }
        if (!robot_->submit_movej(parameters.target, to_move_options(parameters), error)) return false;
        const auto id = robot_->motion().id;
        log_line("MOVE submit id=" + std::to_string(id) + " (100Hz immutable target)");
        for (size_t i = 0; i < motion::axes; ++i) {
            std::ostringstream line;
            line << "MOVE id=" << id << " J" << (i + 1)
                 << " target_rad=" << parameters.target[i]
                 << " vmax_rad_s=" << parameters.velocity_max[i]
                 << " amax_rad_s2=" << parameters.acceleration_max[i]
                 << " kp=" << parameters.kp[i] << " kd=" << parameters.kd[i]
                 << " torque_Nm=" << parameters.torque[i];
            log_line(line.str());
        }
        return true;
    }

    bool movep(const motion::CartesianParameters &parameters, std::string &error) {
        const auto state = snapshot();
        if (!robot_ || state.control_mode != kControlNrtCartesian || !non_realtime_.load() || !state.connected ||
            state.feedback_age_ms > 100 || (state.enabled_mask & 0x7f) != 0x7f) {
            error = "请先在笛卡尔 move 模式使能，并确认七轴在线"; return false;
        }
        if (!robot_->submit_movep(parameters.pose, to_cartesian_options(parameters), error)) return false;
        log_line("MOVE-P submit id=" + std::to_string(robot_->motion().id) + " (100Hz immutable pose)");
        return true;
    }

    void set_target(size_t index, int value) {
        if (index >= kMotorCount) return;
        home_slow_active_.store(false, std::memory_order_release);
        std::lock_guard<std::mutex> lock(target_mutex_);
        if (targets_[index] != value) {
            log_line("slider target motor=" + std::to_string(index + 1) +
                     " position_mrad=" + std::to_string(value));
        }
        targets_[index] = value;
    }

    bool home_slow(std::string &error) {
        const auto state = snapshot();
        if (non_realtime_.load()) {
            if (!state.connected || state.feedback_age_ms > 100 || !state.motion.can_execute) {
                error = "慢速回零要求 movej 已使能并处于可执行状态";
                return false;
            }
            auto parameters = move_defaults();
            parameters.target.fill(0.0f);
            parameters.velocity_max.fill(0.1f);
            parameters.acceleration_max.fill(0.1f);
            parameters.timeout_ms = 180000U;
            log_line("slow home requested in movej: target=0 velocity=0.1 rad/s");
            return movej(parameters, error);
        }
        if (non_realtime_.load() || state.control_mode != kControlRtJoint || !state.connected ||
            !state.enabled || (state.enabled_mask & kArmMask) != kArmMask || !streaming_.load()) {
            error = "慢速回零要求 servej 模式且机械臂 J1–J7 已经使能";
            return false;
        }
        /* Continue from the streamed target (what the MCU holds), moving it
         * toward zero at 0.1 rad/s (1 mrad per 10 ms); restarting from the
         * measured angles would drop the arm by its gravity sag.  This keeps
         * the existing 1 kHz stream and MCU watchdog active. */
        home_slow_divider_ = 0U;
        home_slow_active_.store(true, std::memory_order_release);
        log_line("slow home requested: target=0 velocity=0.1 rad/s");
        return true;
    }

    // Measured joints for the 3D view, only while telemetry is fresh.
    bool measured_position(std::array<int32_t, kMotorCount> &position) const {
        if (!robot_) return false;
        const auto state = robot_->state();
        if (!state.connected || state.telemetry_age_ms > 100) return false;
        for (size_t i = 0; i < kMotorCount; ++i) position[i] = position_rad_to_mrad(state.joints.position[i]);
        return true;
    }

    /* Measured torque of each drive (N·m, joint frame) for the torque bars,
     * only while telemetry is fresh: the mean of the 1 kHz samples since the
     * previous call (the drives' current-based torque is noisy), or the latest
     * sample after a longer pause. */
    bool measured_torque(std::array<float, kMotorCount> &torque) {
        if (!robot_) return false;
        const auto state = robot_->state();
        if (!state.connected || state.telemetry_age_ms > 100) return false;
        std::lock_guard<std::mutex> lock(torque_mutex_);
        const bool mean = torque_samples_ > 0 && torque_samples_ <= 200;
        for (size_t i = 0; i < kMotorCount; ++i)
            torque[i] = mean ? static_cast<float>(torque_sum_[i] / torque_samples_) : state.joints.torque[i];
        torque_sum_.fill(0.0);
        torque_samples_ = 0;
        return true;
    }

    Snapshot snapshot() const {
        Snapshot result;
        if (!robot_) return result;
        const auto state = robot_->state();
        const auto link = robot_->link_stats();
        for (size_t i = 0; i < kMotorCount; ++i) {
            auto &m = result.motors[i];
            m.position_mrad = position_rad_to_mrad(state.joints.position[i]);
            m.velocity_mrad_s = velocity_rad_s_to_mrad_s(state.joints.velocity[i]);
            m.torque_mNm = torque_nm_to_mnm(state.joints.torque[i]);
            m.state = state.joints.drive_state[i];
            m.temperature_mos = state.joints.temperature[i];
            m.temperature_coil = state.joints.temperature_coil[i];
            m.flags = state.joints.motor_flags[i];
            m.feedback_count_lsb = state.joints.feedback_count[i];
            result.position[i] = m.position_mrad;
        }
        if (state.telemetry_age_ms != UINT32_MAX) result.feedback_age_ms = state.telemetry_age_ms;
        result.maintenance_message = maintenance_message();
        result.last_ack = state.command_applied_sequence;
        result.command_active = state.command_active;
        result.enabled = state.enabled;
        result.enabled_mask = state.enabled_mask;
        result.zero_valid = state.zero_valid;
        result.control_mode = static_cast<uint8_t>(state.mode);
        result.controller_status = state.controller_state;
        result.mode_supported = state.modes_supported;
        result.motor_profile = static_cast<uint8_t>(state.profile);
        result.profile_ready = state.profile_ready;
        result.connected = state.connected && result.feedback_age_ms <= 500;
        result.telemetry_version = state.telemetry_version;
        result.safety = state.safety;
        result.link = link;
        result.tx_sent = link.tx_sent;
        result.tx_errors = link.tx_errors;
        result.tx_overruns = tx_overruns_.load(std::memory_order_relaxed);
        result.rx_received = link.rx_telemetry;
        result.rx_invalid = link.rx_invalid;
        result.rx_lost = link.rx_lost;
        result.streaming = streaming_.load(std::memory_order_relaxed) && result.connected &&
                           result.enabled && (result.control_mode == kControlRtJoint ||
                                              result.control_mode == kControlRtCartesian);
        result.non_realtime = result.control_mode == kControlNrtJoint ||
                              result.control_mode == kControlNrtCartesian;
        const auto status = robot_->motion();
        result.motion.phase = static_cast<motion::Phase>(status.phase);
        result.motion.id = status.id;
        result.motion.reply = status.reply;
        result.motion.has_reply = status.has_reply;
        result.motion.error = status.error;
        result.motion.reply_age_us = status.reply_age_us;
        result.motion.can_execute = status.can_execute;
        return result;
    }

private:
    /* A mode was entered (or resumed): the stream and the drafts start from
     * the hold command the SDK read. */
    void entered(uint8_t mode, bool resume) {
        non_realtime_.store(mode == kControlNrtJoint || mode == kControlNrtCartesian);
        if (mode == kControlIdle && resume) return; // a stop in IDLE acknowledged: nothing to seed
        {
            // The SDK read the MCU's hold command on entry: the measured angle
            // sits below it by the gravity sag, so starting from feedback
            // drops the arm.  A disabled gripper keeps its measured value.
            const auto hold_rad = robot_->hold_command();
            std::array<int32_t, kMotorCount> hold{};
            for (size_t i = 0; i < kMotorCount; ++i) hold[i] = position_rad_to_mrad(hold_rad[i]);
            std::lock_guard<std::mutex> lock(target_mutex_);
            targets_ = hold;
            hold_reference_ = hold;
            // A stale/identity Cartesian target would drive the arm on handover.
            // A resumed NRT Cartesian session keeps the interrupted movep target
            // as its draft (movep needs an explicit, confirmed execute).
            if (!(resume && mode == kControlNrtCartesian)) cartesian_pose_ = pose_to_wire(forward_from_mrad(hold));
            arm_velocity_mrad_s_.fill(0);
            arm_torque_mNm_.fill(0);
        }
        if (mode == kControlRtJoint || mode == kControlRtCartesian) streaming_.store(true);
    }

    static std::string hex(unsigned value) {
        std::ostringstream text;
        text << std::hex << value;
        return text.str();
    }

    static infrabot::MoveOptions to_move_options(const motion::Parameters &p) {
        infrabot::MoveOptions o;
        o.timeout_ms = p.timeout_ms; o.dwell_ms = p.dwell_ms;
        o.position_tolerance = p.position_tolerance; o.velocity_tolerance = p.velocity_tolerance;
        o.velocity_max = p.velocity_max; o.acceleration_max = p.acceleration_max;
        o.kp = p.kp; o.kd = p.kd; o.torque = p.torque;
        return o;
    }
    static infrabot::CartesianMoveOptions to_cartesian_options(const motion::CartesianParameters &p) {
        infrabot::CartesianMoveOptions o;
        o.timeout_ms = p.timeout_ms; o.dwell_ms = p.dwell_ms;
        o.position_tolerance = p.position_tolerance; o.velocity_tolerance = p.velocity_tolerance;
        o.limits.linear_velocity = p.cartesian_limits[0] / 1000.0f;
        o.limits.angular_velocity = p.cartesian_limits[1] / 1000.0f;
        o.limits.linear_acceleration = p.cartesian_limits[2] / 1000.0f;
        o.limits.angular_acceleration = p.cartesian_limits[3] / 1000.0f;
        o.velocity_max = p.velocity_max; o.acceleration_max = p.acceleration_max;
        o.kp = p.kp; o.kd = p.kd;
        return o;
    }

    infrabot::StreamProfile stream_profile() const {
        infrabot::StreamProfile profile;
        std::lock_guard<std::mutex> lock(target_mutex_);
        profile.velocity_mrad_s = arm_velocity_mrad_s_;
        profile.kp_milli = arm_kp_milli_;
        profile.kd_milli = arm_kd_milli_;
        profile.torque_mNm = arm_torque_mNm_;
        profile.velocity_limit_mrad_s = arm_velocity_limit_mrad_s_;
        profile.torque_ratio_milli = arm_torque_ratio_milli_;
        return profile;
    }

    bool zero(bool enable_after) {
        const auto state = snapshot();
        if (!robot_ || !state.mode_supported || state.control_mode != kControlIdle || state.enabled_mask != 0 ||
            state.feedback_age_ms > 100) return false;
        std::string error;
        log_line(enable_after ? "set zero and enable" : "set zero");
        const bool ok = robot_->set_zero(enable_after, error);
        if (!ok) log_line("zero failed: " + error);
        return ok;
    }

    bool disable_all_confirmed_state(std::string &error) {
        uint8_t status = 0;
        if (!disable_all_confirmed(status) || status > 1U) {
            error = "无法确认全臂失能，status=" + std::to_string(status);
            return false;
        }
        return true;
    }

    bool maintenance_wait(uint8_t command, uint8_t motor, uint8_t mode,
                          uint8_t value_type, uint8_t register_id,
                          uint32_t value, uint8_t &status, uint32_t &result) {
        if (!robot_) return false;
        infrabot::MaintenanceRequest request;
        request.command = command;
        request.motor = motor;
        request.mode = mode;
        request.value_type = value_type;
        request.register_id = register_id;
        request.value = value;
        uint32_t sequence = 0;
        std::string error;
        if (!robot_->send_maintenance(request, sequence, error)) return false;
        note_sent(sequence);
        InfrabotMaintenanceReply reply{};
        if (!robot_->wait_maintenance(sequence, reply, 900, error)) {
            log_line("maintenance wait timeout seq=" + std::to_string(sequence));
            return false;
        }
        status = reply.status;
        result = reply.value;
        return true;
    }

    /* One v4 operation without waiting; its DMAR shows in the status line. */
    bool send_maintenance(uint8_t command, uint8_t motor, uint8_t mode = 0,
                          uint8_t value_type = 0, uint8_t register_id = 0,
                          uint32_t value = 0, int position_mrad = 0,
                          int velocity_mrad_s = 0, int kp_milli = 0,
                          int kd_milli = 0, int torque_mNm = 0,
                          int velocity_limit_mrad_s = 0,
                          int torque_ratio_milli = 0) {
        if (!robot_) {
            log_line("maintenance rejected: not connected");
            return false;
        }
        // A move owns the arm only: the gripper stays commandable.
        const auto phase = robot_->motion().phase;
        const bool engaged = phase != infrabot::MotionStatus::Phase::Off &&
                             phase != infrabot::MotionStatus::Phase::Cancelled &&
                             phase != infrabot::MotionStatus::Phase::Error;
        if (engaged && command != kGetState && command != kSetControlMode && command != kDisable &&
            command != kGripperCommand) {
            log_line("maintenance rejected: move owns control; cancel/disable first");
            return false;
        }
        infrabot::MaintenanceRequest request;
        request.command = command;
        request.motor = motor;
        request.mode = mode;
        request.value_type = value_type;
        request.register_id = register_id;
        request.value = value;
        request.position_mrad = position_mrad;
        request.velocity_mrad_s = velocity_mrad_s;
        request.kp_milli = kp_milli;
        request.kd_milli = kd_milli;
        request.torque_mNm = torque_mNm;
        request.velocity_limit_mrad_s = velocity_limit_mrad_s;
        request.torque_ratio_milli = torque_ratio_milli;
        uint32_t sequence = 0;
        std::string error;
        if (!robot_->send_maintenance(request, sequence, error)) {
            log_line("maintenance not sent command=" + std::to_string(command) + ": " + error);
            return false;
        }
        note_sent(sequence);
        log_line("maintenance sent command=" + std::to_string(command) +
                 " motor=" + std::to_string(motor) + " seq=" + std::to_string(sequence));
        return true;
    }

    void note_sent(uint32_t sequence) {
        std::lock_guard<std::mutex> lock(message_mutex_);
        sent_sequence_ = sequence;
        sent_us_ = monotonic_us();
    }

    /* The newest maintenance request: waiting, its DMAR, or no answer. */
    std::string maintenance_message() const {
        uint32_t sent;
        uint64_t sent_us;
        {
            std::lock_guard<std::mutex> lock(message_mutex_);
            sent = sent_sequence_;
            sent_us = sent_us_;
        }
        InfrabotMaintenanceReply response{};
        const bool replied = robot_ && robot_->last_maintenance_reply(response);
        if (sent && (!replied || response.sequence != sent)) {
            if (monotonic_us() - sent_us > 1500000)
                return "应答超时：操作结果未知，检查 MCU 日志/固件版本；未自动重试";
            return "已发送，等待 MCU/电机确认 seq=" + std::to_string(sent);
        }
        if (!replied) return sent ? std::string() : "等待 MCU 响应（socket 打开不代表 MCU 在线）";
        std::string message = "command=" + std::to_string(response.command) + " " +
                              infrabot::command_name(response.command) + " " +
                              infrabot::reply_status_text(response.status) +
                              " seq=" + std::to_string(response.sequence);
        if (response.command == kSetArmMode && response.status == 0) {
            message += response.value == kModePosVel ? " (M1-M7=位置/速度，M8=力位)" : " (M1-M7=MIT，M8=力位)";
        }
        if (response.command == kSetArmMode && response.status == INFRABOT_REPLY_MISMATCH)
            message += " " + infrabot::describe_selfcheck_reply(response.value);
        if (response.status == 6 && response.command >= kSendMit && response.command <= kSendForcePos) {
            message += " (MCU当前模式=" + std::to_string(response.value) +
                       ", 1=MIT/2=位置速度/3=速度/4=力位)";
        }
        if (response.command == kGetRegister || response.command == kSetCanTimeout ||
            response.command == kWriteRegister || response.command == kEnsureMode) {
            message += " " + infrabot::register_name(response.register_id) + "=" +
                       infrabot::register_value_text(response.register_id, response.value);
            if (response.register_id == INFRABOT_RID_TIMEOUT && response.status == 0)
                message += " (CAN timeout=" + std::to_string(response.value / 20.0) + " ms)";
        }
        return message;
    }

    bool send_arm_packet() {
        if (!streaming_.load() || non_realtime_.load() || mode_transition_.load() || !robot_) return true;
        const auto state = robot_->state();
        const uint8_t mode = static_cast<uint8_t>(state.mode);
        if (!state.modes_supported || (mode != kControlRtJoint && mode != kControlRtCartesian) ||
            !state.enabled || state.telemetry_age_ms > 100) return false;
        const auto profile = stream_profile();
        std::string error;
        if (mode == kControlRtCartesian) {
            std::array<float, 12> pose;
            std::array<uint16_t, 4> limits;
            {
                std::lock_guard<std::mutex> lock(target_mutex_);
                pose = cartesian_pose_;
                limits = cartesian_limits_;
            }
            if (!robot_->send_cartesian_stream(pose, profile, limits, error)) {
                log_line("Cartesian stream send failed: " + error);
                return false;
            }
            return true;
        }
        std::array<int32_t, kMotorCount> targets;
        {
            std::lock_guard<std::mutex> lock(target_mutex_);
            if (home_slow_active_.load(std::memory_order_acquire) &&
                home_slow_divider_.fetch_add(1U, std::memory_order_relaxed) >= 9U) {
                home_slow_divider_.store(0U, std::memory_order_relaxed);
                bool complete = true;
                constexpr int32_t kHomeStepMrad = 1; // 0.1 rad/s at 100 Hz target updates
                for (size_t i = 0; i < motion::axes; ++i) { // the arm: the gripper keeps its command
                    auto &target = targets_[i];
                    if (target > kHomeStepMrad) target -= kHomeStepMrad;
                    else if (target < -kHomeStepMrad) target += kHomeStepMrad;
                    else target = 0;
                    if (target != 0) complete = false;
                }
                if (complete) {
                    home_slow_active_.store(false, std::memory_order_release);
                    log_line("slow home reached zero target");
                }
            }
            targets = targets_;
        }
        if (!robot_->send_joint_stream(targets, profile, error)) {
            log_line("arm stream send failed: " + error);
            return false;
        }
        return true;
    }

    /* The RT stream: one v5/v7 sample per millisecond while an RT mode runs.
     * The SDK keeps IDLE/NRT alive by itself (heartbeat, move session). */
    void tx_loop() {
        int64_t next_deadline = monotonic_ns() + kPeriodNs;
        log_line("RT stream worker started");
        while (running_.load(std::memory_order_acquire)) {
            sleep_until_ns(next_deadline);
            if (!running_.load(std::memory_order_acquire)) break;
            if (streaming_.load(std::memory_order_acquire)) {
                (void)send_arm_packet();
                // No file I/O in the 1 kHz thread; the UI/logger samples counters.
            }
            next_deadline += kPeriodNs;
            const int64_t now = monotonic_ns();
            if (now > next_deadline) {
                const uint64_t skipped = static_cast<uint64_t>((now - next_deadline) / kPeriodNs) + 1U;
                tx_overruns_.fetch_add(skipped, std::memory_order_relaxed);
                log_line("TX overrun skipped=" + std::to_string(skipped));
                next_deadline += static_cast<int64_t>(skipped) * kPeriodNs;
            }
        }
        log_line("RT stream worker stopped");
    }

    std::unique_ptr<infrabot::Robot> robot_;
    mutable std::mutex target_mutex_;
    std::atomic<bool> non_realtime_{false};
    std::atomic<bool> mode_transition_{false};
    std::array<int32_t, kMotorCount> targets_{};
    std::array<int32_t, kMotorCount> arm_velocity_mrad_s_{};
    std::array<int32_t, kMotorCount> arm_kp_milli_{};
    std::array<int32_t, kMotorCount> arm_kd_milli_{};
    std::array<int32_t, kMotorCount> arm_torque_mNm_{};
    std::array<int32_t, kMotorCount> arm_velocity_limit_mrad_s_{};
    std::array<int32_t, kMotorCount> arm_torque_ratio_milli_{};
    std::array<float, 12> cartesian_pose_{};
    std::array<uint16_t, 4> cartesian_limits_{};
    std::array<int32_t, kMotorCount> hold_reference_{};
    std::atomic<bool> running_{false};
    std::atomic<bool> streaming_{false};
    std::atomic<bool> home_slow_active_{false};
    std::atomic<uint32_t> home_slow_divider_{0};
    std::thread tx_thread_;
    std::atomic<uint64_t> tx_overruns_{0};
    std::mutex torque_mutex_;
    std::array<double, kMotorCount> torque_sum_{}; // since measured_torque()
    uint32_t torque_samples_ = 0;
    mutable std::mutex message_mutex_;
    uint32_t sent_sequence_ = 0;
    uint64_t sent_us_ = 0;
};

/* Signed torque bar, zero in the middle: positive torque fills to the right,
 * negative to the left.  Full scale is the motor's peak torque (at most its
 * feedback range), dashed ticks mark its rated torque; the fill turns amber
 * past the rated torque and red past 90 % of full scale. */
class TorqueBar final : public QWidget {
public:
    TorqueBar(double rated, double scale, QWidget *parent) : QWidget(parent), rated_(rated), scale_(scale) {
        setToolTip(QStringLiteral("满量程 ±%1 N·m，虚线 = 额定 %2 N·m；超过额定变橙色，超过满量程 90% 变红色")
                       .arg(scale, 0, 'g', 3).arg(rated, 0, 'g', 3));
    }
    void set_value(double torque, bool valid) {
        // Redraw on a change of 0.25 % of full scale (under a pixel) or validity.
        if (valid == valid_ && std::fabs(torque - value_) < scale_ * 0.0025) return;
        value_ = torque;
        valid_ = valid;
        update();
    }
    double value() const { return value_; }
    // 0 within rated torque, 1 past it, 2 past 90 % of full scale.
    int level() const {
        const double magnitude = std::fabs(value_);
        return magnitude > 0.9 * scale_ ? 2 : magnitude > rated_ ? 1 : 0;
    }
    QSize sizeHint() const override { return {120, fontMetrics().height() + 8}; }
    QSize minimumSizeHint() const override { return {60, fontMetrics().height() + 4}; }

protected:
    void paintEvent(QPaintEvent *) override {
        QPainter painter(this);
        const QRectF frame = QRectF(rect()).adjusted(0.5, 2.5, -0.5, -2.5);
        painter.fillRect(frame, palette().base());
        const double middle = frame.center().x();
        const double half = frame.width() / 2.0;
        if (valid_) {
            const double length = std::clamp(value_ / scale_, -1.0, 1.0) * half;
            const QColor color = level() == 2 ? QColor(0xd3, 0x2f, 0x2f) :
                                 level() == 1 ? QColor(0xf0, 0x9a, 0x1a) : palette().highlight().color();
            painter.fillRect(QRectF(std::min(middle, middle + length), frame.top(), std::fabs(length), frame.height()),
                             color);
        }
        painter.setPen(QPen(palette().mid().color(), 1, Qt::DashLine));
        for (const double sign : {-1.0, 1.0}) {
            const double x = middle + sign * std::min(rated_ / scale_, 1.0) * half;
            painter.drawLine(QPointF(x, frame.top()), QPointF(x, frame.bottom()));
        }
        painter.setPen(QPen(palette().dark().color(), 1));
        painter.drawLine(QPointF(middle, frame.top()), QPointF(middle, frame.bottom()));
        painter.setPen(palette().mid().color());
        painter.drawRect(frame);
    }

private:
    double rated_, scale_;
    double value_ = 0.0;
    bool valid_ = false;
};

// A tab page in a scroll area that asks for the page's whole size, so the
// window can be sized to show it without scrolling (QScrollArea caps its own
// size hint at a few dozen lines).
class PageScrollArea final : public QScrollArea {
public:
    QSize sizeHint() const override {
        if (!widget()) return QScrollArea::sizeHint();
        const int frame = 2 * frameWidth();
        return widget()->sizeHint() + QSize(frame + verticalScrollBar()->sizeHint().width(), frame);
    }
};

class MainWindow final : public QWidget {
public:
    explicit MainWindow(bool auto_connect = true) {
        setWindowTitle(QStringLiteral("DM 机械臂 UDP 控制"));

        // The connection row and the status line stay in view; each tab
        // scrolls on its own (fit_to_screen() sizes the window to need no
        // scrolling where the screen allows).
        auto *root = new QVBoxLayout(this);
        auto *connection = new QHBoxLayout();
        connection->addWidget(new QLabel(QStringLiteral("MCU IP:"), this));
        ip_edit_ = new QLineEdit(QStringLiteral("192.168.0.50"), this);
        ip_edit_->setObjectName("mcu_ip");
        ip_edit_->setMinimumWidth(150);
        connection->addWidget(ip_edit_);
        connection->addWidget(new QLabel(QStringLiteral("命令端口:"), this));
        command_port_ = new QSpinBox(this);
        command_port_->setObjectName("command_port");
        command_port_->setRange(1, 65535);
        command_port_->setValue(5001);
        connection->addWidget(command_port_);
        connection->addWidget(new QLabel(QStringLiteral("遥测端口:"), this));
        telemetry_port_ = new QSpinBox(this);
        telemetry_port_->setObjectName("telemetry_port");
        telemetry_port_->setRange(1, 65535);
        telemetry_port_->setValue(5000);
        connection->addWidget(telemetry_port_);
        connect_button_ = new QPushButton(QStringLiteral("连接/重连"), this);
        connect_button_->setObjectName("connect_button");
        connection->addWidget(connect_button_);
        connection->addStretch();
        root->addLayout(connection);

        auto *mode_tabs = new QTabWidget(this);
        auto *motor_page = new QWidget;
        motor_page_ = motor_page;
        auto *motor_root = new QVBoxLayout(motor_page);
        auto *arm_page = new QWidget;
        auto *arm_root = new QVBoxLayout(arm_page);
        // Dense rows, so every mode fits one screen page on 1440p.  Rows set
        // their own (default) horizontal spacing: a nested layout would take
        // these 3 px otherwise.
        arm_root->setSpacing(3);
        constexpr int kRowSpacing = 6;
        mode_tabs->addTab(scrolled(motor_page), QStringLiteral("电机通用模式"));
        arm_scroll_ = scrolled(arm_page);
#ifdef INFRABOT_HAS_ARM_VIEW
        // Arm controls on the left, the measured arm in 3D on the right (the
        // motor tab has no 3D view).  The controls keep their width, the 3D
        // view takes up any change of the window width.
        arm_split_ = new QSplitter(Qt::Horizontal);
        arm_split_->addWidget(arm_scroll_);
        arm_split_->addWidget(make_arm_view(arm_split_));
        arm_split_->setStretchFactor(0, 0);
        arm_split_->setStretchFactor(1, 1);
        arm_split_->setChildrenCollapsible(false);
        mode_tabs->addTab(arm_split_, QStringLiteral("机械臂模式"));
#else
        mode_tabs->addTab(arm_scroll_, QStringLiteral("机械臂模式"));
#endif
        mode_tabs->addTab(scrolled(make_diagnostics_page()), QStringLiteral("诊断"));
        root->addWidget(mode_tabs, 1);

        auto *profile_row = new QHBoxLayout();
        profile_row->setSpacing(kRowSpacing);
        profile_row->addWidget(new QLabel(QStringLiteral("① 电机模式（最高层；先选择并应用，再使能）"), this));
        arm_mode_select_ = new QComboBox(this);
        arm_mode_select_->setObjectName("motor_profile");
        arm_mode_select_->addItem(QStringLiteral("J1–J7 MIT；J8 力位混合"), kModeMit);
        arm_mode_select_->addItem(QStringLiteral("J1–J7 位置/速度；J8 力位混合"), kModePosVel);
        profile_row->addWidget(arm_mode_select_);
        auto *set_arm_mode = new QPushButton(QStringLiteral("应用电机模式（失能时）"), this);
        profile_apply_ = set_arm_mode;
        set_arm_mode->setObjectName("apply_motor_profile");
        profile_row->addWidget(set_arm_mode);
        arm_root->addLayout(profile_row);

        auto *actions = new QHBoxLayout();
        actions->setSpacing(kRowSpacing);
        auto *zero_enable = new QPushButton(QStringLiteral("全部校零并使能"), this);
        auto *enable = new QPushButton(QStringLiteral("② 全部使能"), this);
        enable_button_ = enable;
        enable->setObjectName("arm_enable");
        enable->setToolTip(QStringLiteral("机械臂 J1–J7 与夹爪 J8 一起使能，进入 IDLE 保持当前位置（不执行目标）。\n"
                                          "夹爪未连接时整体失败、不使能任何电机：请用“使能机械臂”。"));
        // The arm alone (no gripper, another tool, tool/payload/dynamics
        // identification) or the gripper alone; the other group keeps its state.
        enable_arm_ = new QPushButton(QStringLiteral("使能机械臂"), this);
        enable_arm_->setObjectName("arm_enable_arm");
        enable_arm_->setToolTip(QStringLiteral("只使能机械臂 J1–J7，进入 IDLE 保持当前位置；所有控制模式只需要机械臂。\n"
                                               "已使能的夹爪保持不动。"));
        enable_gripper_ = new QPushButton(QStringLiteral("使能夹爪"), this);
        enable_gripper_->setObjectName("arm_enable_gripper");
        enable_gripper_->setToolTip(QStringLiteral("只使能夹爪 J8（须在 IDLE）；已使能的机械臂继续保持。\n"
                                                   "RT Joint 下由 M8 滑条控制，切换控制模式不改变夹爪指令。"));
        auto *zero = new QPushButton(QStringLiteral("全部校零"), this);
        auto *disable = new QPushButton(QStringLiteral("全部失能"), this);
        disable->setToolTip(QStringLiteral("立即失能全部电机（不是安全制动，承重机械臂会下坠）。\n"
                                           "只想停下运动、保持位置时用“受控停止”。"));
        stop_button_ = new QPushButton(QStringLiteral("受控停止"), this);
        stop_button_->setObjectName("arm_controlled_stop");
        stop_button_->setToolTip(QStringLiteral("任意模式：MCU 把机械臂刹停并在 IDLE 保持（保持使能，不下坠）。\n"
                                                "需要新固件（受控停止能力）。"));
        stop_button_->setStyleSheet(QStringLiteral("background:#ffe2a8"));
        resume_button_ = new QPushButton(QStringLiteral("恢复"), this);
        resume_button_->setObjectName("arm_resume");
        resume_button_->setToolTip(QStringLiteral("受控停止后（不论原因）：一键回到停止前的控制模式，从停下的位置继续，"
                                                  "不用手动切 IDLE；会等机械臂静止后再切。\n"
                                                  "RT Joint / RT 笛卡尔：目标从停下的位置开始；movej / movep：被打断的目标保留为草稿，"
                                                  "确认“执行”即继续。停止发生在 IDLE 时只确认停止。"));
        resume_button_->setStyleSheet(QStringLiteral("background:#cdeccd"));
        disable_arm_ = new QPushButton(QStringLiteral("失能机械臂"), this);
        disable_arm_->setObjectName("arm_disable_arm");
        disable_arm_->setToolTip(QStringLiteral("只失能机械臂 J1–J7：运行中的控制模式随之结束、回到 IDLE；夹爪保持。\n"
                                                "失能不是安全制动，承重机械臂会下坠，请做好支撑。"));
        disable_gripper_ = new QPushButton(QStringLiteral("失能夹爪"), this);
        disable_gripper_->setObjectName("arm_disable_gripper");
        disable_gripper_->setToolTip(QStringLiteral("只失能夹爪 J8（夹持的物体会松开），机械臂继续当前模式。"));
        auto *home_slow = new QPushButton(QStringLiteral("慢速回0位"), this);
        home_slow->setToolTip(QStringLiteral("机械臂慢速回到 0 位（约 0.1 rad/s）"));
        disable->setObjectName("arm_disable");
        home_slow->setObjectName("arm_home_slow");
        zero_enable->setStyleSheet(QStringLiteral("background:#b8e6b8"));
        disable->setStyleSheet(QStringLiteral("background:#f0b0b0"));
        disable_arm_->setStyleSheet(QStringLiteral("background:#f6d4d4"));
        disable_gripper_->setStyleSheet(QStringLiteral("background:#f6d4d4"));
        home_slow->setStyleSheet(QStringLiteral("background:#d7e8ff"));
        zero_enable->hide(); // Calibration and enabling are separate arm actions.
        for (auto *button : {enable, enable_arm_, enable_gripper_}) actions->addWidget(button);
        actions->addSpacing(12);
        actions->addWidget(stop_button_);
        actions->addWidget(resume_button_);
        for (auto *button : {disable, disable_arm_, disable_gripper_}) actions->addWidget(button);
        actions->addSpacing(12);
        actions->addWidget(zero);
        actions->addWidget(home_slow);
        actions->addStretch();
        arm_root->addLayout(actions);

        auto *arm_scan = new QPushButton(QStringLiteral("扫描全部关节"), this);
        auto *arm_defaults = new QPushButton(QStringLiteral("写入内置Flash默认模板"), this);
        auto *arm_import_control_template = new QPushButton(QStringLiteral("写入 MCU 控制模板"), this);
        auto *arm_import_control = new QPushButton(QStringLiteral("导入控制参数"), this);
        auto *arm_export_control = new QPushButton(QStringLiteral("导出控制参数"), this);
        auto *arm_flash_export = new QPushButton(QStringLiteral("导出整臂电机Flash参数"), this);
        auto *parameter_actions = new QGridLayout();
        parameter_actions->setHorizontalSpacing(kRowSpacing);
        const std::array<QPushButton *, 6> parameter_buttons = {arm_scan, arm_defaults,
            arm_import_control_template, arm_import_control, arm_export_control, arm_flash_export};
        for (size_t i = 0; i < parameter_buttons.size(); ++i)
            parameter_actions->addWidget(parameter_buttons[i], 0, static_cast<int>(i));
        arm_root->addLayout(parameter_actions);

        move_conflicting_buttons_ = {zero_enable, zero, set_arm_mode, arm_scan, arm_defaults,
                                     arm_import_control_template, arm_import_control, arm_flash_export};
        zero_buttons_ = {zero_enable, zero};
        home_slow_button_ = home_slow;
        auto *transport = new QHBoxLayout();
        transport->setSpacing(kRowSpacing);
        transport->addWidget(new QLabel(QStringLiteral("③ SwitchMode（经 IDLE 切换）："), this));
        transport_mode_ = new QComboBox(this);
        transport_mode_->setObjectName("transport_mode");
        transport_mode_->addItem(QStringLiteral("IDLE：保持/等待"), kControlIdle);
        transport_mode_->addItem(QStringLiteral("RT Joint：实时关节 1 kHz"), kControlRtJoint);
        transport_mode_->addItem(QStringLiteral("RT Cartesian：实时笛卡尔 1 kHz"), kControlRtCartesian);
        transport_mode_->addItem(QStringLiteral("NRT Joint：非实时关节 100 Hz"), kControlNrtJoint);
        transport_mode_->addItem(QStringLiteral("NRT Cartesian：非实时笛卡尔 100 Hz"), kControlNrtCartesian);
        transport_mode_->addItem(QStringLiteral("JOINT IMP：关节阻抗"), INFRABOT_CONTROL_JOINT_IMPEDANCE);
        transport_mode_->addItem(QStringLiteral("CART IMP：笛卡尔阻抗"), INFRABOT_CONTROL_CARTESIAN_IMPEDANCE);
        transport_mode_->addItem(QStringLiteral("JOINT ADM：关节导纳"), INFRABOT_CONTROL_JOINT_ADMITTANCE);
        transport_mode_->addItem(QStringLiteral("CART ADM：笛卡尔导纳"), INFRABOT_CONTROL_CARTESIAN_ADMITTANCE);
        transport_mode_->setToolTip(QStringLiteral("servep：RT Cartesian 下拖动笛卡尔滑块实时发送；movep：NRT Cartesian 下点击“执行 movep”"));
        transport->addWidget(transport_mode_);
        mode_apply_ = new QPushButton(QStringLiteral("进入所选控制模式"), this);
        mode_apply_->setObjectName("apply_control_mode");
        transport->addWidget(mode_apply_);
        idle_button_ = new QPushButton(QStringLiteral("返回 IDLE（静止保持，不失能）"), this);
        idle_button_->setObjectName("enter_idle");
        transport->addWidget(idle_button_);
        transport->addStretch();
        // Keep the old test/discovery object names, but no longer expose them
        // as a disabled "reserved" control in the user-facing layout.
        for (const auto *name : {"servep_reserved", "movep_reserved"}) {
            auto *legacy = new QPushButton(this);
            legacy->setObjectName(QString::fromLatin1(name));
            legacy->setEnabled(false);
            legacy->hide();
        }
        arm_root->addLayout(transport);
        auto *admit_row=new QHBoxLayout();
        for(auto spec: {std::pair<const char*,int>{"JOINT ADM",INFRABOT_CONTROL_JOINT_ADMITTANCE},
                        {"CART ADM",INFRABOT_CONTROL_CARTESIAN_ADMITTANCE}}){
            auto *button=new QPushButton(QString::fromLatin1(spec.first),this);
            button->setObjectName(spec.second==9?"enter_joint_admittance":"enter_cartesian_admittance");
            admit_row->addWidget(button);
            connect(button,&QPushButton::clicked,this,[this,mode=spec.second]{
                transport_mode_->setCurrentIndex(transport_mode_->findData(mode));
                mode_apply_->click();
            });
        }
        auto *admit_stop=new QPushButton(QStringLiteral("STOP / HOLD"),this);
        admit_stop->setObjectName("admittance_stop");admit_row->addWidget(admit_stop);
        connect(admit_stop,&QPushButton::clicked,this,[this]{
            if(auto *stop=findChild<QPushButton*>("arm_controlled_stop"))stop->click();
        });
        admit_row->addStretch();arm_root->addLayout(admit_row);
        mode_status_ = new QLabel(this);
        mode_status_->setObjectName("control_mode_status");
        mode_status_->setWordWrap(true);
        mode_status_->setMinimumHeight(two_lines(mode_status_));
        mode_status_->setToolTip(QStringLiteral("IDLE 不接受运动目标；使能仅保持。进入 RT Joint 才启动滑条控制；NRT Joint 需点击执行 movej。"
                                                "返回 IDLE 不失能；PC 掉线（关闭 GUI、拔网线）时 MCU 受控停止并保持，不失能。"));
        arm_root->addWidget(mode_status_);

        auto *cartesian_group = new QGroupBox(QStringLiteral("笛卡尔控制：位置 + 3×3 姿态矩阵"), this);
        cartesian_group_ = cartesian_group;
        auto *cartesian_root = new QVBoxLayout(cartesian_group);
        cartesian_root->setSpacing(4);
        auto *cartesian_grid = new QGridLayout();
        for (int i = 0; i < 6; ++i) {
            auto *label = new QLabel(QString::fromUtf8(kCartesianUiSpecs[i].name), this);
            auto *spin = new QDoubleSpinBox(this);
            spin->setObjectName(QStringLiteral("cartesian_target_%1").arg(i));
            spin->setRange(kCartesianUiSpecs[i].lower, kCartesianUiSpecs[i].upper);
            spin->setDecimals(i < 3 ? 2 : 3);
            spin->setSingleStep(kCartesianUiSpecs[i].step);
            auto *unit = new QLabel(QString::fromUtf8(kCartesianUiSpecs[i].unit), this);
            auto *slider = new QSlider(Qt::Horizontal, this);
            slider->setObjectName(QStringLiteral("cartesian_slider_%1").arg(i));
            slider->setTracking(true);
            slider->setMinimumWidth(160);
            if (i < 3) {
                slider->setRange(0, 1000);
                slider->setSingleStep(1);
                slider->setPageStep(50);
            } else {
                slider->setRange(-180000, 180000);
                slider->setSingleStep(100);
                slider->setPageStep(10000);
                slider->setToolTip(QStringLiteral("绕基座固定轴旋转；滑块归中不改变矩阵姿态"));
            }
            cartesian_spins_[i] = spin;
            cartesian_sliders_[i] = slider;
            // Two blocks side by side: X/Y/Z, then ΔRx/ΔRy/ΔRz.
            const int row = i % 3, column = (i / 3) * 5;
            cartesian_grid->addWidget(label, row, column);
            cartesian_grid->addWidget(spin, row, column + 1);
            cartesian_grid->addWidget(unit, row, column + 2);
            cartesian_grid->addWidget(slider, row, column + 3);
            connect(spin, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
                    [this, i](double value) { set_cartesian_control(i, value); });
            connect(slider, &QSlider::valueChanged, this, [this, i](int raw) {
                const double value = i < 3
                    ? kCartesianUiSpecs[i].lower + (kCartesianUiSpecs[i].upper - kCartesianUiSpecs[i].lower) * raw / 1000.0
                    : raw * 0.001;
                const QSignalBlocker blocker(cartesian_spins_[i]);
                cartesian_spins_[i]->setValue(value);
                set_cartesian_control(i, value);
            });
        }
        cartesian_grid->setVerticalSpacing(3);
        cartesian_grid->setColumnStretch(3, 1);
        cartesian_grid->setColumnMinimumWidth(4, 16);
        cartesian_grid->setColumnStretch(8, 1);
        cartesian_controls_ = new QWidget(this);
        cartesian_controls_->setLayout(cartesian_grid);
        cartesian_root->addWidget(cartesian_controls_);
        // Speed limits for RT (next packet) and movep; editable in any mode.
        auto *cartesian_speed = new QHBoxLayout();
        cartesian_speed->setSpacing(kRowSpacing);
        cartesian_speed->addWidget(new QLabel(QStringLiteral("笛卡尔速度上限："), this));
        for (size_t i = 0; i < cartesian_limit_spins_.size(); ++i) {
            const auto &spec = kCartesianLimitSpecs[i];
            auto *spin = new QDoubleSpinBox(this);
            spin->setObjectName(QStringLiteral("cartesian_limit_%1").arg(i));
            spin->setRange(spec.lower, spec.upper);
            spin->setDecimals(0);
            spin->setValue(spec.initial);
            spin->setSuffix(QStringLiteral(" ") + QString::fromUtf8(spec.unit));
            spin->setToolTip(QStringLiteral("末端%1上限；关节限速随之等比例设定，仅在奇异附近起作用")
                                 .arg(QString::fromUtf8(spec.name)));
            cartesian_limit_spins_[i] = spin;
            cartesian_speed->addWidget(new QLabel(QString::fromUtf8(spec.name), this));
            cartesian_speed->addWidget(spin);
            connect(spin, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this] {
                std::array<uint16_t, 4> wire{};
                for (size_t k = 0; k < wire.size(); ++k)
                    wire[k] = cartesian_limit_to_wire(k, cartesian_limit_spins_[k]->value());
                controller_.set_cartesian_limits(wire);
            });
        }
        cartesian_speed->addStretch(1);
        cartesian_root->addLayout(cartesian_speed);
        auto *cartesian_actions = new QHBoxLayout();
        cartesian_actions->setSpacing(kRowSpacing);
        cartesian_use_current_ = new QPushButton(QStringLiteral("当前关节 FK → 笛卡尔目标"), this);
        auto *cartesian_use_current = cartesian_use_current_;
        cartesian_use_current->setObjectName(QStringLiteral("cartesian_use_current"));
        auto *cartesian_recenter = new QPushButton(QStringLiteral("姿态滑块归中"), this);
        cartesian_recenter->setObjectName(QStringLiteral("cartesian_recenter"));
        cartesian_execute_ = new QPushButton(QStringLiteral("执行 movep（NRT 100 Hz）"), this);
        cartesian_execute_->setObjectName(QStringLiteral("movep_execute"));
        cartesian_actions->addWidget(cartesian_use_current);
        cartesian_actions->addWidget(cartesian_recenter);
        cartesian_actions->addWidget(cartesian_execute_);
        cartesian_root->addLayout(cartesian_actions);
        cartesian_result_ = new QLabel(QStringLiteral("笛卡尔 IK：尚未求解"), this);
        cartesian_result_->setObjectName(QStringLiteral("cartesian_ik_result"));
        cartesian_result_->setWordWrap(true);
        cartesian_result_->setMinimumHeight(two_lines(cartesian_result_));
        cartesian_result_->setToolTip(QStringLiteral("ΔRx/ΔRy/ΔRz 滑块为相对当前姿态的增量（绕基座固定轴），不是绝对 RPY。\n"
                                                     "主机 IK 只判断可达性：MCU 沿直线到达同一末端位姿，肘部（冗余）可能与主机参考解不同。"));
        cartesian_root->addWidget(cartesian_result_);
        arm_root->addWidget(cartesian_group);
        // NRT session (movej and movep): status and cancel.  The movej-only
        // rows here and the movej columns of the joint table show in NRT
        // Joint only (move_joint_widgets_).
        move_group_ = new QGroupBox(QStringLiteral("NRT 动作（movej / movep）：J8 保持原力位目标，不参与到位判断"), this);
        auto *move_root = new QVBoxLayout(move_group_);
        move_root->setSpacing(4);
        auto *tolerance_row = new QWidget(this);
        auto *tolerances = new QHBoxLayout(tolerance_row);
        tolerances->setContentsMargins(0, 0, 0, 0);
        auto field = [this, tolerances](const QString &label, const char *name, double lo, double hi, double value, int decimals) {
            tolerances->addWidget(new QLabel(label, this));
            auto *spin = new QDoubleSpinBox(this);
            spin->setObjectName(name); spin->setDecimals(decimals); spin->setRange(lo, hi);
            spin->setSingleStep(std::pow(10.0, -decimals)); spin->setValue(value);
            tolerances->addWidget(spin);
            return spin;
        };
        move_position_tolerance_ = field(QStringLiteral("到位误差 rad"), "move_position_tolerance", .001, .05, .01, 3);
        move_velocity_tolerance_ = field(QStringLiteral("到位速度 rad/s"), "move_velocity_tolerance", .01, .1, .05, 3);
        move_dwell_ = field(QStringLiteral("稳定 ms"), "move_dwell", 20, 5000, 100, 0);
        move_timeout_ = field(QStringLiteral("总超时 s"), "move_timeout", .1, 600, 30, 1);
        tolerances->addStretch();
        move_root->addWidget(tolerance_row);
        auto *move_actions = new QHBoxLayout();
        move_actions->setSpacing(kRowSpacing);
        move_execute_ = new QPushButton(QStringLiteral("执行 movej（使用下方七轴目标）"), this);
        move_execute_->setObjectName("move_execute");
        move_cancel_ = new QPushButton(QStringLiteral("停止 move（受控停止并保持）"), this);
        move_cancel_->setToolTip(QStringLiteral("MCU 把机械臂刹停并在 IDLE 保持（不失能）；需要断电时用“全部失能”。"));
        move_cancel_->setObjectName("move_cancel");
        move_cancel_->setStyleSheet(QStringLiteral("background:#f0b0b0"));
        move_copy_actual_ = new QPushButton(QStringLiteral("复制实际位置到目标（不运动）"), this);
        move_copy_actual_->setObjectName("move_copy_actual");
        auto *load_gains = new QPushButton(QStringLiteral("从控制参数加载增益/前馈"), this);
        move_load_gains_ = load_gains;
        move_actions->addWidget(move_execute_); move_actions->addWidget(move_cancel_);
        move_actions->addWidget(move_copy_actual_); move_actions->addWidget(load_gains);
        move_actions->addStretch();
        move_root->addLayout(move_actions);
        auto *move_help = new QLabel(QStringLiteral("滑条只编辑目标，点击执行才运动，到位前不能提交下一条；超时、掉线、取消会请求失能，承重机械臂可能下坠，请做好支撑。"), this);
        move_help->setToolTip(QStringLiteral("流程：失能 → 应用电机模式 → 使能（IDLE 保持）→ 进入 NRT Joint → 编辑目标 → 执行 movej。\n"
            "move 模式滑条只编辑，不立即运动；到位前不能提交下一条。PC 100 Hz 发送固定目标/心跳，MCU 1 kHz 插值；无 RT 包混发。\n"
            "超时、掉线、取消会请求失能，承重机械臂可能下坠，请做好支撑。速度/加速度是规划上限，不是安全限速。"));
        move_help->setWordWrap(true); move_root->addWidget(move_help);
        move_status_ = new QLabel(QStringLiteral("movej 未启动"), this);
        move_status_->setObjectName("move_status"); move_status_->setWordWrap(true);
        move_status_->setMinimumHeight(two_lines(move_status_));
        move_root->addWidget(move_status_);
        move_joint_widgets_ = {tolerance_row, move_execute_, move_copy_actual_, load_gains, move_help};
        arm_root->addWidget(move_group_);

        connect(cartesian_use_current, &QPushButton::clicked, this, [this] {
            sync_cartesian_from_feedback();
            status_->setText(QStringLiteral("已将当前关节 FK 姿态作为笛卡尔目标"));
        });
        connect(cartesian_recenter, &QPushButton::clicked, this, [this] {
            for (int i = 3; i < 6; ++i) {
                const QSignalBlocker blocker(cartesian_spins_[i]);
                cartesian_values_[i] = 0.0;
                cartesian_spins_[i]->setValue(0.0);
                update_cartesian_slider(i);
            }
            cartesian_result_->setText(QStringLiteral("姿态滑块已归中；内部 3×3 目标矩阵保持不变"));
        });
        connect(cartesian_execute_, &QPushButton::clicked, this, [this] {
            auto p = controller_.cartesian_defaults();
            p.pose = cartesian_target_array();
            std::string error;
            if (!controller_.movep(p, error)) show_error(QString::fromStdString(error));
            refresh();
        });

        connect(mode_apply_, &QPushButton::clicked, this, [this] {
            std::string error;
            const uint8_t mode = static_cast<uint8_t>(transport_mode_->currentData().toInt());
            // After a controlled stop, entering the interrupted mode is a resume.
            const auto current = controller_.snapshot();
            if (mode != kControlIdle && controller_.resume_supported() && current.safety.valid &&
                current.safety.stop_reason != infrabot::StopReason::None &&
                static_cast<uint8_t>(current.safety.stop_mode) == mode) {
                resume_after_stop();
                return;
            }
            if (!controller_.set_control_mode(mode, error)) {
                show_error(QString::fromStdString(error));
            } else {
                sync_targets_without_commands();
                show_cartesian_target(controller_.cartesian_target());
                cartesian_preview_q_ = clamped_to_limits(joints_from_mrad(controller_.targets()));
                run_cartesian_ik();
            }
            refresh();
        });
        connect(idle_button_, &QPushButton::clicked, this, [this] {
            std::string error;
            if (!controller_.set_control_mode(kControlIdle, error)) show_error(QString::fromStdString(error));
            else sync_targets_without_commands();
            refresh();
        });
        connect(move_execute_, &QPushButton::clicked, this, [this] {
            if (!controller_.snapshot().motion.can_execute) return;
            const auto parameters = collect_move_parameters();
            QStringList targets;
            for (float q : parameters.target) targets << QString::number(q, 'f', 3);
            if (QMessageBox::question(this, QStringLiteral("执行七轴 movej"),
                QStringLiteral("目标 [rad]：%1\n请确认机械限位、增益及支撑。J8 不随本次动作改变。继续？").arg(targets.join(", "))) != QMessageBox::Yes) return;
            std::string error;
            if (!controller_.movej(parameters, error)) show_error(QString::fromStdString(error));
            refresh();
        });
        connect(move_cancel_, &QPushButton::clicked, this, [this] {
            if (!controller_.cancel_motion()) show_error(QStringLiteral("没有进行中的 move 会话"));
            refresh();
        });
        connect(move_copy_actual_, &QPushButton::clicked, this, [this] {
            const auto state = controller_.snapshot();
            if (!state.connected || state.feedback_age_ms > 100) { show_error(QStringLiteral("实际反馈过期")); return; }
            for (size_t i = 0; i < motion::axes; ++i) {
                const double q = state.position[i] / 1000.0;
                if (q < target_spins_[i]->minimum() || q > target_spins_[i]->maximum()) {
                    show_error(QStringLiteral("J%1 实际位置超出目标输入范围，未复制，请手动确认目标").arg(i + 1)); return;
                }
            }
            for (size_t i = 0; i < motion::axes; ++i) target_spins_[i]->setValue(state.position[i] / 1000.0);
        });
        connect(load_gains, &QPushButton::clicked, this, [this] { load_move_gains(); });

        /* Unified Damiao/MotorBridge operations.  These packets are low-rate
         * and acknowledged by the MCU; the sliders themselves feed the
         * deterministic 1 kHz DMCP v5 whole-arm stream below. */
        auto *unified_box = new QGridLayout();
        unified_box->addWidget(new QLabel(QStringLiteral("统一接口：电机"), this), 0, 0);
        motor_select_ = new QComboBox(this);
        motor_select_->addItem(QStringLiteral("全部"), static_cast<int>(kMotorAll));
        for (int i = 0; i < static_cast<int>(kMotorCount); ++i) {
            motor_select_->addItem(i == 7 ? QStringLiteral("M8 夹爪")
                                           : QStringLiteral("M%1").arg(i + 1), i);
        }
        motor_select_->setCurrentIndex(1);
        unified_box->addWidget(motor_select_, 0, 1);
        unified_box->addWidget(new QLabel(QStringLiteral("模式"), this), 0, 2);
        mode_select_ = new QComboBox(this);
        mode_select_->addItem(QStringLiteral("MIT"), kModeMit);
        mode_select_->addItem(QStringLiteral("位置/速度"), kModePosVel);
        mode_select_->addItem(QStringLiteral("速度"), kModeVel);
        mode_select_->addItem(QStringLiteral("力位混合"), kModeForcePos);
        /* M8 is the DM4310 gripper and its startup profile is
         * force-position.  Selecting it should not silently leave the mode
         * combo at MIT, which was the main source of status=6 (mode mismatch)
         * when the force-position button was pressed. */
        mode_select_->setCurrentIndex(0);
        unified_box->addWidget(mode_select_, 0, 3);
        unified_box->addWidget(new QLabel(QStringLiteral("寄存器 ID"), this), 0, 4);
        register_id_ = new QSpinBox(this);
        register_id_->setRange(0, 255);
        register_id_->setValue(9);
        unified_box->addWidget(register_id_, 0, 5);
        unified_box->addWidget(new QLabel(QStringLiteral("寄存器值(十进制/0x)"), this), 0, 6);
        register_value_ = new QLineEdit(QStringLiteral("0"), this);
        register_value_->setMaximumWidth(130);
        unified_box->addWidget(register_value_, 0, 7);

        unified_box->addWidget(new QLabel(QStringLiteral("位置 (rad)"), this), 1, 0);
        maintenance_position_ = new QDoubleSpinBox(this);
        maintenance_position_->setRange(kJointPositionMinMrad[0] / 1000.0,
                                        kJointPositionMaxMrad[0] / 1000.0);
        maintenance_position_->setDecimals(4);
        maintenance_position_->setSingleStep(0.01);
        unified_box->addWidget(maintenance_position_, 1, 1);
        velocity_label_ = new QLabel(QStringLiteral("MIT目标速度/前馈（非限速）rad/s"), this);
        velocity_label_->setToolTip(QStringLiteral("MIT/速度模式使用；力位混合模式使用“速度上限”"));
        unified_box->addWidget(velocity_label_, 1, 2);
        maintenance_velocity_ = new QDoubleSpinBox(this);
        maintenance_velocity_->setRange(-100.0, 100.0);
        maintenance_velocity_->setDecimals(3);
        maintenance_velocity_->setSingleStep(0.001);
        unified_box->addWidget(maintenance_velocity_, 1, 3);
        unified_box->addWidget(new QLabel(QStringLiteral("Kp"), this), 1, 4);
        maintenance_kp_ = new QDoubleSpinBox(this);
        maintenance_kp_->setRange(0.0, 500.0);
        maintenance_kp_->setDecimals(3);
        maintenance_kp_->setSingleStep(0.1);
        maintenance_kp_->setValue(1.0);
        unified_box->addWidget(maintenance_kp_, 1, 5);
        unified_box->addWidget(new QLabel(QStringLiteral("Kd"), this), 1, 6);
        maintenance_kd_ = new QDoubleSpinBox(this);
        maintenance_kd_->setRange(0.0, 5.0);
        maintenance_kd_->setDecimals(3);
        maintenance_kd_->setSingleStep(0.01);
        maintenance_kd_->setValue(0.1);
        unified_box->addWidget(maintenance_kd_, 1, 7);

        unified_box->addWidget(new QLabel(QStringLiteral("力/转矩 (Nm)"), this), 2, 0);
        maintenance_torque_ = new QDoubleSpinBox(this);
        maintenance_torque_->setRange(-54.0, 54.0);
        maintenance_torque_->setDecimals(4);
        maintenance_torque_->setSingleStep(0.01);
        unified_box->addWidget(maintenance_torque_, 2, 1);
        velocity_limit_label_ = new QLabel(QStringLiteral("速度上限 rad/s"), this);
        velocity_limit_label_->setToolTip(QStringLiteral("位置/速度、力位混合模式使用"));
        unified_box->addWidget(velocity_limit_label_, 2, 2);
        maintenance_velocity_limit_ = new QDoubleSpinBox(this);
        maintenance_velocity_limit_->setRange(0.0, 100.0);
        maintenance_velocity_limit_->setDecimals(3);
        maintenance_velocity_limit_->setSingleStep(0.001);
        maintenance_velocity_limit_->setValue(1.0);
        unified_box->addWidget(maintenance_velocity_limit_, 2, 3);
        unified_box->addWidget(new QLabel(QStringLiteral("力矩比例 (0..1)"), this), 2, 4);
        maintenance_torque_ratio_ = new QDoubleSpinBox(this);
        maintenance_torque_ratio_->setRange(0.0, 1.0);
        maintenance_torque_ratio_->setDecimals(3);
        maintenance_torque_ratio_->setSingleStep(0.01);
        /* The legacy M8 gripper trajectory uses the full normalized current
         * ratio.  0.1 was too small to overcome the mechanism friction and
         * made a valid force-position frame look ineffective. */
        maintenance_torque_ratio_->setValue(1.0);
        unified_box->addWidget(maintenance_torque_ratio_, 2, 5);
        unified_box->addWidget(new QLabel(QStringLiteral("CAN 超时 ms"), this), 2, 6);
        can_timeout_ms_ = new QSpinBox(this);
        can_timeout_ms_->setRange(1, 60000);
        can_timeout_ms_->setValue(100);
        unified_box->addWidget(can_timeout_ms_, 2, 7);

        connect(motor_select_, qOverload<int>(&QComboBox::currentIndexChanged), this,
                [this](int) {
                    if (!motor_select_ || !mode_select_) return;
                    const int motor = motor_select_->currentData().toInt();
                    if (motor == 7) mode_select_->setCurrentIndex(3);
                    else if (motor >= 0 && motor < static_cast<int>(kMotorCount))
                        mode_select_->setCurrentIndex(0);
                    if (maintenance_position_) {
                        if (motor >= 0 && motor < static_cast<int>(kMotorCount)) {
                            maintenance_position_->setRange(
                                kJointPositionMinMrad[static_cast<size_t>(motor)] / 1000.0,
                                kJointPositionMaxMrad[static_cast<size_t>(motor)] / 1000.0);
                        } else {
                            maintenance_position_->setRange(-12.5, 12.5);
                        }
                    }
                });
        connect(mode_select_, qOverload<int>(&QComboBox::currentIndexChanged), this,
                [this](int) { update_velocity_controls(); });
        update_velocity_controls();

        auto add_unified_button = [this, unified_box](const QString &text, int row, int column,
                                                       auto callback) {
            auto *button = new QPushButton(text, this);
            unified_box->addWidget(button, row, column);
            connect(button, &QPushButton::clicked, this, callback);
        };
        auto selected_motor = [this] {
            return static_cast<uint8_t>(motor_select_->currentData().toInt());
        };
        add_unified_button(QStringLiteral("使能(选中)"), 3, 0, [this, selected_motor] {
            if (!controller_.enable_motor(selected_motor())) show_error(QStringLiteral("发送使能失败"));
        });
        add_unified_button(QStringLiteral("失能(选中)"), 3, 1, [this, selected_motor] {
            if (!controller_.disable_motor(selected_motor())) show_error(QStringLiteral("发送失能失败"));
        });
        add_unified_button(QStringLiteral("停止"), 3, 2, [this, selected_motor] {
            if (!controller_.stop_motor(selected_motor())) show_error(QStringLiteral("发送停止失败"));
        });
        add_unified_button(QStringLiteral("清除错误"), 3, 3, [this, selected_motor] {
            if (!controller_.clear_error(selected_motor())) show_error(QStringLiteral("清除错误发送失败"));
        });
        add_unified_button(QStringLiteral("设零(选中)"), 3, 4, [this, selected_motor] {
            if (QMessageBox::question(this, QStringLiteral("永久设零"),
                QStringLiteral("请支撑机械臂并全部失能。将当前位置写入选中电机零点，确认？")) != QMessageBox::Yes) return;
            if (!controller_.set_zero_motor(selected_motor())) show_error(QStringLiteral("发送设零失败"));
        });
        add_unified_button(QStringLiteral("确保模式"), 3, 5, [this, selected_motor] {
            if (!controller_.ensure_mode(selected_motor(), static_cast<uint8_t>(mode_select_->currentData().toInt())))
                show_error(QStringLiteral("发送模式失败"));
        });
        add_unified_button(QStringLiteral("请求反馈"), 3, 6, [this, selected_motor] {
            if (!controller_.request_feedback(selected_motor())) show_error(QStringLiteral("请求反馈失败"));
        });
        add_unified_button(QStringLiteral("读取状态"), 3, 7, [this, selected_motor] {
            if (!controller_.get_state(selected_motor())) show_error(QStringLiteral("读取状态失败"));
        });
        add_unified_button(QStringLiteral("应用到1kHz整臂流"), 6, 0, [this, selected_motor] {
            const uint8_t motor = selected_motor();
            if (motor == kMotorAll || motor >= kMotorCount) {
                show_error(QStringLiteral("请选择单个电机后再应用参数"));
                return;
            }
            if (!controller_.apply_joint_parameters(
                    motor,
                    velocity_rad_s_to_mrad_s(maintenance_velocity_->value()),
                    static_cast<int>(std::lround(maintenance_kp_->value() * 1000.0)),
                    static_cast<int>(std::lround(maintenance_kd_->value() * 1000.0)),
                    torque_nm_to_mnm(maintenance_torque_->value()),
                    velocity_rad_s_to_mrad_s(maintenance_velocity_limit_->value()),
                    ratio_to_milli(maintenance_torque_ratio_->value()))) {
                show_error(QStringLiteral("应用整臂流参数失败"));
            }
        });
        add_unified_button(QStringLiteral("发送 MIT"), 4, 0, [this, selected_motor] {
            if (!controller_.send_mit_once(selected_motor(), position_rad_to_mrad(maintenance_position_->value()),
                                            velocity_rad_s_to_mrad_s(maintenance_velocity_->value()),
                                            static_cast<int>(std::lround(maintenance_kp_->value() * 1000.0)),
                                            static_cast<int>(std::lround(maintenance_kd_->value() * 1000.0)),
                                            torque_nm_to_mnm(maintenance_torque_->value())))
                show_error(QStringLiteral("发送 MIT 失败"));
        });
        add_unified_button(QStringLiteral("发送位置/速度"), 4, 1, [this, selected_motor] {
            if (!controller_.send_pos_vel(selected_motor(), position_rad_to_mrad(maintenance_position_->value()),
                                           velocity_rad_s_to_mrad_s(maintenance_velocity_limit_->value())))
                show_error(QStringLiteral("发送位置/速度失败"));
        });
        add_unified_button(QStringLiteral("发送速度"), 4, 2, [this, selected_motor] {
            if (!controller_.send_vel(selected_motor(), velocity_rad_s_to_mrad_s(maintenance_velocity_->value())))
                show_error(QStringLiteral("发送速度失败"));
        });
        add_unified_button(QStringLiteral("发送力位混合"), 4, 3, [this, selected_motor] {
            if (!controller_.send_force_pos(selected_motor(), position_rad_to_mrad(maintenance_position_->value()),
                                             velocity_rad_s_to_mrad_s(maintenance_velocity_limit_->value()),
                                             ratio_to_milli(maintenance_torque_ratio_->value())))
                show_error(QStringLiteral("发送力位混合失败"));
        });
        add_unified_button(QStringLiteral("设置 CAN 超时"), 4, 4, [this, selected_motor] {
            if (!controller_.set_can_timeout_ms(selected_motor(), static_cast<uint32_t>(can_timeout_ms_->value())))
                show_error(QStringLiteral("设置 CAN 超时失败"));
        });
        add_unified_button(QStringLiteral("保存电机参数"), 4, 5, [this, selected_motor] {
            if (QMessageBox::question(this, QStringLiteral("保存参数"),
                QStringLiteral("将选中电机当前参数写入电机 Flash，需全部失能；确认？")) != QMessageBox::Yes) return;
            if (!controller_.store_parameters(selected_motor())) show_error(QStringLiteral("保存参数失败"));
        });
        add_unified_button(QStringLiteral("读寄存器"), 4, 6, [this, selected_motor] {
            const uint8_t type = static_cast<uint8_t>(register_type_->currentData().toInt());
            const auto *info = infrabot_register_info(static_cast<uint8_t>(register_id_->value()));
            if (!info || info->type != type) {
                show_error(QString::fromStdString(infrabot::register_summary(static_cast<uint8_t>(register_id_->value()))) +
                           QStringLiteral("：寄存器未知或类型不符"));
                return;
            }
            if (!controller_.get_register(selected_motor(), static_cast<uint8_t>(register_id_->value()), type))
                show_error(QStringLiteral("读寄存器发送失败"));
        });
        add_unified_button(QStringLiteral("写寄存器"), 4, 7, [this, selected_motor] {
            bool ok = false;
            const uint8_t type = static_cast<uint8_t>(register_type_->currentData().toInt());
            uint32_t value = 0;
            if (type == 0) value = register_value_->text().trimmed().toUInt(&ok, 0);
            else {
                const float f = register_value_->text().trimmed().toFloat(&ok);
                ok = ok && std::isfinite(f);
                std::memcpy(&value, &f, 4);
            }
            if (!ok) {
                show_error(QStringLiteral("寄存器值格式错误"));
                return;
            }
            // The register table (protocol.h), as the MCU checks it.
            const auto rid = static_cast<uint8_t>(register_id_->value());
            const auto *info = infrabot_register_info(rid);
            const QString summary = QString::fromStdString(infrabot::register_summary(rid));
            if (!info || info->type != type || !infrabot_register_write_allowed(rid, value)) {
                show_error(QStringLiteral("不能写入：%1%2").arg(summary)
                    .arg(info && info->type == type && info->access == INFRABOT_REGISTER_WRITABLE
                         ? QStringLiteral("；值 %1 超出范围").arg(QString::fromStdString(infrabot::register_value_text(rid, value)))
                         : QString()));
                return;
            }
            if (QMessageBox::question(this, QStringLiteral("写寄存器"),
                QStringLiteral("确认写入选中电机 %1 = %2？需全部失能。")
                    .arg(summary).arg(QString::fromStdString(infrabot::register_value_text(rid, value)))) != QMessageBox::Yes) return;
            if (!controller_.write_register(selected_motor(), static_cast<uint8_t>(register_id_->value()), value, type))
                show_error(QStringLiteral("写寄存器发送失败"));
        });
        register_type_ = new QComboBox(this);
        register_type_->addItem(QStringLiteral("uint32 (十进制/0x)"), 0);
        register_type_->addItem(QStringLiteral("float32 (十进制)"), 1);
        unified_box->addWidget(new QLabel(QStringLiteral("寄存器类型"), this), 5, 0);
        unified_box->addWidget(register_type_, 5, 1, 1, 2);
        // Type, name and range from the register table (protocol.h).
        register_info_ = new QLabel(this);
        register_info_->setObjectName("register_info");
        unified_box->addWidget(register_info_, 6, 1, 1, 7);
        const auto show_register = [this](int rid) {
            const auto *info = infrabot_register_info(static_cast<uint8_t>(rid));
            if (info) register_type_->setCurrentIndex(info->type == INFRABOT_REGISTER_U32 ? 0 : 1);
            register_info_->setText(QString::fromStdString(infrabot::register_summary(static_cast<uint8_t>(rid))));
        };
        connect(register_id_, qOverload<int>(&QSpinBox::valueChanged), this, show_register);
        show_register(register_id_->value());
        auto *help = new QLabel(QStringLiteral("整臂控制：先全部失能，再应用电机模式（M1-M7 为 MIT 或位置/速度，M8 固定力位），然后使能进入 IDLE。显式进入 RT Joint 后滑条才立即生效。"
            "每个电机选中后可用‘应用到1kHz整臂流’设置其速度、Kp/Kd、转矩、速度上限和力矩比例；滑块目标与这些参数一起以 DMCP v5 发送。"
            "电机模式在任何电机使能后均不能修改（包括确保模式/RID10）；单电机运动也仅在 RT Joint 允许。"
            "单次目标由 MCU 以 1 kHz 保持；PC 心跳断开 100 ms（关闭 GUI、拔网线）后 MCU 受控停止并保持，不失能（新固件；旧固件会失能）。停止为当前模式保持/零速度，不是急停。"
            "设零/保存前先请求反馈。高级目标优先于滑块；‘全部使能’只使能并保持当前位置，‘机械臂慢速回到0位’才会执行回零。该按钮在 servej 和 movej 均可用。"), this);
        help->setWordWrap(true);
        unified_box->addWidget(help, 5, 3, 1, 5);
        motor_root->addLayout(unified_box);
        maintenance_status_ = new QLabel(QStringLiteral("统一接口尚无响应"), this);
        maintenance_status_->setWordWrap(true);
        motor_root->addWidget(maintenance_status_);
        motor_root->addStretch(1);

        // One table: target and actual per joint, and in NRT Joint the movej
        // parameters of J1-J7 beside them.
        auto *joints = new QGridLayout();
        joints->setHorizontalSpacing(kRowSpacing);
        joints->addWidget(new QLabel(QStringLiteral("关节"), this), 0, 0);
        joints->addWidget(new QLabel(QStringLiteral("目标位置 (rad)"), this), 0, 1, 1, 2);
        joints->addWidget(new QLabel(QStringLiteral("实际位置 (rad)"), this), 0, 3);
        joints->addWidget(new QLabel(QStringLiteral("实际值"), this), 0, 4);
        auto *torque_header = new QLabel(QStringLiteral("实际力矩 (N·m)"), this);
        torque_header->setToolTip(QStringLiteral("遥测实测力矩（关节坐标），约 30 Hz 刷新，显示两次刷新之间 1 kHz 采样的平均值。\n"
                                                 "条的满量程为电机峰值力矩（不超过驱动器力矩反馈范围），虚线为额定力矩。"));
        joints->addWidget(torque_header, 0, 5, 1, 2);
        const std::array<std::pair<const char *, const char *>, 5> move_columns = {{
            {"速度 rad/s", "movej 速度上限 rad/s"}, {"加速度 rad/s²", "movej 加速度上限 rad/s²"},
            {"Kp", "MIT Kp Nm/rad"}, {"Kd", "MIT Kd Nm/(rad/s)"}, {"前馈 Nm", "前馈力矩 Nm"}}};
        for (size_t c = 0; c < move_columns.size(); ++c) {
            auto *label = new QLabel(QString::fromUtf8(move_columns[c].first), this);
            label->setToolTip(QString::fromUtf8(move_columns[c].second));
            joints->addWidget(label, 0, static_cast<int>(8 + c));
            move_joint_widgets_.push_back(label);
        }
        const auto defaults = controller_.move_defaults();
        constexpr double v_limits[] = {45, 45, 10, 10, 30, 30, 30};
        constexpr double t_limits[] = {54, 54, 28, 28, 10, 10, 10};
        for (size_t i = 0; i < kMotorCount; ++i) {
            const int minimum = kJointPositionMinMrad[i];
            const int maximum = kJointPositionMaxMrad[i];
            auto *name = new QLabel(i == 7 ? QStringLiteral("M8 夹爪")
                                           : QStringLiteral("M%1").arg(static_cast<int>(i + 1)), this);
            auto *slider = new QSlider(Qt::Horizontal, this);
            if (i == kGripperIndex) slider->setObjectName("gripper_slider");
            slider->setRange(minimum, maximum);
            slider->setSingleStep(10);
            slider->setPageStep(100);
            slider->setValue(0);
            auto *spin = new QDoubleSpinBox(this);
            spin->setObjectName(QStringLiteral("joint_target_%1").arg(i + 1));
            spin->setRange(minimum / 1000.0, maximum / 1000.0);
            spin->setDecimals(3);
            spin->setSingleStep(0.01);
            auto *bar = new QProgressBar(this);
            bar->setRange(minimum, maximum);
            bar->setValue(0);
            bar->setTextVisible(false);
            auto *actual = new QLabel(QStringLiteral("0"), this);
            actual->setMinimumWidth(72);
            actual->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
            auto *torque_bar = new TorqueBar(kRatedTorqueNm[i], kTorqueScaleNm[i], this);
            torque_bar->setObjectName(QStringLiteral("torque_bar_%1").arg(i + 1));
            auto *torque = new QLabel(QStringLiteral("--"), this);
            torque->setObjectName(QStringLiteral("torque_value_%1").arg(i + 1));
            torque->setMinimumWidth(fontMetrics().horizontalAdvance(QStringLiteral("-00.00")) + 8);
            torque->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
            sliders_[i] = slider;
            target_spins_[i] = spin;
            actual_bars_[i] = bar;
            actual_labels_[i] = actual;
            torque_bars_[i] = torque_bar;
            torque_labels_[i] = torque;
            const int row = static_cast<int>(i + 1);
            joints->addWidget(name, row, 0);
            joints->addWidget(slider, row, 1);
            joints->addWidget(spin, row, 2);
            joints->addWidget(bar, row, 3);
            joints->addWidget(actual, row, 4);
            joints->addWidget(torque_bar, row, 5);
            joints->addWidget(torque, row, 6);
            if (i < motion::axes) {
                const double lo[] = {.001, .001, .001, 0, -t_limits[i]};
                const double hi[] = {std::min(double(v_limits[i]), double(INFRABOT_ARM_MAX_JOINT_VELOCITY)), 10, 500, 5, t_limits[i]};
                const double initial[] = {.1, .2, defaults.kp[i], defaults.kd[i], defaults.torque[i]};
                for (size_t c = 0; c < 5; ++c) {
                    auto *parameter = new QDoubleSpinBox(this);
                    parameter->setObjectName(QStringLiteral("move_param_%1_%2").arg(i).arg(c));
                    parameter->setDecimals(3); parameter->setRange(lo[c], hi[c]);
                    parameter->setSingleStep(c < 2 ? .01 : .1); parameter->setValue(initial[c]);
                    move_parameters_[i][c] = parameter;
                    joints->addWidget(parameter, row, static_cast<int>(8 + c));
                    move_joint_widgets_.push_back(parameter);
                }
            }
            connect(slider, &QSlider::valueChanged, this, [this, i, spin](int value) {
                spin->setValue(value / 1000.0);
                if (i != kGripperIndex) { controller_.set_target(i, value); return; }
                gripper_position_ = value; // sent by flush_gripper(), in any mode
                gripper_dirty_ = true;
            });
            connect(spin, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
                    [slider](double value) { slider->setValue(static_cast<int>(std::lround(value * 1000.0))); });
        }
        // The gripper's own command, in every control mode: under its position
        // slider, a slider and a value each for its speed limit and its force
        // (the drive's current limit).
        {
            const auto [speed_mrad_s, force_milli] = controller_.gripper_defaults();
            struct Setting {
                const char *name, *unit, *tooltip;
                double lower, upper, step, initial;
                QDoubleSpinBox *&spin;
                QSlider *&slider;
            };
            const Setting settings[] = {
                {"夹爪速度", "rad/s", "夹爪最大速度（驱动器 v_des），0.01–10 rad/s", 0.01, 10.0, 0.1,
                 std::clamp(speed_mrad_s / 1000.0, 0.01, 10.0), gripper_speed_, gripper_speed_slider_},
                {"夹爪力", "× 最大电流", "夹紧力 = 电流上限（驱动器最大相电流的比例，0.01–1，步长 0.01）：\n"
                 "碰到物体停住后最多以该电流推压，越小夹得越轻；实际力矩见 M8 行的力矩条。", 0.01, 1.0, 0.01,
                 std::clamp(force_milli / 1000.0, 0.01, 1.0), gripper_force_, gripper_force_slider_},
            };
            int row = kGripperIndex + 2;
            for (const auto &setting : settings) {
                auto *label = new QLabel(QString::fromUtf8(setting.name), this);
                setting.slider = new QSlider(Qt::Horizontal, this);
                setting.slider->setRange(1, static_cast<int>(std::lround(setting.upper * 100.0))); // 0.01 steps
                setting.slider->setPageStep(10);
                setting.spin = new QDoubleSpinBox(this);
                setting.spin->setRange(setting.lower, setting.upper);
                setting.spin->setDecimals(2);
                setting.spin->setSingleStep(setting.step);
                setting.spin->setValue(setting.initial);
                setting.slider->setValue(static_cast<int>(std::lround(setting.initial * 100.0)));
                for (QWidget *widget : std::initializer_list<QWidget *>{label, setting.slider, setting.spin})
                    widget->setToolTip(QString::fromUtf8(setting.tooltip));
                auto *slider = setting.slider;
                auto *spin = setting.spin;
                connect(slider, &QSlider::valueChanged, this, [this, spin](int raw) {
                    const QSignalBlocker blocker(spin);
                    spin->setValue(raw / 100.0);
                    gripper_dirty_ = true;
                });
                connect(spin, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this, slider](double value) {
                    const QSignalBlocker blocker(slider);
                    slider->setValue(static_cast<int>(std::lround(value * 100.0)));
                    gripper_dirty_ = true;
                });
                joints->addWidget(label, row, 0);
                joints->addWidget(slider, row, 1);
                joints->addWidget(spin, row, 2);
                auto *unit = new QLabel(QString::fromUtf8(setting.unit), this);
                if (row == kGripperIndex + 2) {
                    // The last gripper command's problem, if any, after the unit.
                    auto *box = new QWidget(this);
                    auto *line = new QHBoxLayout(box);
                    line->setContentsMargins(0, 0, 0, 0);
                    line->addWidget(unit);
                    gripper_status_ = new QLabel(this);
                    gripper_status_->setObjectName("gripper_status");
                    gripper_status_->setStyleSheet(QStringLiteral("color:#b00020"));
                    line->addWidget(gripper_status_, 1);
                    joints->addWidget(box, row, 3, 1, 4);
                } else {
                    joints->addWidget(unit, row, 3, 1, 4);
                }
                ++row;
            }
            gripper_speed_->setObjectName("gripper_speed");
            gripper_speed_slider_->setObjectName("gripper_speed_slider");
            gripper_force_->setObjectName("gripper_force");
            gripper_force_slider_->setObjectName("gripper_force_slider");
            gripper_timer_ = new QTimer(this);
            connect(gripper_timer_, &QTimer::timeout, this, [this] { flush_gripper(); });
            gripper_timer_->start(20);
        }
        joints->setColumnStretch(1, 3);
        joints->setColumnStretch(3, 2);
        joints->setColumnStretch(5, 2);
        joints->setColumnMinimumWidth(7, 12);
        joints->setVerticalSpacing(3);
        torque_timer_ = new QTimer(this);
        connect(torque_timer_, &QTimer::timeout, this, [this] { refresh_torque(); });
        torque_timer_->start(33);
        arm_root->addLayout(joints);

        motor_state_ = new QLabel(this);
        motor_state_->setWordWrap(true);
        motor_state_->setMinimumHeight(two_lines(motor_state_));
        arm_root->addWidget(motor_state_);
        arm_root->addStretch(1);
        status_ = new QLabel(QStringLiteral("未连接。连接本身不会使能；确认机械范围后再操作。"), this);
        status_->setWordWrap(true);
        status_->setMinimumHeight(55);
        root->addWidget(status_);

        connect(connect_button_, &QPushButton::clicked, this, [this] { reconnect(); });
        connect(zero_enable, &QPushButton::clicked, this, [this] {
            if (QMessageBox::question(this, QStringLiteral("全部设零并使能"),
                QStringLiteral("会修改所有电机永久零点并使能，请支撑机械臂并确认当前位置。继续？")) != QMessageBox::Yes) return;
            if (!controller_.set_zero_and_enable()) show_error(QStringLiteral("设零并使能发送失败"));
        });
        connect(enable, &QPushButton::clicked, this, [this] { enable_group(kAllMask); });
        connect(enable_arm_, &QPushButton::clicked, this, [this] { enable_group(kArmMask); });
        connect(enable_gripper_, &QPushButton::clicked, this, [this] { enable_group(kGripperMask); });
        connect(disable_arm_, &QPushButton::clicked, this, [this] {
            if (!controller_.disable_group(kArmMask)) show_error(QStringLiteral("机械臂失能发送失败"));
            refresh();
        });
        connect(disable_gripper_, &QPushButton::clicked, this, [this] {
            if (!controller_.disable_group(kGripperMask)) show_error(QStringLiteral("夹爪失能发送失败"));
            refresh();
        });
        connect(zero, &QPushButton::clicked, this, [this] {
            if (QMessageBox::question(this, QStringLiteral("全部永久设零"),
                QStringLiteral("会失能所有电机并修改永久零点，请支撑机械臂。继续？")) != QMessageBox::Yes) return;
            if (!controller_.set_zero()) show_error(QStringLiteral("设零发送失败"));
        });
        connect(disable, &QPushButton::clicked, this, [this] {
            if (!controller_.disable()) show_error(QStringLiteral("失能发送失败"));
        });
        connect(resume_button_, &QPushButton::clicked, this, [this] { resume_after_stop(); });
        connect(stop_button_, &QPushButton::clicked, this, [this] {
            std::string error;
            if (!controller_.controlled_stop(error)) show_error(QString::fromStdString(error));
            else status_->setText(QStringLiteral("受控停止：机械臂已刹停，在 IDLE 保持（仍使能）"));
            refresh();
        });
        connect(home_slow, &QPushButton::clicked, this, [this] {
            std::string error;
            if (!controller_.home_slow(error)) show_error(QString::fromStdString(error));
            else status_->setText(QStringLiteral("已开始慢速回零：目标 0 位，速度约 0.1 rad/s；可随时点击全部失能"));
        });
        connect(set_arm_mode, &QPushButton::clicked, this, [this] {
            const auto state = controller_.snapshot();
            if (state.enabled_mask || state.control_mode != kControlIdle) {
                show_error(QStringLiteral("请先全部失能，再切换整臂模式"));
                return;
            }
            const uint8_t mode = static_cast<uint8_t>(arm_mode_select_->currentData().toInt());
            std::string error;
            if (!controller_.set_arm_mode(mode, error))
                show_error(QStringLiteral("电机模式配置失败，未允许使能：%1").arg(QString::fromStdString(error)));
            refresh();
        });
        connect(arm_scan, &QPushButton::clicked, this, [this] {
            if (!controller_.scan_all_joints()) show_error(QStringLiteral("扫描全部关节请求失败"));
            else status_->setText(QStringLiteral("已请求扫描全部关节；等待遥测反馈和 DMAR 应答"));
        });
        connect(arm_defaults, &QPushButton::clicked, this, [this] {
            // Resolve the bundled template without embedding a machine-specific
            // absolute path.  This works both from the source directory and
            // when the GUI is launched as build/udp_control_gui.
            const QDir app_dir(QCoreApplication::applicationDirPath());
            const QStringList candidates = {
                QDir::current().filePath(QStringLiteral("infrabot.tsv")),
                app_dir.filePath(QStringLiteral("infrabot.tsv")),
                app_dir.filePath(QStringLiteral("../config/infrabot.tsv"))
            };
            QString path;
            for (const QString &candidate : candidates) {
                const QString clean = QDir::cleanPath(candidate);
                if (QFileInfo(clean).isFile()) {
                    path = clean;
                    break;
                }
            }
            if (path.isEmpty()) {
                show_error(QStringLiteral("未找到项目内置模板 infrabot.tsv，请重新构建 udp_control_gui"));
                return;
            }
            if (QMessageBox::question(this, QStringLiteral("写入电机 Flash 默认参数"),
                QStringLiteral("将先全部失能，然后把 infrabot.tsv 模板参数写入并保存到 8 台电机 Flash。\n确认机械臂处于安全状态并继续？")) != QMessageBox::Yes)
                return;
            status_->setText(QStringLiteral("正在逐个写入电机 Flash，请勿断电…"));
            QApplication::setOverrideCursor(Qt::WaitCursor);
            std::string error;
            const bool ok = controller_.import_motorbridge_parameters(
                path.toStdString(), error);
            QApplication::restoreOverrideCursor();
            if (!ok) show_error(QString::fromStdString(error));
            else status_->setText(QStringLiteral("电机 Flash 默认参数已写入并保存（J1-J8）"));
        });
        connect(arm_import_control_template, &QPushButton::clicked, this, [this] {
            const QDir app_dir(QCoreApplication::applicationDirPath());
            const QStringList candidates = {
                QDir::current().filePath(QStringLiteral("control_template.tsv")),
                app_dir.filePath(QStringLiteral("control_template.tsv")),
                app_dir.filePath(QStringLiteral("../config/control_template.tsv"))
            };
            QString path;
            for (const QString &candidate : candidates) {
                const QString clean = QDir::cleanPath(candidate);
                if (QFileInfo(clean).isFile()) { path = clean; break; }
            }
            if (path.isEmpty()) {
                show_error(QStringLiteral("未找到项目内置控制模板 control_template.tsv，请重新构建 udp_control_gui"));
                return;
            }
            std::string error;
            if (!controller_.import_arm_parameters(path.toStdString(), error))
                show_error(QString::fromStdString(error));
            else {
                load_move_gains();
                uint8_t save_status = 0;
                if (!controller_.store_control_profile_confirmed(save_status, error))
                    show_error(QString::fromStdString(error));
                else
                    status_->setText(QStringLiteral("控制模板已写入 MCU Flash，重启后仍保留"));
            }
        });
        connect(arm_import_control, &QPushButton::clicked, this, [this] {
            const QString path = QFileDialog::getOpenFileName(this, QStringLiteral("导入控制参数"),
                QString(), QStringLiteral("控制参数 (*.tsv *.txt);;所有文件 (*)"));
            if (path.isEmpty()) return;
            std::string error;
            if (!controller_.import_arm_parameters(path.toStdString(), error))
                show_error(QString::fromStdString(error));
            else {
                load_move_gains();
                uint8_t save_status = 0;
                if (!controller_.store_control_profile_confirmed(save_status, error))
                    show_error(QString::fromStdString(error));
                else
                    status_->setText(QStringLiteral("控制参数已写入 MCU Flash：%1").arg(path));
            }
        });
        connect(arm_export_control, &QPushButton::clicked, this, [this] {
            const QString path = QFileDialog::getSaveFileName(this, QStringLiteral("导出控制参数"),
                QStringLiteral("arm_control_parameters.tsv"), QStringLiteral("控制参数 (*.tsv);;文本文件 (*.txt)"));
            if (path.isEmpty()) return;
            std::string error;
            if (!controller_.export_arm_parameters(path.toStdString(), error))
                show_error(QString::fromStdString(error));
            else
                status_->setText(QStringLiteral("整臂控制参数已导出：%1").arg(path));
        });
        connect(arm_flash_export, &QPushButton::clicked, this, [this] {
            const QDir app_dir(QCoreApplication::applicationDirPath());
            const QStringList candidates = {
                QDir::current().filePath(QStringLiteral("infrabot.tsv")),
                app_dir.filePath(QStringLiteral("infrabot.tsv")),
                app_dir.filePath(QStringLiteral("../config/infrabot.tsv"))
            };
            QString template_path;
            for (const QString &candidate : candidates) {
                const QString clean = QDir::cleanPath(candidate);
                if (QFileInfo(clean).isFile()) { template_path = clean; break; }
            }
            if (template_path.isEmpty()) {
                show_error(QStringLiteral("未找到项目内置 Flash 模板 infrabot.tsv，请重新构建 udp_control_gui"));
                return;
            }
            const QString path = QFileDialog::getSaveFileName(this, QStringLiteral("导出整臂电机 Flash 参数"),
                QStringLiteral("infrabot-current.tsv"), QStringLiteral("电机 Flash 参数 (*.tsv);;文本文件 (*.txt)"));
            if (path.isEmpty()) return;
            if (QMessageBox::question(this, QStringLiteral("读取整臂电机 Flash 参数"),
                QStringLiteral("将先全部失能，然后逐个读取 J1-J8 的电机 Flash RID 参数并导出。确认继续？")) != QMessageBox::Yes)
                return;
            status_->setText(QStringLiteral("正在读取整臂电机 Flash 参数，请勿断电…"));
            QApplication::setOverrideCursor(Qt::WaitCursor);
            std::string error;
            const bool ok = controller_.export_motorbridge_parameters(
                template_path.toStdString(), path.toStdString(), error);
            QApplication::restoreOverrideCursor();
            if (!ok) show_error(QString::fromStdString(error));
            else status_->setText(QStringLiteral("整臂电机 Flash 参数已导出：%1").arg(path));
        });

        refresh_timer_ = new QTimer(this);
        connect(refresh_timer_, &QTimer::timeout, this, [this] { refresh(); });
        refresh_timer_->start(100);
        refresh(); // the status labels get their texts before fit_to_screen() measures
        fit_to_screen();
        if (auto_connect) reconnect();
        refresh();
    }

protected:
    // Closing the window does not remove power: an enabled arm stops in a
    // controlled way and holds (UdpController::stop).
    void closeEvent(QCloseEvent *event) override {
        controller_.stop();
        event->accept();
    }

private:
    motion::Parameters collect_move_parameters() const {
        motion::Parameters p;
        p.timeout_ms = static_cast<uint32_t>(std::lround(move_timeout_->value() * 1000));
        p.dwell_ms = static_cast<uint32_t>(move_dwell_->value());
        p.position_tolerance = static_cast<float>(move_position_tolerance_->value());
        p.velocity_tolerance = static_cast<float>(move_velocity_tolerance_->value());
        for (size_t i = 0; i < motion::axes; ++i) {
            p.target[i] = static_cast<float>(target_spins_[i]->value());
            p.velocity_max[i] = static_cast<float>(move_parameters_[i][0]->value());
            p.acceleration_max[i] = static_cast<float>(move_parameters_[i][1]->value());
            p.kp[i] = static_cast<float>(move_parameters_[i][2]->value());
            p.kd[i] = static_cast<float>(move_parameters_[i][3]->value());
            p.torque[i] = static_cast<float>(move_parameters_[i][4]->value());
        }
        return p;
    }

    void load_move_gains() {
        const auto p = controller_.move_defaults();
        for (size_t i = 0; i < motion::axes; ++i) {
            move_parameters_[i][2]->setValue(p.kp[i]);
            move_parameters_[i][3]->setValue(p.kd[i]);
            move_parameters_[i][4]->setValue(p.torque[i]);
        }
    }

    void resume_after_stop() {
        std::string error;
        uint8_t mode = kControlIdle;
        if (!controller_.resume(error, mode)) {
            show_error(QStringLiteral("恢复失败：%1").arg(QString::fromStdString(error)));
            refresh();
            return;
        }
        if (mode != kControlIdle) {
            const int index = transport_mode_->findData(mode);
            if (index >= 0) transport_mode_->setCurrentIndex(index);
        }
        if (mode == kControlRtJoint || mode == kControlRtCartesian) {
            // RT streams the sliders at once: they start where the arm stopped.
            sync_targets_without_commands();
            cartesian_preview_q_ = clamped_to_limits(joints_from_mrad(controller_.targets()));
        }
        // NRT keeps the joint drafts (the interrupted movej target); the
        // Cartesian widgets show the controller's target (the movep draft).
        show_cartesian_target(controller_.cartesian_target());
        run_cartesian_ik();
        status_->setText(mode == kControlIdle
            ? QStringLiteral("已确认停止，机械臂在 IDLE 保持")
            : QStringLiteral("已恢复到 %1：从停下的位置继续%2")
                  .arg(QString::fromUtf8(infrabot::mode_text(static_cast<infrabot::Mode>(mode))))
                  .arg(mode == kControlNrtJoint || mode == kControlNrtCartesian
                       ? QStringLiteral("；被打断的目标保留在草稿中，确认“执行”即继续") : QString()));
        refresh();
    }

    void show_error(const QString &message) {
        log_line("UI error: " + message.toStdString());
        status_->setText(message);
        QMessageBox::warning(this, QStringLiteral("UDP 控制"), message);
    }

    /* Sends the gripper command when the position, speed or force changed
     * (at most every 20 ms while dragging), once the gripper is enabled. */
    void flush_gripper() {
        if (!gripper_dirty_ || monotonic_us() < gripper_retry_us_) return;
        const auto state = controller_.snapshot();
        if (!state.connected || !(state.enabled_mask & kGripperMask)) { gripper_dirty_ = false; return; }
        gripper_dirty_ = false;
        const int status = controller_.gripper(gripper_position_,
                                               static_cast<int>(std::lround(gripper_speed_->value() * 1000.0)),
                                               static_cast<int>(std::lround(gripper_force_->value() * 1000.0)));
        if (status == 4 || status < 0) { // busy / another request pending: again shortly
            gripper_dirty_ = true;
            gripper_retry_us_ = monotonic_us() + (status < 0 ? 100000 : 20000);
        }
        gripper_note_ = status == 0 ? QString() : status == 7 ? QStringLiteral("夹爪未使能") :
                        status == 3 ? QStringLiteral("参数超范围或固件不支持") :
                        status == 4 || status < 0 ? QStringLiteral("等待重发") :
                        QStringLiteral("status=%1").arg(status);
    }

    /* ~30 Hz, the arm page only: each motor's torque bar and value. */
    void refresh_torque() {
        if (!arm_scroll_->isVisible()) return;
        std::array<float, kMotorCount> torque{};
        const bool live = controller_.measured_torque(torque);
        for (size_t i = 0; i < kMotorCount; ++i) {
            torque_bars_[i]->set_value(live ? torque[i] : 0.0, live);
            const QString text = live ? QString::number(torque[i], 'f', 2) : QStringLiteral("--");
            if (torque_labels_[i]->text() != text) torque_labels_[i]->setText(text);
        }
    }

    void enable_group(uint8_t group) {
        if (controller_.enable_group(group)) {
            sync_targets_without_commands();
            // Apply the speed and force set here, at the position it holds.
            if (group & kGripperMask) gripper_dirty_ = true;
        } else show_error(group == kGripperMask
            ? QStringLiteral("夹爪使能失败：确认夹爪 J8 已连接并上电（串口日志 ENABLE: J8 ...）、电机模式已应用、处于 IDLE")
            : group == kAllMask
            ? QStringLiteral("使能失败：先应用电机模式并处于 IDLE，确认 MCU 新固件/反馈；夹爪未连接时请用“使能机械臂”")
            : QStringLiteral("机械臂使能失败：先应用电机模式并处于 IDLE，确认 MCU 新固件及 J1–J7 反馈"));
        refresh();
    }

    /* Firmware that enables the arm and the gripper separately (DMIO); when
     * discovery got no answer, let the MCU decide. */
    bool groups_supported() const {
        return !sdk_info_valid_ || (sdk_info_.capabilities & kCapabilityGroups) != 0;
    }

    static QString enabled_groups(uint8_t mask) {
        return mask == kAllMask ? QStringLiteral("机械臂+夹爪") : mask == kArmMask ? QStringLiteral("机械臂") :
               mask == kGripperMask ? QStringLiteral("夹爪") : mask == 0 ? QStringLiteral("无") :
               QStringLiteral("部分电机");
    }

    void sync_targets_without_commands() {
        /* Updating widgets must never emit a movement command at enable or
         * mode handover, especially when the actual pose is outside the
         * previous slider display range. */
        const auto targets = controller_.targets();
        gripper_position_ = targets[kGripperIndex]; // the command it holds
        for (size_t i = 0; i < kMotorCount; ++i) {
            const QSignalBlocker a(sliders_[i]), b(target_spins_[i]);
            const int q = targets[i];
            /* Keep the UI range fixed to the URDF.  Do not expand it when a
             * stale/out-of-range feedback sample arrives, otherwise the GUI
             * could generate a target outside the robot model. */
            sliders_[i]->setRange(kJointPositionMinMrad[i], kJointPositionMaxMrad[i]);
            target_spins_[i]->setRange(kJointPositionMinMrad[i] / 1000.0,
                                      kJointPositionMaxMrad[i] / 1000.0);
            const int clamped = std::clamp(q, kJointPositionMinMrad[i],
                                           kJointPositionMaxMrad[i]);
            sliders_[i]->setValue(clamped);
            target_spins_[i]->setValue(clamped / 1000.0);
        }
    }

    std::array<float, 12> cartesian_target_array() const {
        std::array<float, 12> pose{};
        pose[0] = static_cast<float>(cartesian_values_[0] * 0.001);
        pose[1] = static_cast<float>(cartesian_values_[1] * 0.001);
        pose[2] = static_cast<float>(cartesian_values_[2] * 0.001);
        const auto &r = cartesian_orientation_.rotation();
        for (int row = 0; row < 3; ++row)
            for (int col = 0; col < 3; ++col)
                pose[3 + row * 3 + col] = static_cast<float>(r.m[row][col]);
        return pose;
    }

    void update_cartesian_slider(int index) {
        if (index < 0 || index >= 6) return;
        const QSignalBlocker blocker(cartesian_sliders_[index]);
        cartesian_sliders_[index]->setValue(index < 3
            ? normalized_cartesian_progress(cartesian_values_[index], kCartesianUiSpecs[index].lower,
                                            kCartesianUiSpecs[index].upper)
            : static_cast<int>(std::lround(cartesian_values_[index] * 1000.0)));
    }

    void set_cartesian_control(int index, double value) {
        if (index < 0 || index >= 6 || !std::isfinite(value)) return;
        if (index >= 3)
            cartesian_orientation_.rotate_base(index - 3, (value - cartesian_values_[index]) * kPi / 180.0);
        cartesian_values_[index] = std::clamp(value, kCartesianUiSpecs[index].lower,
                                              kCartesianUiSpecs[index].upper);
        update_cartesian_slider(index);
        // Only reachable targets go to the MCU (same model as its IK): driving
        // the arm into the edge of its reach at speed carries it past the
        // stretch singularity, from where the line back can be blocked.
        if (run_cartesian_ik()) controller_.set_cartesian_target(cartesian_target_array());
    }

    // Display only: mirroring feedback must never write the controller's
    // target, which a mode entry seeded from the MCU hold command.
    void sync_cartesian_from_feedback() {
        const auto position = controller_.snapshot().position;
        show_cartesian_target(pose_to_wire(forward_from_mrad(position)));
        cartesian_preview_q_ = clamped_to_limits(joints_from_mrad(position));
        run_cartesian_ik();
    }

    void show_cartesian_target(const std::array<float, 12> &wire) {
        infrabot_ik::Mat3 rotation;
        for (int row = 0; row < 3; ++row)
            for (int col = 0; col < 3; ++col) rotation.m[row][col] = wire[3 + row * 3 + col];
        cartesian_orientation_.set_rotation(rotation);
        for (int i = 0; i < 3; ++i) cartesian_values_[i] = wire[i] * 1000.0;
        for (int i = 0; i < 3; ++i) {
            const QSignalBlocker blocker(cartesian_spins_[i]);
            cartesian_spins_[i]->setValue(cartesian_values_[i]);
            update_cartesian_slider(i);
        }
        for (int i = 3; i < 6; ++i) {
            const QSignalBlocker blocker(cartesian_spins_[i]);
            cartesian_values_[i] = 0.0;
            cartesian_spins_[i]->setValue(0.0);
            update_cartesian_slider(i);
        }
    }

    // Returns whether the target is reachable within the MCU's following
    // tolerance (0.5 mm, 0.01 rad).
    bool run_cartesian_ik() {
        const auto target = cartesian_target_array();
        infrabot_ik::Pose pose;
        pose.position = {target[0], target[1], target[2]};
        for (int row = 0; row < 3; ++row)
            for (int col = 0; col < 3; ++col) pose.rotation.m[row][col] = target[3 + row * 3 + col];
        // Same solve as the infrabot_ik UI (warm-started from the previous
        // solution, kept only if usable): a reachability check.  The MCU moves
        // the tool along a straight line to the same pose, but lets the
        // redundant elbow drift to neutral slowly, so its joints can differ.
        infrabot_ik::IkOptions options;
        options.max_iterations = 128;
        options.tolerance = 1e-5;
        options.max_step_norm = 0.20;
        options.max_joint_step = 0.12;
        options.posture_weight = 0.02;
        const auto result = cartesian_solver_.solve(pose, cartesian_preview_q_, options);
        const bool usable = result.status == static_cast<int>(infrabot_ik::IkStatus::Converged) ||
                            result.status == static_cast<int>(infrabot_ik::IkStatus::MaxIterations);
        const bool reachable = usable && result.position_error <= 0.0005 &&
                               result.orientation_error <= 0.01;
        // Warm start only from reachable solutions: an out-of-reach probe
        // must not move the preview onto another elbow branch.
        if (reachable) cartesian_preview_q_ = result.q;
        const auto &q = cartesian_preview_q_;
        const auto &r = pose.rotation.m;
        const double roll = std::atan2(r[2][1], r[2][2]) * 180.0 / kPi;
        const double pitch = std::asin(std::clamp(-r[2][0], -1.0, 1.0)) * 180.0 / kPi;
        const double yaw = std::atan2(r[1][0], r[0][0]) * 180.0 / kPi;
        cartesian_result_->setText(QStringLiteral("笛卡尔 IK：%1（%9）| 耗时 %2 ms | 迭代 %3 次 | 位置误差 %4 mm | 姿态误差 %5°\n"
                                                  "目标 RPY（绝对，ZYX）：%6° / %7° / %8° | 主机参考解 q = [%10] rad")
            .arg(reachable ? (result.success ? QStringLiteral("已收敛") : QStringLiteral("可达"))
                           : QStringLiteral("目标超出可达范围，未发送：MCU 保持上一个可达目标"))
            .arg(result.solve_time_ms, 0, 'f', 4).arg(result.iterations)
            .arg(result.position_error * 1000.0, 0, 'f', 3)
            .arg(result.orientation_error * 180.0 / kPi, 0, 'f', 3)
            .arg(roll, 0, 'f', 2).arg(pitch, 0, 'f', 2).arg(yaw, 0, 'f', 2)
            .arg(QString::fromLatin1(infrabot_ik::status_name(result.status)))
            .arg(QStringLiteral("%1, %2, %3, %4, %5, %6, %7")
                 .arg(q[0], 0, 'f', 3).arg(q[1], 0, 'f', 3).arg(q[2], 0, 'f', 3).arg(q[3], 0, 'f', 3)
                 .arg(q[4], 0, 'f', 3).arg(q[5], 0, 'f', 3).arg(q[6], 0, 'f', 3)));
        return reachable;
    }

    void reconnect() {
        log_line("UI reconnect clicked");
        const auto current = controller_.snapshot();
        const bool move_occupied = current.motion.phase != motion::Phase::Off &&
                                   current.motion.phase != motion::Phase::Cancelled &&
                                   current.motion.phase != motion::Phase::Error;
        if (current.enabled_mask != 0U || current.streaming || move_occupied) {
            // Reconnecting does not remove power: the arm stops in a
            // controlled way and holds until the new connection takes it over.
            const QString question = controller_.controlled_stop_supported()
                ? QStringLiteral("机械臂已使能。重连会先让 MCU 受控停止（刹停后在 IDLE 保持，不失能），再重新建立连接。继续？")
                : QStringLiteral("机械臂已使能，而 MCU 固件不支持受控停止：重连会失能机械臂，承重时会下坠。请先支撑机械臂。继续？");
            if (QMessageBox::question(this, QStringLiteral("重连"), question) != QMessageBox::Yes) return;
            log_line("UI reconnect with the arm enabled: controlled stop first");
        }
        controller_.stop();
        sdk_info_valid_ = false;
        std::string error;
        if (!controller_.start(ip_edit_->text().trimmed().toStdString(),
                               command_port_->value(), telemetry_port_->value(), error)) {
            show_error(QString::fromStdString(error));
            return;
        }
        // The SDK asked for the controller's information (DMIO) on connect.
        sdk_info_ = controller_.info();
        sdk_info_valid_ = sdk_info_.received;
        status_->setText(QStringLiteral("socket 已打开，等待 MCU 遥测；连接不会自动使能或设零。"));
        log_line("UI reconnect successful");
    }

    // Height of two lines of text (Chinese: its fallback font's lines are
    // taller): status labels keep it, so a text growing to a second line
    // does not shift every row below.
    static int two_lines(const QWidget *label) {
        return label->fontMetrics().boundingRect(QRect(0, 0, 1000, 1000), Qt::TextWordWrap,
                                                 QStringLiteral("状态 0\n状态 0")).height();
    }

    static QScrollArea *scrolled(QWidget *page) {
        auto *scroll = new PageScrollArea;
        scroll->setWidget(page);
        scroll->setWidgetResizable(true);
        scroll->setFrameShape(QFrame::NoFrame);
        return scroll;
    }

    // Once, before the window is shown: sizes it to show the arm controls in
    // their largest mode (NRT Joint, every row shown) and the motor tab
    // without scrolling, as far as the screen allows.  The next refresh()
    // hides the rows the current mode does not use.
    void fit_to_screen() {
        move_group_->show();
        for (auto *widget : move_joint_widgets_) widget->show();
        ensurePolished();
        QSize size = sizeHint();
        // One line of slack: a status label growing to a third line (an error
        // message) does not bring up the scroll bar.
        size.rheight() += fontMetrics().lineSpacing();
        QSize screen(4096, 4096);
        if (const QScreen *primary = QGuiApplication::primaryScreen())
            screen = primary->availableGeometry().size() - QSize(16, 48); // frame, title bar
#ifdef INFRABOT_HAS_ARM_VIEW
        // The controls keep their full width; the 3D view gets up to 860 px
        // of the width the screen leaves (on a narrow screen the view shrinks,
        // not the controls).
        QWidget *view_panel = arm_split_->widget(1);
        const int without_view = size.width() - view_panel->sizeHint().width();
        const int view = std::clamp(screen.width() - without_view, view_panel->minimumSizeHint().width(), 860);
        arm_split_->setSizes({arm_scroll_->sizeHint().width(), view});
        size.setWidth(std::max(size.width(), without_view + view));
#endif
        resize(size.boundedTo(screen));
    }

    /* Where the 诊断 page records: <repo>/recordings. */
    static std::string recordings_directory() {
        return (executable_path().parent_path().parent_path() / "recordings").string();
    }

    /* 诊断: recording, controlled stop and self-check, link quality, each
     * drive's decoded state and the fault log. */
    QWidget *make_diagnostics_page() {
        auto *page = new QWidget;
        diagnostics_page_ = page;
        auto *root = new QVBoxLayout(page);

        auto *record_group = new QGroupBox(QStringLiteral("1 kHz 数据录制：每包遥测、发出的每条指令和事件，写 CSV"), page);
        auto *record = new QGridLayout(record_group);
        record_label_ = new QLineEdit(page);
        record_label_->setObjectName("record_label");
        record_label_->setPlaceholderText(QStringLiteral("标签（可选，例如：辨识激励1）"));
        record_start_ = new QPushButton(QStringLiteral("开始录制"), page);
        record_start_->setObjectName("record_start");
        record_stop_ = new QPushButton(QStringLiteral("停止录制"), page);
        record_stop_->setObjectName("record_stop");
        marker_text_ = new QLineEdit(page);
        marker_text_->setObjectName("record_marker_text");
        marker_text_->setPlaceholderText(QStringLiteral("标记内容，写入 events.csv"));
        marker_button_ = new QPushButton(QStringLiteral("插入标记"), page);
        marker_button_->setObjectName("record_marker");
        record_status_ = new QLabel(QStringLiteral("未录制"), page);
        record_status_->setObjectName("record_status");
        record_status_->setWordWrap(true);
        record_status_->setTextInteractionFlags(Qt::TextSelectableByMouse);
        record->addWidget(new QLabel(QStringLiteral("标签"), page), 0, 0);
        record->addWidget(record_label_, 0, 1);
        record->addWidget(record_start_, 0, 2);
        record->addWidget(record_stop_, 0, 3);
        record->addWidget(new QLabel(QStringLiteral("标记"), page), 1, 0);
        record->addWidget(marker_text_, 1, 1);
        record->addWidget(marker_button_, 1, 2);
        record->addWidget(record_status_, 2, 0, 1, 4);
        record->setColumnStretch(1, 1);
        root->addWidget(record_group);
        connect(record_start_, &QPushButton::clicked, this, [this] {
            auto *robot = controller_.robot();
            if (!robot) { show_error(QStringLiteral("请先连接")); return; }
            infrabot::RecordOptions options;
            options.label = record_label_->text().trimmed().toStdString();
            std::string error;
            if (!robot->start_recording(recordings_directory(), error, options))
                show_error(QString::fromStdString(error));
            refresh();
        });
        connect(record_stop_, &QPushButton::clicked, this, [this] {
            if (auto *robot = controller_.robot()) robot->stop_recording();
            refresh();
        });
        const auto mark = [this] {
            auto *robot = controller_.robot();
            const QString text = marker_text_->text().trimmed();
            if (!robot || text.isEmpty()) return;
            robot->record_event(text.toStdString());
            marker_text_->clear();
        };
        connect(marker_button_, &QPushButton::clicked, this, mark);
        connect(marker_text_, &QLineEdit::returnPressed, this, mark);

        auto *safety_group = new QGroupBox(QStringLiteral("受控停止与参数自检"), page);
        auto *safety = new QGridLayout(safety_group);
        stop_status_ = new QLabel(page);
        stop_status_->setObjectName("safety_stop");
        stop_status_->setWordWrap(true);
        selfcheck_status_ = new QLabel(page);
        selfcheck_status_->setObjectName("safety_selfcheck");
        selfcheck_status_->setWordWrap(true);
        auto *clear_faults = new QPushButton(QStringLiteral("清除驱动器故障"), page);
        clear_faults->setObjectName("clear_faults");
        clear_faults->setToolTip(QStringLiteral("对报故障的电机发清错并等待恢复；仍使能的电机需先失能它所在的组。"));
        safety->addWidget(new QLabel(QStringLiteral("停止"), page), 0, 0);
        safety->addWidget(stop_status_, 0, 1);
        safety->addWidget(new QLabel(QStringLiteral("自检"), page), 1, 0);
        safety->addWidget(selfcheck_status_, 1, 1);
        safety->addWidget(clear_faults, 0, 2);
        safety->setColumnStretch(1, 1);
        root->addWidget(safety_group);
        connect(clear_faults, &QPushButton::clicked, this, [this] {
            std::string error;
            if (!controller_.clear_faults(error)) show_error(QString::fromStdString(error));
            else status_->setText(QStringLiteral("驱动器故障已清除（或没有故障）"));
            refresh();
        });

        auto *link_group = new QGroupBox(QStringLiteral("链路质量（上一秒）"), page);
        auto *link = new QVBoxLayout(link_group);
        link_status_ = new QLabel(page);
        link_status_->setObjectName("link_quality");
        link_status_->setWordWrap(true);
        link->addWidget(link_status_);
        root->addWidget(link_group);

        auto *motor_group = new QGroupBox(QStringLiteral("各电机状态（驱动器状态码已解码；悬停查看原因与处理）"), page);
        auto *motor = new QVBoxLayout(motor_group);
        motor_table_ = new QTableWidget(kMotorCount, 8, page);
        motor_table_->setObjectName("motor_table");
        motor_table_->setHorizontalHeaderLabels({QStringLiteral("状态"), QStringLiteral("MOS ℃"),
            QStringLiteral("线圈 ℃"), QStringLiteral("指令位置 rad"), QStringLiteral("指令力矩 N·m"),
            QStringLiteral("驱动模式"), QStringLiteral("标志"), QStringLiteral("反馈计数")});
        QStringList rows;
        for (size_t i = 0; i < kMotorCount; ++i) rows << QStringLiteral("J%1").arg(i + 1);
        motor_table_->setVerticalHeaderLabels(rows);
        motor_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
        motor_table_->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
        motor_table_->horizontalHeader()->setStretchLastSection(true);
        for (int row = 0; row < static_cast<int>(kMotorCount); ++row)
            for (int column = 0; column < 8; ++column) motor_table_->setItem(row, column, new QTableWidgetItem);
        motor_table_->setMinimumHeight(motor_table_->horizontalHeader()->sizeHint().height() +
                                       static_cast<int>(kMotorCount) * motor_table_->verticalHeader()->defaultSectionSize() + 4);
        motor->addWidget(motor_table_);
        root->addWidget(motor_group);

        auto *fault_group = new QGroupBox(QStringLiteral("故障与停止记录（最近 200 条）"), page);
        auto *fault = new QVBoxLayout(fault_group);
        fault_log_ = new QPlainTextEdit(page);
        fault_log_->setObjectName("fault_log");
        fault_log_->setReadOnly(true);
        fault_log_->setMinimumHeight(160);
        fault->addWidget(fault_log_);
        root->addWidget(fault_group, 1);
        return page;
    }

    void refresh_diagnostics(const UdpController::Snapshot &state) {
        auto *robot = controller_.robot();
        const auto stats = robot ? robot->recording() : infrabot::RecorderStats{};
        record_start_->setEnabled(robot && !stats.active && state.connected);
        record_stop_->setEnabled(stats.active);
        marker_button_->setEnabled(stats.active);
        QString recording;
        if (stats.active || !stats.directory.empty()) {
            recording = QStringLiteral("%1：%2 | 遥测 %3 行，指令 %4 行，事件 %5 条 | 分段 %6 | 丢弃 %7")
                .arg(stats.active ? QStringLiteral("录制中") : QStringLiteral("已停止"))
                .arg(QString::fromStdString(stats.directory))
                .arg(stats.telemetry_rows).arg(stats.command_rows).arg(stats.events)
                .arg(stats.file_index).arg(stats.dropped);
            if (!stats.error.empty()) recording += QStringLiteral("\n写入错误：") + QString::fromStdString(stats.error);
        } else {
            recording = QStringLiteral("未录制。文件写到 %1/<时间>-<标签>/").arg(QString::fromStdString(recordings_directory()));
        }
        if (record_status_->text() != recording) record_status_->setText(recording);
        record_status_->setStyleSheet(stats.dropped || !stats.error.empty() ? QStringLiteral("color:#b00020") : QString());

        const bool v3 = state.safety.valid;
        const std::string stop = infrabot::describe_stop(state.safety);
        stop_status_->setText(!v3 ? QStringLiteral("固件不报告（需要遥测 v3）") :
                              stop.empty() ? QStringLiteral("无") : QString::fromStdString(stop));
        stop_status_->setStyleSheet(stop.empty() ? QString() : QStringLiteral("color:#b00020"));
        QString selfcheck;
        if (!v3) selfcheck = QStringLiteral("固件不报告（需要遥测 v3）");
        else if (state.safety.selfcheck == infrabot::SelfCheck::NotRun) selfcheck = QStringLiteral("未运行（应用电机模式时自检）");
        else if (state.safety.selfcheck == infrabot::SelfCheck::Ok) selfcheck = QStringLiteral("通过：各电机 ID、控制模式和映射范围与固件一致");
        else selfcheck = QString::fromStdString(infrabot::describe_selfcheck(state.safety));
        selfcheck_status_->setText(selfcheck);
        selfcheck_status_->setStyleSheet(v3 && (state.safety.selfcheck == infrabot::SelfCheck::Mismatch ||
                                                state.safety.selfcheck == infrabot::SelfCheck::NoReply)
                                         ? QStringLiteral("color:#b00020") : QString());

        const auto &link = state.link;
        QString link_text = QStringLiteral("PC：收遥测 %1 包，丢包 %2 包（%3%），最长间隔 %4 ms | 累计：收 %5，丢 %6，无效 %7 | "
                                           "发出 %8，失败 %9，实时线程跳拍 %10")
            .arg(link.rx_last_second).arg(link.lost_last_second).arg(link.loss_percent, 0, 'f', 2)
            .arg(link.rx_interval_max_us / 1000.0, 0, 'f', 1)
            .arg(link.rx_telemetry).arg(link.rx_lost).arg(link.rx_invalid)
            .arg(link.tx_sent).arg(link.tx_errors).arg(state.tx_overruns);
        if (link.mcu_valid) {
            link_text += QStringLiteral("\nMCU：收指令 %1，拒收 %2（最后原因：%3），指令最长间隔 %4 ms，最大抖动 %5 µs | "
                                        "控制环漏拍 %6，最长周期 %7 µs | CAN 发送失败 %8，遥测发送失败 %9")
                .arg(link.mcu.command_rx).arg(link.mcu.command_invalid)
                .arg(QString::fromUtf8(infrabot::invalid_reason_text(link.mcu.invalid_reason)))
                .arg(link.mcu.command_interval_max_us / 1000.0, 0, 'f', 1).arg(link.mcu.command_jitter_max_us)
                .arg(link.mcu.control_missed).arg(link.mcu.control_period_max_us)
                .arg(link.mcu.can_tx_errors).arg(link.mcu.telemetry_errors);
        } else {
            link_text += QStringLiteral("\nMCU 侧统计需要遥测 v3 固件（目前只在串口 STAT 行）");
        }
        link_status_->setText(link_text);
        const bool poor = link.loss_percent > 1.0f || link.rx_interval_max_us > 50000U ||
                          (link.mcu_valid && (link.mcu.command_invalid || link.mcu.control_missed || link.mcu.can_tx_errors));
        link_status_->setStyleSheet(poor ? QStringLiteral("color:#b00020") : QString());

        for (size_t i = 0; i < kMotorCount; ++i) {
            const auto &m = state.motors[i];
            const auto &info = infrabot::drive_state_info(m.state);
            const int row = static_cast<int>(i);
            auto set = [&](int column, const QString &text) {
                auto *item = motor_table_->item(row, column);
                if (item->text() != text) item->setText(text);
            };
            set(0, QString::fromUtf8(info.name));
            auto *status = motor_table_->item(row, 0);
            status->setForeground(info.fault ? QColor(0xb0, 0x00, 0x20) : palette().color(QPalette::Text));
            status->setToolTip(info.fault ? QString::fromStdString(infrabot::describe_drive_fault(static_cast<uint8_t>(i), m.state))
                                          : QString::fromUtf8(info.meaning));
            set(1, state.connected ? QString::number(m.temperature_mos) : QStringLiteral("--"));
            set(2, v3 && state.connected ? QString::number(m.temperature_coil) : QStringLiteral("--"));
            const auto &joints = robot ? robot->state().joints : infrabot::JointState{};
            set(3, joints.has_command ? QString::number(joints.command_position[i], 'f', 3) : QStringLiteral("--"));
            set(4, joints.has_command ? QString::number(joints.command_torque[i], 'f', 2) : QStringLiteral("--"));
            static const char *const modes[] = {"--", "MIT", "位置速度", "速度", "力位"};
            set(5, joints.has_command && joints.drive_mode[i] <= 4 ? QString::fromUtf8(modes[joints.drive_mode[i]]) : QStringLiteral("--"));
            QStringList flags;
            if (m.flags & INFRABOT_MOTOR_ENABLED) flags << QStringLiteral("使能");
            if (m.flags & INFRABOT_MOTOR_OVERRIDE) flags << QStringLiteral("接管");
            if (m.flags & INFRABOT_MOTOR_STOPPED) flags << QStringLiteral("停住");
            if (m.flags & INFRABOT_MOTOR_TORQUE_LIMIT) flags << QStringLiteral("力矩超限");
            if (m.flags & INFRABOT_MOTOR_FEEDBACK_STALE) flags << QStringLiteral("反馈过期");
            set(6, v3 ? flags.join(QStringLiteral("、")) : QStringLiteral("--"));
            set(7, QString::number(m.feedback_count_lsb));
        }

        const auto faults = controller_.fault_log();
        const uint64_t newest = faults.empty() ? 0 : faults.back().host_time_us;
        if (faults.size() != shown_fault_count_ || newest != shown_fault_time_) {
            shown_fault_count_ = faults.size();
            shown_fault_time_ = newest;
            // Steady clock -> wall clock for display.
            const auto wall = std::chrono::system_clock::now();
            const uint64_t steady = monotonic_us();
            QStringList lines;
            for (auto it = faults.rbegin(); it != faults.rend(); ++it) {
                const auto at = wall - std::chrono::microseconds(steady - it->host_time_us);
                const QDateTime time = QDateTime::fromMSecsSinceEpoch(
                    std::chrono::duration_cast<std::chrono::milliseconds>(at.time_since_epoch()).count());
                lines << time.toString(QStringLiteral("HH:mm:ss.zzz  ")) + QString::fromStdString(it->message);
            }
            fault_log_->setPlainText(lines.join(QLatin1Char('\n')));
        }
    }

#ifdef INFRABOT_HAS_ARM_VIEW
    QWidget *make_arm_view(QWidget *parent) {
        auto *group = new QGroupBox(QStringLiteral("机械臂 3D（实测关节角；左键拖动旋转，滚轮缩放）"), parent);
        auto *layout = new QVBoxLayout(group);
        arm_view_ = new infrabot_ik::UrdfViewerWidget(group);
        arm_view_->setObjectName("arm_view");
        arm_view_->setMinimumSize(320, 300); // its own 680x560 is for a standalone window
        // Further out than the infrabot_ik UI's default, so the arm stays in
        // view stretched out to its reach in this narrower panel.
        arm_view_->setHomeView(-45.0f, 25.0f, 1.05f, QVector3D(0.05f, 0.0f, 0.22f));
        layout->addWidget(arm_view_, 1);
        auto *row = new QHBoxLayout();
        arm_view_status_ = new QLabel(group);
        arm_view_status_->setObjectName("arm_view_status");
        arm_view_status_->setWordWrap(true);
        arm_view_status_->setStyleSheet(QStringLiteral("font-family:monospace;"));
        row->addWidget(arm_view_status_, 1);
        auto *reset_view = new QPushButton(QStringLiteral("重置视角"), group);
        row->addWidget(reset_view);
        layout->addLayout(row);
        connect(reset_view, &QPushButton::clicked, this, [this] { arm_view_->resetView(); });
        QString error;
        const QString directory = qEnvironmentVariable("INFRABOT_DESCRIPTION_DIR",
                                                       QStringLiteral(INFRABOT_DEFAULT_DESCRIPTION_DIR));
        if (directory.isEmpty() || !arm_view_->loadUrdf(QDir(directory).filePath("urdf/infrabot_h7c.urdf"), &error)) {
            arm_view_->hide();
            reset_view->hide();
            arm_view_status_->setText(QStringLiteral("设置 INFRABOT_DESCRIPTION_DIR 指向 infrabot-description 目录以显示模型。\n"
                                                     "https://github.com/YaolinDong/infrabot-description\n") + error);
            return group;
        }
        arm_view_timer_ = new QTimer(this);
        connect(arm_view_timer_, &QTimer::timeout, this, [this] { refresh_arm_view(); });
        arm_view_timer_->start(33);
        refresh_arm_view();
        return group;
    }

    // ~30 Hz.  Redraws only when a joint moved more than 2 mrad (about 1 mm at
    // the tool, under a pixel), so an arm at rest costs no rendering.
    void refresh_arm_view() {
        if (!arm_view_->isVisible()) return; // another tab: nothing to draw
        std::array<int32_t, kMotorCount> position{};
        const bool live = controller_.measured_position(position);
        if (live) {
            const auto q = joints_from_mrad(position);
            bool moved = !arm_view_shown_;
            for (size_t i = 0; i < q.size(); ++i) moved = moved || std::fabs(q[i] - arm_view_q_[i]) > 0.002;
            if (moved) {
                arm_view_q_ = q;
                arm_view_shown_ = true;
                arm_view_->setJointValues(q);
            }
        }
        QString text = live ? QStringLiteral("实测") : arm_view_shown_ ? QStringLiteral("无遥测：停在最后一次实测")
                                                             : QStringLiteral("无遥测：显示零位");
        if (arm_view_shown_) {
            const auto tool = cartesian_kinematics_.forward(arm_view_q_).position;
            text += QStringLiteral(" | 末端 xyz = %1, %2, %3 mm")
                .arg(tool.x * 1000.0, 0, 'f', 1).arg(tool.y * 1000.0, 0, 'f', 1).arg(tool.z * 1000.0, 0, 'f', 1);
        }
        if (arm_view_status_->text() != text) arm_view_status_->setText(text);
    }
#endif

    void refresh() {
        const auto state = controller_.snapshot();
        const auto phase = state.motion.phase;
        const bool occupied = state.non_realtime && phase != motion::Phase::Off &&
            phase != motion::Phase::Cancelled && phase != motion::Phase::Error;
        const bool busy = occupied && phase != motion::Phase::Ready && phase != motion::Phase::Done;
        move_group_->setVisible(state.non_realtime);
        const bool movej = state.control_mode == kControlNrtJoint;
        for (auto *widget : move_joint_widgets_) widget->setVisible(movej);
        motor_page_->setEnabled(!state.non_realtime);
        const bool idle = state.control_mode == kControlIdle;
        transport_mode_->setEnabled(state.connected && state.mode_supported && !busy);
        mode_apply_->setEnabled(state.connected && state.mode_supported && !busy);
        for(const char *name: {"enter_joint_admittance","enter_cartesian_admittance"})
            findChild<QPushButton*>(name)->setEnabled(state.connected && idle && state.profile_ready && (state.enabled_mask&0x7f)==0x7f && !busy);
        findChild<QPushButton*>("admittance_stop")->setEnabled(state.connected && state.enabled_mask);
        idle_button_->setEnabled(state.connected && state.mode_supported && !busy && !idle);
        arm_mode_select_->setEnabled(state.connected && idle && !state.enabled_mask && !occupied);
        const bool can_enable = state.connected && state.mode_supported && state.profile_ready && idle && !occupied;
        const bool groups = groups_supported();
        enable_button_->setEnabled(can_enable && state.enabled_mask != kAllMask);
        enable_arm_->setEnabled(groups && can_enable && (state.enabled_mask & kArmMask) != kArmMask);
        enable_gripper_->setEnabled(groups && can_enable && !(state.enabled_mask & kGripperMask));
        // Removing power stays available with stale telemetry, like 全部失能.
        disable_arm_->setEnabled(groups && (!state.connected || (state.enabled_mask & kArmMask)));
        disable_gripper_->setEnabled(groups && (!state.connected || (state.enabled_mask & kGripperMask)));
        for (auto *button : move_conflicting_buttons_) button->setEnabled(!occupied);
        profile_apply_->setEnabled(arm_mode_select_->isEnabled());
        for (auto *button : zero_buttons_) button->setEnabled(state.connected && idle && !state.enabled_mask);
        const QString sdk_line = sdk_info_valid_
            ? QStringLiteral("SDK 发现：%1 轴/%2 电机，能力 0x%3，协议 %4.%5，固件 %6")
                .arg(sdk_info_.axes).arg(sdk_info_.motor_count)
                .arg(QStringLiteral("%1").arg(static_cast<qulonglong>(sdk_info_.capabilities), 0, 16).toUpper())
                .arg(sdk_info_.protocol_major).arg(sdk_info_.protocol_minor).arg(QString::fromStdString(sdk_info_.firmware))
            : QStringLiteral("SDK 发现：未收到 DMIO（请确认 MCU 为新固件）");
        const std::string stop_text = infrabot::describe_stop(state.safety);
        const std::string selfcheck_text = infrabot::describe_selfcheck(state.safety);
        // Resume: back to the interrupted mode (a stop in IDLE: acknowledge).
        const bool stop_latched = state.safety.valid && state.safety.stop_reason != infrabot::StopReason::None;
        const bool needs_arm = state.safety.stop_mode != infrabot::Mode::Idle;
        bool arm_faulted = false;
        for (size_t i = 0; i < motion::axes; ++i)
            arm_faulted = arm_faulted || INFRABOT_DRIVE_IS_FAULT(state.motors[i].state);
        const bool arm_ready = (state.enabled_mask & kArmMask) == kArmMask && !arm_faulted;
        const bool resume_supported = controller_.resume_supported();
        resume_button_->setEnabled(stop_latched && resume_supported && state.connected && idle && !occupied &&
                                   state.safety.stop_phase != infrabot::StopPhase::Braking && (!needs_arm || arm_ready));
        QString resume_hint;
        if (stop_latched) {
            if (!resume_supported) resume_hint = QStringLiteral("固件不支持一键恢复：手动进入所需控制模式");
            else if (needs_arm && arm_faulted) resume_hint = QStringLiteral("驱动器故障：先失能机械臂、清除故障并重新使能，再选择模式");
            else if (needs_arm && !arm_ready) resume_hint = QStringLiteral("机械臂未全部使能：重新使能后再选择模式");
            else if (needs_arm)
                resume_hint = QStringLiteral("点“恢复”回到 %1，从停下的位置继续")
                                  .arg(QString::fromUtf8(infrabot::mode_text(state.safety.stop_mode)));
            else resume_hint = QStringLiteral("点“恢复”确认");
        }
        QString safety_text;
        if (!stop_text.empty())
            safety_text = QStringLiteral("受控停止：") + QString::fromStdString(stop_text) +
                          (resume_hint.isEmpty() ? QString() : QStringLiteral("。") + resume_hint);
        if (!selfcheck_text.empty())
            safety_text += (safety_text.isEmpty() ? QString() : QStringLiteral(" | ")) + QString::fromStdString(selfcheck_text);
        mode_status_->setStyleSheet(safety_text.isEmpty() ? QString() : QStringLiteral("color:#b00020"));
        mode_status_->setText((safety_text.isEmpty() ? QString() : safety_text + QStringLiteral("\n")) +
                              QStringLiteral("MCU 实际：%1（控制器=%2） | 电机模式：%3 | 配置状态：%4")
            .arg(state.mode_supported ? QString::fromLatin1(control_mode_name(state.control_mode)) : QStringLiteral("未知（请更新 MCU 固件）"))
            .arg(QString::fromLatin1(controller_status_name(state.controller_status)))
            .arg(state.motor_profile == kModeMit ? QStringLiteral("MIT + J8 力位") :
                 state.motor_profile == kModePosVel ? QStringLiteral("位置速度 + J8 力位") : QStringLiteral("未统一"))
            .arg(state.profile_ready ? QStringLiteral("已应用") : QStringLiteral("未应用：使能被禁止")) +
            QStringLiteral(" | ") + sdk_line);
        home_slow_button_->setEnabled(
            state.non_realtime ?
                (state.connected && state.motion.can_execute) :
                (state.enabled && (state.enabled_mask & kArmMask) == kArmMask && state.streaming &&
                 state.control_mode == kControlRtJoint));
        move_execute_->setEnabled(state.control_mode == kControlNrtJoint && state.connected &&
                                  state.motion.can_execute);
        cartesian_execute_->setEnabled(state.control_mode == kControlNrtCartesian && state.connected &&
                                       state.motion.can_execute);
        // Entering a Cartesian mode resets the target to FK, so outside those
        // modes the widgets just mirror the measured pose and are read-only.
        const bool cartesian_mode = state.control_mode == kControlRtCartesian ||
                                    state.control_mode == kControlNrtCartesian;
        cartesian_controls_->setEnabled(cartesian_mode && state.connected);
        // In a Cartesian mode the measured pose includes gravity sag; loading it
        // into the widgets would make the next slider edit send it (a drop).
        cartesian_use_current_->setEnabled(!cartesian_mode);
        if (!cartesian_mode && state.connected && state.feedback_age_ms <= 100)
            sync_cartesian_from_feedback();
        move_cancel_->setEnabled(state.non_realtime); // Never disable stop just because telemetry is stale.
        move_copy_actual_->setEnabled(!busy && state.connected);
        move_load_gains_->setEnabled(!busy);
        for (auto &row : move_parameters_) for (auto *spin : row) spin->setEnabled(!busy);
        for (auto *spin : {move_position_tolerance_, move_velocity_tolerance_, move_dwell_, move_timeout_}) spin->setEnabled(!busy);
        for (size_t i = 0; i < motion::axes; ++i) {
            sliders_[i]->setEnabled(!busy && state.control_mode != kControlRtCartesian);
            target_spins_[i]->setEnabled(!busy);
        }
        // The gripper: its own command, in every mode, once enabled.
        const bool gripper_on = state.connected && (state.enabled_mask & kGripperMask);
        sliders_[kGripperIndex]->setEnabled(gripper_on);
        target_spins_[kGripperIndex]->setEnabled(gripper_on);
        if (gripper_status_->text() != gripper_note_) gripper_status_->setText(gripper_note_);
        const auto &m = state.motion.reply;
        move_status_->setText(QStringLiteral("movej %1 | ID=%2 | 规划 %3 ms，已运行 %4 ms | 到位 0x%5/7F | 反馈 0x%6/7F | 最大误差 %7 rad | 应答年龄 %8 ms%9")
            .arg(QString::fromUtf8(motion::phase_name(phase))).arg(state.motion.id)
            .arg(m.duration_ms).arg(m.elapsed_ms)
            .arg(m.arrived_mask, 2, 16, QLatin1Char('0'))
            .arg(m.fresh_mask, 2, 16, QLatin1Char('0'))
            .arg(m.max_error_rad, 0, 'f', 4)
            .arg(state.motion.has_reply ? QString::number(state.motion.reply_age_us / 1000) : QStringLiteral("--"))
            .arg(state.motion.error.empty() ? QString() : QStringLiteral("\n") + QString::fromStdString(state.motion.error)));
        move_status_->setStyleSheet(phase == motion::Phase::Error ? QStringLiteral("color:#b00020") : QString());
        for (size_t i = 0; i < kMotorCount; ++i) {
            const int value = std::clamp(state.position[i], actual_bars_[i]->minimum(), actual_bars_[i]->maximum());
            actual_bars_[i]->setValue(value);
            actual_labels_[i]->setText(QString::number(state.position[i] / 1000.0, 'f', 3));
        }
        status_->setText(QStringLiteral("%1 | 模式=%2 | TX=%3  errors=%4  overruns=%5 | RX=%6  lost=%7  invalid=%8 | "
                                        "MCU=%9 active=%10  使能=%11  mask=0x%12  zero=%13  ack=%14")
                             .arg(state.connected ? QStringLiteral("已连接") : QStringLiteral("未连接"))
                             .arg(QString::fromLatin1(control_mode_name(state.control_mode)))
                             .arg(state.tx_sent)
                             .arg(state.tx_errors)
                             .arg(state.tx_overruns)
                             .arg(state.rx_received)
                             .arg(state.rx_lost)
                             .arg(state.rx_invalid)
                             .arg(QString::fromLatin1(controller_status_name(state.controller_status)))
                             .arg(state.command_active ? QStringLiteral("是") : QStringLiteral("否"))
                             .arg(enabled_groups(state.enabled_mask))
                             .arg(QStringLiteral("%1").arg(static_cast<int>(state.enabled_mask), 2, 16, QLatin1Char('0')).toUpper())
                             .arg(state.zero_valid ? QStringLiteral("是") : QStringLiteral("否"))
                             .arg(state.last_ack) +
                         (state.link.rx_last_second ? QStringLiteral(" | 遥测 %1 Hz 丢包 %2%")
                              .arg(state.link.rx_last_second).arg(state.link.loss_percent, 0, 'f', 1) : QString()));
        stop_button_->setEnabled(state.connected && state.enabled);
        if (diagnostics_page_ && diagnostics_page_->isVisible()) refresh_diagnostics(state);
        maintenance_status_->setText(QString::fromStdString(state.maintenance_message));
        const int selected = motor_select_->currentData().toInt();
        if (selected < kMotorCount) {
            const auto &m = state.motors[selected];
            motor_state_->setText(QStringLiteral("M%1 %2：位置=%3 rad，速度=%4 rad/s，转矩=%5 Nm，state=%6，MOS=%7℃，反馈计数低16位=%8 | UDP 遥测年龄=%9 ms（不代表 CAN 反馈新鲜度）")
                .arg(selected + 1)
                .arg((state.enabled_mask & (1U << selected)) ? QStringLiteral("已使能") : QStringLiteral("未使能"))
                .arg(m.position_mrad / 1000.0, 0, 'f', 3)
                .arg(m.velocity_mrad_s / 1000.0, 0, 'f', 3)
                .arg(m.torque_mNm / 1000.0, 0, 'f', 3)
                .arg(m.state).arg(m.temperature_mos).arg(m.feedback_count_lsb).arg(state.feedback_age_ms));
        } else motor_state_->setText(QStringLiteral("全部电机位置见下方；完整缓存状态请选择单个电机。"));
    }

    void update_velocity_controls() {
        if (!mode_select_ || !maintenance_velocity_ || !maintenance_velocity_limit_) return;
        const int mode = mode_select_->currentData().toInt();
        const bool target_velocity = mode == kModeMit || mode == kModeVel;
        const bool velocity_limit = mode == kModePosVel || mode == kModeForcePos;
        const double maximum = motor_select_ && motor_select_->currentData().toInt() == int(kGripperIndex) ?
            100.0 : double(INFRABOT_ARM_MAX_JOINT_VELOCITY);
        maintenance_velocity_->setRange(-maximum,maximum);
        maintenance_velocity_limit_->setMaximum(maximum);
        maintenance_velocity_->setEnabled(target_velocity);
        maintenance_velocity_limit_->setEnabled(velocity_limit);
        if (velocity_label_) velocity_label_->setEnabled(target_velocity);
        if (velocity_limit_label_) velocity_limit_label_->setEnabled(velocity_limit);
    }

    UdpController controller_;
    QWidget *motor_page_ = nullptr;
    QScrollArea *arm_scroll_ = nullptr;
    QSplitter *arm_split_ = nullptr;
    // NRT Joint only: movej rows and the joint table's movej columns.
    std::vector<QWidget *> move_joint_widgets_;
    QComboBox *transport_mode_ = nullptr;
    QPushButton *mode_apply_ = nullptr, *idle_button_ = nullptr, *profile_apply_ = nullptr;
    QLabel *mode_status_ = nullptr;
    QGroupBox *move_group_ = nullptr;
    QGroupBox *cartesian_group_ = nullptr;
    QWidget *cartesian_controls_ = nullptr;
    QPushButton *cartesian_use_current_ = nullptr;
    QPushButton *enable_button_ = nullptr;
    QPushButton *enable_arm_ = nullptr, *enable_gripper_ = nullptr;
    QDoubleSpinBox *gripper_speed_ = nullptr, *gripper_force_ = nullptr;
    QSlider *gripper_speed_slider_ = nullptr, *gripper_force_slider_ = nullptr;
    QLabel *gripper_status_ = nullptr;
    QTimer *gripper_timer_ = nullptr;
    int gripper_position_ = 0;       // mrad, joint frame
    bool gripper_dirty_ = false;     // changed since the last command sent
    uint64_t gripper_retry_us_ = 0;
    QString gripper_note_;           // last command's problem, if any
    QPushButton *disable_arm_ = nullptr, *disable_gripper_ = nullptr;
    QPushButton *home_slow_button_ = nullptr;
    QPushButton *move_execute_ = nullptr, *move_cancel_ = nullptr, *move_copy_actual_ = nullptr, *move_load_gains_ = nullptr;
    QLabel *move_status_ = nullptr;
    QPushButton *cartesian_execute_ = nullptr;
    QLabel *cartesian_result_ = nullptr;
    QDoubleSpinBox *move_position_tolerance_ = nullptr, *move_velocity_tolerance_ = nullptr, *move_dwell_ = nullptr, *move_timeout_ = nullptr;
    std::array<std::array<QDoubleSpinBox *, 5>, motion::axes> move_parameters_{};
    std::vector<QPushButton *> move_conflicting_buttons_, zero_buttons_;
    QLineEdit *ip_edit_ = nullptr;
    QSpinBox *command_port_ = nullptr;
    QSpinBox *telemetry_port_ = nullptr;
    QPushButton *connect_button_ = nullptr;
    QLabel *status_ = nullptr;
    QComboBox *motor_select_ = nullptr;
    QComboBox *mode_select_ = nullptr;
    QComboBox *arm_mode_select_ = nullptr;
    QLabel *velocity_label_ = nullptr;
    QLabel *velocity_limit_label_ = nullptr;
    QSpinBox *register_id_ = nullptr;
    QLabel *register_info_ = nullptr;
    QComboBox *register_type_ = nullptr;
    QLineEdit *register_value_ = nullptr;
    QDoubleSpinBox *maintenance_position_ = nullptr;
    QDoubleSpinBox *maintenance_velocity_ = nullptr;
    QDoubleSpinBox *maintenance_kp_ = nullptr;
    QDoubleSpinBox *maintenance_kd_ = nullptr;
    QDoubleSpinBox *maintenance_torque_ = nullptr;
    QDoubleSpinBox *maintenance_velocity_limit_ = nullptr;
    QDoubleSpinBox *maintenance_torque_ratio_ = nullptr;
    QSpinBox *can_timeout_ms_ = nullptr;
    QLabel *maintenance_status_ = nullptr;
    QLabel *motor_state_ = nullptr;
    QTimer *refresh_timer_ = nullptr;
    infrabot::RobotInfo sdk_info_{};
    bool sdk_info_valid_ = false;
    QPushButton *stop_button_ = nullptr, *resume_button_ = nullptr;
    QWidget *diagnostics_page_ = nullptr;
    QLineEdit *record_label_ = nullptr, *marker_text_ = nullptr;
    QPushButton *record_start_ = nullptr, *record_stop_ = nullptr, *marker_button_ = nullptr;
    QLabel *record_status_ = nullptr, *stop_status_ = nullptr, *selfcheck_status_ = nullptr, *link_status_ = nullptr;
    QTableWidget *motor_table_ = nullptr;
    QPlainTextEdit *fault_log_ = nullptr;
    size_t shown_fault_count_ = 0;
    uint64_t shown_fault_time_ = 0;
    std::array<QSlider *, kMotorCount> sliders_{};
    std::array<QDoubleSpinBox *, kMotorCount> target_spins_{};
    std::array<QProgressBar *, kMotorCount> actual_bars_{};
    std::array<QLabel *, kMotorCount> actual_labels_{};
    std::array<TorqueBar *, kMotorCount> torque_bars_{};
    std::array<QLabel *, kMotorCount> torque_labels_{};
    QTimer *torque_timer_ = nullptr;
    std::array<QDoubleSpinBox *, 6> cartesian_spins_{};
    std::array<QDoubleSpinBox *, 4> cartesian_limit_spins_{};
    std::array<QSlider *, 6> cartesian_sliders_{};
    std::array<double, 6> cartesian_values_{};
    CartesianOrientationTarget cartesian_orientation_{};
    infrabot_ik::Kinematics cartesian_kinematics_{};
    infrabot_ik::IKSolver cartesian_solver_{cartesian_kinematics_};
    infrabot_ik::JointVector cartesian_preview_q_{};
#ifdef INFRABOT_HAS_ARM_VIEW
    infrabot_ik::UrdfViewerWidget *arm_view_ = nullptr;
    QLabel *arm_view_status_ = nullptr;
    QTimer *arm_view_timer_ = nullptr;
    infrabot_ik::JointVector arm_view_q_{}; // last joint values drawn
    bool arm_view_shown_ = false;           // a measured pose has been drawn
#endif
};

} // namespace

#ifndef UDP_CONTROL_GUI_TEST
int main(int argc, char **argv) {
    init_log();
    log_line("process started");
    qInstallMessageHandler(qt_message_handler);
    QApplication application(argc, argv);
    MainWindow window;
    window.show();
    return application.exec();
}
#endif
