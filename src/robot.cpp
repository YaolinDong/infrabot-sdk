/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "infrabot/robot.hpp"
#include "infrabot/version.hpp"

#include "infrabot/diagnostics.hpp"
#include "infrabot_ik/kinematics.hpp"
#include "motion_session.hpp"
#include "recorder.hpp"
#include "run_log.hpp"
#include "serial_log.hpp"
#include "joint_stream.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <cmath>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <filesystem>
#include <mutex>
#include <random>
#include <sstream>
#include <thread>

namespace infrabot {
namespace {

uint64_t now_us() {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000ULL + static_cast<uint64_t>(ts.tv_nsec) / 1000ULL;
}
int32_t milli(float value) { return static_cast<int32_t>(std::lround(value * 1000.0f)); }
float unmilli(int32_t value) { return static_cast<float>(value) / 1000.0f; }
void sleep_ms(unsigned ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

constexpr std::array<float, 7> kTemplateKp = {300.f, 300.f, 150.f, 150.f, 40.f, 40.f, 30.f};
constexpr std::array<float, 7> kTemplateKd = {2.5f, 2.5f, 2.5f, 2.5f, 0.8f, 0.8f, 0.8f};
/* Position-velocity v_des of the stream (control_template.tsv). */
constexpr float kDefaultVelocityLimit = 3.0f;
/* Application cap, distinct from each motor's MIT encoding range. */
constexpr std::array<float, 7> kCartesianJointVelocityCap = {
    INFRABOT_ARM_MAX_JOINT_VELOCITY, INFRABOT_ARM_MAX_JOINT_VELOCITY, INFRABOT_ARM_MAX_JOINT_VELOCITY,
    INFRABOT_ARM_MAX_JOINT_VELOCITY, INFRABOT_ARM_MAX_JOINT_VELOCITY, INFRABOT_ARM_MAX_JOINT_VELOCITY,
    INFRABOT_ARM_MAX_JOINT_VELOCITY};
constexpr float kCartesianJointAccelerationCap = 100.0f;
constexpr uint32_t kFreshMs = 100;      // telemetry older than this is not trusted for decisions
constexpr float kStillVelocity = 0.05f; // rad/s
constexpr size_t kFaultLogSize = 200;
constexpr size_t kReplyRingSize = 16;

std::string sformat(const char *format, ...) __attribute__((format(printf, 1, 2)));
std::string sformat(const char *format, ...) {
    char text[512];
    va_list args;
    va_start(args, format);
    std::vsnprintf(text, sizeof(text), format, args);
    va_end(args);
    return text;
}

motion::Parameters to_parameters(const MoveOptions &o, const std::array<float, 7> &target) {
    motion::Parameters p;
    p.timeout_ms = o.timeout_ms; p.dwell_ms = o.dwell_ms;
    p.position_tolerance = o.position_tolerance; p.velocity_tolerance = o.velocity_tolerance;
    p.target = target; p.velocity_max = o.velocity_max; p.acceleration_max = o.acceleration_max;
    for (float &v : p.velocity_max)
        if (std::isfinite(v)) v = std::min(v, INFRABOT_ARM_MAX_JOINT_VELOCITY);
    p.kp = o.kp; p.kd = o.kd; p.torque = o.torque;
    return p;
}

uint16_t limit_to_wire(float value, float scale, uint16_t maximum) {
    if (!(value > 0.0f)) return 0;
    return static_cast<uint16_t>(std::min<long>(std::lround(value * scale), maximum));
}
std::array<uint16_t, 4> limits_to_wire(const CartesianLimits &limits) {
    return {limit_to_wire(limits.linear_velocity, 1000.0f, INFRABOT_CARTESIAN_MAX_LINEAR_VELOCITY),
            limit_to_wire(limits.angular_velocity, 1000.0f, INFRABOT_CARTESIAN_MAX_ANGULAR_VELOCITY),
            limit_to_wire(limits.linear_acceleration, 1000.0f, INFRABOT_CARTESIAN_MAX_LINEAR_ACCELERATION),
            limit_to_wire(limits.angular_acceleration, 1000.0f, INFRABOT_CARTESIAN_MAX_ANGULAR_ACCELERATION)};
}

} // namespace

JointCommand::JointCommand() {
    kp = kTemplateKp;
    kd = kTemplateKd;
    velocity_limit.fill(kDefaultVelocityLimit);
}

CartesianCommand::CartesianCommand() {
    pose[3] = pose[7] = pose[11] = 1.0f;
    kp = kTemplateKp;
    kd = kTemplateKd;
    velocity_limit.fill(kDefaultVelocityLimit);
}

MoveOptions::MoveOptions() {
    velocity_max.fill(0.1f);
    acceleration_max.fill(0.2f);
    kp = kTemplateKp;
    kd = kTemplateKd;
}

CartesianMoveOptions::CartesianMoveOptions() {
    kp = kTemplateKp;
    kd = kTemplateKd;
}

struct Robot::Impl {
    std::string address;
    uint16_t command_port = INFRABOT_COMMAND_PORT, telemetry_port = INFRABOT_TELEMETRY_PORT;
    int command_fd = -1, telemetry_fd = -1;
    sockaddr_in board_command{}, board_telemetry{};
    std::atomic<uint32_t> sequence{0};
    std::atomic<bool> running{false}, heartbeat{true};
    std::thread rx_thread, service_thread;
    std::atomic<uint64_t> tx_sent{0}, tx_errors{0};

    // Telemetry, info, link statistics, maintenance replies (mutex).
    mutable std::mutex mutex;
    std::condition_variable cv;
    RobotState state{};
    RobotInfo info{};
    uint64_t last_telemetry_us = 0;
    std::array<uint64_t, 8> feedback_us{};
    bool have_rx_sequence = false;
    uint32_t last_rx_sequence = 0;
    LinkStats link{};
    uint64_t window_start_us = 0, window_last_rx_us = 0, last_link_event_us = 0;
    uint32_t window_rx = 0, window_lost = 0, window_interval_max_us = 0;
    std::array<InfrabotMaintenanceReply, kReplyRingSize> replies{};
    size_t reply_count = 0;
    InfrabotMaintenanceReply last_reply{};
    bool have_last_reply = false;
    uint32_t maintenance_pending = 0;
    uint64_t maintenance_sent_us = 0;
    std::deque<FaultEvent> faults;
    std::array<uint8_t, 8> drive_state{};
    SafetyState last_safety{};
    std::array<float, 8> hold{};

    // The movej/movep session (session_mutex; cv notified on replies).
    mutable std::mutex session_mutex;
    motion::Session session;
    bool session_transition = false;
    std::array<float, 7> hold_reference{};
    MoveOptions default_move{};
    CartesianMoveOptions default_cartesian{};
    motion::Phase reported_phase = motion::Phase::Off;

    std::mutex handler_mutex;
    std::function<void(const RobotState &)> telemetry_handler;

    detail::Recorder recorder;
    detail::SerialLog serial_log;
    detail::JointStream joint_stream;
    std::mutex stream_control_mutex;

    // ---- sending -----------------------------------------------------------------
    bool send_to(const sockaddr_in &destination, const void *data, size_t size, std::string &error) {
        if (command_fd < 0) { error = "未连接"; return false; }
        if (sendto(command_fd, data, size, 0, reinterpret_cast<const sockaddr *>(&destination),
                   sizeof(destination)) != static_cast<ssize_t>(size)) {
            tx_errors.fetch_add(1, std::memory_order_relaxed);
            error = std::string("UDP 发送失败：") + std::strerror(errno);
            return false;
        }
        tx_sent.fetch_add(1, std::memory_order_relaxed);
        if (recorder.active()) recorder.command(now_us(), data, size);
        return true;
    }
    bool send_joint_packet(const JointCommand& c, std::string& error) {
        InfrabotJointStream p{};
        p.magic = INFRABOT_MAGIC_COMMAND; p.version = INFRABOT_COMMAND_V5; p.motor_count = INFRABOT_MOTOR_COUNT;
        p.sequence = ++sequence; p.command = INFRABOT_CMD_MIT; p.pc_timestamp_us = now_us();
        for (size_t i = 0; i < INFRABOT_ARM_AXES; ++i) {
            if (!std::isfinite(c.position[i]) || !std::isfinite(c.velocity[i]) || !std::isfinite(c.torque[i]) ||
                !std::isfinite(c.kp[i]) || !std::isfinite(c.kd[i]) || !std::isfinite(c.velocity_limit[i])) {
                error = "servej 指令包含非有限值";
                return false;
            }
            if (std::fabs(c.velocity[i]) > INFRABOT_ARM_MAX_JOINT_VELOCITY ||
                c.velocity_limit[i] > INFRABOT_ARM_MAX_JOINT_VELOCITY) {
                error = "servej 关节速度超过 3.14 rad/s 上限";
                return false;
            }
            p.target_position_mrad[i] = milli(c.position[i]);
            p.target_velocity_mrad_s[i] = milli(c.velocity[i]);
            p.kp_milli[i] = milli(c.kp[i]);
            p.kd_milli[i] = milli(c.kd[i]);
            p.torque_mNm[i] = milli(c.torque[i]);
            // v_des of position-velocity drives: a speed limit, not the desired
            // velocity (which is 0 at rest and would stop the drive).
            p.velocity_limit_mrad_s[i] = milli(c.velocity_limit[i]);
            p.torque_ratio_milli[i] = 1000;
        }
        p.crc32 = infrabot_crc32(&p, sizeof(p) - 4);
        return send_to(board_command, &p, sizeof(p), error);
    }
    bool send_command_packet(uint8_t command, const std::array<int32_t, 8> &target, std::string &error) {
        InfrabotCommand p{};
        p.magic = INFRABOT_MAGIC_COMMAND; p.version = INFRABOT_COMMAND_V3; p.motor_count = INFRABOT_MOTOR_COUNT;
        p.sequence = ++sequence; p.command = command; p.pc_timestamp_us = now_us();
        std::memcpy(p.target_position_mrad, target.data(), sizeof(p.target_position_mrad));
        p.crc32 = infrabot_crc32(&p, sizeof(p) - 4);
        return send_to(board_command, &p, sizeof(p), error);
    }
    void event(const std::string &text, const char *source = "sdk") {
        if (recorder.active()) recorder.event(now_us(), 0, source, text);
    }
    void push_fault(FaultEvent fault) {
        fault.host_time_us = now_us();
        if (recorder.active()) recorder.event(fault.host_time_us, fault.mcu_time_us, "fault", fault.message);
        faults.push_back(std::move(fault));
        while (faults.size() > kFaultLogSize) faults.pop_front();
    }

    // ---- state helpers (mutex held) ----------------------------------------------
    uint32_t age_ms_locked() const {
        return last_telemetry_us ? static_cast<uint32_t>(std::min<uint64_t>((now_us() - last_telemetry_us) / 1000U, UINT32_MAX))
                                 : UINT32_MAX;
    }
    RobotState state_locked() const {
        RobotState result = state;
        result.telemetry_age_ms = age_ms_locked();
        const uint64_t now = now_us();
        for (size_t i = 0; i < 8; ++i)
            result.joints.feedback_age_ms[i] = feedback_us[i] ?
                uint32_t(std::min<uint64_t>((now - feedback_us[i]) / 1000, UINT32_MAX)) : UINT32_MAX;
        return result;
    }
    bool fresh_locked() const { return age_ms_locked() <= kFreshMs; }
    bool arm_still_locked() const {
        for (size_t i = 0; i < INFRABOT_ARM_AXES; ++i)
            if ((state.enabled_mask & (1U << i)) && std::fabs(state.joints.velocity[i]) > kStillVelocity) return false;
        return true;
    }

    // ---- receiving ------------------------------------------------------------------
    void receive_info(const InfrabotInfoReplyV2 &packet) {
        const auto &reply = packet.info;
        const bool legacy = reply.version == INFRABOT_INFO_V1;
        std::lock_guard<std::mutex> lock(mutex);
        info.protocol_major = legacy ? 1U : packet.protocol_major;
        info.protocol_minor = legacy ? 0U : packet.protocol_minor;
        info.firmware = legacy ? "unknown" : std::string(packet.firmware_version,
            strnlen(packet.firmware_version, sizeof(packet.firmware_version)));
        info.board.assign(packet.board, legacy ? 0 : strnlen(packet.board, sizeof(packet.board)));
        info.arm_model.assign(packet.arm_model, legacy ? 0 : strnlen(packet.arm_model, sizeof(packet.arm_model)));
        info.received = true;
        info.axes = reply.axes; info.motor_count = reply.motor_count; info.protocol_version = reply.version;
        info.capabilities = reply.capabilities; info.controller_status = reply.controller_status;
        info.control_mode = reply.control_mode; info.motor_profile = reply.motor_profile;
        info.profile_ready = reply.profile_ready != 0; info.enabled_mask = reply.enabled_mask;
        for (size_t i = 0; i < 8; ++i) {
            info.position_min[i] = reply.position_min[i]; info.position_max[i] = reply.position_max[i];
            info.velocity_max[i] = reply.velocity_max[i]; info.torque_max[i] = reply.torque_max[i];
        }
    }

    // Synchronous on the command socket before either worker starts. The MCU
    // answers this socket from its telemetry port. Bind replies to this query.
    bool negotiate_info(std::string &error) {
        InfrabotInfoRequest query{};
        query.magic = INFRABOT_MAGIC_INFO_REQUEST;
        query.sequence = ++sequence;
        const uint16_t versions[] = {INFRABOT_INFO_V2, INFRABOT_INFO_V2, INFRABOT_INFO_V1};
        for (uint16_t version : versions) {
            query.version = version;
            query.crc32 = infrabot_crc32(&query, offsetof(InfrabotInfoRequest, crc32));
            if (!send_to(board_command, &query, sizeof(query), error)) return false;
            const uint64_t deadline = now_us() + (version == INFRABOT_INFO_V1 ? 450000U : 25000U);
            for (;;) {
                const uint64_t now = now_us();
                if (now >= deadline) break;
                pollfd socket{command_fd, POLLIN, 0};
                const int ready = poll(&socket, 1, static_cast<int>((deadline - now + 999U) / 1000U));
                if (ready < 0 && errno == EINTR) continue;
                if (ready <= 0) break;
                sockaddr_in peer{};
                socklen_t peer_size = sizeof(peer);
                InfrabotInfoReplyV2 reply{};
                const ssize_t n = recvfrom(command_fd, &reply, sizeof(reply), MSG_TRUNC | MSG_DONTWAIT,
                    reinterpret_cast<sockaddr *>(&peer), &peer_size);
                if (peer.sin_addr.s_addr != board_telemetry.sin_addr.s_addr ||
                    peer.sin_port != board_telemetry.sin_port || reply.info.sequence != query.sequence ||
                    reply.info.magic != INFRABOT_MAGIC_INFO_REPLY || reply.info.status != 0) continue;
                const bool legacy = n == sizeof(InfrabotInfoReply) && reply.info.version == INFRABOT_INFO_V1;
                const bool extended = n == sizeof(reply) && reply.info.version == INFRABOT_INFO_V2 &&
                    reply.crc32 == infrabot_crc32(&reply, offsetof(InfrabotInfoReplyV2, crc32));
                if ((!legacy && !extended) ||
                    reply.info.crc32 != infrabot_crc32(&reply.info, offsetof(InfrabotInfoReply, crc32))) continue;

                const unsigned major = legacy ? 1U : reply.protocol_major;
                if (major != kProtocolMajor) {
                    error = sformat("protocol %u.%u firmware, this SDK speaks %u.%u: update the SDK or the firmware",
                                    major, legacy ? 0U : unsigned(reply.protocol_minor),
                                    unsigned(kProtocolMajor), unsigned(kProtocolMinor));
                    return false;
                }
                receive_info(reply);
                return true;
            }
        }
        error = "No valid DMIO reply to the protocol handshake (check controller address and link)";
        return false;
    }

    void receive_reply(const InfrabotMaintenanceReply &reply) {
        {
            std::lock_guard<std::mutex> lock(mutex);
            replies[reply_count++ % kReplyRingSize] = reply;
            last_reply = reply;
            have_last_reply = true;
            if (reply.sequence == maintenance_pending) maintenance_pending = 0;
        }
        cv.notify_all();
        event(sformat("DMAR %s motor=%u status=%u(%s) rid=%u value=%u seq=%u", command_name(reply.command),
                      reply.motor_index, reply.status, reply_status_text(reply.status), reply.register_id,
                      reply.value, reply.sequence), "mcu");
    }

    void receive_move(const InfrabotMoveReply &reply) {
        std::string change;
        {
            std::lock_guard<std::mutex> lock(session_mutex);
            if (session_transition) return;
            const uint64_t now = now_us();
            if (!session.receive(&reply, sizeof(reply), now)) return;
            const auto snapshot = session.snapshot(now);
            if (snapshot.phase != reported_phase) {
                reported_phase = snapshot.phase;
                change = sformat("MOVE id=%u %s arrived=0x%X max_error=%.4f ", snapshot.id,
                                 motion::phase_name(snapshot.phase), snapshot.reply.arrived_mask,
                                 static_cast<double>(snapshot.reply.max_error_rad)) + snapshot.error;
            }
        }
        cv.notify_all();
        if (!change.empty()) event(change, "mcu");
    }

    void receive_telemetry(const uint8_t *data, size_t size) {
        InfrabotTelemetryV3 packet{};
        std::memcpy(&packet, data, std::min(size, sizeof(packet)));
        const InfrabotTelemetry &head = packet.head;
        const bool v3 = head.version == INFRABOT_TELEMETRY_V3 && size == sizeof(InfrabotTelemetryV3);
        if (head.magic != INFRABOT_MAGIC_TELEMETRY || head.motor_count != INFRABOT_MOTOR_COUNT ||
            !(v3 || (head.version == INFRABOT_TELEMETRY_V2 && size == sizeof(InfrabotTelemetry)))) {
            std::lock_guard<std::mutex> lock(mutex);
            ++link.rx_invalid;
            return;
        }
        const uint64_t now = now_us();
        if (recorder.active()) recorder.telemetry(now, data, size);
        RobotState copy;
        {
            std::lock_guard<std::mutex> lock(mutex);
            // Duplicate/reordered packets cannot refresh state or per-drive ages.
            // After a stale interval, allow a new epoch (e.g. an MCU reboot).
            if (have_rx_sequence) {
                const uint32_t step = head.sequence - last_rx_sequence;
                if (step == 0 || step >= 0x80000000U || head.mcu_timestamp_us <= state.joints.timestamp_us) {
                    if (now - last_telemetry_us <= kFreshMs * 1000U) { ++link.rx_invalid; return; }
                    ++state.telemetry_epoch;
                    have_rx_sequence = false;
                    feedback_us.fill(0);
                    state.joints.feedback_count.fill(0);
                }
            }
            // Link quality: sequence gaps and arrival gaps, per second.
            if (have_rx_sequence) {
                const uint32_t gap = head.sequence - last_rx_sequence;
                if (gap > 1U && gap < 0x80000000U) { link.rx_lost += gap - 1U; window_lost += gap - 1U; }
            }
            have_rx_sequence = true;
            last_rx_sequence = head.sequence;
            ++link.rx_telemetry;
            if (!window_start_us) window_start_us = now;
            if (window_last_rx_us)
                window_interval_max_us = std::max<uint32_t>(window_interval_max_us,
                    static_cast<uint32_t>(std::min<uint64_t>(now - window_last_rx_us, UINT32_MAX)));
            window_last_rx_us = now;
            ++window_rx;
            if (now - window_start_us >= 1000000U) {
                link.rx_last_second = window_rx;
                link.lost_last_second = window_lost;
                link.loss_percent = window_rx + window_lost ? 100.0f * window_lost / (window_rx + window_lost) : 0.0f;
                link.rx_interval_max_us = window_interval_max_us;
                if ((link.loss_percent > 1.0f || window_interval_max_us > 50000U) &&
                    now - last_link_event_us > 5000000U) {
                    last_link_event_us = now;
                    FaultEvent fault;
                    fault.kind = FaultEvent::Kind::Link;
                    fault.mcu_time_us = head.mcu_timestamp_us;
                    fault.message = sformat("遥测链路质量差：上一秒丢包 %.1f%%，最长间隔 %.1f ms。检查网线、交换机和 PC 负载。",
                                            static_cast<double>(link.loss_percent), window_interval_max_us / 1000.0);
                    push_fault(std::move(fault));
                }
                window_start_us = now;
                window_rx = window_lost = window_interval_max_us = 0;
            }
            if (v3) { link.mcu_valid = true; link.mcu = packet.link; }

            last_telemetry_us = now;
            state.connected = true;
            state.telemetry_version = static_cast<uint8_t>(head.version);
            state.enabled = (head.status & INFRABOT_STATUS_FLAG_ENABLED) != 0;
            state.zero_valid = (head.status & INFRABOT_STATUS_FLAG_ZERO_VALID) != 0;
            state.controller_state = static_cast<uint8_t>(INFRABOT_STATUS_FROM_BYTE(head.status));
            state.operational_status = state.controller_state <= INFRABOT_STATUS_CARTESIAN_ADMITTANCE ?
                static_cast<OperationalStatus>(state.controller_state) : OperationalStatus::Unknown;
            state.enabled_mask = head.enabled_mask;
            state.modes_supported = (head.mode & INFRABOT_MODE_FLAG_SUPPORTED) != 0;
            state.profile_ready = (head.mode & INFRABOT_MODE_FLAG_PROFILE_READY) != 0;
            state.mode = static_cast<Mode>(INFRABOT_MODE_FROM_BYTE(head.mode));
            state.profile = static_cast<MotorProfile>(INFRABOT_PROFILE_FROM_BYTE(head.mode));
            state.command_active = head.command_active != 0;
            state.command_applied_sequence = head.command_applied_sequence;
            auto &joints = state.joints;
            joints.sequence = head.sequence;
            joints.timestamp_us = head.mcu_timestamp_us;
            joints.host_time_us = now;
            for (size_t i = 0; i < 8; ++i) {
                const auto &motor = head.motor[i];
                joints.position[i] = unmilli(motor.position_mrad);
                joints.velocity[i] = unmilli(motor.velocity_mrad_s);
                joints.torque[i] = unmilli(motor.torque_mNm);
                joints.drive_state[i] = motor.state;
                joints.temperature[i] = motor.temperature_mos;
                if ((feedback_us[i] == 0 && motor.feedback_count != 0) || motor.feedback_count != joints.feedback_count[i])
                    feedback_us[i] = now;
                joints.feedback_count[i] = motor.feedback_count;
                // Drive faults as they appear and clear.
                if (motor.state != drive_state[i]) {
                    if (INFRABOT_DRIVE_IS_FAULT(motor.state) || INFRABOT_DRIVE_IS_FAULT(drive_state[i])) {
                        FaultEvent fault;
                        fault.kind = INFRABOT_DRIVE_IS_FAULT(motor.state) ? FaultEvent::Kind::DriveFault
                                                                          : FaultEvent::Kind::DriveCleared;
                        fault.mcu_time_us = head.mcu_timestamp_us;
                        fault.joint = static_cast<uint8_t>(i);
                        fault.code = motor.state;
                        fault.detail = drive_state[i];
                        fault.message = fault.kind == FaultEvent::Kind::DriveFault ?
                            describe_drive_fault(static_cast<uint8_t>(i), motor.state) :
                            sformat("J%zu 故障已清除（%s -> %s）", i + 1, drive_state_info(drive_state[i]).name,
                                    drive_state_info(motor.state).name);
                        push_fault(std::move(fault));
                    }
                    drive_state[i] = motor.state;
                }
            }
            joints.has_command = v3;
            if (v3) {
                for (size_t i = 0; i < 8; ++i) {
                    const auto &command = packet.command[i];
                    joints.command_position[i] = unmilli(command.position_mrad);
                    joints.command_velocity[i] = unmilli(command.velocity_mrad_s);
                    joints.command_torque[i] = unmilli(command.torque_mNm);
                    joints.temperature_coil[i] = command.temperature_coil;
                    joints.motor_flags[i] = command.flags;
                    joints.drive_mode[i] = command.drive_mode;
                }
                SafetyState safety;
                safety.valid = true;
                safety.stop_reason = static_cast<StopReason>(std::min<uint8_t>(packet.safety.stop_reason, INFRABOT_STOP_TORQUE_LIMIT));
                safety.stop_phase = static_cast<StopPhase>(std::min<uint8_t>(packet.safety.stop_phase, INFRABOT_STOP_PHASE_HOLDING));
                safety.stop_joint = packet.safety.stop_joint;
                safety.stop_detail = packet.safety.stop_detail;
                safety.stop_mode = packet.safety.stop_mode <= INFRABOT_CONTROL_CARTESIAN_ADMITTANCE ?
                    static_cast<Mode>(packet.safety.stop_mode) : Mode::Idle;
                safety.selfcheck = static_cast<SelfCheck>(std::min<uint8_t>(packet.safety.selfcheck, INFRABOT_SELFCHECK_NO_REPLY));
                safety.selfcheck_joint = packet.safety.selfcheck_joint;
                safety.selfcheck_register = packet.safety.selfcheck_register;
                safety.selfcheck_value = packet.safety.selfcheck_value;
                safety.selfcheck_expected = packet.safety.selfcheck_expected;
                // Controlled stops and self-check results as they change.
                if (safety.stop_reason != last_safety.stop_reason || safety.stop_phase != last_safety.stop_phase) {
                    if (safety.stop_reason != StopReason::None) {
                        FaultEvent fault;
                        fault.kind = FaultEvent::Kind::Stop;
                        fault.mcu_time_us = head.mcu_timestamp_us;
                        fault.joint = safety.stop_joint;
                        fault.code = static_cast<uint8_t>(safety.stop_reason);
                        fault.detail = safety.stop_detail;
                        fault.message = "受控停止：" + describe_stop(safety);
                        if (safety.stop_phase == StopPhase::None && last_safety.stop_phase != StopPhase::None)
                            fault.message += "；PC 在线，机械臂 IDLE 保持";
                        push_fault(std::move(fault));
                    }
                }
                if (safety.selfcheck != last_safety.selfcheck ||
                    safety.selfcheck_joint != last_safety.selfcheck_joint ||
                    safety.selfcheck_register != last_safety.selfcheck_register) {
                    if (safety.selfcheck != SelfCheck::NotRun) {
                        FaultEvent fault;
                        fault.kind = FaultEvent::Kind::SelfCheck;
                        fault.mcu_time_us = head.mcu_timestamp_us;
                        fault.joint = safety.selfcheck_joint;
                        fault.code = static_cast<uint8_t>(safety.selfcheck);
                        fault.detail = safety.selfcheck_register;
                        fault.message = safety.selfcheck == SelfCheck::Ok ? std::string("参数自检通过") :
                                                                           describe_selfcheck(safety);
                        push_fault(std::move(fault));
                    }
                }
                last_safety = safety;
                state.safety = safety;
            } else {
                state.safety = SafetyState{};
            }
            copy = state_locked();
        }
        cv.notify_all();
        std::lock_guard<std::mutex> lock(handler_mutex);
        if (telemetry_handler) telemetry_handler(copy);
    }

    void rx_loop() {
        std::array<uint8_t, 2048> buffer{};
        while (running.load(std::memory_order_acquire)) {
            pollfd fds[2] = {{telemetry_fd, POLLIN, 0}, {command_fd, POLLIN, 0}};
            if (poll(fds, 2, 100) <= 0) continue;
            for (int which = 0; which < 2; ++which) {
                if (!(fds[which].revents & POLLIN)) continue;
                for (;;) {
                    sockaddr_in source{};
                    socklen_t source_len = sizeof(source);
                    const ssize_t n = recvfrom(fds[which].fd, buffer.data(), buffer.size(), MSG_DONTWAIT,
                                               reinterpret_cast<sockaddr *>(&source), &source_len);
                    if (n < 0) break;
                    const size_t size = static_cast<size_t>(n);
                    // Only the board: a second PC or a router broadcast is not the MCU.
                    if (source.sin_addr.s_addr != board_command.sin_addr.s_addr) continue;
                    if (which == 1) {
                        if (size == sizeof(InfrabotMoveReply) && source.sin_port == board_command.sin_port) {
                            InfrabotMoveReply reply;
                            std::memcpy(&reply, buffer.data(), sizeof(reply));
                            receive_move(reply);
                        }
                        continue;
                    }
                    if (source.sin_port != board_telemetry.sin_port) continue;
                    if (size == sizeof(InfrabotMaintenanceReply)) {
                        InfrabotMaintenanceReply reply;
                        std::memcpy(&reply, buffer.data(), sizeof(reply));
                        if (reply.magic == INFRABOT_MAGIC_MAINTENANCE && reply.version == INFRABOT_MAINTENANCE_V1 &&
                            reply.crc32 == infrabot_crc32(&reply, sizeof(reply) - 4)) {
                            receive_reply(reply);
                        } else {
                            std::lock_guard<std::mutex> lock(mutex);
                            ++link.rx_invalid;
                        }
                    } else if (size == sizeof(InfrabotTelemetry) || size == sizeof(InfrabotTelemetryV3)) {
                        receive_telemetry(buffer.data(), size);
                    } else {
                        std::lock_guard<std::mutex> lock(mutex);
                        ++link.rx_invalid;
                    }
                }
            }
        }
    }

    // ---- 1 kHz service: IDLE/NRT heartbeat and the movej/movep session --------------
    void service_session(uint64_t now) {
        std::lock_guard<std::mutex> lock(session_mutex);
        if (session_transition) return;
        if (session.preparing()) {
            bool ready, cartesian;
            {
                std::lock_guard<std::mutex> state_lock(mutex);
                ready = last_telemetry_us > session.started_us() && now - last_telemetry_us <= kFreshMs * 1000 &&
                        state.enabled && (state.mode == Mode::NrtJoint || state.mode == Mode::NrtCartesian) &&
                        (state.enabled_mask & INFRABOT_ARM_MASK) == INFRABOT_ARM_MASK && !maintenance_pending;
                cartesian = state.mode == Mode::NrtCartesian;
                for (size_t i = 0; i < INFRABOT_ARM_AXES; ++i)
                    ready = ready && state.joints.drive_state[i] == INFRABOT_DRIVE_ENABLED &&
                            std::fabs(state.joints.velocity[i]) <= kStillVelocity;
            }
            if (ready && now - session.started_us() >= 50000) {
                if (cartesian) session.begin_hold_cartesian(now);
                else session.begin_hold(hold_reference, now);
            }
        }
        motion::Request packet;
        if (session.poll(now, packet)) {
            std::string error;
            if (!send_to(board_command, &packet, sizeof(packet), error)) session.fail("move UDP 发送失败", now);
        }
    }

    void service_loop() {
        timespec deadline{};
        clock_gettime(CLOCK_MONOTONIC, &deadline);
        uint64_t next_heartbeat = 0;
        while (running.load(std::memory_order_acquire)) {
            deadline.tv_nsec += 1000000;
            if (deadline.tv_nsec >= 1000000000) { deadline.tv_nsec -= 1000000000; ++deadline.tv_sec; }
            while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &deadline, nullptr) == EINTR) {}
            const uint64_t now = now_us();
            joint_stream.tick(
                [this] { std::lock_guard<std::mutex> lock(mutex); return state_locked(); },
                [] { return now_us(); },
                [this](const JointCommand& c, std::string& e) { return send_joint_packet(c, e); },
                [this](const char* why) { stop_without_waiting(why); });
            service_session(now);
            if (heartbeat.load(std::memory_order_relaxed) && now >= next_heartbeat) {
                next_heartbeat = now + 10000;
                bool send;
                {
                    std::lock_guard<std::mutex> lock(mutex);
                    send = state.modes_supported && fresh_locked() &&
                           (state.mode == Mode::Idle || state.mode == Mode::NrtJoint || state.mode == Mode::NrtCartesian);
                }
                std::string error;
                if (send) (void)send_command_packet(INFRABOT_CMD_HOLD_HEARTBEAT, {}, error);
            }
            // Never run late ticks in a burst after a stall.
            timespec now_ts{};
            clock_gettime(CLOCK_MONOTONIC, &now_ts);
            const uint64_t planned_ns = uint64_t(deadline.tv_sec) * 1000000000ULL + deadline.tv_nsec;
            const uint64_t actual_ns = uint64_t(now_ts.tv_sec) * 1000000000ULL + now_ts.tv_nsec;
            if (actual_ns >= planned_ns + 1000000ULL) deadline = now_ts;
        }
    }

    /* Firmware that reports the interrupted mode (resume()). */
    bool resume_supported_locked() const {
        return info.received && (info.capabilities & INFRABOT_CAP_RESUME);
    }
    /* Firmware with the controlled stop: DMIO says so, or it sends telemetry
     * v3 (the same release), which also covers a DMIO answer lost at connect. */
    bool controlled_stop_locked() const {
        return (info.received && (info.capabilities & INFRABOT_CAP_CONTROLLED_STOP)) ||
               state.telemetry_version >= INFRABOT_TELEMETRY_V3;
    }
    /* Leaves a state of unknown outcome without dropping the arm: a controlled
     * stop (the MCU brakes and holds it in IDLE).  Older firmware only knows
     * DISABLE. */
    void stop_without_waiting(const char *why) {
        bool controlled;
        {
            std::lock_guard<std::mutex> lock(mutex);
            controlled = controlled_stop_locked();
        }
        {
            std::lock_guard<std::mutex> lock(session_mutex);
            session.cancel(now_us());
        }
        const uint8_t command = controlled ? INFRABOT_CMD_STOP : INFRABOT_CMD_DISABLE;
        event(std::string(controlled ? "controlled stop: " : "disable (no controlled stop in firmware): ") + why);
        std::string ignored;
        (void)send_command_packet(command, {}, ignored);
        (void)send_command_packet(command, {}, ignored);
    }

    // ---- maintenance ----------------------------------------------------------------
    bool send_maintenance(const MaintenanceRequest &r, uint32_t &seq, std::string &error) {
        InfrabotMaintenance p{};
        p.magic = INFRABOT_MAGIC_COMMAND; p.version = INFRABOT_COMMAND_V4; p.motor_count = INFRABOT_MOTOR_COUNT;
        p.command = r.command; p.motor_index = r.motor; p.mode = r.mode; p.value_type = r.value_type;
        p.register_id = r.register_id; p.pc_timestamp_us = now_us();
        if (r.motor < INFRABOT_MOTOR_COUNT) p.target_position_mrad[r.motor] = r.position_mrad;
        p.target_velocity_mrad_s = r.velocity_mrad_s; p.kp_milli = r.kp_milli; p.kd_milli = r.kd_milli;
        p.torque_mNm = r.torque_mNm; p.velocity_limit_mrad_s = r.velocity_limit_mrad_s;
        p.torque_ratio_milli = r.torque_ratio_milli; p.register_value = r.value;
        {
            std::lock_guard<std::mutex> lock(mutex);
            if (maintenance_pending && now_us() - maintenance_sent_us < 1500000U && r.command != INFRABOT_CMD_DISABLE) {
                error = "请等待当前维护操作完成";
                return false;
            }
            p.sequence = ++sequence;
            maintenance_pending = p.sequence;
            maintenance_sent_us = now_us();
        }
        p.crc32 = infrabot_crc32(&p, sizeof(p) - 4);
        seq = p.sequence;
        if (!send_to(board_command, &p, sizeof(p), error)) {
            std::lock_guard<std::mutex> lock(mutex);
            if (maintenance_pending == p.sequence) maintenance_pending = 0;
            return false;
        }
        event(sformat("-> %s motor=%u mode=%u rid=%u value=%u seq=%u", command_name(r.command), r.motor, r.mode,
                      r.register_id, r.value, p.sequence));
        return true;
    }
    bool find_reply_locked(uint32_t seq, InfrabotMaintenanceReply &reply) const {
        for (size_t i = 0; i < std::min(reply_count, kReplyRingSize); ++i)
            if (replies[i].sequence == seq) { reply = replies[i]; return true; }
        return false;
    }
    bool wait_maintenance(uint32_t seq, InfrabotMaintenanceReply &reply, uint32_t timeout_ms, std::string &error) {
        std::unique_lock<std::mutex> lock(mutex);
        if (!cv.wait_for(lock, std::chrono::milliseconds(timeout_ms), [&] { return find_reply_locked(seq, reply); })) {
            if (maintenance_pending == seq) maintenance_pending = 0;
            error = "MCU 维护应答超时：结果未知，检查 MCU 日志/固件版本";
            return false;
        }
        return true;
    }
    bool maintenance(const MaintenanceRequest &r, InfrabotMaintenanceReply &reply, uint32_t timeout_ms,
                     std::string &error) {
        uint32_t seq = 0;
        return send_maintenance(r, seq, error) && wait_maintenance(seq, reply, timeout_ms, error);
    }
    /* A blocking operation whose DMAR must carry one of the accepted statuses. */
    bool operation(const MaintenanceRequest &r, std::initializer_list<uint8_t> accepted, std::string &error,
                   uint32_t *value = nullptr, uint32_t timeout_ms = 1500) {
        InfrabotMaintenanceReply reply{};
        if (!maintenance(r, reply, timeout_ms, error)) return false;
        if (value) *value = reply.value;
        if (std::find(accepted.begin(), accepted.end(), reply.status) == accepted.end()) {
            error = sformat("MCU 拒绝 %s：%s (status=%u)", command_name(r.command), reply_status_text(reply.status),
                            reply.status);
            return false;
        }
        return true;
    }

    /* Waits until pred(state) holds on telemetry newer than the call. */
    template <class Pred>
    bool wait_state(uint32_t timeout_ms, Pred pred) {
        std::unique_lock<std::mutex> lock(mutex);
        const uint64_t since = now_us();
        return cv.wait_for(lock, std::chrono::milliseconds(timeout_ms),
                           [&] { return last_telemetry_us > since && pred(state); });
    }

    bool read_hold(std::array<float, 8> &hold_out, std::string &error) {
        RobotState current;
        {
            std::lock_guard<std::mutex> lock(mutex);
            current = state_locked();
        }
        std::array<float, 8> result = current.joints.position;
        for (uint8_t i = 0; i < INFRABOT_MOTOR_COUNT; ++i) {
            const bool arm = i < INFRABOT_ARM_AXES;
            if (!arm && !(current.enabled_mask & INFRABOT_GRIPPER_MASK)) break;
            MaintenanceRequest r;
            r.command = INFRABOT_CMD_GET_HOLD_POSITION;
            r.motor = i;
            InfrabotMaintenanceReply reply{};
            if (!maintenance(r, reply, 900, error) || reply.status != INFRABOT_REPLY_OK) {
                if (!arm) break; // the gripper keeps its measured angle
                if (error.empty()) error = sformat("J%u 保持指令读取失败：%s", i + 1, reply_status_text(reply.status));
                return false;
            }
            result[i] = static_cast<float>(static_cast<int32_t>(reply.value)) / 1000.0f;
        }
        hold_out = result;
        return true;
    }
};

Robot::Robot(std::string address, uint16_t command_port, uint16_t telemetry_port) : impl_(new Impl) {
    impl_->address = std::move(address);
    impl_->command_port = command_port;
    impl_->telemetry_port = telemetry_port;
    for (size_t i = 0; i < 8; ++i) {
        impl_->info.position_min[i] = i == INFRABOT_GRIPPER_INDEX ? 0.0f : -12.5f;
        impl_->info.position_max[i] = i == INFRABOT_GRIPPER_INDEX ? 5.94f : 12.5f;
        impl_->info.velocity_max[i] = i < 2 ? 45.f : (i < 4 ? 10.f : 30.f);
        impl_->info.torque_max[i] = i < 2 ? 54.f : (i < 4 ? 28.f : 10.f);
    }
}

Robot::~Robot() { disconnect(); }

bool Robot::connect(std::string &error) {
    error.clear();
    disconnect();
    auto &d = *impl_;
    d.joint_stream.release();
    {
        std::lock_guard<std::mutex> lock(d.mutex);
        d.state = RobotState{};
        d.last_telemetry_us = 0;
        d.feedback_us.fill(0);
        d.have_rx_sequence = false;
        d.link = LinkStats{};
        d.window_start_us = d.window_last_rx_us = 0;
        d.window_rx = d.window_lost = d.window_interval_max_us = 0;
        d.reply_count = 0;
        d.have_last_reply = false;
        d.maintenance_pending = 0;
        d.drive_state.fill(0);
        d.last_safety = SafetyState{};
        d.info.received = false;
        d.info.protocol_version = d.info.protocol_major = d.info.protocol_minor = 0;
        d.info.firmware.clear();
        d.info.board.clear();
        d.info.arm_model.clear();
        d.info.capabilities = 0;
        d.info.controller_status = static_cast<uint8_t>(OperationalStatus::Unknown);
        d.info.control_mode = static_cast<uint8_t>(Mode::Unknown);
    }
    {
        std::lock_guard<std::mutex> lock(d.session_mutex);
        d.session = motion::Session{};
        d.session_transition = false;
        d.reported_phase = motion::Phase::Off;
    }
    d.tx_sent = 0;
    d.tx_errors = 0;
    sockaddr_in board{};
    board.sin_family = AF_INET;
    if (inet_pton(AF_INET, d.address.c_str(), &board.sin_addr) != 1) { error = "MCU IP 地址无效"; return false; }
    d.board_command = board;
    d.board_command.sin_port = htons(d.command_port);
    d.board_telemetry = board;
    d.board_telemetry.sin_port = htons(d.telemetry_port);
    d.command_fd = socket(AF_INET, SOCK_DGRAM, 0);
    d.telemetry_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (d.command_fd < 0 || d.telemetry_fd < 0) {
        error = std::string("创建 socket 失败：") + std::strerror(errno);
        disconnect();
        return false;
    }
    int reuse = 1;
    setsockopt(d.telemetry_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    int receive_buffer = 4 * 1024 * 1024;
    setsockopt(d.telemetry_fd, SOL_SOCKET, SO_RCVBUF, &receive_buffer, sizeof(receive_buffer));
    sockaddr_in local{};
    local.sin_family = AF_INET;
    local.sin_addr.s_addr = htonl(INADDR_ANY);
    local.sin_port = htons(d.telemetry_port);
    if (bind(d.telemetry_fd, reinterpret_cast<sockaddr *>(&local), sizeof(local)) != 0) {
        error = std::string("绑定本地遥测端口失败：") + std::strerror(errno);
        disconnect();
        return false;
    }
    if (!d.negotiate_info(error)) { disconnect(); return false; }
    // Any datagram to the telemetry port registers this socket for telemetry.
    const char hello[] = "infrabot_sdk";
    if (sendto(d.telemetry_fd, hello, sizeof(hello) - 1, 0, reinterpret_cast<const sockaddr *>(&d.board_telemetry),
               sizeof(d.board_telemetry)) < 0) {
        error = std::string("遥测注册发送失败：") + std::strerror(errno);
        disconnect();
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(d.mutex);
        d.state.connected = true;
    }
    d.running.store(true, std::memory_order_release);
    d.rx_thread = std::thread(&Impl::rx_loop, impl_.get());
    d.service_thread = std::thread(&Impl::service_loop, impl_.get());
    d.event("connect " + d.address);
    return true;
}

void Robot::disconnect() {
    if (!impl_) return;
    auto &d = *impl_;
    std::lock_guard<std::mutex> control(d.stream_control_mutex);
    d.joint_stream.halt();
    d.running.store(false, std::memory_order_release);
    if (d.command_fd >= 0) shutdown(d.command_fd, SHUT_RDWR);
    if (d.telemetry_fd >= 0) shutdown(d.telemetry_fd, SHUT_RDWR);
    if (d.rx_thread.joinable()) d.rx_thread.join();
    if (d.service_thread.joinable()) d.service_thread.join();
    if (d.command_fd >= 0) close(d.command_fd);
    if (d.telemetry_fd >= 0) close(d.telemetry_fd);
    d.command_fd = d.telemetry_fd = -1;
    std::lock_guard<std::mutex> lock(d.mutex);
    d.state.connected = false;
    d.last_telemetry_us = 0;
}

bool Robot::connected() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->state.connected;
}
RobotInfo Robot::info() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->info;
}
RobotState Robot::state() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->state_locked();
}
LinkStats Robot::link_stats() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    LinkStats stats = impl_->link;
    stats.tx_sent = impl_->tx_sent.load(std::memory_order_relaxed);
    stats.tx_errors = impl_->tx_errors.load(std::memory_order_relaxed);
    return stats;
}
std::vector<FaultEvent> Robot::fault_log() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return {impl_->faults.begin(), impl_->faults.end()};
}
void Robot::set_heartbeat(bool enabled) { impl_->heartbeat.store(enabled); }

bool Robot::set_motor_profile(MotorProfile profile, std::string &error) {
    auto &d = *impl_;
    {
        std::lock_guard<std::mutex> lock(d.mutex);
        if (!d.fresh_locked()) { error = "MCU 遥测未连接或已过期"; return false; }
        if (!d.state.modes_supported) { error = "固件不支持控制模式状态机"; return false; }
        if (d.state.enabled_mask || d.state.mode != Mode::Idle) { error = "应用电机模式前请全部失能并回到 IDLE"; return false; }
    }
    MaintenanceRequest r;
    r.command = INFRABOT_CMD_SET_ARM_MODE;
    r.mode = static_cast<uint8_t>(profile);
    InfrabotMaintenanceReply reply{};
    // The MCU writes CTRL_MODE to every drive and then reads back their
    // parameters (self-check): a few hundred milliseconds.
    if (!d.maintenance(r, reply, 3000, error)) return false;
    if (reply.status == INFRABOT_REPLY_MISMATCH) {
        // The telemetry right after the reply carries the details.
        SafetyState safety;
        d.wait_state(200, [](const RobotState &s) { return s.safety.valid; });
        {
            std::lock_guard<std::mutex> lock(d.mutex);
            safety = d.state.safety;
        }
        error = safety.valid && (safety.selfcheck == SelfCheck::Mismatch || safety.selfcheck == SelfCheck::NoReply)
            ? describe_selfcheck(safety) : describe_selfcheck_reply(reply.value);
        return false;
    }
    if (reply.status != INFRABOT_REPLY_OK || reply.value != static_cast<uint32_t>(profile)) {
        error = std::string("MCU 拒绝电机模式：") + reply_status_text(reply.status);
        return false;
    }
    if (!d.wait_state(500, [&](const RobotState &s) { return s.profile_ready && s.profile == profile; })) {
        error = "MCU 未在遥测中确认电机模式";
        return false;
    }
    return true;
}

bool Robot::set_zero(bool enable_after, std::string &error) {
    auto &d = *impl_;
    {
        std::lock_guard<std::mutex> lock(d.mutex);
        if (!d.fresh_locked() || !d.state.modes_supported) { error = "MCU 遥测未连接或固件不支持"; return false; }
        if (d.state.enabled_mask || d.state.mode != Mode::Idle) { error = "设零前请全部失能并回到 IDLE"; return false; }
        if (enable_after && !d.state.profile_ready) { error = "请先应用电机模式"; return false; }
    }
    const uint8_t command = enable_after ? INFRABOT_CMD_SET_ZERO_ENABLE : INFRABOT_CMD_SET_ZERO;
    d.event(enable_after ? "set zero and enable" : "set zero");
    if (!d.send_command_packet(command, {}, error)) return false;
    sleep_ms(2);
    if (!d.send_command_packet(command, {}, error)) return false;
    const bool done = enable_after ?
        d.wait_state(3000, [](const RobotState &s) { return s.enabled_mask == INFRABOT_ALL_MASK && s.zero_valid; }) :
        d.wait_state(2000, [](const RobotState &s) { return s.zero_valid; });
    if (!done) { error = enable_after ? "设零后使能未确认" : "MCU 未确认设零"; return false; }
    return true;
}

bool Robot::servo_on(MotorGroup group, std::string &error) {
    auto &d = *impl_;
    const uint8_t mask = static_cast<uint8_t>(group);
    {
        std::lock_guard<std::mutex> lock(d.mutex);
        if (!d.fresh_locked()) { error = "MCU 遥测未连接或已过期"; return false; }
        if (!d.state.modes_supported) { error = "固件不支持控制模式状态机"; return false; }
        if (d.state.mode != Mode::Idle) { error = "使能需要 IDLE"; return false; }
        if (!d.state.profile_ready) { error = "请先应用电机模式（MIT/位置速度）"; return false; }
        if ((d.state.enabled_mask & mask) == mask) { error = "该组已经使能"; return false; }
    }
    const uint8_t command = group == MotorGroup::Arm ? INFRABOT_CMD_ENABLE_ARM :
                            group == MotorGroup::Gripper ? INFRABOT_CMD_ENABLE_GRIPPER : INFRABOT_CMD_ENABLE;
    d.event(sformat("servo_on mask=0x%02X", mask));
    // Twice: a single lost datagram must not cost a retry period.  The MCU
    // ignores duplicates while its staged enable runs.
    if (!d.send_command_packet(command, {}, error)) return false;
    sleep_ms(2);
    if (!d.send_command_packet(command, {}, error)) return false;
    const uint64_t deadline = now_us() + 3000000U;
    uint64_t next_retry = now_us() + 250000U;
    for (;;) {
        {
            std::unique_lock<std::mutex> lock(d.mutex);
            d.cv.wait_for(lock, std::chrono::milliseconds(5));
            if (d.fresh_locked() && d.state.enabled && (d.state.enabled_mask & mask) == mask) return true;
        }
        const uint64_t now = now_us();
        if (now >= deadline) break;
        if (now >= next_retry) {
            // A staged enable that failed after the MCU finished it starts again.
            std::string ignored;
            (void)d.send_command_packet(command, {}, ignored);
            next_retry = now + 250000U;
        }
    }
    // Only this group: a gripper that does not come up must not drop an arm
    // holding in IDLE.
    std::string ignored;
    (void)d.send_command_packet(group == MotorGroup::All ? INFRABOT_CMD_DISABLE :
                                group == MotorGroup::Arm ? INFRABOT_CMD_DISABLE_ARM : INFRABOT_CMD_DISABLE_GRIPPER,
                                {}, ignored);
    error = group == MotorGroup::Gripper ? "夹爪使能未确认（夹爪未连接？），已失能夹爪组" :
            "使能未确认（电机未连接或报故障？），已失能该组";
    return false;
}

bool Robot::servo_on(std::string &error) { return servo_on(MotorGroup::All, error); }

bool Robot::servo_off(MotorGroup group, std::string &error, bool wait) {
    auto &d = *impl_;
    std::lock_guard<std::mutex> control(d.stream_control_mutex);
    if (group != MotorGroup::Gripper) d.joint_stream.halt();
    const uint8_t mask = static_cast<uint8_t>(group);
    {
        std::lock_guard<std::mutex> lock(d.session_mutex);
        // A disabled arm ends the session.  The full DISABLE also cancels it;
        // the arm alone just drops it (a cancel of a gripper-holding arm is moot).
        if (group == MotorGroup::All) d.session.cancel(now_us());
        else if (group == MotorGroup::Arm) d.session = motion::Session{};
    }
    const uint8_t command = group == MotorGroup::Arm ? INFRABOT_CMD_DISABLE_ARM :
                            group == MotorGroup::Gripper ? INFRABOT_CMD_DISABLE_GRIPPER : INFRABOT_CMD_DISABLE;
    d.event(sformat("servo_off mask=0x%02X", mask));
    bool ok = d.send_command_packet(command, {}, error);
    // Repeat once so a single dropped datagram cannot leave the drives enabled.
    ok = d.send_command_packet(command, {}, error) && ok;
    if (!ok || !wait) return ok;
    if (!d.wait_state(2000, [&](const RobotState &s) { return (s.enabled_mask & mask) == 0; })) {
        error = "MCU 未确认失能";
        return false;
    }
    return true;
}

bool Robot::servo_off(std::string &error) { return servo_off(MotorGroup::All, error); }
bool Robot::cancel(std::string &error) { return servo_off(MotorGroup::All, error, false); }

bool Robot::stop(std::string &error, bool wait) {
    auto &d = *impl_;
    std::lock_guard<std::mutex> control(d.stream_control_mutex);
    d.joint_stream.halt();
    {
        std::lock_guard<std::mutex> lock(d.mutex);
        if (wait && !d.fresh_locked()) { error = "MCU 遥测未连接或已过期"; return false; }
        if (!d.controlled_stop_locked()) {
            error = "固件不支持受控停止（INFRABOT_CAP_CONTROLLED_STOP）；未发送。可用 servo_off 失能";
            return false;
        }
        if (!d.state.enabled) return true; // nothing to stop
    }
    {
        std::lock_guard<std::mutex> lock(d.session_mutex);
        d.session.cancel(now_us());
    }
    d.event("stop");
    bool ok = d.send_command_packet(INFRABOT_CMD_STOP, {}, error);
    ok = d.send_command_packet(INFRABOT_CMD_STOP, {}, error) && ok;
    if (!ok || !wait) return ok;
    if (!d.wait_state(2000, [&](const RobotState &s) {
            if (s.mode != Mode::Idle || s.operational_status == OperationalStatus::Recovery ||
                (s.safety.valid && s.safety.stop_phase == StopPhase::Braking)) return false;
            for (size_t i = 0; i < INFRABOT_ARM_AXES; ++i)
                if ((s.enabled_mask & (1U << i)) && std::fabs(s.joints.velocity[i]) > kStillVelocity) return false;
            return true;
        })) {
        error = "受控停止未在 2 s 内确认";
        return false;
    }
    return true;
}

bool Robot::clear_fault(std::string &error, uint32_t timeout_ms) {
    auto &d = *impl_;
    RobotState current;
    {
        std::lock_guard<std::mutex> lock(d.mutex);
        if (!d.fresh_locked()) { error = "MCU 遥测未连接或已过期"; return false; }
        current = d.state_locked();
    }
    uint8_t faulted = 0;
    for (uint8_t i = 0; i < INFRABOT_MOTOR_COUNT; ++i)
        if (INFRABOT_DRIVE_IS_FAULT(current.joints.drive_state[i])) faulted |= static_cast<uint8_t>(1U << i);
    if (!faulted) return true;
    for (uint8_t i = 0; i < INFRABOT_MOTOR_COUNT; ++i) {
        if (!(faulted & (1U << i))) continue;
        MaintenanceRequest r;
        r.command = INFRABOT_CMD_CLEAR_ERROR;
        r.motor = i;
        InfrabotMaintenanceReply reply{};
        if (!d.maintenance(r, reply, 1500, error)) return false;
        if (reply.status == INFRABOT_REPLY_DISABLE_FIRST) {
            error = sformat("J%u 仍处于使能，先失能它所在的组再清除故障", i + 1);
            return false;
        }
        if (reply.status != INFRABOT_REPLY_OK && reply.status != INFRABOT_REPLY_QUEUED) {
            error = sformat("J%u 清除故障失败：%s", i + 1, reply_status_text(reply.status));
            return false;
        }
    }
    if (!d.wait_state(timeout_ms, [&](const RobotState &s) {
            for (uint8_t i = 0; i < INFRABOT_MOTOR_COUNT; ++i)
                if ((faulted & (1U << i)) && INFRABOT_DRIVE_IS_FAULT(s.joints.drive_state[i])) return false;
            return true;
        })) {
        std::lock_guard<std::mutex> lock(d.mutex);
        for (uint8_t i = 0; i < INFRABOT_MOTOR_COUNT; ++i)
            if ((faulted & (1U << i)) && INFRABOT_DRIVE_IS_FAULT(d.state.joints.drive_state[i])) {
                error = "故障未清除：" + describe_drive_fault(i, d.state.joints.drive_state[i]);
                return false;
            }
        error = "故障清除未确认";
        return false;
    }
    return true;
}

bool Robot::switch_mode(Mode mode, std::string &error) { return enter_mode(mode, error, nullptr); }

bool Robot::enter_mode(Mode mode, std::string &error, uint8_t *reply_status) {
    auto &d = *impl_;
    std::lock_guard<std::mutex> control(d.stream_control_mutex);
    if (reply_status) *reply_status = INFRABOT_REPLY_OK;
    if (mode == Mode::Unknown || static_cast<uint8_t>(mode) > INFRABOT_CONTROL_CARTESIAN_ADMITTANCE) {
        error = "控制模式无效";
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(d.mutex);
        if (!d.fresh_locked() || !d.state.modes_supported) { error = "请连接支持模式状态机的固件，并确认遥测新鲜"; return false; }
        if (mode == d.state.mode) return true;
        if (mode != Mode::Idle) {
            if (!d.state.profile_ready || (d.state.enabled_mask & INFRABOT_ARM_MASK) != INFRABOT_ARM_MASK ||
                d.state.mode != Mode::Idle) {
                error = "先应用电机模式并使能机械臂 J1–J7；切换 RT/NRT 前必须回到 IDLE";
                return false;
            }
            const uint32_t needed = mode == Mode::RtCartesian ? INFRABOT_CAP_CARTESIAN_RT :
                                    mode == Mode::NrtCartesian ? INFRABOT_CAP_CARTESIAN_NRT :
                                    (mode==Mode::JointAdmittance || mode==Mode::CartesianAdmittance)?INFRABOT_CAP_ADMITTANCE:
                                    (mode == Mode::GravityFree || mode == Mode::GravityLock || mode==Mode::JointImpedance || mode==Mode::CartesianImpedance) ? INFRABOT_CAP_LOCAL_GRAVITY : 0U;
            if (needed && ((needed==INFRABOT_CAP_ADMITTANCE && !d.info.received) ||
                (d.info.received && !(d.info.capabilities & needed)))) {
                if(reply_status)*reply_status=INFRABOT_REPLY_UNSUPPORTED;
                error = "固件不支持请求的控制模式";
                return false;
            }
        }
        if (!d.arm_still_locked()) {
            if (reply_status) *reply_status = INFRABOT_REPLY_BUSY;
            error = "切换前机械臂必须静止（速度 ≤ 0.05 rad/s）";
            return false;
        }
    }
    {
        std::lock_guard<std::mutex> lock(d.session_mutex);
        const auto snapshot = d.session.snapshot(now_us());
        // A stopping session (cancel sent) does not block: the arm is
        // already stopping and the switch replaces the session.
        if (snapshot.phase != motion::Phase::Off && snapshot.phase != motion::Phase::Cancelled &&
            snapshot.phase != motion::Phase::Cancelling && snapshot.phase != motion::Phase::Error &&
            !snapshot.can_execute) {
            error = "当前 move 尚未完成，请等待到位或先停止";
            return false;
        }
        // Stop the session's producer while switching: late replies from a
        // closed NRT session must not race the handover.
        d.session_transition = true;
    }
    struct TransitionEnd {
        Impl &d;
        ~TransitionEnd() { std::lock_guard<std::mutex> lock(d.session_mutex); d.session_transition = false; }
    } transition{d};
    d.joint_stream.halt();
    d.event(std::string("switch_mode ") + mode_name(mode));
    MaintenanceRequest r;
    r.command = INFRABOT_CMD_SET_CONTROL_MODE;
    r.mode = static_cast<uint8_t>(mode);
    InfrabotMaintenanceReply reply{};
    if (!d.maintenance(r, reply, 1000, error)) {
        d.stop_without_waiting("switch_mode not answered");
        error = "MCU 控制模式应答超时，结果未知；已请求受控停止（刹停并保持）";
        return false;
    }
    if (reply.status != INFRABOT_REPLY_OK || reply.value != static_cast<uint32_t>(mode)) {
        if (reply_status) *reply_status = reply.status != INFRABOT_REPLY_OK ? reply.status : INFRABOT_REPLY_MISMATCH;
        error = std::string("MCU 拒绝控制模式：") + reply_status_text(reply.status);
        return false;
    }
    // Seed from the MCU's hold command: the measured angle sits below it by
    // the gravity sag, so starting from feedback drops the arm.
    std::array<float, 8> hold{};
    bool posvel;
    {
        std::lock_guard<std::mutex> lock(d.mutex);
        hold = d.state.joints.position;
        posvel = d.state.profile == MotorProfile::PositionVelocity;
    }
    if (mode != Mode::Idle) {
        std::string hold_error;
        if (!d.read_hold(hold, hold_error)) d.event("hold command query failed; seeding from feedback: " + hold_error);
    }
    {
        std::lock_guard<std::mutex> lock(d.mutex);
        d.hold = hold;
    }
    std::string prepare_error;
    bool prepared = true;
    {
        std::lock_guard<std::mutex> lock(d.session_mutex);
        d.session = motion::Session{};
        d.reported_phase = motion::Phase::Off;
        for (size_t i = 0; i < INFRABOT_ARM_AXES; ++i) d.hold_reference[i] = hold[i];
        std::random_device random;
        const uint64_t id = ((uint64_t(random()) << 32) ^ random() ^ now_us()) | 1U;
        if (mode == Mode::NrtJoint) {
            prepared = d.session.prepare(to_parameters(d.default_move, d.hold_reference), id, now_us(), prepare_error);
        } else if (mode == Mode::NrtCartesian) {
            std::array<float, 7> joints{};
            std::copy_n(hold.begin(), 7, joints.begin());
            motion::CartesianParameters p;
            const auto &o = d.default_cartesian;
            p.timeout_ms = o.timeout_ms; p.dwell_ms = o.dwell_ms;
            p.position_tolerance = o.position_tolerance; p.velocity_tolerance = o.velocity_tolerance;
            p.pose = forward(joints);
            p.cartesian_limits = limits_to_wire(o.limits);
            for (size_t i = 0; i < INFRABOT_ARM_AXES; ++i) {
                p.velocity_max[i] = o.velocity_max[i] > 0 ? std::min(o.velocity_max[i], kCartesianJointVelocityCap[i]) :
                                    posvel ? kDefaultVelocityLimit : kCartesianJointVelocityCap[i];
                p.acceleration_max[i] = o.acceleration_max[i] > 0 ? o.acceleration_max[i] : kCartesianJointAccelerationCap;
                p.kp[i] = o.kp[i];
                p.kd[i] = o.kd[i];
            }
            prepared = d.session.prepare_cartesian(p, id, now_us(), prepare_error);
        }
    }
    if (!prepared) {
        d.stop_without_waiting("move session not prepared");
        error = "move 会话准备失败：" + prepare_error + "；已请求受控停止（刹停并保持）";
        return false;
    }
    if (!d.wait_state(500, [&](const RobotState &s) { return s.mode == mode; })) {
        error = "MCU 未在遥测中确认控制模式";
        return false;
    }
    return true;
}

bool Robot::resume(std::string &error, Mode *resumed) {
    auto &d = *impl_;
    if (resumed) *resumed = Mode::Unknown;
    {
        std::lock_guard<std::mutex> lock(d.mutex);
        if (!d.fresh_locked()) { error = "MCU 遥测未连接或已过期"; return false; }
        if (!d.resume_supported_locked()) {
            error = "固件不支持恢复（INFRABOT_CAP_RESUME，需要新固件）；可手动切回控制模式";
            return false;
        }
        if (!d.state.safety.valid || d.state.safety.stop_reason == StopReason::None) {
            error = "没有需要恢复的受控停止";
            return false;
        }
    }
    // The brake must be over and the PC seen (phase NONE), and the arm still:
    // the MCU switches modes only with every arm joint below 0.05 rad/s, and
    // right after a stop position-velocity drives still trail their stopped
    // command (J3/J4 ~0.74 s) while MIT joints ring briefly.
    const uint64_t deadline = now_us() + 3000000U;
    Mode target = Mode::Idle;
    std::string waiting;
    for (;;) {
        RobotState s;
        {
            std::lock_guard<std::mutex> lock(d.mutex);
            s = d.state_locked();
        }
        if (s.safety.stop_reason == StopReason::None) { // acknowledged meanwhile
            if (resumed) *resumed = s.mode;
            return true;
        }
        target = s.safety.stop_mode;
        if (target != Mode::Idle) {
            if ((s.enabled_mask & INFRABOT_ARM_MASK) != INFRABOT_ARM_MASK) {
                error = std::string("机械臂 J1–J7 未全部使能，不能恢复到 ") + mode_name(target) +
                        "：重新使能后再切换模式";
                return false;
            }
            for (uint8_t i = 0; i < INFRABOT_ARM_AXES; ++i)
                if (INFRABOT_DRIVE_IS_FAULT(s.joints.drive_state[i])) {
                    error = describe_drive_fault(i, s.joints.drive_state[i]) +
                            "。不能恢复：先失能机械臂、清除故障并重新使能，再切换模式";
                    return false;
                }
        }
        if (s.safety.stop_phase == StopPhase::Braking) waiting = "仍在刹车";
        else if (s.safety.stop_phase == StopPhase::Holding) waiting = "等待 MCU 确认 PC 在线";
        else if (s.mode != Mode::Idle) waiting = "等待 MCU 回到 IDLE";
        else {
            bool still = true;
            for (size_t i = 0; i < INFRABOT_ARM_AXES; ++i)
                if ((s.enabled_mask & (1U << i)) && std::fabs(s.joints.velocity[i]) > kStillVelocity) still = false;
            if (still) break;
            waiting = "等待机械臂静止（速度 ≤ 0.05 rad/s）";
        }
        if (now_us() >= deadline) { error = "恢复超时：" + waiting; return false; }
        std::unique_lock<std::mutex> lock(d.mutex);
        d.cv.wait_for(lock, std::chrono::milliseconds(10));
    }
    d.event(std::string("resume ") + mode_name(target));
    if (target == Mode::Idle) {
        // The stop happened in IDLE: nothing to re-enter, acknowledge it.
        MaintenanceRequest r;
        r.command = INFRABOT_CMD_SET_CONTROL_MODE;
        r.mode = INFRABOT_CONTROL_IDLE;
        if (!d.operation(r, {INFRABOT_REPLY_OK}, error, nullptr, 1000)) return false;
        if (!d.wait_state(500, [](const RobotState &s) { return s.safety.stop_reason == StopReason::None; })) {
            error = "MCU 未确认停止";
            return false;
        }
        if (resumed) *resumed = Mode::Idle;
        return true;
    }
    // Re-enter the interrupted mode.  The MCU checks stillness against its
    // newest sample: busy means a joint moved again, try once more.
    for (;;) {
        uint8_t status = INFRABOT_REPLY_OK;
        if (enter_mode(target, error, &status)) {
            if (resumed) *resumed = target;
            return true;
        }
        if (status != INFRABOT_REPLY_BUSY || now_us() >= deadline) return false;
        error.clear();
        sleep_ms(20);
    }
}

std::array<float, 8> Robot::hold_command() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->hold;
}

bool Robot::read_hold_command(std::array<float, 8> &position, std::string &error) {
    return impl_->read_hold(position, error);
}

bool Robot::servej(const JointCommand &c, std::string &error) {
    auto &d = *impl_;
    return d.joint_stream.serve(c, now_us(), [&](const JointCommand& command, std::string& e) {
        {
            std::lock_guard<std::mutex> lock(d.mutex);
            if (!d.state.connected) { e = "未连接"; return false; }
            if (d.state.mode != Mode::RtJoint) { e = "不在 RT_JOINT"; return false; }
        }
        return d.send_joint_packet(command, e);
    }, error);
}

bool Robot::start_joint_stream(const JointCommand& initial, const JointStreamOptions& options, std::string& error) {
    auto& d = *impl_;
    std::lock_guard<std::mutex> control(d.stream_control_mutex);
    return d.joint_stream.start(initial, options, state(), info(), now_us(), error);
}
void Robot::stop_joint_stream() {
    std::lock_guard<std::mutex> control(impl_->stream_control_mutex);
    impl_->joint_stream.release();
}
JointStreamStats Robot::joint_stream_stats() const { return impl_->joint_stream.stats(); }

bool Robot::servej(const std::array<float, 7> &position, const std::array<float, 7> &velocity,
                   const std::array<float, 7> &kp, const std::array<float, 7> &kd, std::string &error) {
    JointCommand command;
    command.position = position;
    command.velocity = velocity;
    command.kp = kp;
    command.kd = kd;
    return servej(command, error);
}

bool Robot::servep(const CartesianCommand &c, std::string &error) {
    auto &d = *impl_;
    {
        std::lock_guard<std::mutex> lock(d.mutex);
        if (!d.state.connected) { error = "未连接"; return false; }
        if (d.state.mode != Mode::RtCartesian) { error = "不在 RT_CARTESIAN"; return false; }
    }
    for (float value : c.pose)
        if (!std::isfinite(value)) { error = "servep 位姿包含非有限值"; return false; }
    StreamProfile profile;
    for (size_t i = 0; i < INFRABOT_ARM_AXES; ++i) {
        profile.kp_milli[i] = milli(c.kp[i]);
        profile.kd_milli[i] = milli(c.kd[i]);
        profile.torque_mNm[i] = milli(c.torque[i]);
        profile.velocity_limit_mrad_s[i] = milli(c.velocity_limit[i]);
        profile.torque_ratio_milli[i] = 1000;
    }
    return send_cartesian_stream(c.pose, profile, limits_to_wire(c.limits), error);
}

bool Robot::submit_movej(const std::array<float, 7> &position, const MoveOptions &options, std::string &error) {
    auto &d = *impl_;
    {
        std::lock_guard<std::mutex> lock(d.mutex);
        if (d.state.mode != Mode::NrtJoint || !d.fresh_locked()) { error = "请先切换到 NRT_JOINT（遥测需新鲜）"; return false; }
    }
    std::lock_guard<std::mutex> lock(d.session_mutex);
    if (!d.session.movej(to_parameters(options, position), now_us(), error)) return false;
    d.event(sformat("movej id=%u", d.session.snapshot(now_us()).id));
    return true;
}

bool Robot::submit_movep(const Pose &pose, const CartesianMoveOptions &o, std::string &error) {
    auto &d = *impl_;
    bool posvel;
    {
        std::lock_guard<std::mutex> lock(d.mutex);
        if (d.state.mode != Mode::NrtCartesian || !d.fresh_locked()) { error = "请先切换到 NRT_CARTESIAN（遥测需新鲜）"; return false; }
        posvel = d.state.profile == MotorProfile::PositionVelocity;
    }
    motion::CartesianParameters p;
    p.timeout_ms = o.timeout_ms; p.dwell_ms = o.dwell_ms;
    p.position_tolerance = o.position_tolerance; p.velocity_tolerance = o.velocity_tolerance;
    p.pose = pose;
    p.cartesian_limits = limits_to_wire(o.limits);
    for (size_t i = 0; i < INFRABOT_ARM_AXES; ++i) {
        p.velocity_max[i] = o.velocity_max[i] > 0 ? std::min(o.velocity_max[i], kCartesianJointVelocityCap[i]) :
                            posvel ? kDefaultVelocityLimit : kCartesianJointVelocityCap[i];
        p.acceleration_max[i] = o.acceleration_max[i] > 0 ? o.acceleration_max[i] : kCartesianJointAccelerationCap;
        p.kp[i] = o.kp[i];
        p.kd[i] = o.kd[i];
    }
    std::lock_guard<std::mutex> lock(d.session_mutex);
    if (!d.session.movep(p, now_us(), error)) return false;
    d.event(sformat("movep id=%u", d.session.snapshot(now_us()).id));
    return true;
}

MotionStatus Robot::motion() const {
    std::lock_guard<std::mutex> lock(impl_->session_mutex);
    const auto s = impl_->session.snapshot(now_us());
    MotionStatus status;
    status.phase = static_cast<MotionStatus::Phase>(s.phase);
    status.id = s.id;
    status.can_execute = s.can_execute;
    status.has_reply = s.has_reply;
    status.reply = s.reply;
    status.reply_age_us = s.reply_age_us;
    status.error = s.error;
    return status;
}

bool Robot::wait_motion(uint32_t timeout_ms, MotionResult &result, std::string &error) {
    auto &d = *impl_;
    const uint64_t deadline = now_us() + uint64_t(timeout_ms) * 1000U;
    for (;;) {
        const MotionStatus status = motion();
        result.motion_id = status.id;
        result.state = status.reply.state;
        result.arrived_mask = status.reply.arrived_mask;
        result.fresh_mask = status.reply.fresh_mask;
        result.max_error_rad = status.reply.max_error_rad;
        result.reason = status.reply.reason;
        using Phase = MotionStatus::Phase;
        if (status.phase == Phase::Done || status.phase == Phase::Ready) {
            result.result = INFRABOT_MOVE_OK;
            return true;
        }
        if (status.phase == Phase::Error || status.phase == Phase::Cancelled || status.phase == Phase::Off) {
            result.result = status.reply.result ? status.reply.result :
                            status.reply.reason ? static_cast<uint8_t>(status.reply.reason) : INFRABOT_MOVE_CANCEL;
            result.message = status.error.empty() ? motion::phase_name(static_cast<motion::Phase>(status.phase))
                                                  : status.error;
            error = result.message;
            return false;
        }
        if (now_us() >= deadline) {
            result.result = INFRABOT_MOVE_MOTION_TIMEOUT;
            result.message = error = "等待到位超时";
            return false;
        }
        std::unique_lock<std::mutex> lock(d.mutex);
        d.cv.wait_for(lock, std::chrono::milliseconds(5));
    }
}

MotionResult Robot::movej(const std::array<float, 7> &position, const MoveOptions &options, std::string &error) {
    MotionResult result;
    MotionResult hold;
    // Right after switch_mode the session first holds the current command.
    std::string ignored;
    const MotionStatus status = motion();
    if (status.phase == MotionStatus::Phase::Preparing || status.phase == MotionStatus::Phase::Waiting ||
        status.phase == MotionStatus::Phase::Running || status.phase == MotionStatus::Phase::Settling)
        (void)wait_motion(12000, hold, ignored);
    if (!submit_movej(position, options, error)) {
        result.result = INFRABOT_MOVE_NOT_READY;
        result.message = error;
        return result;
    }
    (void)wait_motion(options.timeout_ms + 3000U, result, error);
    return result;
}

MotionResult Robot::movep(const Pose &pose, const CartesianMoveOptions &options, std::string &error) {
    MotionResult result;
    MotionResult hold;
    std::string ignored;
    const MotionStatus status = motion();
    if (status.phase == MotionStatus::Phase::Preparing || status.phase == MotionStatus::Phase::Waiting ||
        status.phase == MotionStatus::Phase::Running || status.phase == MotionStatus::Phase::Settling)
        (void)wait_motion(12000, hold, ignored);
    if (!submit_movep(pose, options, error)) {
        result.result = INFRABOT_MOVE_NOT_READY;
        result.message = error;
        return result;
    }
    (void)wait_motion(options.timeout_ms + 3000U, result, error);
    return result;
}

bool Robot::cancel_motion(std::string &error) {
    auto &d = *impl_;
    std::lock_guard<std::mutex> lock(d.session_mutex);
    if (!d.session.engaged()) { error = "没有进行中的 move 会话"; return false; }
    d.session.cancel(now_us());
    d.event("cancel motion");
    return true;
}

void Robot::set_default_move_options(const MoveOptions &options) {
    std::lock_guard<std::mutex> lock(impl_->session_mutex);
    impl_->default_move = options;
}
void Robot::set_default_cartesian_options(const CartesianMoveOptions &options) {
    std::lock_guard<std::mutex> lock(impl_->session_mutex);
    impl_->default_cartesian = options;
}

bool Robot::gripper(float position_rad, float speed_rad_s, float force_ratio, std::string &error) {
    MaintenanceRequest r;
    r.command = INFRABOT_CMD_GRIPPER;
    r.motor = INFRABOT_GRIPPER_INDEX;
    r.mode = INFRABOT_DRIVE_MODE_FORCE_POS;
    r.position_mrad = milli(position_rad);
    r.velocity_limit_mrad_s = milli(speed_rad_s);
    r.torque_ratio_milli = milli(force_ratio);
    InfrabotMaintenanceReply reply{};
    if (!impl_->maintenance(r, reply, 1000, error)) return false;
    if (reply.status == INFRABOT_REPLY_NOT_READY) { error = "夹爪未使能"; return false; }
    if (reply.status == INFRABOT_REPLY_INVALID) { error = "夹爪位置/速度/力超出范围，或 J8 不在力位模式"; return false; }
    if (reply.status != INFRABOT_REPLY_OK) { error = std::string("夹爪指令被拒绝：") + reply_status_text(reply.status); return false; }
    return true;
}

bool Robot::start_recording(const std::string &directory, std::string &error, const RecordOptions &options) {
    auto &d = *impl_;
    std::ostringstream info;
    const RobotInfo robot = this->info();
    info << "infrabot SDK recording\n"
         << "robot " << d.address << " command_port " << d.command_port << " telemetry_port " << d.telemetry_port << "\n"
         << "capabilities 0x" << std::hex << robot.capabilities << std::dec
         << " firmware " << robot.firmware << " protocol " << robot.protocol_major << "." << robot.protocol_minor << (robot.received ? "" : " (no DMIO)") << "\n"
         << "label " << options.label << "\n\n"
         << "telemetry_NNN.csv: one row per DMTL packet (1 kHz). Joint frame, SI: q rad, dq rad/s, tau N·m,\n"
         << "  temperatures degC; host_time_s = PC steady clock, mcu_time_s = MCU clock; lost = sequence gap;\n"
         << "  status/mode/profile/state/flags/stop_* codes: include/infrabot/protocol.h.\n"
         << "  q_cmd/dq_cmd/tau_cmd/temp_coil/flags/stop_*/mcu_*: telemetry v3 only (empty for v2 firmware).\n"
         << "commands_NNN.csv: every command datagram sent (v3 control, v4 maintenance, v5/v7 streams,\n"
         << "  DMOV moves); heartbeats and DMOV keepalives are not logged.\n"
         << "events.csv: faults, controlled stops, self-check, maintenance replies, moves, markers.\n";
    if (!d.recorder.start(directory, options, info.str(), error)) return false;
    d.event("recording started " + options.label);
    return true;
}

void Robot::stop_recording() {
    impl_->event("recording stopped");
    impl_->recorder.stop();
}

RecorderStats Robot::recording() const { return impl_->recorder.stats(); }
void Robot::record_event(const std::string &text) { impl_->event(text, "user"); }

std::string Robot::log_directory() {
    if (const char *dir = std::getenv("INFRABOT_LOG_DIR"); dir && *dir) return dir;
    if (const char *state = std::getenv("XDG_STATE_HOME"); state && *state && std::filesystem::path(state).is_absolute())
        return (std::filesystem::path(state) / "infrabot/log").string();
    if (const char *home = std::getenv("HOME"); home && *home)
        return (std::filesystem::path(home) / ".local/state/infrabot/log").string();
    return {};  // no default directory for this environment
}

bool Robot::start_serial_log(std::string &error, const SerialLogOptions &options) {
    SerialLogOptions resolved = options;
    if (resolved.directory.empty()) resolved.directory = log_directory();
    if (resolved.directory.empty()) {
        error = "Set a log directory, HOME, XDG_STATE_HOME, or INFRABOT_LOG_DIR";
        return false;
    }
    return impl_->serial_log.start(resolved, error);
}

void Robot::stop_serial_log() { impl_->serial_log.stop(); }
SerialLogStats Robot::serial_log() const { return impl_->serial_log.stats(); }

bool Robot::download_mcu_log(McuLogResult &result, std::string &error, const McuLogOptions &options) {
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        if (impl_->info.received && !(impl_->info.capabilities & INFRABOT_CAP_RUN_LOG)) {
            result = McuLogResult{};
            error = "当前固件不支持 MCU 运行日志（无 INFRABOT_CAP_RUN_LOG），请先升级固件";
            return false;
        }
    }
    McuLogOptions resolved = options;
    if (resolved.directory.empty()) resolved.directory = log_directory();
    if (resolved.directory.empty()) {
        error = "Set a log directory, HOME, XDG_STATE_HOME, or INFRABOT_LOG_DIR";
        return false;
    }
    const bool ok = detail::download_run_log(impl_->address, resolved, result, error);
    impl_->event(ok ? "mcu log downloaded: " + result.text_path : "mcu log download failed: " + error);
    return ok;
}

Pose Robot::forward(const std::array<float, 7> &joints) {
    static const infrabot_ik::Kinematics kinematics;
    infrabot_ik::JointVector q{};
    for (size_t i = 0; i < 7; ++i) q[i] = joints[i];
    const auto pose = kinematics.forward(q);
    Pose wire{};
    wire[0] = static_cast<float>(pose.position.x);
    wire[1] = static_cast<float>(pose.position.y);
    wire[2] = static_cast<float>(pose.position.z);
    for (int row = 0; row < 3; ++row)
        for (int col = 0; col < 3; ++col) wire[3 + row * 3 + col] = static_cast<float>(pose.rotation.m[row][col]);
    return wire;
}

// ---- single motor maintenance ---------------------------------------------------------
namespace {
MaintenanceRequest request(uint8_t command, uint8_t motor, uint8_t mode = 0) {
    MaintenanceRequest r;
    r.command = command;
    r.motor = motor;
    r.mode = mode;
    return r;
}
} // namespace

bool Robot::enable(uint8_t motor, std::string &e) {
    return impl_->operation(request(INFRABOT_CMD_ENABLE, motor), {INFRABOT_REPLY_QUEUED}, e);
}
bool Robot::disable(uint8_t motor, std::string &e) {
    return impl_->operation(request(INFRABOT_CMD_DISABLE, motor), {INFRABOT_REPLY_QUEUED}, e);
}
bool Robot::stop(uint8_t motor, std::string &e) {
    return impl_->operation(request(INFRABOT_CMD_STOP, motor), {INFRABOT_REPLY_QUEUED}, e);
}
bool Robot::clear_error(uint8_t motor, std::string &e) {
    return impl_->operation(request(INFRABOT_CMD_CLEAR_ERROR, motor), {INFRABOT_REPLY_QUEUED}, e);
}
bool Robot::set_zero_position(uint8_t motor, std::string &e) {
    return impl_->operation(request(INFRABOT_CMD_SET_ZERO, motor), {INFRABOT_REPLY_OK}, e);
}
bool Robot::ensure_mode(uint8_t motor, uint8_t mode, std::string &e) {
    if (mode < INFRABOT_DRIVE_MODE_MIT || mode > INFRABOT_DRIVE_MODE_FORCE_POS) { e = "达妙模式无效（应为 1..4）"; return false; }
    return impl_->operation(request(INFRABOT_CMD_ENSURE_MODE, motor, mode), {INFRABOT_REPLY_OK}, e);
}
bool Robot::send_mit(uint8_t motor, float position_rad, float velocity_rad_s, float kp, float kd, float torque_nm,
                     std::string &e) {
    auto r = request(INFRABOT_CMD_SEND_MIT, motor, INFRABOT_DRIVE_MODE_MIT);
    r.position_mrad = milli(position_rad); r.velocity_mrad_s = milli(velocity_rad_s);
    r.kp_milli = milli(kp); r.kd_milli = milli(kd); r.torque_mNm = milli(torque_nm);
    return impl_->operation(r, {INFRABOT_REPLY_QUEUED}, e);
}
bool Robot::send_pos_vel(uint8_t motor, float position_rad, float velocity_limit_rad_s, std::string &e) {
    auto r = request(INFRABOT_CMD_SEND_POS_VEL, motor, INFRABOT_DRIVE_MODE_POS_VEL);
    r.position_mrad = milli(position_rad); r.velocity_limit_mrad_s = milli(velocity_limit_rad_s);
    return impl_->operation(r, {INFRABOT_REPLY_QUEUED}, e);
}
bool Robot::send_vel(uint8_t motor, float velocity_rad_s, std::string &e) {
    auto r = request(INFRABOT_CMD_SEND_VEL, motor, INFRABOT_DRIVE_MODE_VEL);
    r.velocity_mrad_s = milli(velocity_rad_s);
    return impl_->operation(r, {INFRABOT_REPLY_QUEUED}, e);
}
bool Robot::send_force_pos(uint8_t motor, float position_rad, float velocity_limit_rad_s, float torque_ratio,
                           std::string &e) {
    auto r = request(INFRABOT_CMD_SEND_FORCE_POS, motor, INFRABOT_DRIVE_MODE_FORCE_POS);
    r.position_mrad = milli(position_rad); r.velocity_limit_mrad_s = milli(velocity_limit_rad_s);
    r.torque_ratio_milli = milli(torque_ratio);
    return impl_->operation(r, {INFRABOT_REPLY_QUEUED}, e);
}
bool Robot::request_feedback(uint8_t motor, std::string &e) {
    return impl_->operation(request(INFRABOT_CMD_REQUEST_FEEDBACK, motor), {INFRABOT_REPLY_QUEUED}, e);
}
bool Robot::get_state(uint8_t motor, std::string &e) {
    return impl_->operation(request(INFRABOT_CMD_GET_STATE, motor), {INFRABOT_REPLY_OK}, e);
}
bool Robot::set_can_timeout_ms(uint8_t motor, uint32_t timeout_ms, std::string &e) {
    auto r = request(INFRABOT_CMD_SET_CAN_TIMEOUT, motor);
    r.value = timeout_ms;
    return impl_->operation(r, {INFRABOT_REPLY_OK}, e);
}
bool Robot::store_parameters(uint8_t motor, std::string &e) {
    return impl_->operation(request(INFRABOT_CMD_STORE_PARAMETERS, motor), {INFRABOT_REPLY_QUEUED}, e);
}
bool Robot::get_register(uint8_t motor, uint8_t register_id, uint8_t value_type, uint32_t &value, std::string &e) {
    const auto *info = infrabot_register_info(register_id);
    if (!info) { e = "未知寄存器 RID " + std::to_string(register_id); return false; }
    if (info->type != value_type) { e = register_name(register_id) + " 的类型不是请求的 u32/f32"; return false; }
    auto r = request(INFRABOT_CMD_GET_REGISTER, motor);
    r.value_type = value_type;
    r.register_id = register_id;
    return impl_->operation(r, {INFRABOT_REPLY_OK}, e, &value);
}
bool Robot::write_register(uint8_t motor, uint8_t register_id, uint32_t value, uint8_t value_type, std::string &e) {
    const auto *info = infrabot_register_info(register_id);
    if (!info) { e = "未知寄存器 RID " + std::to_string(register_id); return false; }
    if (info->type != value_type) { e = register_name(register_id) + " 的类型不是请求的 u32/f32"; return false; }
    if (!infrabot_register_write_allowed(register_id, value)) {
        e = register_name(register_id) + (info->access == INFRABOT_REGISTER_WRITABLE ?
            " 的值 " + register_value_text(register_id, value) + " 超出允许范围" : " 不允许写入");
        return false;
    }
    auto r = request(INFRABOT_CMD_WRITE_REGISTER, motor);
    r.value_type = value_type;
    r.register_id = register_id;
    r.value = value;
    uint32_t readback = 0;
    if (!impl_->operation(r, {INFRABOT_REPLY_OK}, e, &readback)) return false;
    if (readback != value) { e = register_name(register_id) + " 回读不符"; return false; }
    return true;
}

// ---- protocol level -------------------------------------------------------------------
bool Robot::send_command(uint8_t command, const std::array<int32_t, 8> &target_mrad, std::string &error) {
    return impl_->send_command_packet(command, target_mrad, error);
}

bool Robot::send_joint_stream(const std::array<int32_t, 8> &target_mrad, const StreamProfile &s, std::string &error) {
    auto &d = *impl_;
    InfrabotJointStream p{};
    p.magic = INFRABOT_MAGIC_COMMAND; p.version = INFRABOT_COMMAND_V5; p.motor_count = INFRABOT_MOTOR_COUNT;
    p.sequence = ++d.sequence; p.command = INFRABOT_CMD_MIT; p.pc_timestamp_us = now_us();
    std::memcpy(p.target_position_mrad, target_mrad.data(), sizeof(p.target_position_mrad));
    std::memcpy(p.target_velocity_mrad_s, s.velocity_mrad_s.data(), sizeof(p.target_velocity_mrad_s));
    std::memcpy(p.kp_milli, s.kp_milli.data(), sizeof(p.kp_milli));
    std::memcpy(p.kd_milli, s.kd_milli.data(), sizeof(p.kd_milli));
    std::memcpy(p.torque_mNm, s.torque_mNm.data(), sizeof(p.torque_mNm));
    std::memcpy(p.velocity_limit_mrad_s, s.velocity_limit_mrad_s.data(), sizeof(p.velocity_limit_mrad_s));
    std::memcpy(p.torque_ratio_milli, s.torque_ratio_milli.data(), sizeof(p.torque_ratio_milli));
    p.crc32 = infrabot_crc32(&p, sizeof(p) - 4);
    return d.send_to(d.board_command, &p, sizeof(p), error);
}

bool Robot::send_cartesian_stream(const Pose &pose, const StreamProfile &s, const std::array<uint16_t, 4> &limits,
                                  std::string &error) {
    auto &d = *impl_;
    InfrabotCartesianStream p{};
    p.magic = INFRABOT_MAGIC_COMMAND; p.version = INFRABOT_COMMAND_V7; p.motor_count = INFRABOT_MOTOR_COUNT;
    p.sequence = ++d.sequence; p.command = INFRABOT_CMD_MIT; p.pc_timestamp_us = now_us();
    std::memcpy(p.pose, pose.data(), sizeof(p.pose));
    std::memcpy(p.target_velocity_mrad_s, s.velocity_mrad_s.data(), sizeof(p.target_velocity_mrad_s));
    std::memcpy(p.kp_milli, s.kp_milli.data(), sizeof(p.kp_milli));
    std::memcpy(p.kd_milli, s.kd_milli.data(), sizeof(p.kd_milli));
    std::memcpy(p.torque_mNm, s.torque_mNm.data(), sizeof(p.torque_mNm));
    std::memcpy(p.velocity_limit_mrad_s, s.velocity_limit_mrad_s.data(), sizeof(p.velocity_limit_mrad_s));
    std::memcpy(p.torque_ratio_milli, s.torque_ratio_milli.data(), sizeof(p.torque_ratio_milli));
    std::memcpy(p.cartesian_limits, limits.data(), sizeof(p.cartesian_limits));
    p.crc32 = infrabot_crc32(&p, sizeof(p) - 4);
    return d.send_to(d.board_command, &p, sizeof(p), error);
}

bool Robot::send_maintenance(const MaintenanceRequest &request, uint32_t &sequence, std::string &error) {
    return impl_->send_maintenance(request, sequence, error);
}
bool Robot::wait_maintenance(uint32_t sequence, InfrabotMaintenanceReply &reply, uint32_t timeout_ms,
                             std::string &error) {
    return impl_->wait_maintenance(sequence, reply, timeout_ms, error);
}
bool Robot::maintenance(const MaintenanceRequest &request, InfrabotMaintenanceReply &reply, uint32_t timeout_ms,
                        std::string &error) {
    return impl_->maintenance(request, reply, timeout_ms, error);
}
bool Robot::maintenance_pending() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->maintenance_pending && now_us() - impl_->maintenance_sent_us < 1500000U;
}
bool Robot::last_maintenance_reply(InfrabotMaintenanceReply &reply) const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    reply = impl_->last_reply;
    return impl_->have_last_reply;
}

bool Robot::store_control_profile(const StreamProfile &s, uint8_t &status, std::string &error) {
    auto &d = *impl_;
    {
        std::lock_guard<std::mutex> lock(d.mutex);
        if (d.maintenance_pending && now_us() - d.maintenance_sent_us < 1500000U) { error = "请等待当前维护操作完成"; return false; }
        if (d.state.enabled_mask != 0U || d.state.enabled) { error = "请先全部失能并等待遥测确认"; return false; }
    }
    InfrabotJointStream p{};
    p.magic = INFRABOT_MAGIC_COMMAND; p.version = INFRABOT_COMMAND_V5; p.motor_count = INFRABOT_MOTOR_COUNT;
    p.command = INFRABOT_CMD_STORE_CONTROL_PROFILE; p.pc_timestamp_us = now_us();
    std::memcpy(p.target_velocity_mrad_s, s.velocity_mrad_s.data(), sizeof(p.target_velocity_mrad_s));
    std::memcpy(p.kp_milli, s.kp_milli.data(), sizeof(p.kp_milli));
    std::memcpy(p.kd_milli, s.kd_milli.data(), sizeof(p.kd_milli));
    std::memcpy(p.torque_mNm, s.torque_mNm.data(), sizeof(p.torque_mNm));
    std::memcpy(p.velocity_limit_mrad_s, s.velocity_limit_mrad_s.data(), sizeof(p.velocity_limit_mrad_s));
    std::memcpy(p.torque_ratio_milli, s.torque_ratio_milli.data(), sizeof(p.torque_ratio_milli));
    {
        std::lock_guard<std::mutex> lock(d.mutex);
        p.sequence = ++d.sequence;
        d.maintenance_pending = p.sequence;
        d.maintenance_sent_us = now_us();
    }
    p.crc32 = infrabot_crc32(&p, sizeof(p) - 4);
    if (!d.send_to(d.board_command, &p, sizeof(p), error)) return false;
    InfrabotMaintenanceReply reply{};
    if (!d.wait_maintenance(p.sequence, reply, 3000, error)) { error = "等待 MCU 控制参数保存应答超时"; return false; }
    status = reply.status;
    if (status != INFRABOT_REPLY_OK) { error = "MCU 保存控制参数失败：" + std::string(reply_status_text(status)); return false; }
    return true;
}

void Robot::set_telemetry_handler(std::function<void(const RobotState &)> handler) {
    std::lock_guard<std::mutex> lock(impl_->handler_mutex);
    impl_->telemetry_handler = std::move(handler);
}

const char *mode_name(Mode mode) {
    switch (mode) {
    case Mode::Idle: return "IDLE";
    case Mode::RtJoint: return "RT_JOINT";
    case Mode::RtCartesian: return "RT_CARTESIAN";
    case Mode::NrtJoint: return "NRT_JOINT";
    case Mode::NrtCartesian: return "NRT_CARTESIAN";
    case Mode::GravityFree: return "GRAVITY_FREE";
    case Mode::GravityLock: return "GRAVITY_LOCK";
    case Mode::JointImpedance:return "JOINT_IMP";
    case Mode::CartesianImpedance:return "CART_IMP";
    case Mode::JointAdmittance:return "JOINT_ADM";
    case Mode::CartesianAdmittance:return "CART_ADM";
    default: return "UNKNOWN";
    }
}

const char *status_name(OperationalStatus status) {
    static const char *const names[] = {"BOOTING", "PRECHECK", "PROFILE_REQUIRED", "DISABLED", "ENABLING",
        "IDLE_HOLD", "RT_JOINT", "RT_CARTESIAN", "NRT_JOINT", "NRT_CARTESIAN", "STOPPING", "FAULT", "RECOVERY",
        "ESTOP", "UNKNOWN", "GRAVITY_FREE", "GRAVITY_LOCK","JOINT_IMP","CART_IMP","JOINT_ADM","CART_ADM"};
    const auto index = static_cast<size_t>(status);
    return index < sizeof(names) / sizeof(names[0]) ? names[index] : "UNKNOWN";
}

} // namespace infrabot
