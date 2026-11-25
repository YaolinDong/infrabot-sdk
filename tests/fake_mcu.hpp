/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#pragma once

/* A loopback stand-in for the arm controller, speaking the real UDP protocol
 * (infrabot/protocol.h) on 127.0.0.2: telemetry (v2 or v3), DMAR, DMIO, DMOS.
 * It models what the PC relies on, with the firmware's controlled stop: a
 * DMOV cancel or a v3 STOP leaves the arm enabled, holding in IDLE. */

#include "infrabot/protocol.h"
#include "motion_session.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <thread>

namespace fake {

inline uint64_t now_us() {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000ULL + static_cast<uint64_t>(ts.tv_nsec) / 1000ULL;
}

class FakeMcu {
public:
    FakeMcu() {
        command_fd = open_socket(command_port);
        telemetry_fd = open_socket(telemetry_port);
        for (auto &position : positions) position = 150;
        positions[7] = 300;
        for (auto &state : drive_state) state = INFRABOT_DRIVE_DISABLED;
        worker = std::thread([this] { loop(); });
    }
    ~FakeMcu() {
        running = false; worker.join(); close(command_fd); close(telemetry_fd);
    }
    motion::Request last_request() {
        std::lock_guard<std::mutex> lock(mutex); return last;
    }
    int command_port = 0, telemetry_port = 0;
    std::atomic<unsigned> motion_count{0}, legacy_count{0}, legacy_after_move{0}, disable_count{0};
    std::atomic<uint32_t> last_id{0};
    std::atomic<bool> enabled{false}, drop_motion_replies{false}, block_done{false}, drop_telemetry{false};
    // Enabled drives (bit i = J(i+1)) as the MCU reports them; enabled = any.
    std::atomic<uint8_t> enabled_mask{0};
    std::atomic<bool> gripper_absent{false}; // J8 not connected: it never comes up
    void calibrate_for_test() { zero_valid = true; }
    void shift_arm_for_test(int32_t delta_mrad) {
        for (size_t i = 0; i < INFRABOT_ARM_AXES; ++i) positions[i] += delta_mrad;
    }
    std::atomic<uint8_t> frozen_feedback_mask{0};
    // Optional ideal tracking for PC workflow tests; default transport tests
    // keep the original fixed feedback positions.
    std::atomic<bool> follow_joint_commands{false};
    // Opt-in firmware-like IDLE heartbeat expiry (0 keeps existing tests unchanged).
    std::atomic<uint32_t> idle_heartbeat_timeout_ms{0};
    std::atomic<unsigned> arm_disable_count{0}, gripper_disable_count{0};
    // Gripper commands: count and the last position (mrad), speed limit
    // (mrad/s) and force (per mille), as the MCU would apply them.
    std::atomic<unsigned> gripper_count{0};
    // Measured torque reported for each motor, mNm (joint frame).
    std::array<std::atomic<int32_t>, 8> torques{};
    std::atomic<int32_t> gripper_position{0}, gripper_speed{0}, gripper_force{0};
    std::array<int32_t, 8> last_arm_targets() {
        std::lock_guard<std::mutex> lock(mutex); return last_arm;
    }
    InfrabotJointStream last_joint_stream() {
        std::lock_guard<std::mutex> lock(mutex); return last_v5;
    }
    std::atomic<uint8_t> mode{INFRABOT_CONTROL_IDLE}, profile{INFRABOT_DRIVE_MODE_MIT};
    std::atomic<bool> profile_ready{false}, supports_modes{true}, supports_cartesian{false};
    std::atomic<bool> supports_gravity{true},supports_admittance{true};
    // Hold command = measured + offset: a stationary arm sags below its command.
    std::atomic<bool> supports_hold_query{true};
    std::atomic<int32_t> hold_offset_mrad{0};
    std::array<int32_t, 8> first_arm_targets() {
        std::lock_guard<std::mutex> lock(mutex); return first_arm;
    }
    std::atomic<unsigned> cartesian_count{0};
    std::array<float, 12> first_cartesian_pose() {
        std::lock_guard<std::mutex> lock(mutex); return first_pose;
    }
    std::array<float, 12> last_cartesian_pose() {
        std::lock_guard<std::mutex> lock(mutex); return last_pose;
    }
    std::array<uint16_t, 4> last_cartesian_limits() {
        std::lock_guard<std::mutex> lock(mutex); return last_limits;
    }
    // Firmware features of the newer protocol.
    std::atomic<bool> telemetry_v3{false};
    std::atomic<bool> controlled_stop{true};
    std::atomic<bool> v1_only{false}, corrupt_info{false}, stale_info{false};
    std::atomic<uint16_t> protocol_major{INFRABOT_PROTOCOL_MAJOR}, protocol_minor{INFRABOT_PROTOCOL_MINOR};
    std::atomic<unsigned> info_count{0}, stop_count{0}, heartbeat_count{0}, clear_error_count{0};
    // Parameter service (DMPR/DMPA): supported by default; replies can be withheld or refused for ownership.
    std::atomic<bool> supports_parameters{true}, param_not_owner{false};
    std::atomic<unsigned> param_requests{0}, param_drop_replies{0};
    // Diagnostic snapshot (DMDR/DMDA): supported by default; replies can be withheld.
    std::atomic<bool> supports_diagnostics{true};
    std::atomic<unsigned> diag_requests{0}, diag_drop_replies{0};
    float param_requested(uint16_t id) {
        std::lock_guard<std::mutex> lock(param_mutex);
        for (const auto &p : params) if (p.id == id) return p.requested;
        return -1.f;
    }
    std::atomic<unsigned> keepalive_count{0};
    // SET_ARM_MODE: the self-check fails on this joint/register (0xFF: passes).
    std::atomic<uint8_t> selfcheck_fail_joint{INFRABOT_NO_JOINT}, selfcheck_fail_rid{INFRABOT_RID_VMAX};
    // A drive fault on this joint (0xFF none), with this status code.
    std::atomic<uint8_t> fault_joint{INFRABOT_NO_JOINT}, fault_state{INFRABOT_DRIVE_OVERCURRENT};
    // Controlled stop as telemetry v3 reports it.
    std::atomic<uint8_t> stop_reason{INFRABOT_STOP_NONE}, stop_phase{INFRABOT_STOP_PHASE_NONE};
    std::atomic<uint16_t> mcu_command_rx{0};
    // Resume after a controlled stop (INFRABOT_CAP_RESUME): the mode the stop
    // interrupted, the arm's measured speed (a switch refuses busy while a
    // joint moves faster than 0.05 rad/s, like the MCU), and switches to
    // refuse busy regardless (the MCU's newest sample still moving).
    std::atomic<bool> resume_supported{true};
    std::atomic<uint8_t> stop_mode{INFRABOT_CONTROL_IDLE};
    std::atomic<int32_t> arm_velocity_mrad_s{0};
    std::atomic<unsigned> busy_replies{0}, mode_requests{0};
    // SET_CONTROL_MODE answered as the MCU answers a new endpoint while the
    // previous owner of the enabled arm spoke less than 0.5 s ago (status 5).
    std::atomic<unsigned> owner_reject_replies{0};

private:
    static int open_socket(int &port) {
        int fd = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);
        assert(fd >= 0);
        int reuse = 1;
        assert(setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) == 0);
        sockaddr_in address{};
        address.sin_family = AF_INET;
        assert(inet_pton(AF_INET, "127.0.0.2", &address.sin_addr) == 1);
        assert(bind(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) == 0);
        socklen_t length = sizeof(address);
        assert(getsockname(fd, reinterpret_cast<sockaddr *>(&address), &length) == 0);
        port = ntohs(address.sin_port);
        return fd;
    }
    void reply_dmar(const InfrabotMaintenance &q, uint8_t status, uint32_t value) {
        InfrabotMaintenanceReply r{};
        r.magic = INFRABOT_MAGIC_MAINTENANCE; r.version = INFRABOT_MAINTENANCE_V1; r.command = q.command;
        r.motor_index = q.motor_index; r.sequence = q.sequence; r.status = status; r.value = value;
        r.value_type = q.value_type; r.register_id = q.register_id;
        r.crc32 = infrabot_crc32(&r, sizeof(r) - 4);
        assert(telemetry_peer.sin_port);
        assert(sendto(telemetry_fd, &r, sizeof(r), 0,
            reinterpret_cast<const sockaddr *>(&telemetry_peer), sizeof(telemetry_peer)) == sizeof(r));
    }
    void stop_motion(uint8_t reason) {
        // The firmware's controlled stop: IDLE, still enabled; older
        // firmware disabled the drives instead.
        cancelled = true;
        if(mode>=INFRABOT_CONTROL_GRAVITY_FREE && mode<=INFRABOT_CONTROL_CARTESIAN_ADMITTANCE)
            gravity_recovery_until=now_us()+500000;
        if (controlled_stop && stop_reason == INFRABOT_STOP_NONE) stop_mode = mode.load();
        mode = INFRABOT_CONTROL_IDLE;
        if (controlled_stop) { stop_reason = reason; stop_phase = INFRABOT_STOP_PHASE_NONE; ++stop_count; }
        else set_mask(0);
    }
    struct FakeParam {
        uint16_t id;
        uint8_t group, flags;
        const char *name, *unit;
        float requested, applied, minimum, maximum, default_value, rate;
    };
    // A four-entry table: two gains, a switch and a read-only value.
    std::array<FakeParam, 4> params{{
        {INFRABOT_PARAM_ID(INFRABOT_PARAM_GROUP_GRAVITY, 0), INFRABOT_PARAM_GROUP_GRAVITY, INFRABOT_PARAM_FLAG_WRITABLE,
         "gravity.damping", "Nms/rad", .8f, .8f, .1f, 3.f, .8f, 2.f},
        {INFRABOT_PARAM_ID(INFRABOT_PARAM_GROUP_IMPEDANCE, 0), INFRABOT_PARAM_GROUP_IMPEDANCE, INFRABOT_PARAM_FLAG_WRITABLE,
         "imp.joint_k", "Nm/rad", 8.f, 8.f, .5f, 30.f, 8.f, 20.f},
        {INFRABOT_PARAM_ID(INFRABOT_PARAM_GROUP_ADMITTANCE, 12), INFRABOT_PARAM_GROUP_ADMITTANCE,
         INFRABOT_PARAM_FLAG_WRITABLE | INFRABOT_PARAM_FLAG_BOOLEAN, "adm.bias_learning", "", 1.f, 1.f, 0.f, 1.f, 1.f, 0.f},
        {INFRABOT_PARAM_ID(INFRABOT_PARAM_GROUP_ADMITTANCE, 13), INFRABOT_PARAM_GROUP_ADMITTANCE, 0,
         "adm.fixed", "", 2.f, 2.f, 2.f, 2.f, 2.f, 0.f},
    }};
    std::mutex param_mutex;
    void handle_param(const uint8_t *bytes, const sockaddr_in &peer) {
        InfrabotParamRequest q{}; std::memcpy(&q, bytes, sizeof(q));
        if (q.version != INFRABOT_PARAM_V1 || q.crc32 != infrabot_crc32(&q, offsetof(InfrabotParamRequest, crc32))) return;
        ++param_requests;
        if (!supports_parameters) return;
        if (const unsigned drop = param_drop_replies) { param_drop_replies = drop - 1U; return; }
        InfrabotParamReply r{};
        r.magic = INFRABOT_MAGIC_PARAM_REPLY; r.version = INFRABOT_PARAM_V1; r.op = q.op;
        r.request_id = q.request_id; r.param_count = static_cast<uint16_t>(params.size());
        {
            std::lock_guard<std::mutex> lock(param_mutex);
            FakeParam *found = nullptr;
            if (q.op > INFRABOT_PARAM_OP_SAVE || q.reserved0 != 0 || q.reserved1 != 0) r.result = INFRABOT_PARAM_INVALID;
            else if ((q.op == INFRABOT_PARAM_OP_SET || q.op == INFRABOT_PARAM_OP_SAVE) && param_not_owner) r.result = INFRABOT_PARAM_BUSY;
            else if (q.op == INFRABOT_PARAM_OP_SAVE) r.result = INFRABOT_PARAM_UNSUPPORTED;
            else {
                if (q.op == INFRABOT_PARAM_OP_INFO) { if (q.param_id < params.size()) found = &params[q.param_id]; }
                else for (auto &p : params) if (p.id == q.param_id) found = &p;
                if (!found) r.result = INFRABOT_PARAM_UNKNOWN;
                else if (q.op == INFRABOT_PARAM_OP_SET) {
                    const bool boolean = (found->flags & INFRABOT_PARAM_FLAG_BOOLEAN) != 0;
                    if (!(found->flags & INFRABOT_PARAM_FLAG_WRITABLE)) r.result = INFRABOT_PARAM_READ_ONLY;
                    else if (!std::isfinite(q.value) || q.value < found->minimum || q.value > found->maximum ||
                             (boolean && q.value != 0.f && q.value != 1.f)) r.result = INFRABOT_PARAM_RANGE;
                    else { found->requested = q.value; if (boolean) found->applied = q.value; }
                }
            }
            if (found && r.result == INFRABOT_PARAM_OK) {
                r.param_id = found->id; r.group = found->group; r.flags = found->flags;
                r.requested = found->requested; r.applied = found->applied;
                r.minimum = found->minimum; r.maximum = found->maximum;
                r.default_value = found->default_value; r.rate = found->rate;
                std::strncpy(r.name, found->name, sizeof(r.name) - 1U);
                std::strncpy(r.unit, found->unit, sizeof(r.unit) - 1U);
            }
        }
        r.crc32 = infrabot_crc32(&r, offsetof(InfrabotParamReply, crc32));
        const ssize_t sent = sendto(command_fd, &r, sizeof(r), 0, reinterpret_cast<const sockaddr *>(&peer), sizeof(peer));
        assert(sent == static_cast<ssize_t>(sizeof(r)));
        (void)sent;
    }
    void handle_diag(const uint8_t *bytes, const sockaddr_in &peer) {
        InfrabotDiagRequest q{}; std::memcpy(&q, bytes, sizeof(q));
        if (q.version != INFRABOT_DIAG_V1 || q.reserved != 0 ||
            q.crc32 != infrabot_crc32(&q, offsetof(InfrabotDiagRequest, crc32))) return;
        ++diag_requests;
        if (!supports_diagnostics) return;
        if (const unsigned drop = diag_drop_replies) { diag_drop_replies = drop - 1U; return; }
        InfrabotDiagReply r{};
        r.magic = INFRABOT_MAGIC_DIAG_REPLY; r.version = INFRABOT_DIAG_V1; r.request_id = q.request_id;
        r.uptime_ms = 123456U; r.snapshot_seconds = 123U;
        r.control_hz = 1000U; r.control_missed = 1U; r.control_period_max_us = 1042U;
        r.can_tx = 8000U; r.can_rx = 7998U; r.can_tx_errors = 2U;
        r.command_rx = 1000U; r.command_invalid = 3U; r.telemetry_tx = 500U;
        r.can_error_events = 4U; r.control_missed_total = 5U; r.can_tx_errors_total = 6U; r.command_invalid_total = 7U;
        r.can_tec = 9U; r.can_rec = 10U; r.can_error_passive = 1U; r.can_bus_off = 0U;
        r.heap_free_bytes = 20000U; r.heap_min_free_bytes = 15000U;
        r.stack_free_control_bytes = 1200U; r.stack_free_status_bytes = 800U; r.stack_free_ethernet_bytes = 600U;
        for (uint8_t i = 0; i < 8U; ++i) r.drive_state[i] = static_cast<uint8_t>(i < 7U ? INFRABOT_DRIVE_ENABLED : INFRABOT_DRIVE_DISABLED);
        r.controller_status = INFRABOT_STATUS_IDLE_HOLD; r.invalid_reason = INFRABOT_INVALID_FORMAT;
        r.crc32 = infrabot_crc32(&r, offsetof(InfrabotDiagReply, crc32));
        const ssize_t sent = sendto(command_fd, &r, sizeof(r), 0, reinterpret_cast<const sockaddr *>(&peer), sizeof(peer));
        assert(sent == static_cast<ssize_t>(sizeof(r)));
        (void)sent;
    }
    void command(const uint8_t *bytes, size_t size, const sockaddr_in &peer) {
        uint32_t magic = 0;
        std::memcpy(&magic, bytes, sizeof(magic));
        if (size == sizeof(InfrabotParamRequest) && magic == INFRABOT_MAGIC_PARAM_REQUEST) {
            handle_param(bytes, peer);
        } else if (size == sizeof(InfrabotDiagRequest) && magic == INFRABOT_MAGIC_DIAG_REQUEST) {
            handle_diag(bytes, peer);
        } else if (size == sizeof(InfrabotInfoRequest) && magic == INFRABOT_MAGIC_INFO_REQUEST) {
            InfrabotInfoRequest q{}; std::memcpy(&q, bytes, size);
            assert(q.crc32 == infrabot_crc32(&q, sizeof(q) - 4));
            if (q.version != INFRABOT_INFO_V1 && (q.version != INFRABOT_INFO_V2 || v1_only)) return;
            InfrabotInfoReplyV2 packet{};
            auto &r = packet.info;
            r.magic = INFRABOT_MAGIC_INFO_REPLY; r.version = q.version; r.sequence = stale_info ? q.sequence - 1U : q.sequence;
            r.axes = INFRABOT_ARM_AXES; r.motor_count = INFRABOT_MOTOR_COUNT;
            r.control_mode = mode; r.motor_profile = profile; r.profile_ready = profile_ready;
            r.enabled_mask = enabled_mask;
            for (size_t i=0;i<8;++i) {
                r.position_min[i] = i == 7 ? 0.f : -12.5f;
                r.position_max[i] = i == 7 ? 5.94f : 12.5f;
                r.velocity_max[i] = i < 2 ? 10.f : 30.f;
                r.torque_max[i] = i < 2 ? 54.f : (i < 4 ? 28.f : 10.f);
            }
            r.capabilities = INFRABOT_CAP_JOINT_RT | INFRABOT_CAP_JOINT_NRT | INFRABOT_CAP_MAINTENANCE |
                             INFRABOT_CAP_PERSISTENCE | INFRABOT_CAP_GROUPS |
                             (supports_cartesian ? INFRABOT_CAP_CARTESIAN_RT | INFRABOT_CAP_CARTESIAN_NRT : 0U) |
                             (telemetry_v3 ? INFRABOT_CAP_TELEMETRY_V3 | INFRABOT_CAP_SELFCHECK : 0U) |
                             (controlled_stop ? INFRABOT_CAP_CONTROLLED_STOP : 0U) |
                             (controlled_stop && resume_supported ? INFRABOT_CAP_RESUME : 0U) |
                             (supports_gravity ? INFRABOT_CAP_LOCAL_GRAVITY : 0U) | (supports_admittance?INFRABOT_CAP_ADMITTANCE:0U) |
                             (supports_parameters ? INFRABOT_CAP_PARAMETERS : 0U) |
                             (supports_diagnostics ? INFRABOT_CAP_DIAGNOSTICS : 0U);
            r.crc32 = infrabot_crc32(&r, sizeof(r) - 4);
            size_t length = sizeof(r);
            if (q.version == INFRABOT_INFO_V2) {
                length = sizeof(packet);
                packet.protocol_major = protocol_major;
                packet.protocol_minor = protocol_minor;
                std::strcpy(packet.firmware_version, "fixture-123");
                std::strcpy(packet.board, "test-h743");
                std::strcpy(packet.arm_model, "H7C bare v1");
                packet.crc32 = infrabot_crc32(&packet, offsetof(InfrabotInfoReplyV2, crc32));
            }
            if (corrupt_info) r.crc32 ^= 1;
            ++info_count;
            // From the telemetry port to the requesting command socket.
            assert(sendto(telemetry_fd, &packet, length, 0, reinterpret_cast<const sockaddr *>(&peer),
                          sizeof(peer)) == static_cast<ssize_t>(length));
        } else if (size == sizeof(InfrabotCommand)) {
            InfrabotCommand packet{}; std::memcpy(&packet, bytes, size);
            assert(packet.magic == INFRABOT_MAGIC_COMMAND && packet.version == INFRABOT_COMMAND_V3);
            assert(packet.crc32 == infrabot_crc32(&packet, sizeof(packet) - 4));
            // Like the MCU: an enable stages only drives that answer (an
            // absent gripper fails the whole group); without the arm, IDLE.
            const uint8_t c = packet.command;
            const bool enable = c == INFRABOT_CMD_ENABLE || c == INFRABOT_CMD_ENABLE_ARM ||
                                c == INFRABOT_CMD_ENABLE_GRIPPER || c == INFRABOT_CMD_SET_ZERO_ENABLE;
            if (enable) {
                assert(profile_ready && mode == INFRABOT_CONTROL_IDLE);
                stop_reason = INFRABOT_STOP_NONE;
            }
            if (c == INFRABOT_CMD_ENABLE && !gripper_absent) set_mask(0xff);
            if (c == INFRABOT_CMD_ENABLE_ARM) set_mask(enabled_mask | 0x7f);
            if (c == INFRABOT_CMD_ENABLE_GRIPPER && !gripper_absent) set_mask(enabled_mask | 0x80);
            if (c == INFRABOT_CMD_SET_ZERO || c == INFRABOT_CMD_SET_ZERO_ENABLE) {
                zero_valid = true;
                if (c == INFRABOT_CMD_SET_ZERO_ENABLE) set_mask(0xff);
            }
            if (c == INFRABOT_CMD_DISABLE) {
                set_mask(0); mode = INFRABOT_CONTROL_IDLE; ++disable_count; cancelled = true;
            }
            if (c == INFRABOT_CMD_DISABLE_ARM) {
                set_mask(enabled_mask & 0x80); mode = INFRABOT_CONTROL_IDLE; ++arm_disable_count; cancelled = true;
            }
            if (c == INFRABOT_CMD_DISABLE_GRIPPER) { set_mask(enabled_mask & 0x7f); ++gripper_disable_count; }
            if (c == INFRABOT_CMD_STOP && controlled_stop && enabled) stop_motion(INFRABOT_STOP_REQUESTED);
            if (c == INFRABOT_CMD_HOLD_HEARTBEAT) {
                ++heartbeat_count;
                last_heartbeat_us = now_us();
                if (idle_heartbeat_timeout_ms && stop_phase == INFRABOT_STOP_PHASE_HOLDING)
                    stop_phase = INFRABOT_STOP_PHASE_NONE;
            }
            ack = packet.sequence;
        } else if (size == sizeof(InfrabotMaintenance)) {
            InfrabotMaintenance q{}; std::memcpy(&q, bytes, size);
            assert(q.crc32 == infrabot_crc32(bytes, size - 4));
            uint8_t status = INFRABOT_REPLY_OK;
            uint32_t value = 0;
            if (q.command == INFRABOT_CMD_SET_ARM_MODE) {
                assert(!enabled && mode == INFRABOT_CONTROL_IDLE);
                profile = q.mode;
                if (selfcheck_fail_joint != INFRABOT_NO_JOINT) {
                    profile_ready = false; status = INFRABOT_REPLY_MISMATCH;
                    value = INFRABOT_SELFCHECK_REPLY_VALUE(selfcheck_fail_joint.load(), selfcheck_fail_rid.load(),
                                                           INFRABOT_SELFCHECK_MISMATCH);
                    selfcheck = INFRABOT_SELFCHECK_MISMATCH;
                } else {
                    profile_ready = true; value = q.mode; selfcheck = INFRABOT_SELFCHECK_OK;
                }
            } else if (q.command == INFRABOT_CMD_SET_CONTROL_MODE) {
                ++mode_requests;
                if (owner_reject_replies > 0) { --owner_reject_replies; status = INFRABOT_REPLY_DISABLE_FIRST; }
                else if(q.mode>INFRABOT_CONTROL_CARTESIAN_ADMITTANCE || (q.mode>=INFRABOT_CONTROL_JOINT_ADMITTANCE && !supports_admittance))
                    status=INFRABOT_REPLY_UNSUPPORTED;
                else if (!supports_cartesian && (q.mode == INFRABOT_CONTROL_RT_CARTESIAN || q.mode == INFRABOT_CONTROL_NRT_CARTESIAN))
                    status = INFRABOT_REPLY_UNSUPPORTED;
                else if (q.mode == mode) {
                    // Retries change nothing; IDLE -> IDLE acknowledges a stop that is over.
                    if (q.mode == INFRABOT_CONTROL_IDLE && resume_supported && stop_phase == INFRABOT_STOP_PHASE_NONE) {
                        stop_reason = INFRABOT_STOP_NONE; stop_mode = INFRABOT_CONTROL_IDLE;
                    }
                }
                else if (q.mode != INFRABOT_CONTROL_IDLE && (enabled_mask & 0x7f) != 0x7f) status = INFRABOT_REPLY_NOT_READY; // the arm
                else if (std::abs(arm_velocity_mrad_s.load()) > 50 || busy_replies > 0) {
                    status = INFRABOT_REPLY_BUSY;
                    if (busy_replies > 0) --busy_replies;
                }
                else {
                    mode = q.mode; session = 0;
                    stop_reason = INFRABOT_STOP_NONE; stop_mode = INFRABOT_CONTROL_IDLE;
                }
                value = mode;
            } else if (q.command == INFRABOT_CMD_GRIPPER) {
                assert(q.motor_index == INFRABOT_GRIPPER_INDEX);
                if (!(enabled_mask & 0x80)) status = INFRABOT_REPLY_NOT_READY;
                else {
                    gripper_position = q.target_position_mrad[INFRABOT_GRIPPER_INDEX];
                    gripper_speed = q.velocity_limit_mrad_s;
                    gripper_force = q.torque_ratio_milli;
                    ++gripper_count;
                }
            } else if (q.command == INFRABOT_CMD_GET_HOLD_POSITION) {
                assert(q.motor_index < 8);
                if (!supports_hold_query) status = INFRABOT_REPLY_INVALID;
                else if (!(enabled_mask & (1u << q.motor_index))) status = INFRABOT_REPLY_NOT_READY;
                else value = static_cast<uint32_t>(positions[q.motor_index] + hold_offset_mrad);
            } else if (q.command == INFRABOT_CMD_REQUEST_FEEDBACK) {
                assert(q.motor_index < 8);
                status = INFRABOT_REPLY_QUEUED;
            } else if (q.command == INFRABOT_CMD_CLEAR_ERROR) {
                assert(q.motor_index < 8);
                if (enabled_mask & (1u << q.motor_index)) status = INFRABOT_REPLY_DISABLE_FIRST;
                else {
                    status = INFRABOT_REPLY_QUEUED;
                    ++clear_error_count;
                    if (fault_joint == q.motor_index) fault_joint = INFRABOT_NO_JOINT;
                }
            } else assert(false && "Unexpected maintenance command");
            reply_dmar(q, status, value);
        } else if (size == sizeof(InfrabotJointStream)) {
            InfrabotJointStream packet{}; std::memcpy(&packet, bytes, size);
            assert(packet.version == INFRABOT_COMMAND_V5 && packet.crc32 == infrabot_crc32(bytes, size - 4));
            // One in-flight old RT datagram may arrive after an IDLE ack.
            // Real MCU rejects it without updating the hold target.
            if (!enabled || mode != INFRABOT_CONTROL_RT_JOINT) return;
            if (session) ++legacy_after_move;
            if (legacy_count == 0) {
                std::lock_guard<std::mutex> lock(mutex);
                std::memcpy(first_arm.data(), packet.target_position_mrad, sizeof(first_arm));
            }
            {
                std::lock_guard<std::mutex> lock(mutex);
                std::memcpy(last_arm.data(), packet.target_position_mrad, sizeof(last_arm));
                last_v5 = packet;
            }
            ++legacy_count;
            if (follow_joint_commands)
                for (size_t i = 0; i < INFRABOT_ARM_AXES; ++i)
                    positions[i] = packet.target_position_mrad[i] - hold_offset_mrad.load();
            ack = packet.sequence;
        } else if (size == sizeof(InfrabotCartesianStream)) {
            InfrabotCartesianStream packet{}; std::memcpy(&packet, bytes, size);
            assert(packet.version == INFRABOT_COMMAND_V7 && packet.crc32 == infrabot_crc32(bytes, size - 4));
            if (!enabled || mode != INFRABOT_CONTROL_RT_CARTESIAN) return;
            if (cartesian_count == 0) {
                std::lock_guard<std::mutex> lock(mutex);
                std::memcpy(first_pose.data(), packet.pose, sizeof(packet.pose));
            }
            {
                std::lock_guard<std::mutex> lock(mutex);
                std::memcpy(last_pose.data(), packet.pose, sizeof(packet.pose));
                std::memcpy(last_limits.data(), packet.cartesian_limits, sizeof(packet.cartesian_limits));
            }
            ++cartesian_count;
            ack = packet.sequence;
        } else if (size == sizeof(motion::Request)) {
            motion::Request q{}; std::memcpy(&q, bytes, size);
            assert(q.magic == INFRABOT_MAGIC_MOVE_REQUEST && q.crc32 == motion::crc32(&q, size - 4));
            if ((q.op == motion::MoveJ || q.op == motion::MoveP) && (q.session != session || q.motion_id != last_id)) {
                assert(enabled && (mode == INFRABOT_CONTROL_NRT_JOINT || mode == INFRABOT_CONTROL_NRT_CARTESIAN));
                { std::lock_guard<std::mutex> lock(mutex); last = q; }
                session = q.session; last_id = q.motion_id;
                move_start = now_us(); cancelled = false; ++motion_count;
            }
            if (q.op == motion::Keepalive) ++keepalive_count;
            if (q.op == motion::Cancel && !cancelled) stop_motion(INFRABOT_STOP_REQUESTED);
            if (drop_motion_replies) return;
            motion::Reply r{};
            r.magic = INFRABOT_MAGIC_MOVE_REPLY; r.version = INFRABOT_MOVE_V1; r.session = q.session;
            r.request_id = r.active_id = q.motion_id;
            r.elapsed_ms = static_cast<uint32_t>((now_us() - move_start) / 1000);
            r.duration_ms = 150;
            r.state = cancelled ? motion::Cancelled :
                (r.elapsed_ms >= 150 && !block_done) ? motion::Done :
                r.elapsed_ms >= 80 ? motion::Settling : motion::Running;
            if (r.state == motion::Done) {
                const auto request = last_request();
                if (request.op == motion::MoveJ)
                    for (size_t i = 0; i < 7; ++i)
                        positions[i] = static_cast<int32_t>(std::lround(request.command.joint.target[i] * 1000.0f));
                r.arrived_mask = 0x7f;
            }
            r.enabled_mask = enabled_mask;
            r.fresh_mask = 0xff;
            r.crc32 = motion::crc32(&r, sizeof(r) - 4);
            assert(sendto(command_fd, &r, sizeof(r), 0, reinterpret_cast<const sockaddr *>(&peer),
                          sizeof(peer)) == sizeof(r));
        } else assert(false && "Unexpected command packet");
    }
    void set_mask(uint8_t mask) { enabled_mask = mask; enabled = mask != 0; }
    uint8_t controller_status() const {
        if (!enabled) return INFRABOT_STATUS_DISABLED;
        if (now_us()<gravity_recovery_until) return INFRABOT_STATUS_RECOVERY;
        switch (mode.load()) {
        case INFRABOT_CONTROL_RT_JOINT: return INFRABOT_STATUS_RT_JOINT;
        case INFRABOT_CONTROL_RT_CARTESIAN: return INFRABOT_STATUS_RT_CARTESIAN;
        case INFRABOT_CONTROL_NRT_JOINT: return INFRABOT_STATUS_NRT_JOINT;
        case INFRABOT_CONTROL_NRT_CARTESIAN: return INFRABOT_STATUS_NRT_CARTESIAN;
        case INFRABOT_CONTROL_GRAVITY_FREE: return INFRABOT_STATUS_GRAVITY_FREE;
        case INFRABOT_CONTROL_GRAVITY_LOCK: return INFRABOT_STATUS_GRAVITY_LOCK;
        case INFRABOT_CONTROL_JOINT_IMPEDANCE:return INFRABOT_STATUS_JOINT_IMPEDANCE;
        case INFRABOT_CONTROL_CARTESIAN_IMPEDANCE:return INFRABOT_STATUS_CARTESIAN_IMPEDANCE;
        case INFRABOT_CONTROL_JOINT_ADMITTANCE:return INFRABOT_STATUS_JOINT_ADMITTANCE;
        case INFRABOT_CONTROL_CARTESIAN_ADMITTANCE:return INFRABOT_STATUS_CARTESIAN_ADMITTANCE;
        default: return INFRABOT_STATUS_IDLE_HOLD;
        }
    }
    void send_telemetry(uint32_t sequence) {
        InfrabotTelemetryV3 packet{};
        InfrabotTelemetry &head = packet.head;
        head.magic = INFRABOT_MAGIC_TELEMETRY;
        head.version = telemetry_v3 ? INFRABOT_TELEMETRY_V3 : INFRABOT_TELEMETRY_V2;
        head.motor_count = INFRABOT_MOTOR_COUNT; head.sequence = sequence;
        head.mcu_timestamp_us = now_us();
        head.command_applied_sequence = ack;
        head.command_active = enabled;
        head.status = static_cast<uint8_t>((enabled ? INFRABOT_STATUS_FLAG_ENABLED : 0U) |
                                           (zero_valid ? INFRABOT_STATUS_FLAG_ZERO_VALID : 0U) |
                                           (controller_status() << 2));
        head.enabled_mask = enabled_mask;
        head.mode = supports_modes ? static_cast<uint8_t>(INFRABOT_MODE_FLAG_SUPPORTED |
            (profile_ready ? INFRABOT_MODE_FLAG_PROFILE_READY : 0U) | (profile.load() << 4) | mode.load()) : 0;
        for (size_t i = 0; i < 8; ++i) {
            head.motor[i].position_mrad = positions[i];
            if (i < 7) head.motor[i].velocity_mrad_s = arm_velocity_mrad_s;
            head.motor[i].state = fault_joint == i ? fault_state.load() :
                (enabled_mask & (1u << i)) ? INFRABOT_DRIVE_ENABLED : INFRABOT_DRIVE_DISABLED;
            head.motor[i].torque_mNm = torques[i];
            head.motor[i].feedback_count = (frozen_feedback_mask & (1u << i)) ? 1 : static_cast<uint16_t>(sequence);
            head.motor[i].temperature_mos = 30;
            packet.command[i].position_mrad = positions[i] + hold_offset_mrad;
            packet.command[i].torque_mNm = 100 * static_cast<int32_t>(i);
            packet.command[i].temperature_coil = 35;
            packet.command[i].flags = (enabled_mask & (1u << i)) ? INFRABOT_MOTOR_ENABLED : 0U;
            packet.command[i].drive_mode = i == 7 ? INFRABOT_DRIVE_MODE_FORCE_POS : profile.load();
        }
        packet.link.command_rx = mcu_command_rx;
        packet.safety.stop_reason = stop_reason;
        packet.safety.stop_phase = stop_phase;
        packet.safety.stop_mode = stop_reason != INFRABOT_STOP_NONE ? stop_mode.load() : INFRABOT_CONTROL_IDLE;
        packet.safety.stop_joint = INFRABOT_NO_JOINT;
        packet.safety.selfcheck = selfcheck;
        packet.safety.selfcheck_joint = selfcheck == INFRABOT_SELFCHECK_MISMATCH ? selfcheck_fail_joint.load() : INFRABOT_NO_JOINT;
        packet.safety.selfcheck_register = selfcheck_fail_rid;
        const float read = 8.0f, expected = 10.0f;
        std::memcpy(&packet.safety.selfcheck_value, &read, 4);
        std::memcpy(&packet.safety.selfcheck_expected, &expected, 4);
        const size_t size = telemetry_v3 ? sizeof(InfrabotTelemetryV3) : sizeof(InfrabotTelemetry);
        assert(sendto(telemetry_fd, &packet, size, 0, reinterpret_cast<const sockaddr *>(&telemetry_peer),
                      sizeof(telemetry_peer)) == static_cast<ssize_t>(size));
    }
    void loop() {
        uint64_t next_telemetry = 0;
        uint32_t sequence = 0;
        while (running) {
            pollfd fd[2] = {{command_fd, POLLIN, 0}, {telemetry_fd, POLLIN, 0}};
            poll(fd, 2, 1);
            for (int i = 0; i < 2; ++i) {
                if (!(fd[i].revents & POLLIN)) continue;
                uint8_t buffer[512]; sockaddr_in peer{}; socklen_t length = sizeof(peer);
                const auto size = recvfrom(fd[i].fd, buffer, sizeof(buffer), 0,
                                           reinterpret_cast<sockaddr *>(&peer), &length);
                if (size > 0) {
                    // Reject anything not sourced from the loopback test client.
                    assert((ntohl(peer.sin_addr.s_addr) >> 24) == 127);
                    if (i == 0) command(buffer, static_cast<size_t>(size), peer);
                    else telemetry_peer = peer;
                }
            }
            const auto now = now_us();
            if (idle_heartbeat_timeout_ms && enabled && mode == INFRABOT_CONTROL_IDLE && last_heartbeat_us &&
                now > last_heartbeat_us + uint64_t(idle_heartbeat_timeout_ms) * 1000 &&
                stop_phase == INFRABOT_STOP_PHASE_NONE) {
                stop_motion(INFRABOT_STOP_STREAM_LOST);
                stop_phase = INFRABOT_STOP_PHASE_HOLDING;
            }
            if (telemetry_peer.sin_port && now >= next_telemetry && !drop_telemetry) {
                send_telemetry(++sequence);
                next_telemetry = now + 1000;
            }
        }
    }
    int command_fd = -1, telemetry_fd = -1;
    sockaddr_in telemetry_peer{};
    std::atomic<bool> running{true};
    std::thread worker;
    std::mutex mutex;
    motion::Request last{};
    InfrabotJointStream last_v5{};
    std::array<float, 12> first_pose{}, last_pose{};
    std::array<uint16_t, 4> last_limits{};
    std::array<int32_t, 8> first_arm{}, last_arm{};
    std::atomic<uint64_t> session{0};
    uint64_t move_start = 0;
    uint64_t last_heartbeat_us = 0; // only read/written by the MCU worker
    uint64_t gravity_recovery_until = 0;
    std::atomic<uint32_t> ack{0};
    std::atomic<bool> cancelled{false}, zero_valid{false};
    std::atomic<uint8_t> selfcheck{INFRABOT_SELFCHECK_NOT_RUN};
    std::array<std::atomic<int32_t>, 8> positions{};
    std::array<std::atomic<uint8_t>, 8> drive_state{};
};

} // namespace fake
