/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#pragma once

// Native, nonblocking DMOV state machine of the movej/movep session.  The SDK
// (infrabot::Robot) owns one and serializes it with its session mutex.
#include "infrabot/protocol.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>

namespace motion {
constexpr size_t axes = INFRABOT_ARM_AXES;
enum Op : uint8_t {
    MoveJ = INFRABOT_MOVE_OP_MOVEJ, MoveP = INFRABOT_MOVE_OP_MOVEP,
    Keepalive = INFRABOT_MOVE_OP_KEEPALIVE, Cancel = INFRABOT_MOVE_OP_CANCEL,
    Query = INFRABOT_MOVE_OP_QUERY
};
enum State : uint8_t {
    Idle = INFRABOT_MOVE_STATE_IDLE, Running = INFRABOT_MOVE_STATE_RUNNING,
    Settling = INFRABOT_MOVE_STATE_SETTLING, Done = INFRABOT_MOVE_STATE_DONE,
    Failed = INFRABOT_MOVE_STATE_FAILED, Cancelled = INFRABOT_MOVE_STATE_CANCELLED
};
enum class Phase { Off, Preparing, Waiting, Running, Settling, Ready, Done, Cancelling, Cancelled, Error };

using Request = InfrabotMoveRequest;
using Reply = InfrabotMoveReply;

// The DMOV payloads (protocol.h) with defaults and std::array members.
struct Parameters {
    uint32_t timeout_ms = 30000, dwell_ms = 100;
    float position_tolerance = 0.01f, velocity_tolerance = 0.05f;
    std::array<float, axes> target{}, velocity_max{}, acceleration_max{}, kp{}, kd{}, torque{};
};

// pose is xyz in metres followed by a row-major 3x3 rotation matrix.
// velocity_max/acceleration_max are per-joint caps; the tool speed comes from
// cartesian_limits {mm/s, mrad/s, mm/s^2, mrad/s^2}, 0 = MCU default.
struct CartesianParameters {
    uint32_t timeout_ms = 30000, dwell_ms = 100;
    float position_tolerance = 0.01f, velocity_tolerance = 0.05f;
    std::array<float, 12> pose{};
    std::array<float, axes> velocity_max{}, acceleration_max{}, kp{}, kd{};
    std::array<uint16_t, 4> cartesian_limits{};
};
static_assert(sizeof(Parameters) == sizeof(InfrabotMoveJoint) &&
              offsetof(Parameters, target) == offsetof(InfrabotMoveJoint, target) &&
              offsetof(Parameters, torque) == offsetof(InfrabotMoveJoint, torque),
              "Parameters must have the InfrabotMoveJoint layout");
static_assert(sizeof(CartesianParameters) == sizeof(InfrabotMoveCartesian) &&
              offsetof(CartesianParameters, pose) == offsetof(InfrabotMoveCartesian, pose) &&
              offsetof(CartesianParameters, kd) == offsetof(InfrabotMoveCartesian, kd) &&
              offsetof(CartesianParameters, cartesian_limits) ==
                  offsetof(InfrabotMoveCartesian, cartesian_limits),
              "CartesianParameters must have the InfrabotMoveCartesian layout");

inline uint32_t crc32(const void *bytes, size_t length) { return infrabot_crc32(bytes, length); }

inline Request blank_request() {
    Request request{};
    request.magic = INFRABOT_MAGIC_MOVE_REQUEST;
    request.version = INFRABOT_MOVE_V1;
    return request;
}

inline const char *phase_name(Phase phase) {
    switch (phase) {
    case Phase::Off: return "未启动";
    case Phase::Preparing: return "使能准备 / 固定目标保持";
    case Phase::Waiting: return "等待 MCU 接受";
    case Phase::Running: return "运行中";
    case Phase::Settling: return "等待实际到位并稳定";
    case Phase::Ready: return "已就绪 / 保持当前位置";
    case Phase::Done: return "已到位 / 保持目标";
    case Phase::Cancelling: return "停止中 / 已请求受控停止";
    case Phase::Cancelled: return "已受控停止；机械臂在 IDLE 保持";
    case Phase::Error: return "失败 / 已请求受控停止";
    }
    return "未知";
}
inline const char *reason_name(uint32_t reason) {
    static const char *const names[] = {"OK", "BUSY", "NOT_READY", "INVALID", "UNSUPPORTED",
        "BAD_SESSION", "STALE_ID", "COMM_TIMEOUT", "MOTION_TIMEOUT", "FEEDBACK_STALE",
        "MOTOR_FAULT", "CANCELLED", "TX_ERROR"};
    return reason < sizeof(names) / sizeof(names[0]) ? names[reason] : "UNKNOWN";
}
inline bool valid(const Parameters &p, std::string &error) {
    constexpr float vmax[] = {45, 45, 10, 10, 30, 30, 30};
    constexpr float tmax[] = {54, 54, 28, 28, 10, 10, 10};
    if (p.timeout_ms < 100 || p.timeout_ms > 600000 || p.dwell_ms < 20 || p.dwell_ms > 5000 ||
        !std::isfinite(p.position_tolerance) || p.position_tolerance < .001f || p.position_tolerance > .05f ||
        !std::isfinite(p.velocity_tolerance) || p.velocity_tolerance < .01f || p.velocity_tolerance > .1f) {
        error = "到位阈值/稳定时间/总超时超出允许范围"; return false;
    }
    for (size_t i = 0; i < axes; ++i) {
        if (!std::isfinite(p.target[i]) || std::fabs(p.target[i]) > 12.5f ||
            !std::isfinite(p.velocity_max[i]) || p.velocity_max[i] <= 0 || p.velocity_max[i] > vmax[i] ||
            !std::isfinite(p.acceleration_max[i]) || p.acceleration_max[i] <= 0 || p.acceleration_max[i] > 10 ||
            !std::isfinite(p.kp[i]) || p.kp[i] <= 0 || p.kp[i] > 500 ||
            !std::isfinite(p.kd[i]) || p.kd[i] < 0 || p.kd[i] > 5 ||
            !std::isfinite(p.torque[i]) || std::fabs(p.torque[i]) > tmax[i]) {
            error = "J" + std::to_string(i + 1) + " 的 movej 参数无效"; return false;
        }
    }
    return true;
}

inline bool valid(const CartesianParameters &p, std::string &error) {
    constexpr float vmax[] = {45, 45, 10, 10, 30, 30, 30};
    constexpr float tmax[] = {500, 500, 500, 500, 500, 500, 500};
    if (p.timeout_ms < 100 || p.timeout_ms > 600000 || p.dwell_ms < 20 || p.dwell_ms > 5000 ||
        !std::isfinite(p.position_tolerance) || p.position_tolerance < .001f || p.position_tolerance > .05f ||
        !std::isfinite(p.velocity_tolerance) || p.velocity_tolerance < .01f || p.velocity_tolerance > .1f) {
        error = "笛卡尔到位阈值/稳定时间/总超时超出允许范围"; return false;
    }
    for (size_t i = 0; i < 12; ++i) {
        if (!std::isfinite(p.pose[i]) || (i < 3 && std::fabs(p.pose[i]) > 2.0f)) {
            error = "笛卡尔目标位姿无效"; return false;
        }
    }
    constexpr uint16_t limit_max[] = {INFRABOT_CARTESIAN_MAX_LINEAR_VELOCITY,
        INFRABOT_CARTESIAN_MAX_ANGULAR_VELOCITY, INFRABOT_CARTESIAN_MAX_LINEAR_ACCELERATION,
        INFRABOT_CARTESIAN_MAX_ANGULAR_ACCELERATION};
    for (size_t i = 0; i < 4; ++i) {
        if (p.cartesian_limits[i] > limit_max[i]) { error = "笛卡尔速度/加速度上限超出允许范围"; return false; }
    }
    for (size_t i = 0; i < axes; ++i) {
        if (!std::isfinite(p.velocity_max[i]) || p.velocity_max[i] <= 0 || p.velocity_max[i] > vmax[i] ||
            // Per-joint caps only; the MCU accepts up to 100 rad/s^2.
            !std::isfinite(p.acceleration_max[i]) || p.acceleration_max[i] <= 0 || p.acceleration_max[i] > 100 ||
            !std::isfinite(p.kp[i]) || p.kp[i] <= 0 || p.kp[i] > tmax[i] ||
            !std::isfinite(p.kd[i]) || p.kd[i] < 0 || p.kd[i] > 5) {
            error = "笛卡尔运动参数无效（速度/加速度/Kp/Kd）"; return false;
        }
    }
    return true;
}

struct Snapshot {
    Phase phase = Phase::Off;
    uint32_t id = 0;
    Reply reply{};
    bool has_reply = false;
    std::string error;
    uint64_t reply_age_us = UINT64_MAX;
    bool can_execute = false;
};

/* One NRT session: the hold move (id 1) on entering the mode, then the
 * caller's moves, kept alive at 100 Hz.  A failure, a lost DMOS stream or a
 * cancel ends it with DMOV cancels for 250 ms: the controller stops the arm
 * in a controlled way and holds it (it is not disabled). */
class Session {
public:
    Session() : request_(blank_request()) {}
    bool preparing() const { return phase_ == Phase::Preparing; }
    bool engaged() const {
        return phase_ != Phase::Off && phase_ != Phase::Cancelled && phase_ != Phase::Error;
    }
    bool cartesian() const { return cartesian_mode_; }
    const Parameters &parameters() const { return parameters_; }
    const CartesianParameters &cartesian_parameters() const { return cartesian_parameters_; }
    uint64_t started_us() const { return started_; }
    Snapshot snapshot(uint64_t now) const {
        return {phase_, request_.motion_id, reply_, has_reply_, error_,
            last_reply_ ? now - last_reply_ : UINT64_MAX,
            (phase_ == Phase::Ready || phase_ == Phase::Done) && now - last_reply_ <= 100000};
    }
    bool prepare(const Parameters &p, uint64_t session, uint64_t now, std::string &error) {
        if (engaged() || cleanup_until_ || !session) { error = "请先停止当前会话"; return false; }
        if (!valid(p, error)) return false;
        *this = Session{};
        parameters_ = p;
        cartesian_mode_ = false;
        request_.session = session;
        request_.motion_id = 1;
        phase_ = Phase::Preparing;
        started_ = now;
        return true;
    }
    void begin_hold(const std::array<float, axes> &positions, uint64_t now) {
        if (!preparing()) return;
        parameters_.target = positions;
        // Startup hold gets its own settle budget, independent of the next requested move.
        parameters_.timeout_ms = 10000;
        begin(now, true);
    }
    void begin_hold_cartesian(uint64_t now) {
        if (!preparing() || !cartesian_mode_) return;
        cartesian_parameters_.timeout_ms = 10000;
        begin(now, true);
    }
    bool movej(const Parameters &p, uint64_t now, std::string &error) {
        if (!snapshot(now).can_execute) { error = "当前动作未完成、应答过期或尚未使能就绪"; return false; }
        if (!valid(p, error)) return false;
        if (request_.motion_id == UINT32_MAX) { error = "动作 ID 已用尽，请重新进入 move 模式"; return false; }
        parameters_ = p;
        cartesian_mode_ = false;
        ++request_.motion_id;
        begin(now, false);
        return true;
    }
    bool prepare_cartesian(const CartesianParameters &p, uint64_t session,
                           uint64_t now, std::string &error) {
        if (engaged() || cleanup_until_ || !session) { error = "请先停止当前会话"; return false; }
        if (!valid(p, error)) return false;
        *this = Session{};
        cartesian_parameters_ = p;
        request_.session = session;
        request_.motion_id = 1;
        cartesian_mode_ = true;
        phase_ = Phase::Preparing;
        started_ = now;
        return true;
    }
    bool movep(const CartesianParameters &p, uint64_t now, std::string &error) {
        if (!snapshot(now).can_execute) { error = "当前动作未完成、应答过期或尚未使能就绪"; return false; }
        if (!valid(p, error)) return false;
        if (request_.motion_id == UINT32_MAX) { error = "动作 ID 已用尽，请重新进入 move 模式"; return false; }
        cartesian_parameters_ = p;
        ++request_.motion_id;
        cartesian_mode_ = true;
        begin(now, false);
        return true;
    }
    void cancel(uint64_t now) {
        if (phase_ == Phase::Off || phase_ == Phase::Cancelled) return;
        if (phase_ != Phase::Error) phase_ = sent_ ? Phase::Cancelling : Phase::Cancelled;
        cleanup_until_ = now + 250000;
        next_send_ = 0;
    }
    void fail(const std::string &message, uint64_t now) {
        if (phase_ == Phase::Error) return;
        error_ = message;
        phase_ = Phase::Error;
        cleanup_until_ = now + 250000;
        next_send_ = 0;
    }

    // Called from the service thread, never sleeps. Returns a packet at most every 10 ms.
    bool poll(uint64_t now, Request &out) {
        if (phase_ == Phase::Preparing && now - started_ > 2000000)
            fail("使能/静止确认超时，检查各电机状态、模式和负载", now);
        if (engaged() && !preparing() && phase_ != Phase::Cancelling) {
            if (now - (last_reply_ ? last_reply_ : started_) > 300000)
                fail("DMOV 应答超时：检查固件版本/网线，未视为到位", now);
            else if (phase_ != Phase::Ready && phase_ != Phase::Done &&
                     now - started_ > uint64_t(parameters_.timeout_ms) * 1000 + 1000000)
                fail("运动总超时，未视为到位", now);
        }
        if (cleanup_until_ && now >= cleanup_until_) {
            if (phase_ == Phase::Cancelling) {
                phase_ = Phase::Error;
                error_ = "停止未获确认；检查 MCU 和各轴状态";
            }
            cleanup_until_ = 0;
        }
        if (now < next_send_) return false;
        if (cleanup_until_) {
            out = request_;
            out.op = Cancel;
        } else {
            if (!engaged() || preparing()) return false;
            out = request_;
            out.op = (phase_ == Phase::Ready || phase_ == Phase::Done) ? Keepalive :
                     (cartesian_mode_ ? MoveP : MoveJ);
            sent_ = true;
        }
        if (out.op != MoveJ && out.op != MoveP) std::memset(&out.command, 0, sizeof(out.command));
        out.crc32 = crc32(&out, sizeof(out) - 4);
        next_send_ = next_send_ ? next_send_ + 10000 : now + 10000;
        if (next_send_ <= now) next_send_ = now + 10000; // No catch-up burst.
        return true;
    }
    bool receive(const void *data, size_t length, uint64_t now) {
        if (length != sizeof(Reply)) return false;
        Reply r{};
        std::memcpy(&r, data, sizeof(r));
        if (r.magic != INFRABOT_MAGIC_MOVE_REPLY || r.version != INFRABOT_MOVE_V1 ||
            r.crc32 != crc32(&r, sizeof(r) - 4) ||
            r.session != request_.session || r.request_id != request_.motion_id ||
            !std::isfinite(r.max_error_rad) || r.state > Cancelled) return false;
        if (phase_ == Phase::Error || phase_ == Phase::Cancelled) return true; // Latched fault stays visible.
        if (phase_ == Phase::Cancelling) {
            if (r.result == 0 && r.state == Cancelled && r.active_id == request_.motion_id) {
                phase_ = Phase::Cancelled;
                cleanup_until_ = 0;
            }
            return true;
        }
        if (!engaged() || preparing()) return true;
        if (r.result || r.state == Failed || r.state == Cancelled) {
            fail(std::string("MCU: ") + reason_name(r.result ? r.result : r.reason), now);
            reply_ = r; has_reply_ = true; last_reply_ = now;
            return true;
        }
        if (r.active_id != request_.motion_id || r.state == Idle) return false;
        // Delayed/reordered status cannot turn DONE back into RUNNING.
        if ((phase_ == Phase::Ready || phase_ == Phase::Done) && r.state != Done) return false;
        if (has_reply_ && r.elapsed_ms < reply_.elapsed_ms) return false;
        reply_ = r; has_reply_ = true; last_reply_ = now;
        phase_ = r.state == Done ? (hold_ ? Phase::Ready : Phase::Done) :
                 r.state == Settling ? Phase::Settling : Phase::Running;
        return true;
    }
private:
    void begin(uint64_t now, bool hold) {
        request_.op = cartesian_mode_ ? MoveP : MoveJ;
        if (cartesian_mode_)
            std::memcpy(&request_.command.cartesian, &cartesian_parameters_, sizeof(cartesian_parameters_));
        else
            std::memcpy(&request_.command.joint, &parameters_, sizeof(parameters_));
        phase_ = Phase::Waiting;
        hold_ = hold;
        has_reply_ = false;
        reply_ = {};
        started_ = now;
        last_reply_ = 0;
        next_send_ = 0;
    }
    Parameters parameters_{};
    CartesianParameters cartesian_parameters_{};
    Request request_{};
    Reply reply_{};
    Phase phase_ = Phase::Off;
    uint64_t started_ = 0, last_reply_ = 0, next_send_ = 0, cleanup_until_ = 0;
    bool has_reply_ = false, hold_ = false, sent_ = false, cartesian_mode_ = false;
    std::string error_;
};
} // namespace motion
