/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#pragma once
// Private Robot component: no sockets, no second Robot, no Python callbacks,
// no drive lifecycle. Robot's service thread owns periodic calls to tick().
#include "infrabot/robot.hpp"
#include <cmath>
#include <mutex>

namespace infrabot::detail {
class JointStream {
public:
    bool start(const JointCommand& initial, const JointStreamOptions& options,
               const RobotState& state, const RobotInfo& info, uint64_t now, std::string& error) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (stats_.active) { error = "joint stream already active"; return false; }
        if (options.transmit_hz < 100 || options.transmit_hz > 1000 ||
            options.command_timeout_ms < 20 || options.command_timeout_ms > 1000 ||
            options.feedback_timeout_ms < 10 || options.feedback_timeout_ms > 200) {
            error = "invalid joint stream rate/watchdog timeout"; return false;
        }
        const uint32_t required = INFRABOT_CAP_JOINT_RT | INFRABOT_CAP_TELEMETRY_V3 | INFRABOT_CAP_CONTROLLED_STOP;
        if (!info.received || info.axes != 7 || (info.capabilities & required) != required) {
            error = "joint stream requires seven-axis RT Joint, telemetry v3 and controlled stop"; return false;
        }
        if (!healthy(state, options, error) || !valid(initial, options, info, error)) return false;
        options_ = options; info_ = info; command_ = initial;
        updated_us_ = next_us_ = source_progress_us_ = now;
        epoch_ = state.telemetry_epoch;
        sequence_ = state.joints.sequence; mcu_us_ = state.joints.timestamp_us;
        managed_ = true; stats_ = {}; stats_.active = true;
        return true;
    }

    template<class Send>
    bool serve(const JointCommand& command, uint64_t now, Send&& send, std::string& error) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!managed_) return send(command, error); // original one-packet SDK path
        if (!stats_.active) {
            error = stats_.fault.empty() ? "joint stream stopped; explicitly start it again" : stats_.fault;
            return false;
        }
        if (now > updated_us_ + uint64_t(options_.command_timeout_ms) * 1000) {
            error = "command producer timed out; wait for STOP before explicitly restarting";
            return false; // do not refresh an already expired producer; tick sends STOP
        }
        if (!valid(command, options_, info_, error)) return false;
        command_ = command;
        updated_us_ = now;
        return true;
    }

    // Hold this mutex through send/STOP: cancellation waits for an in-flight
    // send, and no queued RT packet can follow the caller's mode/stop command.
    template<class ReadState, class Clock, class Send, class Stop>
    void tick(ReadState&& read_state, Clock&& clock, Send&& send, Stop&& stop) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!stats_.active) return;
        // Sample after locking: a producer/start could otherwise publish a
        // newer timestamp than a tick's pre-lock snapshot and falsely time out.
        const uint64_t now = clock();
        const RobotState state = read_state();
        std::string error;
        if (state.telemetry_epoch != epoch_ || state.joints.timestamp_us < mcu_us_) {
            error = "telemetry source restarted; controlled stop requested";
        } else if (state.joints.sequence != sequence_ && state.joints.timestamp_us > mcu_us_) {
            sequence_ = state.joints.sequence; mcu_us_ = state.joints.timestamp_us;
            source_progress_us_ = now;
        }
        if (error.empty() && now > updated_us_ + uint64_t(options_.command_timeout_ms) * 1000)
            error = "command producer timed out; controlled stop requested";
        if (error.empty() && now > source_progress_us_ + uint64_t(options_.feedback_timeout_ms) * 1000)
            error = "telemetry stopped advancing; controlled stop requested";
        if (error.empty()) healthy(state, options_, error);
        if (error.empty() && now >= next_us_) {
            const uint64_t period = 1000000 / options_.transmit_hz;
            stats_.skipped_deadlines += (now - next_us_) / period;
            next_us_ += ((now - next_us_) / period + 1) * period;
            if (send(command_, error)) ++stats_.sent;
            else if (error.empty()) error = "joint stream send failed";
        }
        if (!error.empty()) {
            stats_.fault = error; stats_.active = false;
            stop(error.c_str());
        }
    }

    void halt() { // lifecycle stop: do not silently fall back to direct sending
        std::lock_guard<std::mutex> lock(mutex_); stats_.active = false;
    }
    void release() { // explicit opt-out/reconnect: original servej semantics
        std::lock_guard<std::mutex> lock(mutex_); stats_.active = false; managed_ = false;
    }
    JointStreamStats stats() const {
        std::lock_guard<std::mutex> lock(mutex_); return stats_;
    }
private:
    static bool healthy(const RobotState& s, const JointStreamOptions& o, std::string& error) {
        if (!s.connected || s.telemetry_age_ms > o.feedback_timeout_ms || s.telemetry_version < 3) {
            error = "arm feedback stale; controlled stop requested"; return false;
        }
        if (s.mode != Mode::RtJoint || s.profile != MotorProfile::Mit || !s.profile_ready ||
            (s.enabled_mask & INFRABOT_ARM_MASK) != INFRABOT_ARM_MASK || s.safety.stop_reason != StopReason::None) {
            error = "MCU not in enabled RT_JOINT/MIT or stopped"; return false;
        }
        for (size_t i = 0; i < 7; ++i) {
            if (s.joints.feedback_age_ms[i] > o.feedback_timeout_ms ||
                (s.joints.motor_flags[i] & INFRABOT_MOTOR_FEEDBACK_STALE)) {
                error = "arm feedback stale; controlled stop requested"; return false;
            }
            if (INFRABOT_DRIVE_IS_FAULT(s.joints.drive_state[i])) {
                error = "arm drive fault; controlled stop requested"; return false;
            }
        }
        return true;
    }
    static bool valid(const JointCommand& c, const JointStreamOptions& o, const RobotInfo& info, std::string& error) {
        for (size_t i = 0; i < 7; ++i) {
            if (!std::isfinite(info.velocity_max[i]) || info.velocity_max[i] <= 0 ||
                !std::isfinite(info.torque_max[i]) || info.torque_max[i] <= 0 ||
                !std::isfinite(c.position[i]) || !std::isfinite(c.velocity[i]) || !std::isfinite(c.torque[i]) ||
                !std::isfinite(c.kp[i]) || !std::isfinite(c.kd[i]) || !std::isfinite(c.velocity_limit[i]) ||
                std::fabs(c.position[i]) > 12.5f || std::fabs(c.velocity[i]) > info.velocity_max[i] ||
                std::fabs(c.velocity[i]) > INFRABOT_ARM_MAX_JOINT_VELOCITY ||
                std::fabs(c.torque[i]) > info.torque_max[i] || c.kp[i] < 0 || c.kp[i] > 500 ||
                (!o.allow_zero_kp && c.kp[i] < .001f) || c.kd[i] < 0 || c.kd[i] > 5 ||
                c.velocity_limit[i] < 0 || c.velocity_limit[i] > info.velocity_max[i] ||
                c.velocity_limit[i] > INFRABOT_ARM_MAX_JOINT_VELOCITY) {
                error = "invalid joint stream command (finite values, motor ranges, gains/allow_zero_kp)";
                return false;
            }
        }
        return true;
    }
    mutable std::mutex mutex_;
    bool managed_ = false;
    JointStreamOptions options_{};
    JointStreamStats stats_{};
    RobotInfo info_{};
    JointCommand command_{};
    uint64_t updated_us_ = 0, next_us_ = 0, source_progress_us_ = 0, mcu_us_ = 0;
    uint32_t epoch_ = 0, sequence_ = 0;
};
} // namespace infrabot::detail
