/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#pragma once

#include "infrabot/robot.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <stdexcept>
#include <thread>

namespace example {
inline void check(bool ok, const std::string& error) {
    if (!ok) throw std::runtime_error(error);
}

// Ports from the environment, for a loopback board; the controller uses the defaults.
inline uint16_t port(const char* name, uint16_t fallback) {
    const char* value = std::getenv(name);
    return value && *value ? static_cast<uint16_t>(std::stoi(value)) : fallback;
}

// Examples take an optional controller IP. STOP returns an enabled arm to hold.
template <class Action>
int connected(int argc, char** argv, bool enable_arm, Action action) {
    infrabot::Robot robot(argc > 1 ? argv[1] : "192.168.0.50",
                          port("INFRABOT_EXAMPLE_COMMAND_PORT", INFRABOT_COMMAND_PORT),
                          port("INFRABOT_EXAMPLE_TELEMETRY_PORT", INFRABOT_TELEMETRY_PORT));
    // A read-only session sends no heartbeat, so it never looks like the controlling PC.
    robot.set_heartbeat(enable_arm);
    std::string error;
    bool enabled = false;
    try {
        check(robot.connect(error), error);
        if (!robot.info().received) throw std::runtime_error("Controller did not reply");
        if (enable_arm) {
            // DMIO can arrive before the first DMTL. Match the SDK's 100 ms freshness check.
            const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
            while (robot.state().telemetry_age_ms > 100) {
                if (std::chrono::steady_clock::now() >= deadline)
                    throw std::runtime_error("No fresh telemetry before drive setup");
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
            robot.set_heartbeat(true);
            const auto s = robot.state();
            const bool arm_enabled = (s.enabled_mask & INFRABOT_ARM_MASK) == INFRABOT_ARM_MASK;
            if (s.enabled && !(arm_enabled && s.mode == infrabot::Mode::Idle && s.profile_ready &&
                               s.profile == infrabot::MotorProfile::Mit))
                throw std::runtime_error("The arm is enabled outside IDLE/MIT or only partly: stop or disable it first");
            if (!arm_enabled) {
                // A previous example left the arm enabled and holding in IDLE: reuse it as it is.
                check(robot.set_motor_profile(infrabot::MotorProfile::Mit, error), error);
                check(robot.servo_on(infrabot::MotorGroup::Arm, error), error);
            }
            // A mode switch needs a still arm. Right after a previous example stopped a local mode, the
            // hold is still settling for a moment.
            const auto rest = std::chrono::steady_clock::now() + std::chrono::seconds(3);
            for (;;) {
                const auto v = robot.state().joints.velocity;
                if (std::all_of(v.begin(), v.begin() + 7, [](float x) { return std::fabs(x) <= 0.05f; })) break;
                if (std::chrono::steady_clock::now() >= rest)
                    throw std::runtime_error("The arm does not come to rest (above 0.05 rad/s): let go of it");
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            enabled = true;
        }
        action(robot, error);
        if (enabled) check(robot.stop(error), error);
        return 0;
    } catch (const std::exception& failure) {
        std::cerr << failure.what() << '\n';
        if (enabled && !robot.stop(error)) std::cerr << error << '\n';
        return 1;
    }
}

inline std::array<float, 7> hold(infrabot::Robot& robot) {
    auto q = robot.hold_command();
    std::array<float, 7> arm;
    std::copy_n(q.begin(), arm.size(), arm.begin());
    return arm;
}
}  // namespace example
