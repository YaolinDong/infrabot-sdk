/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#pragma once

#include "infrabot/robot.hpp"
#include <algorithm>
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>

namespace example {
inline void check(bool ok, const std::string& error) {
    if (!ok) throw std::runtime_error(error);
}

// Examples take an optional controller IP. STOP returns an enabled arm to hold.
template <class Action>
int connected(int argc, char** argv, bool enable_arm, Action action) {
    infrabot::Robot robot(argc > 1 ? argv[1] : "192.168.0.50");
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
            check(robot.set_motor_profile(infrabot::MotorProfile::Mit, error), error);
            robot.set_heartbeat(true);
            check(robot.servo_on(infrabot::MotorGroup::Arm, error), error);
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
