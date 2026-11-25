/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
// Keep fingers and objects clear of the gripper.
// Enables only the J8 gripper group (the arm stays as it is), moves it to POSITION and back to where it
// started. FORCE is a current limit as a fraction of the drive maximum. It stays enabled and holding.
// Usage: 16_gripper [IP [POSITION SPEED FORCE]]. Default 1.0 rad at 1.0 rad/s, force 0.3.
#include "example.hpp"

#include <cmath>
#include <cstdio>

int main(int argc, char** argv) {
    const float position = argc > 4 ? std::stof(argv[2]) : 1.0f;
    const float speed = argc > 4 ? std::stof(argv[3]) : 1.0f;
    const float force = argc > 4 ? std::stof(argv[4]) : 0.3f;
    return example::connected(argc, argv, false, [&](infrabot::Robot& robot, std::string& error) {
        robot.set_heartbeat(true);
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
        while (robot.state().telemetry_age_ms > 100) {
            if (std::chrono::steady_clock::now() >= deadline)
                throw std::runtime_error("No fresh telemetry before drive setup");
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        const auto s = robot.state();
        if (s.joints.feedback_age_ms[7] > 1000)
            throw std::runtime_error("J8 sends no feedback: no gripper connected or powered, nothing enabled");
        if (!s.profile_ready) example::check(robot.set_motor_profile(infrabot::MotorProfile::Mit, error), error);
        if (!(s.enabled_mask & 0x80)) example::check(robot.servo_on(infrabot::MotorGroup::Gripper, error), error);
        const float start = robot.state().joints.position[7];
        float here = start;
        for (const float target : {position, start}) {
            example::check(robot.gripper(target, speed, force, error), error);
            std::this_thread::sleep_for(std::chrono::duration<double>(std::fabs(target - here) / speed + 0.5));
            here = robot.state().joints.position[7];
            std::printf("gripper at %.3f rad (target %.3f)\n", here, target);
        }
    });
}
