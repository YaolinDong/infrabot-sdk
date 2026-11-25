/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
// Before enabling the arm, clear its workspace and verify the hardware emergency stop.
// NRT Cartesian move: the tool goes DZ m along the base z axis in a straight line, then back. The start
// pose is the forward kinematics of the held joints. Usage: 12_movep_offset [IP [DZ]]. Default +0.02 m.
// MIT holds by stiffness alone: without gravity feed-forward the tool stays about 10 mm short of its
// target. The arrival tolerance below allows for that; the printed error is what the arm actually reached.
#include "example.hpp"

#include <cstdio>

static void check(const infrabot::MotionResult& result, const std::string& error) {
    if (!error.empty() || result.result != 0) throw std::runtime_error(error.empty() ? result.message : error);
}

int main(int argc, char** argv) {
    const float dz = argc > 2 ? std::stof(argv[2]) : 0.02f;
    return example::connected(argc, argv, true, [&](infrabot::Robot& robot, std::string& error) {
        example::check(robot.switch_mode(infrabot::Mode::NrtCartesian, error), error);
        const auto start = infrabot::Robot::forward(example::hold(robot));
        auto target = start;
        target[2] += dz;
        infrabot::CartesianMoveOptions options;
        options.limits.linear_velocity = 0.02f;
        options.limits.angular_velocity = 0.2f;
        options.position_tolerance = 0.03f;
        auto result = robot.movep(target, options, error);
        check(result, error);
        std::printf("tool at z = %.3f m, error %.1f mm/mrad\n", target[2], result.max_error_rad * 1000.0f);
        result = robot.movep(start, options, error);
        check(result, error);
        std::printf("back at the start, error %.1f mm/mrad\n", result.max_error_rad * 1000.0f);
    });
}
