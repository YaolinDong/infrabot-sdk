/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
// Before enabling the arm, clear its workspace and verify the hardware emergency stop.
// NRT joint move: one joint goes by DELTA rad from where the arm holds, then back.
// Usage: 11_movej_offset [IP [JOINT DELTA]]. Default: J7 +0.2 rad at 0.1 rad/s.
// MIT holds by stiffness alone: without gravity feed-forward a loaded joint stays 10-30 mrad short of its
// target, and a light one (J7) can stall a little short on friction. The arrival tolerance below allows for
// that; the printed error is what the arm actually reached.
#include "example.hpp"

#include <cstdio>

static void check(const infrabot::MotionResult& result, const std::string& error) {
    if (!error.empty() || result.result != 0) throw std::runtime_error(error.empty() ? result.message : error);
}

int main(int argc, char** argv) {
    const int joint = argc > 3 ? std::stoi(argv[2]) : 7;
    const float delta = argc > 3 ? std::stof(argv[3]) : 0.2f;
    if (argc == 3 || argc > 4 || joint < 1 || joint > 7) {
        std::cerr << "Usage: " << argv[0] << " [IP [JOINT(1-7) DELTA_RAD]]\n";
        return 2;
    }
    return example::connected(argc, argv, true, [&](infrabot::Robot& robot, std::string& error) {
        example::check(robot.switch_mode(infrabot::Mode::NrtJoint, error), error);
        const auto start = example::hold(robot);
        auto target = start;
        target[joint - 1] += delta;
        infrabot::MoveOptions options;  // 0.1 rad/s, 0.2 rad/s^2 per joint
        options.position_tolerance = 0.04f;
        auto result = robot.movej(target, options, error);
        check(result, error);
        std::printf("J%d at %+.3f rad, largest joint error %.0f mrad\n", joint,
                    robot.state().joints.position[joint - 1], result.max_error_rad * 1000.0f);
        result = robot.movej(start, options, error);
        check(result, error);
        std::printf("back at the start, largest joint error %.0f mrad\n", result.max_error_rad * 1000.0f);
    });
}
