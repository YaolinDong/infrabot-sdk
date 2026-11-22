/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
// Before enabling the arm, clear its workspace and verify the hardware emergency stop.
#include "example.hpp"

int main(int argc, char** argv) {
    return example::connected(argc, argv, true, [](infrabot::Robot& robot, std::string& error) {
        example::check(robot.switch_mode(infrabot::Mode::NrtJoint, error), error);
        // Use the held pose as the first target. Joint positions are in radians.
        const auto result = robot.movej(example::hold(robot), infrabot::MoveOptions{}, error);
        if (!error.empty() || result.result != 0)
            throw std::runtime_error(error.empty() ? result.message : error);
    });
}
