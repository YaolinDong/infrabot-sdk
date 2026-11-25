/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
// Before enabling the arm, clear its workspace and verify the hardware emergency stop.
#include "example.hpp"

int main(int argc, char** argv) {
    return example::connected(argc, argv, true, [](infrabot::Robot& robot, std::string& error) {
        example::check(robot.switch_mode(infrabot::Mode::NrtJoint, error), error);
        // Use the held pose as the first target. Joint positions are in radians.
        // MIT holds by stiffness alone and sags below its command by tens of mrad.
        infrabot::MoveOptions options;
        options.position_tolerance = 0.04f;
        const auto result = robot.movej(example::hold(robot), options, error);
        if (!error.empty() || result.result != 0)
            throw std::runtime_error(error.empty() ? result.message : error);
    });
}
