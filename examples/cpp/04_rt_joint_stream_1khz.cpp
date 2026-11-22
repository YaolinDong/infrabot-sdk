/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
// Before enabling the arm, clear its workspace and verify the hardware emergency stop.
#include "example.hpp"

int main(int argc, char** argv) {
    return example::connected(argc, argv, true, [](infrabot::Robot& robot, std::string& error) {
        example::check(robot.switch_mode(infrabot::Mode::RtJoint, error), error);
        infrabot::JointCommand command;
        command.position = example::hold(robot);
        example::check(robot.start_joint_stream(command, infrabot::JointStreamOptions{}, error), error);
        // Native pump sends at 1 kHz. Refresh the held target at 100 Hz.
        for (int i = 0; i < 100; ++i) {
            example::check(robot.servej(command, error), error);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        robot.stop_joint_stream();
    });
}
