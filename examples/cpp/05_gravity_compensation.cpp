/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
// Before enabling the arm, clear its workspace and verify the hardware emergency stop.
#include "example.hpp"

int main(int argc, char** argv) {
    return example::connected(argc, argv, true, [](infrabot::Robot& robot, std::string& error) {
        example::check(robot.switch_mode(infrabot::Mode::GravityLock, error), error);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    });
}
