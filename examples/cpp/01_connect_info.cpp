/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "example.hpp"

int main(int argc, char** argv) {
    return example::connected(argc, argv, false, [](infrabot::Robot& robot, std::string& error) {
        const auto info = robot.info();
        std::cout << info.product << " " << info.firmware << "\n";
    });
}
