/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "infrabot/robot.hpp"
#include <iostream>

int main(int argc, char** argv) {
    infrabot::Robot robot(argc > 1 ? argv[1] : "192.168.0.50");
    infrabot::McuLogResult result;
    std::string error;
    if (!robot.download_mcu_log(result, error)) {
        std::cerr << error << '\n';
        return 1;
    }
    std::cout << result.text_path << '\n' << result.telemetry_path << '\n';
}
