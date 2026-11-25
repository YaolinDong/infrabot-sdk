/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
// Before enabling the arm, clear its workspace and verify the hardware emergency stop.
// RT Cartesian stream: servep at 100 Hz (the MCU interpolates at 1 kHz and stops after a 100 ms gap).
// The target rises DZ m along z and comes back on a smooth cosine over SECONDS.
// Usage: 13_rt_cartesian_stream [IP [DZ SECONDS]]. Default 0.01 m over 4 s.
#include "example.hpp"

#include <cmath>
#include <cstdio>

int main(int argc, char** argv) {
    const float dz = argc > 3 ? std::stof(argv[2]) : 0.01f;
    const double seconds = argc > 3 ? std::stod(argv[3]) : 4.0;
    return example::connected(argc, argv, true, [&](infrabot::Robot& robot, std::string& error) {
        example::check(robot.switch_mode(infrabot::Mode::RtCartesian, error), error);
        const auto start = infrabot::Robot::forward(example::hold(robot));
        infrabot::CartesianCommand command;
        command.limits.linear_velocity = 0.05f;
        command.limits.angular_velocity = 0.3f;
        command.pose = start;
        const auto t0 = std::chrono::steady_clock::now();
        for (;;) {
            const double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
            if (t >= seconds) break;
            command.pose[2] = start[2] + float(dz * 0.5 * (1.0 - std::cos(2.0 * M_PI * t / seconds)));
            example::check(robot.servep(command, error), error);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        command.pose = start;
        example::check(robot.servep(command, error), error);
        std::printf("stream ended at the start pose\n");
    });
}
