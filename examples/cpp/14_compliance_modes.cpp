/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
// Before enabling the arm, clear its workspace and verify the hardware emergency stop.
// Local compliance modes run on the MCU at 1 kHz without a PC stream. Push the arm by hand while it runs.
// STOP at the end brakes it and holds it in IDLE. Gains: see 08_parameters.
// Usage: 14_compliance_modes [IP [MODE SECONDS]], MODE one of gravity-free, gravity-lock, joint-impedance,
// cartesian-impedance (default), joint-admittance, cartesian-admittance.
#include "example.hpp"

#include <cstdio>
#include <map>

int main(int argc, char** argv) {
    const std::map<std::string, infrabot::Mode> modes = {
        {"gravity-free", infrabot::Mode::GravityFree},
        {"gravity-lock", infrabot::Mode::GravityLock},
        {"joint-impedance", infrabot::Mode::JointImpedance},
        {"cartesian-impedance", infrabot::Mode::CartesianImpedance},
        {"joint-admittance", infrabot::Mode::JointAdmittance},
        {"cartesian-admittance", infrabot::Mode::CartesianAdmittance}};
    const std::string name = argc > 2 ? argv[2] : "cartesian-impedance";
    const double seconds = argc > 3 ? std::stod(argv[3]) : 1.0;
    const auto mode = modes.find(name);
    if (mode == modes.end()) {
        std::cerr << "Unknown mode " << name << '\n';
        return 2;
    }
    return example::connected(argc, argv, true, [&](infrabot::Robot& robot, std::string& error) {
        example::check(robot.switch_mode(mode->second, error), error);
        std::printf("%s for %g s\n", name.c_str(), seconds);
        const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
        while (std::chrono::steady_clock::now() < end) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            const auto q = robot.state().joints.position;
            for (size_t i = 0; i < 7; ++i) std::printf("%+.2f ", q[i]);
            std::printf("\n");
        }
    });
}
