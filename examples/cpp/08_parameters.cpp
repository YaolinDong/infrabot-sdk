/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
// Lists the controller's tunable parameters (gravity, impedance, admittance).
// Usage: 08_parameters [IP [NAME VALUE]]. A write is RAM only: the controller slews its
// applied value toward the requested one, and every parameter returns to its default at reboot.
// The arm is not enabled and does not move.
#include "example.hpp"

#include <cstdio>
#include <vector>

int main(int argc, char** argv) {
    if (argc != 1 && argc != 2 && argc != 4) {
        std::cerr << "Usage: " << argv[0] << " [IP [NAME VALUE]]\n";
        return 2;
    }
    return example::connected(argc, argv, false, [&](infrabot::Robot& robot, std::string& error) {
        if (argc == 4) {
            infrabot::ParameterInfo written;
            example::check(robot.set_parameter(argv[2], std::stof(argv[3]), written, error), error);
            std::printf("set %s: requested %g, applied %g\n", written.name.c_str(), written.requested,
                        written.applied);
        }
        std::vector<infrabot::ParameterInfo> list;
        example::check(robot.list_parameters(list, error), error);
        for (const auto& p : list)
            std::printf("%-20s %9.3f (applied %9.3f) [%g..%g] %s\n", p.name.c_str(), p.requested, p.applied,
                        p.minimum, p.maximum, p.unit.c_str());
    });
}
