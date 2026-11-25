/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
// Before enabling the arm, clear its workspace and verify the hardware emergency stop.
// Controlled stop and resume: STOP brakes the arm and holds it enabled in IDLE; resume() re-enters the
// mode the stop interrupted, from where the arm stopped. The arm does not move.
#include "example.hpp"
#include "infrabot/diagnostics.hpp"

#include <cstdio>

int main(int argc, char** argv) {
    return example::connected(argc, argv, true, [](infrabot::Robot& robot, std::string& error) {
        example::check(robot.switch_mode(infrabot::Mode::NrtJoint, error), error);
        example::check(robot.stop(error), error);
        const auto s = robot.state();
        std::printf("stopped: mode %s, %s, interrupted %s, enabled 0x%02X\n", infrabot::mode_text(s.mode),
                    infrabot::describe_stop(s.safety).c_str(), infrabot::mode_text(s.safety.stop_mode),
                    s.enabled_mask);
        infrabot::Mode resumed = infrabot::Mode::Unknown;
        example::check(robot.resume(error, &resumed), error);
        std::printf("resumed %s\n", infrabot::mode_text(resumed));
    });
}
