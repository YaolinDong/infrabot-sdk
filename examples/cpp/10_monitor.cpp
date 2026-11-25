/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
// Read only: prints the controller state, every joint and the link quality for a few seconds, then the
// fault log. Usage: 10_monitor [IP [SECONDS]]. No heartbeat is sent and nothing is enabled.
#include "example.hpp"
#include "infrabot/diagnostics.hpp"

#include <cstdio>

int main(int argc, char** argv) {
    const double seconds = argc > 2 ? std::stod(argv[2]) : 1.0;
    return example::connected(argc, argv, false, [&](infrabot::Robot& robot, std::string&) {
        // The first telemetry packet follows the handshake.
        const auto first = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
        while (robot.state().telemetry_age_ms > 100) {
            if (std::chrono::steady_clock::now() >= first) throw std::runtime_error("No telemetry");
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
        for (;;) {
            const auto s = robot.state();
            std::printf("%s mode=%s enabled=0x%02X telemetry age %u ms\n",
                        infrabot::controller_status_text(s.controller_state), infrabot::mode_text(s.mode),
                        s.enabled_mask, s.telemetry_age_ms);
            for (size_t i = 0; i < 8; ++i)
                std::printf("  J%zu q=%+.3f rad dq=%+.3f rad/s tau=%+.2f Nm %s %u C\n", i + 1,
                            s.joints.position[i], s.joints.velocity[i], s.joints.torque[i],
                            infrabot::drive_state_info(s.joints.drive_state[i]).name, s.joints.temperature[i]);
            if (std::chrono::steady_clock::now() >= end) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        const auto link = robot.link_stats();
        std::printf("link: %u packets/s, lost %u, longest gap %u us\n", link.rx_last_second,
                    link.lost_last_second, link.rx_interval_max_us);
        for (const auto& event : robot.fault_log()) std::printf("fault: %s\n", event.message.c_str());
    });
}
