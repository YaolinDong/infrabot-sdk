/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
// Prints one diagnostic snapshot: loop timing, CAN and command counters, memory, drive states.
// Usage: 09_diagnostics [IP]. Read only: any PC may ask, even while another one controls the arm.
// The arm is not enabled and does not move.
#include "example.hpp"
#include "infrabot/diagnostics.hpp"

#include <cstdio>

int main(int argc, char** argv) {
    if (argc > 2) {
        std::cerr << "Usage: " << argv[0] << " [IP]\n";
        return 2;
    }
    return example::connected(argc, argv, false, [&](infrabot::Robot& robot, std::string& error) {
        infrabot::DiagnosticsInfo d;
        example::check(robot.diagnostics(d, error), error);
        std::printf("uptime %.1f s, status %s\n", d.uptime_ms / 1000.0,
                    infrabot::controller_status_text(d.controller_status));
        std::printf("last second: control %u Hz, missed %u, period max %u us\n", d.control_hz, d.control_missed,
                    d.control_period_max_us);
        std::printf("  CAN tx %u rx %u tx errors %u; commands %u, invalid %u; telemetry %u\n", d.can_tx, d.can_rx,
                    d.can_tx_errors, d.command_rx, d.command_invalid, d.telemetry_tx);
        std::printf("since boot: missed %u, CAN tx errors %u, CAN error events %u, invalid commands %u\n",
                    d.control_missed_total, d.can_tx_errors_total, d.can_error_events, d.command_invalid_total);
        if (d.command_invalid_total != 0)
            std::printf("  last invalid command: %s\n", infrabot::invalid_reason_text(d.invalid_reason));
        std::printf("CAN TEC %u REC %u%s%s\n", d.can_tec, d.can_rec, d.can_error_passive ? " error-passive" : "",
                    d.can_bus_off ? " BUS-OFF" : "");
        std::printf("heap free %u B (lowest %u B); least stack free: control %u B, status %u B, ethernet %u B\n",
                    d.heap_free_bytes, d.heap_min_free_bytes, d.stack_free_control_bytes,
                    d.stack_free_status_bytes, d.stack_free_ethernet_bytes);
        for (size_t i = 0; i < d.drive_state.size(); ++i)
            std::printf("J%zu %s\n", i + 1, infrabot::drive_state_info(d.drive_state[i]).name);
    });
}
