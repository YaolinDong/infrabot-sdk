/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
// Read only: records the 1 kHz telemetry, the commands sent and the events as CSV for a few seconds.
// Nothing is enabled. Usage: 17_record [IP [DIRECTORY SECONDS]]; default: the SDK log directory, 1 s.
#include "example.hpp"

#include <cstdio>

int main(int argc, char** argv) {
    const std::string directory = argc > 3 ? argv[2] : infrabot::Robot::log_directory();
    const double seconds = argc > 3 ? std::stod(argv[3]) : 1.0;
    return example::connected(argc, argv, false, [&](infrabot::Robot& robot, std::string& error) {
        infrabot::RecordOptions options;
        options.label = "example";
        example::check(robot.start_recording(directory, error, options), error);
        robot.record_event("example marker");
        std::this_thread::sleep_for(std::chrono::duration<double>(seconds));
        robot.stop_recording();
        const auto stats = robot.recording();
        std::printf("%s: %llu telemetry rows, %llu events, %llu dropped\n", stats.directory.c_str(),
                    static_cast<unsigned long long>(stats.telemetry_rows),
                    static_cast<unsigned long long>(stats.events), static_cast<unsigned long long>(stats.dropped));
        if (stats.telemetry_rows == 0) throw std::runtime_error("no telemetry was recorded");
    });
}
