/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#pragma once

/* MCU UART debug log recorder (Robot::start_serial_log).  A background thread
 * reads the serial port (read-only: it never writes to the board) and appends
 * one timestamped line per received text line to a file.  It shares nothing
 * with the UDP threads, so it cannot affect the 1 kHz control path. */

#include "infrabot/robot.hpp"

#include <atomic>
#include <cstdio>
#include <ctime>
#include <mutex>
#include <string>
#include <thread>

namespace infrabot {
namespace detail {

/* <directory>/serial_YYYYMMDDHHMMSS.log, with -1, -2 ... if that name exists. */
std::string serial_log_path(const std::string &directory, std::time_t when);
/* Processes holding device open, "pid(name)" separated by ", "; empty if none. */
std::string serial_port_users(const std::string &device);

class SerialLog {
public:
    ~SerialLog() { stop(); }
    bool start(const SerialLogOptions &options, std::string &error);
    void stop();
    bool active() const { return active_.load(std::memory_order_acquire); }
    SerialLogStats stats() const;

private:
    void run();
    void stop_locked();
    bool open_port(std::string &error);
    void close_port();
    void write_line(const char *text, size_t size);

    std::mutex control_mutex_;  // serialises start()/stop() (GIL is released in Python)
    SerialLogOptions options_;
    std::string path_;
    std::FILE *file_ = nullptr;
    int fd_ = -1;
    int wake_[2] = {-1, -1};   // pipe: stop() wakes the blocked poll()
    std::thread thread_;
    std::atomic<bool> active_{false}, running_{false};
    mutable std::mutex mutex_;  // stats and error
    SerialLogStats stats_;
};

}  // namespace detail
}  // namespace infrabot
