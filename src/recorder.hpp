/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#pragma once

/* CSV recorder of the SDK (Robot::start_recording).  The receive thread and
 * the senders only copy the raw datagram into a queue (a mutex and a memcpy);
 * a background thread formats and writes it, rotating the files. */

#include "infrabot/protocol.h"
#include "infrabot/robot.hpp"

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdio>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace infrabot {
namespace detail {

/* The telemetry_NNN.csv format (header with '\n', and one DMTL v2/v3 row).
 * lost counts the sequence gap to the previous row of the same file. */
std::string telemetry_csv_header();
void telemetry_csv_row(uint64_t host_us, const void *data, size_t size, bool &have_sequence,
                       uint32_t &last_sequence, std::string &line);

class Recorder {
public:
    ~Recorder() { stop(); }
    /* Creates directory/<time>[-label]/ with info.txt, events.csv and the
     * first telemetry/commands files; info is written into info.txt. */
    bool start(const std::string &directory, const RecordOptions &options, const std::string &info,
               std::string &error);
    void stop();
    bool active() const { return active_.load(std::memory_order_acquire); }
    void telemetry(uint64_t host_us, const void *data, size_t size);
    void command(uint64_t host_us, const void *data, size_t size);
    void event(uint64_t host_us, uint64_t mcu_us, const char *source, const std::string &text);
    RecorderStats stats() const;

private:
    enum Kind : uint8_t { kTelemetry, kCommand };
    struct Item {
        uint8_t kind;
        uint16_t size;
        uint64_t host_us;
        std::array<uint8_t, sizeof(InfrabotTelemetryV3)> bytes;
    };
    struct Event {
        uint64_t host_us, mcu_us;
        const char *source;
        std::string text;
    };
    void push(Kind kind, uint64_t host_us, const void *data, size_t size);
    void run();
    bool open_part(std::string &error);
    void close_part();
    void write_telemetry(const Item &item, std::string &line);
    void write_command(const Item &item, std::string &line);

    std::atomic<bool> active_{false};
    RecordOptions options_{};
    std::string directory_;
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::vector<Item> pending_, writing_;
    std::vector<Event> pending_events_, writing_events_;
    bool stopping_ = false;
    std::thread thread_;
    // Writer thread only.
    std::FILE *telemetry_file_ = nullptr, *command_file_ = nullptr, *event_file_ = nullptr;
    uint64_t part_start_us_ = 0;
    bool have_sequence_ = false;
    uint32_t last_sequence_ = 0;
    // Shared counters (mutex_).
    uint64_t telemetry_rows_ = 0, command_rows_ = 0, events_ = 0, dropped_ = 0;
    uint32_t file_index_ = 0;
    std::string error_;
};

} // namespace detail
} // namespace infrabot
