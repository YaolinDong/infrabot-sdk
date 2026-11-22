/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#pragma once
#include "infrabot/robot.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cassert>
#include <cstdio>
#include <cstring>
#include <deque>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#ifdef RUN_LOG_WITH_FIRMWARE
extern "C" {
#include "app_log.h"
}
#endif

namespace fake_log {
#ifndef RUN_LOG_WITH_FIRMWARE
/* Text records as the firmware stores them: seq, time_ms, length, text. */
class Model {
public:
    void init(uint32_t text_count, uint32_t telemetry_count) {
        text_count_ = text_count; telemetry_count_ = telemetry_count;
        text_.clear(); telemetry_.clear(); text_first_ = text_next_ = telemetry_first_ = telemetry_next_ = 0;
        frozen = latched = false;
    }
    void add_text(uint32_t time_ms, const std::string &line) {
        text_.push_back({text_next_++, time_ms, line.substr(0, 246)});  // APP_LOG_TEXT_LINE_MAX
        if (text_.size() > text_count_ - 1U) { text_.pop_front(); ++text_first_; }
    }
    void add_telemetry(const InfrabotTelemetryV3 &packet) {
        if (frozen || latched) return;
        telemetry_.push_back(packet);
        ++telemetry_next_;
        if (telemetry_.size() > telemetry_count_ - 1U) { telemetry_.pop_front(); ++telemetry_first_; }
    }
    size_t serve(const uint8_t *bytes, size_t size, uint8_t *reply, bool busy) {
        InfrabotLogRequest request{};
        if (size != sizeof(request)) return 0;
        std::memcpy(&request, bytes, sizeof(request));
        if (request.magic != INFRABOT_MAGIC_LOG_REQUEST) return 0;
        InfrabotLogReplyHeader header{};
        header.magic = INFRABOT_MAGIC_LOG_REPLY;
        header.version = INFRABOT_LOG_V1;
        header.op = request.op;
        header.request_id = request.request_id;
        uint8_t *data = reply + sizeof(header);
        size_t used = 0;
        if (request.version != INFRABOT_LOG_V1 || request.op > INFRABOT_LOG_OP_RESUME) header.result = INFRABOT_LOG_INVALID;
        else if (!ready) header.result = INFRABOT_LOG_UNAVAILABLE;
        else if (busy) header.result = INFRABOT_LOG_BUSY;
        else {
            uint32_t next_cursor = 0;
            uint8_t count = 0;
            if (request.op == INFRABOT_LOG_OP_TEXT) {
                uint32_t index = (request.cursor - text_first_) > (text_next_ - text_first_) ? text_first_ : request.cursor;
                header.cursor = index;
                while (index != text_next_ && count < 255U) {
                    const Text &t = text_[index - text_first_];
                    const size_t record = (INFRABOT_LOG_TEXT_HEADER + t.text.size() + 3U) & ~size_t{3};
                    if (used + record > INFRABOT_LOG_DATA_MAX) break;
                    std::memset(data + used, 0, record);
                    const uint16_t length = static_cast<uint16_t>(t.text.size());
                    std::memcpy(data + used, &t.seq, 4);
                    std::memcpy(data + used + 4, &t.time_ms, 4);
                    std::memcpy(data + used + 8, &length, 2);
                    std::memcpy(data + used + 10, t.text.data(), length);
                    used += record; ++index; ++count;
                }
                next_cursor = index;
            } else if (request.op == INFRABOT_LOG_OP_TELEMETRY) {
                uint32_t index = (request.cursor - telemetry_first_) > (telemetry_next_ - telemetry_first_) ?
                                 telemetry_first_ : request.cursor;
                header.cursor = index;
                while (index != telemetry_next_ && used + sizeof(InfrabotTelemetryV3) <= INFRABOT_LOG_DATA_MAX) {
                    std::memcpy(data + used, &telemetry_[index - telemetry_first_], sizeof(InfrabotTelemetryV3));
                    used += sizeof(InfrabotTelemetryV3); ++index; ++count;
                }
                next_cursor = index;
            } else if (request.op == INFRABOT_LOG_OP_FREEZE) frozen = true;
            else if (request.op == INFRABOT_LOG_OP_RESUME) frozen = latched = false;
            else { header.writer_max_us = writer_max_us; writer_max_us = 0; }
            header.next_cursor = next_cursor;
            header.record_count = count;
            header.data_bytes = static_cast<uint16_t>(used);
            header.boot_id = boot_id;
            header.text_first = text_first_; header.text_next = text_next_;
            header.telemetry_first = telemetry_first_; header.telemetry_next = telemetry_next_;
            header.frozen = static_cast<uint8_t>((frozen ? 1U : 0U) | (latched ? 2U : 0U));
            header.lost_bytes = lost_bytes;
        }
        std::memcpy(reply, &header, sizeof(header));
        return sizeof(header) + used;
    }
    void latch() { latched = true; }  // the arm faulted: the ring is held until a RESUME
    bool ready = true, frozen = false, latched = false;
    uint32_t boot_id = 0x1234U, writer_max_us = 0, lost_bytes = 0;

private:
    struct Text { uint32_t seq, time_ms; std::string text; };
    std::deque<Text> text_;
    std::deque<InfrabotTelemetryV3> telemetry_;
    uint32_t text_count_ = 0, telemetry_count_ = 0;
    uint32_t text_first_ = 0, text_next_ = 0, telemetry_first_ = 0, telemetry_next_ = 0;
};
#else
/* The firmware's own rings and AppLog_Serve. */
class Model {
public:
    void init(uint32_t text_count, uint32_t telemetry_count) {
        text_memory_.assign(size_t(text_count) * sizeof(AppLogTextRecord), 0);
        telemetry_memory_.assign(size_t(telemetry_count) * sizeof(InfrabotTelemetryV3), 0);
        std::memset(&server_, 0, sizeof(server_));
        AppLogRing_Init(&server_.text, text_memory_.data(), sizeof(AppLogTextRecord), text_count);
        AppLogRing_Init(&server_.telemetry, telemetry_memory_.data(), sizeof(InfrabotTelemetryV3), telemetry_count);
        frozen = latched = false;
    }
    void add_text(uint32_t time_ms, const std::string &line) {
        AppLogText_Append(&server_.text, time_ms, line.data(), static_cast<uint32_t>(line.size()));
    }
    void add_telemetry(const InfrabotTelemetryV3 &packet) {
        if (server_.frozen || server_.latched) return;
        std::memcpy(AppLogRing_Begin(&server_.telemetry), &packet, sizeof(packet));
        AppLogRing_Commit(&server_.telemetry);
    }
    size_t serve(const uint8_t *bytes, size_t size, uint8_t *reply, bool busy) {
        server_.ready = ready ? 1U : 0U;
        server_.boot_id = boot_id;
        server_.lost_bytes = lost_bytes;
        if (writer_max_us != 0U) { server_.writer_max_us = writer_max_us; writer_max_us = 0U; }
        AppLogContext context{};
        context.now_ms = ++now_ms_;  // one request per "ms": the rate limit never bites here
        context.busy = busy ? 1U : 0U;
        const size_t length = AppLog_Serve(&server_, bytes, size, &context, reply);
        frozen = server_.frozen != 0U;
        latched = server_.latched != 0U;
        return length;
    }
    /* The arm faulted: the firmware's own FaultTick latches the ring. */
    void latch() {
        AppLog_FaultTick(&server_, INFRABOT_STATUS_STOPPING, INFRABOT_STOP_STREAM_LOST);
        latched = server_.latched != 0U;
    }
    bool ready = true, frozen = false, latched = false;
    uint32_t boot_id = 0x1234U, writer_max_us = 0, lost_bytes = 0;

private:
    AppLogServer server_{};
    std::vector<uint8_t> text_memory_, telemetry_memory_;
    uint32_t now_ms_ = 0;
};
#endif

/* UDP on 127.0.0.1:<ephemeral>; hooks let a test drop replies, reset the
 * board or append telemetry while a download runs. */
class Board {
public:
    Board() {
        fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        assert(bind(fd_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) == 0);
        socklen_t length = sizeof(address);
        getsockname(fd_, reinterpret_cast<sockaddr *>(&address), &length);
        port = ntohs(address.sin_port);
        timeval timeout{0, 20000};
        setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
        thread_ = std::thread([this] { run(); });
    }
    ~Board() { stop_ = true; thread_.join(); close(fd_); }

    std::mutex mutex;  // guards everything below while the thread runs
    Model model;
    bool busy = false;
    uint32_t drop_every = 0;          // drop every Nth reply (0 = never)
    uint32_t reset_after = 0;         // change boot_id after this many requests (0 = never)
    uint32_t telemetry_per_request = 0;  // the 1 kHz writer keeps going while unfrozen
    uint32_t requests = 0, freezes = 0, resumes = 0;
    uint32_t next_sequence = 0;       // of telemetry_per_request packets
    uint16_t port = 0;

private:
    void run() {
        uint8_t request[256], reply[sizeof(InfrabotLogReplyHeader) + INFRABOT_LOG_DATA_MAX];
        while (!stop_) {
            sockaddr_in source{};
            socklen_t length = sizeof(source);
            const ssize_t got = recvfrom(fd_, request, sizeof(request), 0, reinterpret_cast<sockaddr *>(&source), &length);
            if (got <= 0) continue;
            std::lock_guard<std::mutex> lock(mutex);
            ++requests;
            if (got >= 7) {
                if (request[6] == INFRABOT_LOG_OP_FREEZE) ++freezes;
                if (request[6] == INFRABOT_LOG_OP_RESUME) ++resumes;
            }
            if (reset_after != 0U && requests > reset_after) model.boot_id = 0x9999U;
            for (uint32_t i = 0; i < telemetry_per_request; ++i) {
                InfrabotTelemetryV3 packet{};
                packet.head.version = INFRABOT_TELEMETRY_V3;
                packet.head.sequence = next_sequence++;
                model.add_telemetry(packet);
            }
            const size_t size = model.serve(request, static_cast<size_t>(got), reply, busy);
            if (size == 0U) continue;
            if (drop_every != 0U && requests % drop_every == 0U) continue;
            sendto(fd_, reply, size, 0, reinterpret_cast<sockaddr *>(&source), length);
        }
    }
    int fd_ = -1;
    std::atomic<bool> stop_{false};
    std::thread thread_;
};
} // namespace fake_log
