/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "run_log.hpp"

#include "recorder.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

namespace infrabot {
namespace detail {
namespace {

namespace fs = std::filesystem;

std::string stamp(std::time_t when) {
    std::tm local{};
    localtime_r(&when, &local);
    char text[32];
    std::strftime(text, sizeof(text), "%Y%m%d%H%M%S", &local);
    return text;
}

/* <directory>/<base><ext>, with -1, -2 ... if that name exists. */
std::string unique_path(const std::string &directory, const std::string &base, const std::string &ext) {
    const fs::path dir = directory;
    fs::path path = dir / (base + ext);
    for (int i = 1; fs::exists(path); ++i) path = dir / (base + "-" + std::to_string(i) + ext);
    return path.string();
}

class Client {
public:
    Client(const McuLogOptions &options, std::string &error) : options_(options), error_(error) {}
    ~Client() { if (fd_ >= 0) close(fd_); }

    bool open(const std::string &address, uint16_t port) {
        board_.sin_family = AF_INET;
        board_.sin_port = htons(port);
        port_ = port;
        if (inet_pton(AF_INET, address.c_str(), &board_.sin_addr) != 1) { error_ = "MCU IP 地址无效"; return false; }
        fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (fd_ < 0) { error_ = std::string("创建 socket 失败：") + std::strerror(errno); return false; }
        const uint32_t ms = options_.timeout_ms == 0U ? 1U : options_.timeout_ms;
        timeval timeout{static_cast<time_t>(ms / 1000U), static_cast<suseconds_t>(ms % 1000U) * 1000};
        setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
        return true;
    }

    /* One request, retried on timeout.  Replies from elsewhere, or to an
     * earlier (timed-out) request, are skipped.  header.result is not checked. */
    bool request(uint8_t op, uint32_t cursor, InfrabotLogReplyHeader &header, std::vector<uint8_t> &data) {
        const uint32_t id = ++request_id_;
        const InfrabotLogRequest request{INFRABOT_MAGIC_LOG_REQUEST, INFRABOT_LOG_V1, op, 0U, id, cursor, 0U};
        uint8_t buffer[2048];
        for (uint32_t attempt = 0; attempt <= options_.retries; ++attempt) {
            /* The board answers at most 2 requests per ms and stays silent on a third, which
             * would cost a whole timeout. Keep >= 700 us between sends, retries included. */
            const auto due = last_send_ + std::chrono::microseconds(700);
            if (std::chrono::steady_clock::now() < due) std::this_thread::sleep_until(due);
            last_send_ = std::chrono::steady_clock::now();
            if (sendto(fd_, &request, sizeof(request), 0, reinterpret_cast<const sockaddr *>(&board_),
                       sizeof(board_)) != static_cast<ssize_t>(sizeof(request))) {
                error_ = std::string("发送日志请求失败：") + std::strerror(errno);
                return false;
            }
            for (;;) {
                sockaddr_in source{};
                socklen_t length = sizeof(source);
                const ssize_t got = recvfrom(fd_, buffer, sizeof(buffer), 0,
                                             reinterpret_cast<sockaddr *>(&source), &length);
                if (got < 0) {
                    if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) break;  // retry
                    error_ = std::string("接收日志回复失败：") + std::strerror(errno);
                    return false;
                }
                if (source.sin_addr.s_addr != board_.sin_addr.s_addr || source.sin_port != board_.sin_port) continue;
                InfrabotLogReplyHeader reply{};
                if (got < static_cast<ssize_t>(sizeof(reply))) continue;
                std::memcpy(&reply, buffer, sizeof(reply));
                if (reply.magic != INFRABOT_MAGIC_LOG_REPLY || reply.request_id != id) continue;
                if (reply.data_bytes > INFRABOT_LOG_DATA_MAX ||
                    static_cast<size_t>(got) != sizeof(reply) + reply.data_bytes) {
                    error_ = "日志回复长度与 data_bytes 不符";
                    return false;
                }
                if (reply.result == INFRABOT_LOG_OK) latched_ = (reply.frozen & 2U) != 0U;
                header = reply;
                data.assign(buffer + sizeof(reply), buffer + got);
                return true;
            }
        }
        error_ = "MCU 未响应日志请求（UDP 端口 " + std::to_string(port_) +
                 "；固件是否支持 INFRABOT_CAP_RUN_LOG？）";
        return false;
    }

    /* request() plus: result OK, same boot. */
    bool page(uint8_t op, uint32_t cursor, uint32_t boot_id, InfrabotLogReplyHeader &header,
              std::vector<uint8_t> &data) {
        if (!request(op, cursor, header, data)) return false;
        if (header.result == INFRABOT_LOG_BUSY) {
            error_ = "下载中途机械臂开始使能或运动，MCU 停止发送日志";
            return false;
        }
        if (header.result != INFRABOT_LOG_OK) {
            error_ = "MCU 中断了日志下载（result=" + std::to_string(header.result) + "）";
            return false;
        }
        if (header.boot_id != boot_id) { error_ = "下载期间 MCU 复位，日志已重新开始"; return false; }
        return true;
    }

    /* Best effort; the board also lifts a download freeze by itself 5 s after the last
     * request.  A RESUME also releases a fault latch, which is right only once the whole
     * telemetry ring has been downloaded. */
    void release() {
        InfrabotLogReplyHeader header{};
        std::vector<uint8_t> data;
        std::string saved = error_;
        request(INFRABOT_LOG_OP_RESUME, 0U, header, data);
        error_ = saved;
    }
    /* A failed or cancelled download keeps a fault latch, so the next try still finds the
     * ring; a plain download freeze is lifted. */
    void abandon() {
        if (!latched_) release();
    }

private:
    const McuLogOptions &options_;
    std::string &error_;
    int fd_ = -1;
    uint16_t port_ = 0;
    sockaddr_in board_{};
    uint32_t request_id_ = 0;
    bool latched_ = false;  // the board's telemetry ring is held by a fault (last OK reply)
    std::chrono::steady_clock::time_point last_send_{};
};

bool cancelled(const McuLogOptions &options, const McuLogResult &result, uint32_t text_total,
               uint32_t telemetry_total) {
    return options.progress &&
           !options.progress(result.text_lines, text_total, result.telemetry_rows, telemetry_total);
}

} // namespace

bool download_run_log(const std::string &address, const McuLogOptions &options,
                      McuLogResult &result, std::string &error) {
    result = McuLogResult{};
    error.clear();
    Client client(options, error);
    if (!client.open(address, options.port)) return false;

    InfrabotLogReplyHeader header{};
    std::vector<uint8_t> data;
    if (!client.request(INFRABOT_LOG_OP_STATUS, 0U, header, data)) return false;
    if (header.result == INFRABOT_LOG_UNAVAILABLE) { error = "MCU 运行日志不可用（SDRAM 未运行，见启动串口 LOG: 行）"; return false; }
    if (header.result == INFRABOT_LOG_BUSY) {
        error = "MCU 正在使能或运动（使能过程、RT/NRT、阻抗、导纳、重力模式），拒绝日志下载以免占用控制链路；失能、空闲、受控停止或故障时可下载，请先停止或回到空闲再试（故障时的遥测已锁定保留）";
        return false;
    }
    if (header.result != INFRABOT_LOG_OK) { error = "MCU 拒绝日志请求（result=" + std::to_string(header.result) + "）"; return false; }
    const uint32_t boot_id = header.boot_id;
    result.boot_id = boot_id;
    result.writer_max_us = header.writer_max_us;

    std::error_code ec;
    fs::create_directories(options.directory, ec);
    if (ec) { error = "创建日志目录失败：" + options.directory + "（" + ec.message() + "）"; return false; }
    std::string base = "mcu_" + stamp(std::time(nullptr));
    if (!options.label.empty()) {
        base += '_';
        for (char c : options.label) base += (c == '/' || c == '\\' || c == ' ' || c == ':') ? '_' : c;
    }
    result.text_path = unique_path(options.directory, base, ".log");
    result.telemetry_path = unique_path(options.directory, base + "_telemetry", ".csv");
    std::ofstream text(result.text_path, std::ios::binary);
    std::ofstream telemetry(result.telemetry_path, std::ios::binary);
    if (!text || !telemetry) { error = "打开日志文件失败：" + result.text_path; return false; }

    /* Telemetry first, frozen so the 10 s window does not move under the
     * download; then resume and fetch the text (10 minutes, barely moves). */
    if (!client.page(INFRABOT_LOG_OP_FREEZE, 0U, boot_id, header, data)) return false;
    result.fault_latched = (header.frozen & 2U) != 0U;  // bit1: held by a fault, not by this download
    const uint32_t telemetry_total = header.telemetry_next - header.telemetry_first;
    uint32_t text_total = header.text_next - header.text_first;
    telemetry << telemetry_csv_header();
    bool have_sequence = false;
    uint32_t last_sequence = 0U;
    std::string line;
    line.reserve(2048);
    const uint32_t telemetry_end = header.telemetry_next;
    for (uint32_t cursor = header.telemetry_first; cursor != telemetry_end;) {
        if (!client.page(INFRABOT_LOG_OP_TELEMETRY, cursor, boot_id, header, data)) { client.abandon(); return false; }
        if (header.record_count == 0U) break;
        for (uint8_t r = 0; r < header.record_count; ++r) {
            const size_t offset = static_cast<size_t>(r) * sizeof(InfrabotTelemetryV3);
            if (offset + sizeof(InfrabotTelemetryV3) > data.size()) break;
            /* host_time_s = 0: the PC did not receive these live; use mcu_time_s. */
            telemetry_csv_row(0U, data.data() + offset, sizeof(InfrabotTelemetryV3), have_sequence,
                              last_sequence, line);
            telemetry << line;
            ++result.telemetry_rows;
        }
        cursor = header.next_cursor;
        if (cancelled(options, result, text_total, telemetry_total)) {
            client.abandon();
            error = "下载已取消";
            return false;
        }
    }
    telemetry.flush();
    client.release();

    if (!client.page(INFRABOT_LOG_OP_STATUS, 0U, boot_id, header, data)) return false;
    text_total = header.text_next - header.text_first;
    const uint32_t text_end = header.text_next;
    for (uint32_t cursor = header.text_first; cursor != text_end;) {
        if (!client.page(INFRABOT_LOG_OP_TEXT, cursor, boot_id, header, data)) return false;
        if (header.record_count == 0U) break;
        const uint8_t *p = data.data();
        const uint8_t *const end = p + data.size();
        for (uint8_t r = 0; r < header.record_count; ++r) {
            if (end - p < static_cast<ptrdiff_t>(INFRABOT_LOG_TEXT_HEADER)) break;
            uint32_t time_ms = 0U;
            uint16_t length = 0U;
            std::memcpy(&time_ms, p + 4, 4);
            std::memcpy(&length, p + 8, 2);
            const size_t size = (INFRABOT_LOG_TEXT_HEADER + length + 3U) & ~size_t{3};
            if (static_cast<size_t>(end - p) < size) break;
            char time_text[24];
            std::snprintf(time_text, sizeof(time_text), "%u.%03u ", time_ms / 1000U, time_ms % 1000U);
            text << time_text;
            text.write(reinterpret_cast<const char *>(p + INFRABOT_LOG_TEXT_HEADER), length);
            text << '\n';
            ++result.text_lines;
            p += size;
        }
        cursor = header.next_cursor;
        if (cancelled(options, result, text_total, telemetry_total)) { error = "下载已取消"; return false; }
    }
    text.flush();
    result.lost_bytes = header.lost_bytes;
    if (!text.good() || !telemetry.good()) { error = "写入日志文件失败（磁盘满？）"; return false; }
    return true;
}

} // namespace detail
} // namespace infrabot
