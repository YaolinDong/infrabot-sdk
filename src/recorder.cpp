/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "recorder.hpp"

#include "infrabot/diagnostics.hpp"

#include <algorithm>
#include <charconv>
#include <chrono>
#include <cstring>
#include <ctime>
#include <filesystem>

namespace infrabot {
namespace detail {
namespace {

/* At 1 kHz telemetry plus 1 kHz commands: ten seconds the disk may stall. */
constexpr size_t kMaxPending = 20000;

void append_u(std::string &line, uint64_t value) {
    char text[24];
    const auto result = std::to_chars(text, text + sizeof(text), value);
    line.append(text, result.ptr);
}
void append_i(std::string &line, int64_t value) {
    char text[24];
    const auto result = std::to_chars(text, text + sizeof(text), value);
    line.append(text, result.ptr);
}
/* A wire value in thousandths (mrad, mN·m) as an exact decimal: -1.234. */
void append_milli(std::string &line, int32_t milli) {
    int64_t value = milli;
    if (value < 0) { line += '-'; value = -value; }
    append_u(line, static_cast<uint64_t>(value / 1000));
    char text[5] = {'.', 0, 0, 0, 0};
    const int64_t fraction = value % 1000;
    text[1] = static_cast<char>('0' + fraction / 100);
    text[2] = static_cast<char>('0' + fraction / 10 % 10);
    text[3] = static_cast<char>('0' + fraction % 10);
    line.append(text, 4);
}
/* Microseconds as seconds with six decimals. */
void append_seconds(std::string &line, uint64_t us) {
    append_u(line, us / 1000000U);
    char text[8] = {'.'};
    uint64_t fraction = us % 1000000U;
    for (int i = 6; i >= 1; --i) { text[i] = static_cast<char>('0' + fraction % 10); fraction /= 10; }
    line.append(text, 7);
}
void append_f(std::string &line, float value) {
    char text[32];
    const auto result = std::to_chars(text, text + sizeof(text), value);
    line.append(text, result.ptr);
}
void append_columns(std::string &line, const char *name, int count, int first = 1) {
    for (int i = 0; i < count; ++i) {
        line += ',';
        line += name;
        append_i(line, first + i);
    }
}
std::string quoted(const std::string &text) {
    std::string out = "\"";
    for (char c : text) {
        if (c == '"') out += '"';
        out += c == '\n' ? ' ' : c;
    }
    return out + "\"";
}
std::string local_time(const char *format) {
    const std::time_t now = std::time(nullptr);
    std::tm local{};
    localtime_r(&now, &local);
    char text[64];
    std::strftime(text, sizeof(text), format, &local);
    return text;
}
std::string safe_label(const std::string &label) {
    std::string out;
    for (char c : label) out += (c == '/' || c == '\\' || c == ' ' || c == ':') ? '_' : c;
    return out;
}

} // namespace

bool Recorder::start(const std::string &directory, const RecordOptions &options, const std::string &info,
                     std::string &error) {
    stop();
    namespace fs = std::filesystem;
    std::string name = local_time("%Y%m%d-%H%M%S");
    if (!options.label.empty()) name += "-" + safe_label(options.label);
    fs::path path = fs::path(directory.empty() ? "." : directory) / name;
    for (unsigned suffix = 1; fs::exists(path); ++suffix)
        path = fs::path(directory.empty() ? "." : directory) / (name + "-" + std::to_string(suffix));
    std::error_code code;
    fs::create_directories(path, code);
    if (code) { error = "无法创建录制目录 " + path.string() + "：" + code.message(); return false; }
    directory_ = path.string();
    options_ = options;
    {
        std::FILE *file = std::fopen((path / "info.txt").c_str(), "w");
        if (!file) { error = "无法写入 " + (path / "info.txt").string(); return false; }
        std::fputs(info.c_str(), file);
        std::fclose(file);
    }
    event_file_ = std::fopen((path / "events.csv").c_str(), "w");
    if (!event_file_) { error = "无法写入 events.csv"; return false; }
    std::fputs("host_time_s,mcu_time_s,source,text\n", event_file_);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        telemetry_rows_ = command_rows_ = events_ = dropped_ = 0;
        file_index_ = 0;
        error_.clear();
        pending_.clear();
        pending_events_.clear();
        pending_.reserve(4096);
        stopping_ = false;
    }
    part_start_us_ = 0;
    have_sequence_ = false;
    if (!open_part(error)) { close_part(); std::fclose(event_file_); event_file_ = nullptr; return false; }
    active_.store(true, std::memory_order_release);
    thread_ = std::thread(&Recorder::run, this);
    return true;
}

void Recorder::stop() {
    if (!thread_.joinable()) return;
    active_.store(false, std::memory_order_release);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stopping_ = true;
    }
    cv_.notify_all();
    thread_.join();
    close_part();
    if (event_file_) { std::fclose(event_file_); event_file_ = nullptr; }
}

void Recorder::push(Kind kind, uint64_t host_us, const void *data, size_t size) {
    if (!active() || size > sizeof(Item::bytes)) return;
    std::lock_guard<std::mutex> lock(mutex_);
    if (pending_.size() >= kMaxPending) { ++dropped_; return; }
    pending_.emplace_back();
    Item &item = pending_.back();
    item.kind = kind;
    item.size = static_cast<uint16_t>(size);
    item.host_us = host_us;
    std::memcpy(item.bytes.data(), data, size);
}

void Recorder::telemetry(uint64_t host_us, const void *data, size_t size) { push(kTelemetry, host_us, data, size); }
void Recorder::command(uint64_t host_us, const void *data, size_t size) {
    if (options_.commands) push(kCommand, host_us, data, size);
}

void Recorder::event(uint64_t host_us, uint64_t mcu_us, const char *source, const std::string &text) {
    if (!active()) return;
    std::lock_guard<std::mutex> lock(mutex_);
    if (pending_events_.size() >= kMaxPending) { ++dropped_; return; }
    pending_events_.push_back({host_us, mcu_us, source, text});
}

RecorderStats Recorder::stats() const {
    RecorderStats stats;
    std::lock_guard<std::mutex> lock(mutex_);
    stats.active = active();
    stats.directory = directory_;
    stats.telemetry_rows = telemetry_rows_;
    stats.command_rows = command_rows_;
    stats.events = events_;
    stats.dropped = dropped_;
    stats.file_index = file_index_;
    stats.error = error_;
    return stats;
}

bool Recorder::open_part(std::string &error) {
    namespace fs = std::filesystem;
    char suffix[16];
    std::snprintf(suffix, sizeof(suffix), "_%03u.csv", file_index_);
    telemetry_file_ = std::fopen((fs::path(directory_) / ("telemetry" + std::string(suffix))).c_str(), "w");
    if (!telemetry_file_) { error = "无法写入 telemetry 文件"; return false; }
    std::setvbuf(telemetry_file_, nullptr, _IOFBF, 1 << 20);
    std::fputs(telemetry_csv_header().c_str(), telemetry_file_);
    std::string header;
    if (options_.commands) {
        command_file_ = std::fopen((fs::path(directory_) / ("commands" + std::string(suffix))).c_str(), "w");
        if (!command_file_) { error = "无法写入 commands 文件"; return false; }
        std::setvbuf(command_file_, nullptr, _IOFBF, 1 << 20);
        header = "host_time_s,type,sequence,command,motor";
        append_columns(header, "q", 8);
        append_columns(header, "dq", 8);
        append_columns(header, "kp", 8);
        append_columns(header, "kd", 8);
        append_columns(header, "tau", 8);
        append_columns(header, "vlim", 8);
        append_columns(header, "ratio", 8);
        append_columns(header, "pose", 12, 0);
        append_columns(header, "limit", 4, 0);
        header += ",move_op,move_id,mode,rid,value\n";
        std::fputs(header.c_str(), command_file_);
    }
    return true;
}

void Recorder::close_part() {
    if (telemetry_file_) { std::fclose(telemetry_file_); telemetry_file_ = nullptr; }
    if (command_file_) { std::fclose(command_file_); command_file_ = nullptr; }
}

void Recorder::write_telemetry(const Item &item, std::string &line) {
    telemetry_csv_row(item.host_us, item.bytes.data(), item.size, have_sequence_, last_sequence_, line);
}

std::string telemetry_csv_header() {
    std::string header = "host_time_s,mcu_time_s,sequence,lost,status,mode,profile,enabled_mask,"
                         "command_active,command_applied_sequence";
    append_columns(header, "q", 8);
    append_columns(header, "dq", 8);
    append_columns(header, "tau", 8);
    append_columns(header, "state", 8);
    append_columns(header, "temp_mos", 8);
    append_columns(header, "q_cmd", 8);
    append_columns(header, "dq_cmd", 8);
    append_columns(header, "tau_cmd", 8);
    append_columns(header, "temp_coil", 8);
    append_columns(header, "flags", 8);
    header += ",stop_reason,stop_phase,stop_joint,stop_detail,"
              "mcu_cmd_rx,mcu_cmd_invalid,mcu_cmd_interval_max_us,mcu_control_missed\n";
    return header;
}

void telemetry_csv_row(uint64_t host_us, const void *data, size_t size, bool &have_sequence,
                       uint32_t &last_sequence, std::string &line) {
    InfrabotTelemetryV3 packet{};
    std::memcpy(&packet, data, std::min<size_t>(size, sizeof(packet)));
    const InfrabotTelemetry &head = packet.head;
    const bool v3 = head.version == INFRABOT_TELEMETRY_V3 && size == sizeof(InfrabotTelemetryV3);
    uint32_t lost = 0;
    if (have_sequence) {
        const uint32_t gap = head.sequence - last_sequence;
        if (gap > 1U && gap < 0x80000000U) lost = gap - 1U;
    }
    have_sequence = true;
    last_sequence = head.sequence;
    line.clear();
    append_seconds(line, host_us); line += ',';
    append_seconds(line, head.mcu_timestamp_us); line += ',';
    append_u(line, head.sequence); line += ',';
    append_u(line, lost); line += ',';
    append_u(line, INFRABOT_STATUS_FROM_BYTE(head.status)); line += ',';
    append_u(line, INFRABOT_MODE_FROM_BYTE(head.mode)); line += ',';
    append_u(line, INFRABOT_PROFILE_FROM_BYTE(head.mode)); line += ',';
    append_u(line, head.enabled_mask); line += ',';
    append_u(line, head.command_active); line += ',';
    append_u(line, head.command_applied_sequence);
    for (const auto &motor : head.motor) { line += ','; append_milli(line, motor.position_mrad); }
    for (const auto &motor : head.motor) { line += ','; append_milli(line, motor.velocity_mrad_s); }
    for (const auto &motor : head.motor) { line += ','; append_milli(line, motor.torque_mNm); }
    for (const auto &motor : head.motor) { line += ','; append_u(line, motor.state); }
    for (const auto &motor : head.motor) { line += ','; append_u(line, motor.temperature_mos); }
    if (v3) {
        for (const auto &command : packet.command) { line += ','; append_milli(line, command.position_mrad); }
        for (const auto &command : packet.command) { line += ','; append_milli(line, command.velocity_mrad_s); }
        for (const auto &command : packet.command) { line += ','; append_milli(line, command.torque_mNm); }
        for (const auto &command : packet.command) { line += ','; append_u(line, command.temperature_coil); }
        for (const auto &command : packet.command) { line += ','; append_u(line, command.flags); }
        line += ','; append_u(line, packet.safety.stop_reason);
        line += ','; append_u(line, packet.safety.stop_phase);
        line += ','; append_u(line, packet.safety.stop_joint);
        line += ','; append_u(line, packet.safety.stop_detail);
        line += ','; append_u(line, packet.link.command_rx);
        line += ','; append_u(line, packet.link.command_invalid);
        line += ','; append_u(line, packet.link.command_interval_max_us);
        line += ','; append_u(line, packet.link.control_missed);
    } else {
        line.append(40 + 8, ',');
    }
    line += '\n';
}

void Recorder::write_command(const Item &item, std::string &line) {
    line.clear();
    uint32_t magic = 0;
    std::memcpy(&magic, item.bytes.data(), sizeof(magic));
    std::array<std::string, 8> q, dq, kp, kd, tau, vlim, ratio;
    std::array<std::string, 12> pose;
    std::array<std::string, 4> limits;
    std::string type, sequence, command, motor, move_op, move_id, mode, rid, value;
    auto milli = [](int32_t v) { std::string s; append_milli(s, v); return s; };
    auto real = [](float v) { std::string s; append_f(s, v); return s; };
    if (magic == INFRABOT_MAGIC_MOVE_REQUEST && item.size == sizeof(InfrabotMoveRequest)) {
        InfrabotMoveRequest request;
        std::memcpy(&request, item.bytes.data(), sizeof(request));
        if (request.op == INFRABOT_MOVE_OP_KEEPALIVE || request.op == INFRABOT_MOVE_OP_QUERY) return;
        type = "dmov";
        move_op = std::to_string(request.op);
        move_id = std::to_string(request.motion_id);
        if (request.op == INFRABOT_MOVE_OP_MOVEJ) {
            const auto &joint = request.command.joint;
            for (size_t i = 0; i < INFRABOT_ARM_AXES; ++i) {
                q[i] = real(joint.target[i]); dq[i] = real(joint.velocity_max[i]);
                kp[i] = real(joint.kp[i]); kd[i] = real(joint.kd[i]); tau[i] = real(joint.torque[i]);
            }
        } else if (request.op == INFRABOT_MOVE_OP_MOVEP) {
            const auto &cartesian = request.command.cartesian;
            for (size_t i = 0; i < 12; ++i) pose[i] = real(cartesian.pose[i]);
            for (size_t i = 0; i < INFRABOT_ARM_AXES; ++i) {
                vlim[i] = real(cartesian.velocity_max[i]);
                kp[i] = real(cartesian.kp[i]); kd[i] = real(cartesian.kd[i]);
            }
            for (size_t i = 0; i < 4; ++i) limits[i] = std::to_string(cartesian.cartesian_limits[i]);
        }
    } else if (magic == INFRABOT_MAGIC_COMMAND && item.size >= 8) {
        uint16_t version = 0;
        std::memcpy(&version, item.bytes.data() + 4, sizeof(version));
        if (version == INFRABOT_COMMAND_V3 && item.size == sizeof(InfrabotCommand)) {
            InfrabotCommand packet;
            std::memcpy(&packet, item.bytes.data(), sizeof(packet));
            if (packet.command == INFRABOT_CMD_HOLD_HEARTBEAT) return;
            type = "v3"; sequence = std::to_string(packet.sequence); command = std::to_string(packet.command);
            for (size_t i = 0; i < 8; ++i) q[i] = milli(packet.target_position_mrad[i]);
        } else if (version == INFRABOT_COMMAND_V4 && item.size == sizeof(InfrabotMaintenance)) {
            InfrabotMaintenance packet;
            std::memcpy(&packet, item.bytes.data(), sizeof(packet));
            type = "v4"; sequence = std::to_string(packet.sequence); command = std::to_string(packet.command);
            motor = std::to_string(packet.motor_index);
            mode = std::to_string(packet.mode);
            rid = std::to_string(packet.register_id);
            value = std::to_string(packet.register_value);
            const size_t i = packet.motor_index < 8 ? packet.motor_index : 0;
            q[i] = milli(packet.target_position_mrad[i]);
            dq[i] = milli(packet.target_velocity_mrad_s);
            kp[i] = milli(packet.kp_milli); kd[i] = milli(packet.kd_milli);
            tau[i] = milli(packet.torque_mNm); vlim[i] = milli(packet.velocity_limit_mrad_s);
            ratio[i] = milli(packet.torque_ratio_milli);
        } else if (version == INFRABOT_COMMAND_V5 && item.size == sizeof(InfrabotJointStream)) {
            InfrabotJointStream packet;
            std::memcpy(&packet, item.bytes.data(), sizeof(packet));
            type = "v5"; sequence = std::to_string(packet.sequence); command = std::to_string(packet.command);
            for (size_t i = 0; i < 8; ++i) {
                q[i] = milli(packet.target_position_mrad[i]); dq[i] = milli(packet.target_velocity_mrad_s[i]);
                kp[i] = milli(packet.kp_milli[i]); kd[i] = milli(packet.kd_milli[i]);
                tau[i] = milli(packet.torque_mNm[i]); vlim[i] = milli(packet.velocity_limit_mrad_s[i]);
                ratio[i] = milli(packet.torque_ratio_milli[i]);
            }
        } else if (version == INFRABOT_COMMAND_V7 && item.size == sizeof(InfrabotCartesianStream)) {
            InfrabotCartesianStream packet;
            std::memcpy(&packet, item.bytes.data(), sizeof(packet));
            type = "v7"; sequence = std::to_string(packet.sequence); command = std::to_string(packet.command);
            for (size_t i = 0; i < 12; ++i) pose[i] = real(packet.pose[i]);
            for (size_t i = 0; i < 8; ++i) {
                dq[i] = milli(packet.target_velocity_mrad_s[i]);
                kp[i] = milli(packet.kp_milli[i]); kd[i] = milli(packet.kd_milli[i]);
                tau[i] = milli(packet.torque_mNm[i]); vlim[i] = milli(packet.velocity_limit_mrad_s[i]);
                ratio[i] = milli(packet.torque_ratio_milli[i]);
            }
            for (size_t i = 0; i < 4; ++i) limits[i] = std::to_string(packet.cartesian_limits[i]);
        } else return;
    } else return;
    append_seconds(line, item.host_us);
    line += ',' + type + ',' + sequence + ',' + command + ',' + motor;
    for (const auto *column : {&q, &dq, &kp, &kd, &tau, &vlim, &ratio})
        for (const auto &cell : *column) { line += ','; line += cell; }
    for (const auto &cell : pose) { line += ','; line += cell; }
    for (const auto &cell : limits) { line += ','; line += cell; }
    line += ',' + move_op + ',' + move_id + ',' + mode + ',' + rid + ',' + value + '\n';
}

void Recorder::run() {
    std::string line;
    line.reserve(2048);
    for (;;) {
        bool stopping;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            cv_.wait_for(lock, std::chrono::milliseconds(20), [this] { return stopping_; });
            writing_.swap(pending_);
            writing_events_.swap(pending_events_);
            stopping = stopping_;
        }
        uint64_t telemetry_rows = 0, command_rows = 0;
        std::string error;
        for (const Item &item : writing_) {
            if (options_.rotate_seconds && item.kind == kTelemetry) {
                if (!part_start_us_) part_start_us_ = item.host_us;
                else if (item.host_us - part_start_us_ >= uint64_t(options_.rotate_seconds) * 1000000U) {
                    close_part();
                    {
                        std::lock_guard<std::mutex> lock(mutex_);
                        ++file_index_;
                    }
                    if (!open_part(error)) break;
                    part_start_us_ = item.host_us;
                }
            }
            if (item.kind == kTelemetry && telemetry_file_) {
                write_telemetry(item, line);
                if (std::fwrite(line.data(), 1, line.size(), telemetry_file_) != line.size())
                    error = "写入 telemetry 失败（磁盘满？）";
                ++telemetry_rows;
            } else if (item.kind == kCommand && command_file_) {
                write_command(item, line);
                if (!line.empty()) {
                    if (std::fwrite(line.data(), 1, line.size(), command_file_) != line.size())
                        error = "写入 commands 失败（磁盘满？）";
                    ++command_rows;
                }
            }
        }
        for (const Event &event : writing_events_) {
            line.clear();
            append_seconds(line, event.host_us);
            line += ',';
            if (event.mcu_us) append_seconds(line, event.mcu_us);
            line += ',';
            line += event.source;
            line += ',' + quoted(event.text) + '\n';
            if (event_file_) std::fwrite(line.data(), 1, line.size(), event_file_);
        }
        if (event_file_ && !writing_events_.empty()) std::fflush(event_file_);
        {
            std::lock_guard<std::mutex> lock(mutex_);
            telemetry_rows_ += telemetry_rows;
            command_rows_ += command_rows;
            events_ += writing_events_.size();
            if (!error.empty()) error_ = error;
        }
        writing_.clear();
        writing_events_.clear();
        if (stopping) break;
    }
    if (telemetry_file_) std::fflush(telemetry_file_);
    if (command_file_) std::fflush(command_file_);
}

} // namespace detail
} // namespace infrabot
