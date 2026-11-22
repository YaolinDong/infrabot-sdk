/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "serial_log.hpp"

#include <dirent.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <filesystem>
#include <vector>

namespace infrabot {
namespace detail {
namespace {

double monotonic_s() {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<double>(ts.tv_sec) + static_cast<double>(ts.tv_nsec) * 1e-9;
}

bool baud_constant(uint32_t baud, speed_t &out) {
    switch (baud) {
    case 9600: out = B9600; return true;
    case 19200: out = B19200; return true;
    case 38400: out = B38400; return true;
    case 57600: out = B57600; return true;
    case 115200: out = B115200; return true;
    case 230400: out = B230400; return true;
    case 460800: out = B460800; return true;
    case 921600: out = B921600; return true;
    case 1000000: out = B1000000; return true;
    case 2000000: out = B2000000; return true;
    default: return false;
    }
}

std::string canonical(const std::string &path) {
    std::error_code ec;
    auto resolved = std::filesystem::canonical(path, ec);
    return ec ? path : resolved.string();
}

}  // namespace

std::string serial_log_path(const std::string &directory, std::time_t when) {
    std::tm local{};
    localtime_r(&when, &local);
    char stamp[32]{};
    std::strftime(stamp, sizeof(stamp), "%Y%m%d%H%M%S", &local);
    const std::filesystem::path base(directory);
    auto candidate = base / ("serial_" + std::string(stamp) + ".log");
    for (unsigned suffix = 1; std::filesystem::exists(candidate); ++suffix)
        candidate = base / ("serial_" + std::string(stamp) + "-" + std::to_string(suffix) + ".log");
    return candidate.string();
}

std::string serial_port_users(const std::string &device) {
    const std::string target = canonical(device);
    std::string users;
    DIR *proc = opendir("/proc");
    if (!proc) return users;
    const std::string self = std::to_string(getpid());
    while (dirent *entry = readdir(proc)) {
        const std::string pid = entry->d_name;
        if (pid.empty() || pid.find_first_not_of("0123456789") != std::string::npos || pid == self)
            continue;
        const std::string fd_dir = "/proc/" + pid + "/fd";
        DIR *fds = opendir(fd_dir.c_str());
        if (!fds) continue;
        bool holds = false;
        while (dirent *fd = readdir(fds)) {
            if (fd->d_name[0] == '.') continue;
            char link[4096];
            const ssize_t n = readlink((fd_dir + "/" + fd->d_name).c_str(), link, sizeof(link) - 1);
            if (n > 0) { link[n] = '\0'; if (target == link) { holds = true; break; } }
        }
        closedir(fds);
        if (!holds) continue;
        std::string name;
        if (std::FILE *comm = std::fopen(("/proc/" + pid + "/comm").c_str(), "r")) {
            char buffer[64]{};
            if (std::fgets(buffer, sizeof(buffer), comm)) name = buffer;
            std::fclose(comm);
            while (!name.empty() && (name.back() == '\n' || name.back() == '\r')) name.pop_back();
        }
        if (!users.empty()) users += ", ";
        users += pid + "(" + name + ")";
    }
    closedir(proc);
    return users;
}

bool SerialLog::start(const SerialLogOptions &options, std::string &error) {
    std::lock_guard<std::mutex> control(control_mutex_);
    stop_locked();
    speed_t speed{};
    if (!baud_constant(options.baud, speed)) {
        error = "不支持的串口波特率 " + std::to_string(options.baud);
        return false;
    }
    if (options.directory.empty()) { error = "日志目录为空"; return false; }
    const std::string users = serial_port_users(options.port);
    if (!users.empty()) {
        error = "串口 " + options.port + " 已被其他程序占用: " + users +
                "（同时读取会互相丢数据，请先关闭它）";
        return false;
    }
    std::error_code ec;
    std::filesystem::create_directories(options.directory, ec);
    if (ec) { error = "无法创建日志目录 " + options.directory + ": " + ec.message(); return false; }

    options_ = options;
    // The port need not be present yet: the thread waits for it.  But a
    // missing port at start is almost always a typo, so report it now.
    std::string port_error;
    if (!open_port(port_error) && errno != ENOENT) { error = port_error; return false; }

    path_ = serial_log_path(options.directory, std::time(nullptr));
    file_ = std::fopen(path_.c_str(), "a");
    if (!file_) {
        error = "无法创建 " + path_ + ": " + std::strerror(errno);
        close_port();
        return false;
    }
    if (pipe2(wake_, O_CLOEXEC | O_NONBLOCK) != 0) {
        error = std::string("pipe: ") + std::strerror(errno);
        std::fclose(file_); file_ = nullptr; close_port();
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stats_ = SerialLogStats{};
        stats_.active = true;
        stats_.path = path_;
        stats_.port_open = fd_ >= 0;
        if (fd_ < 0) stats_.error = port_error;
    }
    running_.store(true, std::memory_order_release);
    active_.store(true, std::memory_order_release);
    thread_ = std::thread(&SerialLog::run, this);
    return true;
}

void SerialLog::stop() {
    std::lock_guard<std::mutex> control(control_mutex_);
    stop_locked();
}

void SerialLog::stop_locked() {
    if (!thread_.joinable()) return;
    running_.store(false, std::memory_order_release);
    const char byte = 1;
    if (write(wake_[1], &byte, 1) < 0) {}  // poll() wakes either way within 200 ms
    thread_.join();
    close_port();
    if (file_) { std::fclose(file_); file_ = nullptr; }
    for (int &fd : wake_) { if (fd >= 0) close(fd); fd = -1; }
    active_.store(false, std::memory_order_release);
    std::lock_guard<std::mutex> lock(mutex_);
    stats_.active = false;
    stats_.port_open = false;
}

SerialLogStats SerialLog::stats() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return stats_;
}

bool SerialLog::open_port(std::string &error) {
    close_port();
    const int fd = open(options_.port.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0) {
        const int saved = errno;
        error = "无法打开串口 " + options_.port + ": " + std::strerror(saved);
        errno = saved;
        return false;
    }
    termios tio{};
    speed_t speed{};
    baud_constant(options_.baud, speed);
    if (tcgetattr(fd, &tio) != 0) {
        const int saved = errno;
        error = "串口 " + options_.port + " 不是终端设备: " + std::strerror(saved);
        close(fd);
        errno = saved;
        return false;
    }
    cfmakeraw(&tio);
    tio.c_cflag |= CLOCAL | CREAD;
    tio.c_cflag &= ~CRTSCTS;
    tio.c_cc[VMIN] = 1;
    tio.c_cc[VTIME] = 0;
    cfsetispeed(&tio, speed);
    cfsetospeed(&tio, speed);
    if (tcsetattr(fd, TCSANOW, &tio) != 0) {
        const int saved = errno;
        error = "无法设置串口参数: " + std::string(std::strerror(saved));
        close(fd);
        errno = saved;
        return false;
    }
    tcflush(fd, TCIFLUSH);
    fd_ = fd;
    return true;
}

void SerialLog::close_port() {
    if (fd_ >= 0) close(fd_);
    fd_ = -1;
}

void SerialLog::write_line(const char *text, size_t size) {
    while (size && (text[size - 1] == '\r' || text[size - 1] == '\n')) --size;
    const std::time_t now = std::time(nullptr);
    std::tm local{};
    localtime_r(&now, &local);
    char wall[32]{};
    std::strftime(wall, sizeof(wall), "%Y-%m-%d %H:%M:%S", &local);
    // fwrite, not "%.*s": bytes after a reset can include NUL, which would cut the line.
    const int prefix = std::fprintf(file_, "%.6f %s ", monotonic_s(), wall);
    const size_t body = std::fwrite(text, 1, size, file_);
    const bool ok = prefix >= 0 && body == size && std::fputc('\n', file_) != EOF;
    std::lock_guard<std::mutex> lock(mutex_);
    if (!ok) { stats_.error = std::string("写日志失败: ") + std::strerror(errno); return; }
    ++stats_.lines;
    stats_.bytes += static_cast<uint64_t>(prefix) + size + 1;
}

void SerialLog::run() {
    std::vector<char> pending;
    pending.reserve(4096);
    char buffer[4096];
    while (running_.load(std::memory_order_acquire)) {
        if (fd_ < 0) {
            std::string error;
            if (open_port(error)) {
                std::lock_guard<std::mutex> lock(mutex_);
                stats_.port_open = true;
                stats_.error.clear();
            } else {
                std::lock_guard<std::mutex> lock(mutex_);
                stats_.error = error;
            }
            if (fd_ < 0) {
                pollfd wake{wake_[0], POLLIN, 0};
                poll(&wake, 1, 500);
                continue;
            }
        }
        pollfd fds[2] = {{fd_, POLLIN, 0}, {wake_[0], POLLIN, 0}};
        const int ready = poll(fds, 2, 200);
        if (ready < 0) { if (errno == EINTR) continue; break; }
        if (fds[1].revents) break;
        bool lost = (fds[0].revents & (POLLERR | POLLHUP | POLLNVAL)) != 0;
        if (fds[0].revents & POLLIN) {
            const ssize_t n = read(fd_, buffer, sizeof(buffer));
            if (n > 0) {
                size_t start = 0;
                for (ssize_t i = 0; i < n; ++i) {
                    if (buffer[i] != '\n') continue;
                    pending.insert(pending.end(), buffer + start, buffer + i);
                    write_line(pending.data(), pending.size());
                    pending.clear();
                    start = static_cast<size_t>(i) + 1;
                }
                pending.insert(pending.end(), buffer + start, buffer + n);
                // A runaway line without '\n' (binary noise) must not grow forever.
                if (pending.size() > 64 * 1024) { write_line(pending.data(), pending.size()); pending.clear(); }
                std::fflush(file_);
                lost = false;
            } else if (n == 0 || (errno != EAGAIN && errno != EINTR)) {
                lost = true;
            }
        }
        if (lost) {
            if (!pending.empty()) { write_line(pending.data(), pending.size()); pending.clear(); std::fflush(file_); }
            close_port();
            std::lock_guard<std::mutex> lock(mutex_);
            stats_.port_open = false;
            ++stats_.reconnects;
            stats_.error = "串口断开，等待重新连接";
        }
    }
    if (!pending.empty()) write_line(pending.data(), pending.size());
    if (file_) std::fflush(file_);
}

}  // namespace detail
}  // namespace infrabot
