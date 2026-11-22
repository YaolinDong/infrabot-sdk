/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
/* Robot::start_serial_log against a pseudo-terminal standing in for the
 * MCU's USB-serial adapter: file naming, line framing (split reads, CRLF),
 * busy-port refusal, unplug/reopen and stop. No board or network needed. */
#include "infrabot/robot.hpp"
#include "serial_log.hpp"

#include <fcntl.h>
#include <pty.h>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cassert>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <regex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace fs = std::filesystem;
using infrabot::Robot;
using infrabot::SerialLogOptions;

static void sleep_ms(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

static std::vector<std::string> lines_of(const std::string &path) {
    std::ifstream in(path);
    std::vector<std::string> out;
    for (std::string line; std::getline(in, line);) out.push_back(line);
    return out;
}

static bool wait_lines(const Robot &robot, uint64_t count) {
    for (int i = 0; i < 100; ++i) { if (robot.serial_log().lines >= count) return true; sleep_ms(20); }
    return false;
}

static void write_all(int fd, const std::string &text) {
    assert(write(fd, text.data(), text.size()) == static_cast<ssize_t>(text.size()));
}

int main() {
    const fs::path dir = fs::temp_directory_path() / ("infrabot_serial_log_" + std::to_string(getpid()));
    fs::remove_all(dir);

    // Naming: serial_YYYYMMDDHHMMSS.log, suffixed when the name exists.
    {
        fs::create_directories(dir);
        std::tm t{}; t.tm_year = 126; t.tm_mon = 10; t.tm_mday = 20; t.tm_hour = 0; t.tm_min = 19; t.tm_sec = 17; t.tm_isdst = -1;
        const std::time_t when = std::mktime(&t);
        const std::string first = infrabot::detail::serial_log_path(dir.string(), when);
        assert(fs::path(first).filename() == "serial_20261120001917.log");
        std::ofstream(first).put('x');
        const std::string second = infrabot::detail::serial_log_path(dir.string(), when);
        assert(fs::path(second).filename() == "serial_20261120001917-1.log");
        fs::remove(first);
    }

    int master = -1, slave = -1;
    char name[128]{};
    assert(openpty(&master, &slave, name, nullptr, nullptr) == 0);
    close(slave);  // the logger opens the slave by name, like a real tty
    // Keep a slave reference open so the pty does not report hang-up while
    // the logger is between reopen attempts; it must not count as a "user".
    const int keep = open(name, O_RDWR | O_NOCTTY);
    assert(keep >= 0);

    Robot robot;
    SerialLogOptions options;
    options.port = name;
    options.directory = dir.string();

    // A port held by another process is refused, naming the holder.
    {
        const pid_t child = fork();
        if (child == 0) {
            const int fd = open(name, O_RDONLY | O_NOCTTY);
            (void)fd;
            sleep(5);
            _exit(0);
        }
        sleep_ms(200);
        std::string error;
        assert(!robot.start_serial_log(error, options));
        assert(error.find(std::to_string(child)) != std::string::npos);
        std::printf("busy port refused: %s\n", error.c_str());
        kill(child, SIGKILL);
        waitpid(child, nullptr, 0);
    }
    // Our own extra handle (keep) is in this process, so it is not a "user".
    std::string error;
    assert(robot.start_serial_log(error, options));
    const auto stats = robot.serial_log();
    assert(stats.active && stats.port_open);
    assert(std::regex_match(fs::path(stats.path).filename().string(), std::regex(R"(serial_\d{14}\.log)")));
    sleep_ms(100);

    write_all(master, "STAT ACTIVE en=7F ctrl=1000Hz miss=0\r\nADM JITTER rev=3 ff_step_max_mNm=41\r\nhal");
    sleep_ms(60);
    write_all(master, "f line\r\n");
    write_all(master, std::string("boot\0noise\r\n", 12));  // NUL after a reset must not cut the line
    assert(wait_lines(robot, 4));

    auto lines = lines_of(stats.path);
    assert(lines.size() == 4);
    const std::regex row(R"((\d+\.\d{6}) (\d{4}-\d\d-\d\d \d\d:\d\d:\d\d) (.*))");
    std::smatch m;
    assert(std::regex_match(lines[0], m, row) && m[3] == "STAT ACTIVE en=7F ctrl=1000Hz miss=0");
    assert(std::regex_match(lines[1], m, row) && m[3] == "ADM JITTER rev=3 ff_step_max_mNm=41");
    assert(std::regex_match(lines[2], m, row) && m[3] == "half line");
    assert(lines[3].size() >= 10 && lines[3].compare(lines[3].size() - 10, 10, std::string("boot\0noise", 10)) == 0);
    // host_time_s is CLOCK_MONOTONIC, comparable with start_recording()'s host_time_s.
    timespec now{};
    clock_gettime(CLOCK_MONOTONIC, &now);
    std::regex_match(lines[2], m, row);
    const double logged = std::stod(m[1]);
    assert(logged <= now.tv_sec + 1.0 && logged > now.tv_sec - 5.0);

    // Unplug: the device disappears, then comes back as a new pty under the
    // same path. Simulate with a symlink like /dev/serial/by-id does.
    robot.stop_serial_log();
    assert(!robot.serial_log().active);
    {
        const fs::path link = dir / "ttyLINK";
        fs::create_symlink(name, link);
        options.port = link.string();
        assert(robot.start_serial_log(error, options));
        const std::string second_path = robot.serial_log().path;
        assert(second_path != stats.path);
        sleep_ms(100);
        write_all(master, "before unplug\r\n");
        assert(wait_lines(robot, 1));

        close(keep);
        close(master);           // hang-up: the logger loses the port
        fs::remove(link);        // and it is gone, as with a pulled USB cable
        for (int i = 0; i < 100 && robot.serial_log().port_open; ++i) sleep_ms(20);
        assert(!robot.serial_log().port_open);
        assert(robot.serial_log().reconnects == 1);

        int master2 = -1, slave2 = -1;
        char name2[128]{};
        assert(openpty(&master2, &slave2, name2, nullptr, nullptr) == 0);
        fs::create_symlink(name2, link);
        for (int i = 0; i < 150 && !robot.serial_log().port_open; ++i) sleep_ms(20);
        assert(robot.serial_log().port_open);
        write_all(master2, "after replug\r\n");
        assert(wait_lines(robot, 2));
        lines = lines_of(second_path);
        assert(lines.size() == 2);
        assert(std::regex_match(lines[0], m, row) && m[3] == "before unplug");
        assert(std::regex_match(lines[1], m, row) && m[3] == "after replug");
        robot.stop_serial_log();
        close(slave2);
        close(master2);
    }

    unsetenv("INFRABOT_LOG_DIR");
    unsetenv("XDG_STATE_HOME");
    setenv("HOME", dir.c_str(), 1);
    assert(fs::path(Robot::log_directory()) == dir / ".local/state/infrabot/log");
    setenv("XDG_STATE_HOME", dir.c_str(), 1);
    assert(fs::path(Robot::log_directory()) == dir / "infrabot/log");
    setenv("INFRABOT_LOG_DIR", dir.c_str(), 1);
    assert(fs::path(Robot::log_directory()) == dir);

    fs::remove_all(dir);
    std::puts("serial log: naming, framing, busy-port refusal, unplug/replug, stop: OK");
    return 0;
}
