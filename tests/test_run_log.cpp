/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
/* Robot::download_mcu_log against a loopback board serving DMLG/DMLR: paging,
 * file names and contents, freeze/resume, a fault latch that only a finished
 * download releases, refusal while the arm is not idle, lost replies,
 * MCU reset mid-download, cancel and missing capability.
 *
 * The board model mirrors firmware app/app_log.c.  Built with
 * -DRUN_LOG_WITH_FIRMWARE -D_Static_assert=static_assert -I<firmware>/app -I<firmware>/protocol
 * and an object of <firmware>/app/app_log.c, the firmware code answers. */
#include "fake_log_board.hpp"

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

namespace fs = std::filesystem;
using infrabot::McuLogOptions;
using infrabot::McuLogResult;
using infrabot::Robot;

namespace {

using fake_log::Board;


std::vector<std::string> lines_of(const std::string &path) {
    std::ifstream in(path);
    std::vector<std::string> out;
    for (std::string line; std::getline(in, line);) out.push_back(line);
    return out;
}

std::string field(const std::string &line, size_t index) {
    size_t start = 0;
    for (size_t i = 0; i < index; ++i) start = line.find(',', start) + 1;
    return line.substr(start, line.find(',', start) - start);
}

/* text_count/telemetry_count ring slots; fills 'lines' lines (lengths 0..300,
 * so some are cut at 246) and 'rows' telemetry packets with sequence 100.. */
void fill(Board &board, uint32_t text_count, uint32_t lines, uint32_t telemetry_count, uint32_t rows) {
    std::lock_guard<std::mutex> lock(board.mutex);
    board.model.init(text_count, telemetry_count);
    board.model.boot_id = 0x1234U;  // each case is a fresh boot; an earlier reset case moved it
    for (uint32_t i = 0; i < lines; ++i) {
        std::string line = "L" + std::to_string(i) + " ";
        line += std::string(i % 300U, char('a' + i % 26U));
        board.model.add_text(1000U + i * 7U, line);
    }
    for (uint32_t i = 0; i < rows; ++i) {
        InfrabotTelemetryV3 packet{};
        packet.head.version = INFRABOT_TELEMETRY_V3;
        packet.head.sequence = 100U + i;
        packet.head.mcu_timestamp_us = 5000000ULL + i * 1000ULL;
        packet.head.motor[0].position_mrad = static_cast<int32_t>(i);
        board.model.add_telemetry(packet);
    }
    board.next_sequence = 100U + rows;
    board.requests = board.freezes = board.resumes = 0;
}

McuLogOptions options_for(const Board &board, const fs::path &directory) {
    McuLogOptions options;
    options.directory = directory.string();
    options.port = board.port;
    options.timeout_ms = 50;
    options.retries = 3;
    return options;
}

} // namespace

static void check_full_download(Board &board, const fs::path &dir) {
    /* Rings keep count-1 records: 1999 of 2500 lines, 1000 of 1500 rows.
     * The writer adds 3 rows per request until FREEZE. */
    fill(board, 2000, 2500, 1001, 1500);
    { std::lock_guard<std::mutex> lock(board.mutex); board.telemetry_per_request = 3; board.model.writer_max_us = 42; board.model.lost_bytes = 5; }
    Robot robot("127.0.0.1");
    McuLogOptions options = options_for(board, dir);
    uint32_t calls = 0, last_total = 0;
    options.progress = [&](uint32_t, uint32_t, uint32_t, uint32_t telemetry_total) { ++calls; last_total = telemetry_total; return true; };
    McuLogResult result;
    std::string error;
    assert(robot.download_mcu_log(result, error, options));
    assert(error.empty());
    assert(result.boot_id == 0x1234U && result.writer_max_us == 42U && result.lost_bytes == 5U);
    assert(result.text_lines == 1999U && result.telemetry_rows == 1000U);
    assert(calls > 10U && last_total == 1000U);
    {
        std::lock_guard<std::mutex> lock(board.mutex);
        assert(board.freezes == 1U && board.resumes == 1U && !board.model.frozen);
        board.telemetry_per_request = 0;
    }
    const std::string name = fs::path(result.text_path).filename().string();
    assert(name.size() == std::string("mcu_20261120001917.log").size() && name.rfind("mcu_", 0) == 0);
    assert(fs::path(result.telemetry_path).filename().string() == name.substr(0, 18) + "_telemetry.csv");

    const auto text = lines_of(result.text_path);
    assert(text.size() == 1999U);
    for (uint32_t k = 0; k < text.size(); ++k) {
        const uint32_t i = 501U + k, ms = 1000U + i * 7U;
        char prefix[64];
        std::snprintf(prefix, sizeof(prefix), "%u.%03u L%u ", ms / 1000U, ms % 1000U, i);
        std::string body = "L" + std::to_string(i) + " " + std::string(i % 300U, char('a' + i % 26U));
        body = body.substr(0, 246);
        const std::string expected = std::string(prefix).substr(0, std::string(prefix).find(' ') + 1) + body;
        assert(text[k] == expected);
    }

    const auto csv = lines_of(result.telemetry_path);
    assert(csv.size() == 1001U && csv[0].rfind("host_time_s,mcu_time_s,sequence,lost", 0) == 0);
    /* 1500 + 3 (STATUS) + 3 (FREEZE) written; the newest 1000 kept. */
    const uint32_t first = 100U + 1506U - 1000U;
    for (uint32_t k = 1; k < csv.size(); ++k) {
        assert(field(csv[k], 2) == std::to_string(first + k - 1U));
        assert(field(csv[k], 3) == "0");
        assert(field(csv[k], 0) == "0.000000");
    }
    assert(field(csv[1000], 2) == "1605");

    /* Same second again: a new name, never an overwrite; label appended. */
    McuLogResult again;
    assert(robot.download_mcu_log(again, error, options));
    assert(again.writer_max_us == 0U);  // reset by the previous STATUS
    assert(again.text_path != result.text_path && again.telemetry_path != result.telemetry_path);
    options.label = "after crash";
    assert(robot.download_mcu_log(again, error, options));
    assert(fs::path(again.text_path).filename().string().find("_after_crash") == 18U);
    std::printf("full download ok: %s\n", name.c_str());
}

static void check_refusals(Board &board, const fs::path &dir) {
    Robot robot("127.0.0.1");
    McuLogResult result;
    std::string error;
    fill(board, 100, 10, 100, 10);
    { std::lock_guard<std::mutex> lock(board.mutex); board.busy = true; }
    assert(!robot.download_mcu_log(result, error, options_for(board, dir / "busy")));
    assert(error.find("拒绝日志下载") != std::string::npos);
    assert(!fs::exists(dir / "busy"));
    { std::lock_guard<std::mutex> lock(board.mutex); board.busy = false; board.model.ready = false; }
    assert(!robot.download_mcu_log(result, error, options_for(board, dir / "sdram")));
    assert(error.find("SDRAM") != std::string::npos);
    { std::lock_guard<std::mutex> lock(board.mutex); board.model.ready = true; }

    McuLogOptions nobody = options_for(board, dir / "nobody");
    { Board closed; nobody.port = closed.port; }  // bound then closed: nothing answers
    nobody.timeout_ms = 20;
    nobody.retries = 1;
    assert(!robot.download_mcu_log(result, error, nobody));
    assert(error.find("未响应") != std::string::npos);

    McuLogOptions bad = options_for(board, dir);
    Robot nowhere("not-an-ip");
    assert(!nowhere.download_mcu_log(result, error, bad));
    assert(error.find("IP") != std::string::npos);
    std::printf("refusals ok\n");
}

static void check_lossy_link(Board &board, const fs::path &dir) {
    fill(board, 600, 500, 300, 299);
    { std::lock_guard<std::mutex> lock(board.mutex); board.drop_every = 5; }
    Robot robot("127.0.0.1");
    McuLogResult result;
    std::string error;
    assert(robot.download_mcu_log(result, error, options_for(board, dir)));
    assert(result.text_lines == 500U && result.telemetry_rows == 299U);
    const auto csv = lines_of(result.telemetry_path);
    for (uint32_t k = 1; k < csv.size(); ++k) assert(field(csv[k], 2) == std::to_string(99U + k));
    { std::lock_guard<std::mutex> lock(board.mutex); board.drop_every = 0; }
    std::printf("lossy link ok\n");
}

static void check_reset_and_cancel(Board &board, const fs::path &dir) {
    Robot robot("127.0.0.1");
    McuLogResult result;
    std::string error;
    fill(board, 2000, 1500, 1001, 1000);
    { std::lock_guard<std::mutex> lock(board.mutex); board.reset_after = 20; }
    assert(!robot.download_mcu_log(result, error, options_for(board, dir)));
    assert(error.find("复位") != std::string::npos);
    { std::lock_guard<std::mutex> lock(board.mutex); assert(board.resumes == 1U); board.reset_after = 0; }

    fill(board, 2000, 1500, 1001, 1000);
    McuLogOptions options = options_for(board, dir);
    options.progress = [](uint32_t, uint32_t, uint32_t rows, uint32_t) { return rows < 100U; };
    assert(!robot.download_mcu_log(result, error, options));
    assert(error.find("取消") != std::string::npos);
    { std::lock_guard<std::mutex> lock(board.mutex); assert(board.resumes == 1U && !board.model.frozen); }
    std::printf("reset and cancel ok\n");
}

static void check_fault_latch(Board &board, const fs::path &dir) {
    Robot robot("127.0.0.1");
    McuLogResult result;
    std::string error;
    /* 1500 rows into 1001 slots keep sequences 600..1599.  The arm faults and the board
     * latches the ring: the 3 rows per request that the writer would add never arrive,
     * so the CSV ends at the fault.  Only a finished download releases the latch. */
    fill(board, 200, 50, 1001, 1500);
    { std::lock_guard<std::mutex> lock(board.mutex); board.model.latch(); board.telemetry_per_request = 3; board.busy = true; }

    /* The arm is stopped, not idle: refused, and a refusal cannot release the latch. */
    assert(!robot.download_mcu_log(result, error, options_for(board, dir / "refused")));
    assert(error.find("拒绝日志下载") != std::string::npos);
    assert(!fs::exists(dir / "refused"));
    { std::lock_guard<std::mutex> lock(board.mutex); assert(board.model.latched && board.resumes == 0U); board.busy = false; }

    /* Back to idle.  A cancelled download keeps the latch, so the next try still finds the fault. */
    McuLogOptions cancel = options_for(board, dir);
    cancel.progress = [](uint32_t, uint32_t, uint32_t rows, uint32_t) { return rows < 100U; };
    assert(!robot.download_mcu_log(result, error, cancel));
    assert(error.find("取消") != std::string::npos);
    { std::lock_guard<std::mutex> lock(board.mutex); assert(board.model.latched && board.resumes == 0U); }

    /* So does an MCU reset in the middle of the download. */
    { std::lock_guard<std::mutex> lock(board.mutex); board.reset_after = board.requests + 20U; }
    assert(!robot.download_mcu_log(result, error, options_for(board, dir)));
    assert(error.find("复位") != std::string::npos);
    { std::lock_guard<std::mutex> lock(board.mutex); assert(board.model.latched && board.resumes == 0U); board.reset_after = 0; }

    /* A whole download: the CSV is the lead-up to the fault, then the board resumes. */
    assert(robot.download_mcu_log(result, error, options_for(board, dir)));
    assert(result.fault_latched && result.telemetry_rows == 1000U && result.text_lines == 50U);
    const auto csv = lines_of(result.telemetry_path);
    assert(csv.size() == 1001U);
    for (uint32_t k = 1; k < csv.size(); ++k) assert(field(csv[k], 2) == std::to_string(599U + k));
    {
        std::lock_guard<std::mutex> lock(board.mutex);
        assert(board.resumes == 1U && !board.model.latched && !board.model.frozen);
        board.telemetry_per_request = 0;
    }
    McuLogResult again;
    assert(robot.download_mcu_log(again, error, options_for(board, dir)));
    assert(!again.fault_latched);
    std::printf("fault latch ok\n");
}

int main() {
    const fs::path dir = fs::temp_directory_path() / ("infrabot_run_log_" + std::to_string(getpid()));
    fs::remove_all(dir);
    {
        Board board;
        check_full_download(board, dir / "full");
        check_refusals(board, dir / "refuse");
        check_lossy_link(board, dir / "lossy");
        check_reset_and_cancel(board, dir / "reset");
        check_fault_latch(board, dir / "latch");
    }
    fs::remove_all(dir);
#ifdef RUN_LOG_WITH_FIRMWARE
    std::printf("test_run_log (firmware app_log.c): all ok\n");
#else
    std::printf("test_run_log: all ok\n");
#endif
    return 0;
}
