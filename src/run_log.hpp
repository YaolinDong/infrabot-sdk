/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#pragma once

/* Download of the MCU run log (INFRABOT_LOG_PORT, DMLG/DMLR): the text the
 * board printed (newest 10 minutes) and its 1 kHz telemetry (newest 10 s).
 * One request, one reply; the board never sends on its own and refuses while
 * a PC real-time stream is active, so a download cannot disturb 1 kHz control. */

#include "infrabot/protocol.h"
#include "infrabot/robot.hpp"

#include <cstdint>
#include <string>

namespace infrabot {
namespace detail {

/* Downloads both rings into new files in options.directory (must be set). */
bool download_run_log(const std::string &address, const McuLogOptions &options,
                      McuLogResult &result, std::string &error);

} // namespace detail
} // namespace infrabot
