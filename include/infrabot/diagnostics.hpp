/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#pragma once

/**
 * @file diagnostics.hpp
 * @brief Human-readable meaning of the codes in infrabot/protocol.h.
 *
 * Drive faults with their probable causes and what to do, controlled stops, the
 * parameter self-check, DMAR replies and the drive registers. The texts are in
 * Chinese. All functions are [Non-blocking] local lookups for supervisory code.
 */

#include "infrabot/protocol.h"
#include "infrabot/robot.hpp"

#include <cstdint>
#include <string>

namespace infrabot {

/// The meaning of one Damiao drive status code. Returned by drive_state_info().
struct DriveStateInfo {
    uint8_t code;         ///< INFRABOT_DRIVE_*.
    const char *name;     ///< Short name, e.g. "过压".
    const char *meaning;  ///< What happened.
    const char *causes;   ///< Probable causes.
    const char *actions;  ///< What to do.
    bool fault;           ///< The code is a fault.
};

/// Every Damiao status code (INFRABOT_DRIVE_*); unknown codes get a generic entry.
const DriveStateInfo &drive_state_info(uint8_t state);
/// One sentence for a drive fault: "J3 过流：……可能原因：……处理：……". `joint` is 0-based.
std::string describe_drive_fault(uint8_t joint, uint8_t state);

const char *controller_status_text(uint8_t status);   ///< INFRABOT_STATUS_*, in Chinese.
const char *reply_status_text(uint8_t status);        ///< INFRABOT_REPLY_*, the status of a DMAR reply.
const char *invalid_reason_text(uint8_t reason);      ///< INFRABOT_INVALID_*, why a command datagram was rejected.
const char *move_reason_text(uint32_t reason);        ///< INFRABOT_MOVE_*, why a move ended.
const char *command_name(uint8_t command);            ///< INFRABOT_CMD_*, e.g. "SET_ARM_MODE".
const char *stop_phase_text(StopPhase phase);         ///< Phase of a controlled stop.
/// Mode for display: "RT Joint", "RT 笛卡尔", "movej（NRT Joint）", ...
const char *mode_text(Mode mode);
/// The latched controlled stop, e.g. "通信中断（实时流或心跳 100 ms 未到）", with joint and detail filled in.
std::string describe_stop(const SafetyState &safety);
/// The self-check result with what to do; "" when it passed or did not run.
std::string describe_selfcheck(const SafetyState &safety);
/// The DMAR value of a failed SET_ARM_MODE (INFRABOT_SELFCHECK_REPLY_VALUE).
std::string describe_selfcheck_reply(uint32_t value);

/// Register name; "RID 57" for an unknown one.
std::string register_name(uint8_t rid);
/// The raw value as the register's type: "54", "0.0038" or "0x11".
std::string register_value_text(uint8_t rid, uint32_t raw);
/// One-line summary, e.g. "KP_APR 位置环 Kp | f32 可写 [0, ∞)"; "RID 57：未知寄存器".
std::string register_summary(uint8_t rid);

} // namespace infrabot
