/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#pragma once

/* Human-readable (Chinese) meaning of the codes in infrabot/protocol.h: drive
 * faults with their probable causes and what to do (Flexiv RobotEvent
 * style), controlled stops, the parameter self-check, DMAR replies and the
 * drive registers. */

#include "infrabot/protocol.h"
#include "infrabot/robot.hpp"

#include <cstdint>
#include <string>

namespace infrabot {
/* [Non-blocking] Local diagnostic lookups and formatting for supervisory code. */

struct DriveStateInfo {
    uint8_t code;
    const char *name;     // short, e.g. "过压"
    const char *meaning;  // what happened
    const char *causes;   // probable causes
    const char *actions;  // what to do
    bool fault;
};

/* Every Damiao status code (INFRABOT_DRIVE_*); unknown codes get a generic entry. */
const DriveStateInfo &drive_state_info(uint8_t state);
/* "J3 过流：……可能原因：……处理：……" */
std::string describe_drive_fault(uint8_t joint, uint8_t state);

const char *controller_status_text(uint8_t status);   // INFRABOT_STATUS_*, Chinese
const char *reply_status_text(uint8_t status);        // INFRABOT_REPLY_*
const char *invalid_reason_text(uint8_t reason);      // INFRABOT_INVALID_*
const char *move_reason_text(uint32_t reason);        // INFRABOT_MOVE_*
const char *command_name(uint8_t command);            // INFRABOT_CMD_*, e.g. "SET_ARM_MODE"
const char *stop_phase_text(StopPhase phase);
/* "RT Joint", "RT 笛卡尔", "movej（NRT Joint）", ... */
const char *mode_text(Mode mode);
/* "通信中断（实时流或心跳 100 ms 未到）"; joint/detail filled in. */
std::string describe_stop(const SafetyState &safety);
/* The self-check result with what to do, "" when it passed or did not run. */
std::string describe_selfcheck(const SafetyState &safety);
/* The DMAR value of a failed SET_ARM_MODE (INFRABOT_SELFCHECK_REPLY_VALUE). */
std::string describe_selfcheck_reply(uint32_t value);

/* Register name, "RID 57" for an unknown one. */
std::string register_name(uint8_t rid);
/* The raw value as the register's type: "54" / "0.0038" / "0x11". */
std::string register_value_text(uint8_t rid, uint32_t raw);
/* "KP_APR 位置环 Kp | f32 可写 [0, ∞)"; "RID 57：未知寄存器". */
std::string register_summary(uint8_t rid);

} // namespace infrabot
