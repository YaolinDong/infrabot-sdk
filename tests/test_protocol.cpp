/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
/* Validate the C++ wire layout and SDK status names. --golden outputs
 * reference DMOV/DMOS datagrams as hexadecimal bytes. */
#include "infrabot/diagnostics.hpp"
#include "infrabot/protocol.h"
#include "motion_session.hpp"

#include <cassert>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>

static void print_hex(const char *name, const void *data, size_t size) {
    std::printf("%s ", name);
    const auto *bytes = static_cast<const uint8_t *>(data);
    for (size_t i = 0; i < size; ++i) std::printf("%02x", bytes[i]);
    std::printf("\n");
}

static InfrabotMoveRequest golden_request() {
    InfrabotMoveRequest request = motion::blank_request();
    request.op = INFRABOT_MOVE_OP_MOVEJ;
    request.session = 0x1122334455667788ULL;
    request.motion_id = 7;
    auto &joint = request.command.joint;
    joint.timeout_ms = 30000; joint.dwell_ms = 100;
    joint.position_tolerance = 0.01f; joint.velocity_tolerance = 0.05f;
    for (int i = 0; i < 7; ++i) {
        joint.target[i] = 0.125f * static_cast<float>(i + 1) - 0.5f;
        joint.velocity_max[i] = 0.25f; joint.acceleration_max[i] = 0.5f;
        joint.kp[i] = 30.0f + static_cast<float>(i); joint.kd[i] = 0.75f; joint.torque[i] = -1.5f;
    }
    request.crc32 = infrabot_crc32(&request, sizeof(request) - 4);
    return request;
}

static InfrabotMoveReply golden_reply() {
    InfrabotMoveReply reply{};
    reply.magic = INFRABOT_MAGIC_MOVE_REPLY; reply.version = INFRABOT_MOVE_V1;
    reply.result = 0; reply.state = INFRABOT_MOVE_STATE_SETTLING;
    reply.session = 0x1122334455667788ULL; reply.request_id = 7; reply.active_id = 7;
    reply.elapsed_ms = 1234; reply.duration_ms = 2000; reply.arrived_mask = 0x3f;
    reply.enabled_mask = 0x7f; reply.fresh_mask = 0xff; reply.max_error_rad = 0.0625f;
    reply.reason = INFRABOT_MOVE_OK;
    reply.crc32 = infrabot_crc32(&reply, sizeof(reply) - 4);
    return reply;
}

int main(int argc, char **argv) {
    if (argc > 1 && std::string(argv[1]) == "--golden") {
        const auto request = golden_request();
        const auto reply = golden_reply();
        print_hex("request", &request, sizeof(request));
        print_hex("reply", &reply, sizeof(reply));
        return 0;
    }
    static_assert(sizeof(InfrabotTelemetryV3) == 348, "DMTL v3");
    assert(infrabot_crc32("123456789", 9) == 0xCBF43926U);
    // motion::Parameters is the wire payload.
    motion::Parameters p;
    p.target[6] = 1.5f;
    InfrabotMoveJoint wire;
    std::memcpy(&wire, &p, sizeof(wire));
    assert(wire.timeout_ms == 30000 && wire.target[6] == 1.5f);
    // Names and descriptions.
    assert(std::string(infrabot::command_name(INFRABOT_CMD_GRIPPER)) == "GRIPPER");
    assert(std::string(infrabot::command_name(INFRABOT_CMD_SET_ARM_MODE)) == "SET_ARM_MODE");
    assert(infrabot::drive_state_info(INFRABOT_DRIVE_OVERCURRENT).fault &&
           !infrabot::drive_state_info(INFRABOT_DRIVE_ENABLED).fault);
    for (uint8_t state = INFRABOT_DRIVE_OVERVOLTAGE; state <= INFRABOT_DRIVE_OVERLOAD; ++state) {
        const auto text = infrabot::describe_drive_fault(4, state);
        assert(text.rfind("J5 ", 0) == 0 && text.find("可能原因：") != std::string::npos &&
               text.find("处理：") != std::string::npos);
    }
    assert(std::string(infrabot::drive_state_info(0x5).name) == "未知");
    infrabot::SafetyState safety;
    safety.valid = true;
    safety.stop_reason = infrabot::StopReason::TorqueLimit;
    safety.stop_phase = infrabot::StopPhase::Holding;
    safety.stop_joint = 1;
    const auto stop = infrabot::describe_stop(safety);
    // Held without a PC: enabled, no countdown to a disable.
    assert(stop.find("J2 力矩持续超限") != std::string::npos && stop.find("保持使能，等待 PC") != std::string::npos &&
           stop.find("失能") == std::string::npos);
    safety.stop_reason = infrabot::StopReason::StreamLost;
    safety.stop_phase = infrabot::StopPhase::Braking;
    safety.stop_mode = infrabot::Mode::RtJoint;
    assert(infrabot::describe_stop(safety).find("（停止前：RT Joint）；刹车中") != std::string::npos);
    safety.stop_mode = infrabot::Mode::Idle;
    assert(infrabot::describe_stop(safety).find("通信中断") != std::string::npos &&
           infrabot::describe_stop(safety).find("刹车中") != std::string::npos);
    safety.stop_reason = infrabot::StopReason::DriveFault;
    safety.stop_joint = INFRABOT_NO_JOINT;
    assert(infrabot::describe_stop(safety).find("CAN 控制器故障") != std::string::npos);
    safety.stop_phase = infrabot::StopPhase::Holding;
    safety.stop_reason = infrabot::StopReason::MoveFailed;
    safety.stop_detail = INFRABOT_MOVE_MOTION_TIMEOUT;
    assert(infrabot::describe_stop(safety).find("运动超时") != std::string::npos);
    safety.selfcheck = infrabot::SelfCheck::Mismatch;
    safety.selfcheck_joint = 2;
    safety.selfcheck_register = INFRABOT_RID_VMAX;
    const float read = 8.0f, expected = 10.0f;
    std::memcpy(&safety.selfcheck_value, &read, 4);
    std::memcpy(&safety.selfcheck_expected, &expected, 4);
    assert(infrabot::describe_selfcheck(safety).find("J3 VMAX 读回 8，固件期望 10") != std::string::npos);
    safety.selfcheck_register = INFRABOT_RID_MST_ID;
    safety.selfcheck_value = 0x14; safety.selfcheck_expected = 0x13;
    assert(infrabot::describe_selfcheck(safety).find("MST_ID 读回 0x14，固件期望 0x13") != std::string::npos);
    assert(infrabot::describe_selfcheck_reply(INFRABOT_SELFCHECK_REPLY_VALUE(7, INFRABOT_RID_PMAX, INFRABOT_SELFCHECK_NO_REPLY))
               .find("J8 没有回复 PMAX") != std::string::npos);
    assert(infrabot::register_name(57) == "RID 57" && infrabot::register_name(27) == "KP_APR");
    assert(infrabot::register_summary(9).find("u32 可写 [20, 4294967295]") != std::string::npos);
    assert(infrabot::register_summary(27).find("f32 可写 [0, ∞]") != std::string::npos);
    assert(infrabot::register_summary(0).find("可写 (10, ∞]") != std::string::npos);
    assert(infrabot::register_summary(7).find("受保护") != std::string::npos &&
           infrabot::register_summary(80).find("只读") != std::string::npos &&
           infrabot::register_summary(57) == "RID 57：未知寄存器");
    float gain = 54.0f; uint32_t raw; std::memcpy(&raw, &gain, 4);
    assert(infrabot::register_value_text(27, raw) == "54" && infrabot::register_value_text(10, 2) == "2");
    std::cout << "protocol.h (C++): payload layout, names, fault/stop/self-check texts passed\n";
    return 0;
}
