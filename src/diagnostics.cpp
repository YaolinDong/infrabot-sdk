/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "infrabot/diagnostics.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace infrabot {
namespace {

const DriveStateInfo kDriveStates[] = {
    {INFRABOT_DRIVE_DISABLED, "失能", "驱动器未输出力矩。", "", "", false},
    {INFRABOT_DRIVE_ENABLED, "使能", "驱动器正常输出。", "", "", false},
    {INFRABOT_DRIVE_OVERVOLTAGE, "过压",
     "母线电压超过过压保护值（RID 29 OV_Value），驱动器已停止输出。",
     "电源电压过高；急减速或被外力反拖时电机回馈的能量抬高了母线电压，而电源不能吸收。",
     "检查电源电压；降低减速度和速度；电源侧加能量吸收（制动电阻、电池或可回馈电源）；清除故障后重新使能。", true},
    {INFRABOT_DRIVE_UNDERVOLTAGE, "欠压",
     "母线电压低于低压保护值（RID 0 UV_Value），驱动器已停止输出。",
     "电源电压不足或功率不够；线缆过细、过长或接头接触不良造成压降；多个关节同时急加速时电压跌落。",
     "检查电源电压和电流能力、线缆和接头；降低加速度；清除故障后重新使能。", true},
    {INFRABOT_DRIVE_OVERCURRENT, "过流",
     "相电流超过过流保护值（RID 3 OC_Value），驱动器已停止输出。",
     "增益过高引起振荡；堵转、碰撞或机械卡死；电机相线短路或接触不良。",
     "降低 Kp/Kd 或加速度；检查关节是否被卡住、有无碰撞；检查电机线；清除故障后重新使能。", true},
    {INFRABOT_DRIVE_MOS_OVERTEMP, "MOS 过温",
     "驱动板 MOS 温度超过过温保护值（RID 2 OT_Value），驱动器已停止输出。",
     "长时间大电流（重载保持、频繁加减速）；散热差或环境温度高。",
     "停机冷却；减轻负载或降低持续力矩（例如加重力补偿）；改善散热。", true},
    {INFRABOT_DRIVE_COIL_OVERTEMP, "线圈过温",
     "电机绕组温度过高，驱动器已停止输出。",
     "持续大力矩：负载重、姿态不利时长时间保持、被卡住。",
     "停机冷却；换到力矩更小的姿态保持，或降低持续力矩；检查机械是否卡滞。", true},
    {INFRABOT_DRIVE_COMM_LOST, "通信丢失",
     "驱动器在 CAN 超时时间（RID 9 TIMEOUT）内没有收到指令，自行失能。",
     "MCU 停发指令；CAN 线松动、终端电阻缺失或受干扰；CAN 超时设得过短。",
     "检查 CAN 线、接头和终端电阻，查看 MCU 串口日志；必要时调大或关闭 CAN 超时；清除故障后重新使能。", true},
    {INFRABOT_DRIVE_OVERLOAD, "过载",
     "电机长时间超过额定负载，驱动器已停止输出。",
     "负载过重、长时间大力矩保持、关节被卡住。",
     "减轻负载，检查机械；降低持续力矩；冷却后清除故障再使能。", true},
};
const DriveStateInfo kUnknownState = {0xFF, "未知", "驱动器报告了未定义的状态码。",
    "驱动器固件版本不同，或反馈帧被误解析。", "查看串口日志中的 MOTOR 行，确认驱动器固件版本。", true};

std::string joint_label(uint8_t joint) {
    return joint < INFRABOT_MOTOR_COUNT ? "J" + std::to_string(joint + 1) : std::string("机械臂");
}

std::string hex(uint32_t value) {
    char text[16];
    std::snprintf(text, sizeof(text), "0x%X", static_cast<unsigned>(value));
    return text;
}

} // namespace

const DriveStateInfo &drive_state_info(uint8_t state) {
    for (const auto &info : kDriveStates)
        if (info.code == state) return info;
    return kUnknownState;
}

std::string describe_drive_fault(uint8_t joint, uint8_t state) {
    const auto &info = drive_state_info(state);
    std::string text = joint_label(joint) + " " + info.name + "：" + info.meaning;
    if (*info.causes) text += "可能原因：" + std::string(info.causes);
    if (*info.actions) text += "处理：" + std::string(info.actions);
    return text;
}

const char *controller_status_text(uint8_t status) {
    static const char *const names[] = {"启动中", "预检", "需先应用电机模式", "已失能", "使能中",
        "IDLE 保持", "实时关节", "实时笛卡尔", "关节 move", "笛卡尔 move", "受控停止中", "故障",
        "恢复中", "急停", "未知"};
    return status < sizeof(names) / sizeof(names[0]) ? names[status] : "未知";
}

const char *reply_status_text(uint8_t status) {
    static const char *const texts[] = {"已确认", "已提交 CAN（未确认物理执行）",
        "CAN 失败/等待电机超时", "参数错误", "忙/重复请求/非控制者", "请先全部失能",
        "模式不匹配或回读不符", "未使能/反馈过期/电机故障", "只读、保护寄存器或数值越界",
        "固件不支持"};
    return status < sizeof(texts) / sizeof(texts[0]) ? texts[status] : "未知状态";
}

const char *invalid_reason_text(uint8_t reason) {
    static const char *const texts[] = {"无", "报文格式/CRC 错误", "实时关节流不在 RT_JOINT",
        "当前状态不允许使能/设零", "move 占用机械臂", "未知命令码", "笛卡尔流不在 RT_CARTESIAN",
        "笛卡尔速度上限超出最大值", "维护参数无效", "该报文不支持定时执行"};
    return reason < sizeof(texts) / sizeof(texts[0]) ? texts[reason] : "未知原因";
}

const char *move_reason_text(uint32_t reason) {
    static const char *const texts[] = {"正常", "忙（上一个动作未完成）", "未就绪", "参数无效",
        "不支持", "会话不匹配", "动作 ID 过期", "通信超时（保活中断）", "运动超时",
        "反馈过期", "电机故障", "已取消", "CAN 发送失败"};
    return reason < sizeof(texts) / sizeof(texts[0]) ? texts[reason] : "未知原因";
}

const char *command_name(uint8_t command) {
    static const char *const names[] = {"STREAM", "ENABLE", "DISABLE", "SET_ZERO", "SET_ZERO_ENABLE",
        "STOP", "CLEAR_ERROR", "REQUEST_FEEDBACK", "ENSURE_MODE", "SEND_MIT", "SEND_POS_VEL",
        "SEND_VEL", "SEND_FORCE_POS", "SET_CAN_TIMEOUT", "STORE_PARAMETERS", "GET_REGISTER",
        "WRITE_REGISTER", "GET_STATE", "SET_ARM_MODE", "STORE_CONTROL_PROFILE", "SET_CONTROL_MODE",
        "HOLD_HEARTBEAT", "GET_HOLD_POSITION", "ENABLE_ARM", "ENABLE_GRIPPER", "DISABLE_ARM",
        "DISABLE_GRIPPER", "GRIPPER"};
    return command < sizeof(names) / sizeof(names[0]) ? names[command] : "UNKNOWN";
}

const char *stop_phase_text(StopPhase phase) {
    switch (phase) {
    case StopPhase::None: return "";
    case StopPhase::Braking: return "刹车中";
    case StopPhase::Holding: return "已停止，保持使能，等待 PC";
    }
    return "";
}

const char *mode_text(Mode mode) {
    switch (mode) {
    case Mode::Idle: return "IDLE";
    case Mode::RtJoint: return "RT Joint";
    case Mode::RtCartesian: return "RT 笛卡尔";
    case Mode::NrtJoint: return "movej（NRT Joint）";
    case Mode::NrtCartesian: return "movep（NRT 笛卡尔）";
    case Mode::GravityFree:return "GRAVITY FREE";
    case Mode::GravityLock:return "GRAVITY LOCK";
    case Mode::JointImpedance:return "JOINT IMP";
    case Mode::CartesianImpedance:return "CART IMP";
    case Mode::JointAdmittance:return "JOINT ADM";
    case Mode::CartesianAdmittance:return "CART ADM";
    default: return "未知模式";
    }
}

std::string describe_stop(const SafetyState &safety) {
    if (!safety.valid || safety.stop_reason == StopReason::None) return {};
    const std::string joint = joint_label(safety.stop_joint);
    std::string text;
    switch (safety.stop_reason) {
    case StopReason::None: break;
    case StopReason::StreamLost: text = "通信中断：实时指令流或 IDLE 心跳 100 ms 未到"; break;
    case StopReason::MoveLost: text = "movej/movep 保活中断"; break;
    case StopReason::MoveFailed:
        text = std::string("movej/movep 失败：") + move_reason_text(safety.stop_detail); break;
    case StopReason::Requested: text = "PC 请求停止"; break;
    case StopReason::NotFollowing: text = joint + " 跟不上指令（可能被卡住或发生碰撞）"; break;
    case StopReason::InvalidTarget:
        text = "笛卡尔目标被 IK 拒绝（数据无效）";
        if (safety.stop_detail) text += "，IK 状态 " + std::to_string(safety.stop_detail);
        break;
    case StopReason::FeedbackLost: text = joint + " 反馈丢失（检查该电机的电源和 CAN）"; break;
    case StopReason::DriveFault:
        text = safety.stop_joint < INFRABOT_MOTOR_COUNT ?
            joint + " 驱动器故障：" + drive_state_info(safety.stop_detail).name :
            std::string("CAN 控制器故障"); break;
    case StopReason::TorqueLimit: text = joint + " 力矩持续超限（碰撞或负载过重）"; break;
    }
    if (safety.stop_mode != Mode::Idle) text += std::string("（停止前：") + mode_text(safety.stop_mode) + "）";
    const char *phase = stop_phase_text(safety.stop_phase);
    if (*phase) text += std::string("；") + phase;
    return text;
}

std::string describe_selfcheck(const SafetyState &safety) {
    if (!safety.valid || safety.selfcheck == SelfCheck::NotRun || safety.selfcheck == SelfCheck::Ok)
        return {};
    const std::string joint = joint_label(safety.selfcheck_joint);
    const std::string rid = register_name(safety.selfcheck_register);
    if (safety.selfcheck == SelfCheck::NoReply)
        return "参数自检失败：" + joint + " 没有回复 " + rid + "。检查该电机的电源和 CAN 连接后重新应用电机模式。";
    std::string text = "参数自检失败：" + joint + " " + rid + " 读回 " +
        register_value_text(safety.selfcheck_register, safety.selfcheck_value) + "，固件期望 " +
        register_value_text(safety.selfcheck_register, safety.selfcheck_expected) + "。";
    if (safety.selfcheck_register == INFRABOT_RID_MST_ID || safety.selfcheck_register == INFRABOT_RID_ESC_ID)
        text += "电机 ID 不对（可能装错了位置），用调试工具改 ID、重新上电后再应用电机模式。";
    else if (safety.selfcheck_register == INFRABOT_RID_CTRL_MODE)
        text += "控制模式没有写进去，重新应用电机模式；仍失败时检查该电机。";
    else
        text += "电机映射范围与固件不一致：MIT 指令和遥测会被按比例缩放。用电机参数模板（infrabot.tsv）"
                "重写该电机参数，或修正固件常量表，然后重新应用电机模式。";
    return text;
}

std::string describe_selfcheck_reply(uint32_t value) {
    SafetyState safety;
    safety.valid = true;
    safety.selfcheck_joint = static_cast<uint8_t>(value & 0xFFU);
    safety.selfcheck_register = static_cast<uint8_t>((value >> 8) & 0xFFU);
    const uint8_t result = static_cast<uint8_t>((value >> 16) & 0xFFU);
    safety.selfcheck = result == INFRABOT_SELFCHECK_NO_REPLY ? SelfCheck::NoReply : SelfCheck::Mismatch;
    if (safety.selfcheck == SelfCheck::NoReply) return describe_selfcheck(safety);
    return "参数自检失败：" + joint_label(safety.selfcheck_joint) + " " +
           register_name(safety.selfcheck_register) + " 与固件常量表不一致。";
}

std::string register_name(uint8_t rid) {
    const auto *info = infrabot_register_info(rid);
    return info ? std::string(info->name) : "RID " + std::to_string(rid);
}

std::string register_value_text(uint8_t rid, uint32_t raw) {
    const auto *info = infrabot_register_info(rid);
    if (info && info->type == INFRABOT_REGISTER_F32) {
        float value;
        std::memcpy(&value, &raw, sizeof(value));
        char text[32];
        std::snprintf(text, sizeof(text), "%.4g", static_cast<double>(value));
        return text;
    }
    if (rid == INFRABOT_RID_MST_ID || rid == INFRABOT_RID_ESC_ID) return hex(raw);
    return std::to_string(raw);
}

std::string register_summary(uint8_t rid) {
    const auto *info = infrabot_register_info(rid);
    if (!info) return "RID " + std::to_string(rid) + "：未知寄存器";
    std::string text = std::string(info->name) + " " + info->description + " | " +
                       (info->type == INFRABOT_REGISTER_F32 ? "f32" : "u32");
    if (info->access == INFRABOT_REGISTER_READ_ONLY) return text + " 只读";
    if (info->access == INFRABOT_REGISTER_PROTECTED) return text + " 受保护（MCU 拒绝在线写入）";
    const auto bound = [&](float value) -> std::string {
        if (info->type == INFRABOT_REGISTER_U32) // exact integers, the whole u32 range at most
            return std::to_string(static_cast<unsigned long>(std::min(static_cast<double>(value), 4294967295.0)));
        if (value >= INFRABOT_FLOAT_MAX) return "∞";
        if (value <= -INFRABOT_FLOAT_MAX) return "-∞";
        char number[32];
        std::snprintf(number, sizeof(number), "%g", static_cast<double>(value));
        return number;
    };
    return text + " 可写 " + ((info->bounds & INFRABOT_REGISTER_LOW_OPEN) ? "(" : "[") + bound(info->low) + ", " +
           bound(info->high) + ((info->bounds & INFRABOT_REGISTER_HIGH_OPEN) ? ")" : "]");
}

} // namespace infrabot
