/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
/* The SDK against the loopback controller (tests/fake_mcu.hpp): the servej
 * v_des fix, groups, modes, the NRT session and its keepalive, controlled
 * stop, heartbeat, telemetry v3, faults, the parameter self-check and the
 * recorder.  Offline checks first: without a controller every call fails at
 * once instead of blocking. */
#include "infrabot/diagnostics.hpp"
#include "infrabot/robot.hpp"
#include "fake_mcu.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <iterator>
#include <string>
#include <thread>

using fake::FakeMcu;
using infrabot::Mode;
using infrabot::MotorGroup;
using infrabot::MotorProfile;
using infrabot::Robot;

static void wait_for(const std::function<bool()> &predicate, const char *message, int timeout_ms = 3000) {
    const auto end = fake::now_us() + uint64_t(timeout_ms) * 1000;
    while (!predicate() && fake::now_us() < end) std::this_thread::sleep_for(std::chrono::milliseconds(1));
    if (!predicate()) { std::cerr << "Timeout: " << message << std::endl; std::abort(); }
}
static void pause_ms(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }
static bool near(float a, float b, float tolerance = 1e-4f) { return std::fabs(a - b) <= tolerance; }

static void offline() {
    Robot robot;
    const auto info = robot.info();
    assert(info.axes == 7 && info.motor_count == 8 && !info.received);
    std::string error;
    infrabot::CartesianCommand pose;
    assert(!robot.servep(pose, error));
    assert(!robot.movep(pose.pose, {}, error).message.empty());
    assert(!robot.servo_on(MotorGroup::Arm, error));
    assert(!robot.servo_off(MotorGroup::Gripper, error));
    assert(!robot.gripper(0.5f, 1.0f, 0.5f, error));
    assert(!robot.enable(0, error));
    assert(!robot.stop(0, error));
    assert(!robot.ensure_mode(0, 99, error));
    assert(!robot.send_mit(0, 0.f, 0.f, 1.f, 1.f, 0.f, error));
    assert(!robot.set_can_timeout_ms(0, 100, error));
    uint32_t value = 0;
    // Registers are checked against protocol.h before anything is sent.
    assert(!robot.get_register(0, 57, INFRABOT_REGISTER_F32, value, error) && error.find("未知寄存器") != std::string::npos);
    assert(!robot.get_register(0, INFRABOT_RID_KP_APR, INFRABOT_REGISTER_U32, value, error));
    assert(!robot.write_register(0, INFRABOT_RID_MST_ID, 0x11, INFRABOT_REGISTER_U32, error) &&
           error.find("不允许写入") != std::string::npos);
    float uv = 5.0f; uint32_t raw; std::memcpy(&raw, &uv, 4);
    assert(!robot.write_register(0, 0, raw, INFRABOT_REGISTER_F32, error) && error.find("超出允许范围") != std::string::npos);
    assert(std::string(infrabot::mode_name(Mode::RtJoint)) == "RT_JOINT");
}

static void test_info_handshake() {
    FakeMcu board;
    Robot robot("127.0.0.2", board.command_port, board.telemetry_port);
    std::string error;
    board.protocol_minor = 17;
    assert(robot.connect(error));
    auto info = robot.info();
    assert(info.protocol_version == 2 && info.protocol_major == 1 && info.protocol_minor == 17);
    assert(info.firmware == "fixture-123" && info.board == "test-h743" && info.arm_model == "H7C bare v1");
    robot.disconnect();

    board.v1_only = true;  // ignores V2 DMIF just like the original MCU
    assert(robot.connect(error));
    info = robot.info();
    assert(info.received && info.protocol_version == 1 && info.protocol_major == 1 && info.protocol_minor == 0);
    assert(info.firmware == "unknown" && info.board.empty() && info.arm_model.empty());
    robot.disconnect();

    board.v1_only = false;
    board.protocol_major = 2;
    const auto heartbeats = board.heartbeat_count.load();
    assert(!robot.connect(error) && !robot.connected() && !robot.info().received);
    assert(error.find("2.17") != std::string::npos && error.find("1.1") != std::string::npos);
    assert(board.heartbeat_count == heartbeats);
    board.protocol_major = 1;
    board.stale_info = true;
    assert(!robot.connect(error) && !robot.connected());
    board.stale_info = false;
    board.corrupt_info = true;
    assert(!robot.connect(error) && !robot.connected());
    board.corrupt_info = false;
    assert(robot.connect(error) && robot.info().protocol_minor == 17);
}

static void test_parameters() {
    FakeMcu board;
    Robot robot("127.0.0.2", board.command_port, board.telemetry_port);
    std::string error;
    std::vector<infrabot::ParameterInfo> list;
    infrabot::ParameterInfo p;
    const auto npos = std::string::npos;
    // Not connected: refused before anything is sent.
    assert(!robot.list_parameters(list, error) && !error.empty());
    assert(board.param_requests == 0);
    assert(robot.connect(error));
    assert(robot.list_parameters(list, error));
    assert(list.size() == 4 && list[0].name == "gravity.damping" && list[0].unit == "Nms/rad");
    assert(list[1].writable && !list[1].boolean && list[2].boolean && !list[3].writable);
    assert(near(list[1].minimum, .5f) && near(list[1].maximum, 30.f) && near(list[1].default_value, 8.f) && near(list[1].rate, 20.f));
    // By name and by id; a write shows the requested value while the applied value has not moved yet.
    assert(robot.get_parameter("imp.joint_k", p, error) && p.id == list[1].id && near(p.requested, 8.f));
    assert(robot.set_parameter("imp.joint_k", 12.f, p, error) && near(p.requested, 12.f) && near(p.applied, 8.f));
    assert(near(board.param_requested(list[1].id), 12.f));
    assert(robot.set_parameter(list[2].id, 0.f, p, error) && near(p.requested, 0.f) && near(p.applied, 0.f));
    // Refusals leave the stored value alone.
    assert(!robot.set_parameter("imp.joint_k", 31.f, p, error) && error.find("超出范围") != npos);
    assert(!robot.set_parameter("imp.joint_k", std::nanf(""), p, error) && error.find("超出范围") != npos);
    assert(near(board.param_requested(list[1].id), 12.f));
    assert(!robot.set_parameter("adm.fixed", 2.f, p, error) && error.find("只读") != npos);
    assert(!robot.set_parameter("adm.bias_learning", .5f, p, error) && error.find("超出范围") != npos);
    assert(!robot.get_parameter("nope", p, error) && error.find("nope") != npos);
    assert(!robot.get_parameter(uint16_t(0x0909), p, error) && error.find("未知参数") != npos);
    // A PC that does not control the arm may read but not write.
    board.param_not_owner = true;
    assert(!robot.set_parameter("imp.joint_k", 10.f, p, error) && error.find("另一台 PC") != npos);
    assert(robot.get_parameter("imp.joint_k", p, error) && near(p.requested, 12.f));
    board.param_not_owner = false;
    // Lost datagrams are sent again; a silent board ends in a timeout.
    board.param_drop_replies = 2;
    assert(robot.get_parameter("gravity.damping", p, error) && board.param_drop_replies == 0);
    board.param_drop_replies = 100;
    assert(!robot.get_parameter("gravity.damping", p, error) && error.find("超时") != npos);
    board.param_drop_replies = 0;
    // Firmware without the service: refused at once, nothing is sent, and the name list is not reused.
    robot.disconnect();
    board.supports_parameters = false;
    const unsigned before = board.param_requests;
    assert(robot.connect(error));
    assert(!robot.list_parameters(list, error) && error.find("INFRABOT_CAP_PARAMETERS") != npos);
    assert(!robot.set_parameter("imp.joint_k", 9.f, p, error) && error.find("INFRABOT_CAP_PARAMETERS") != npos);
    assert(board.param_requests == before);
}

int main() {
    offline();
    test_info_handshake();
    test_parameters();
    const auto directory = std::filesystem::temp_directory_path() / ("infrabot_sdk_test_" + std::to_string(getpid()));
    {
        FakeMcu board;
        board.supports_cartesian = true;
        board.hold_offset_mrad = 25;
        Robot robot("127.0.0.2", static_cast<uint16_t>(board.command_port), static_cast<uint16_t>(board.telemetry_port));
        std::string error;
        assert(robot.connect(error));
        assert(robot.info().received && (robot.info().capabilities & INFRABOT_CAP_GROUPS) && board.info_count == 1);
        wait_for([&] { return robot.state().telemetry_age_ms < 50; }, "telemetry");
        assert(robot.state().telemetry_version == 2 && !robot.state().joints.has_command);

        // Recording from the start: every telemetry packet and command.
        infrabot::RecordOptions record;
        record.label = "sdk test";
        assert(robot.start_recording(directory.string(), error, record));
        robot.record_event("开始 \"标记\"");

        assert(!robot.servo_on(MotorGroup::Arm, error) && error.find("电机模式") != std::string::npos);
        assert(robot.set_motor_profile(MotorProfile::Mit, error));
        assert(robot.servo_on(MotorGroup::Arm, error) && board.enabled_mask == 0x7f);
        assert(!robot.servo_on(MotorGroup::Arm, error));

        // IDLE: the SDK keeps the controller alive with a 100 Hz heartbeat.
        const unsigned beats = board.heartbeat_count;
        pause_ms(200);
        assert(board.heartbeat_count >= beats + 10);

        // RT joint: the stream starts at the hold command, and servej sends
        // v_des = the velocity limit, never the desired velocity (0 at rest
        // stopped position-velocity drives).
        assert(robot.switch_mode(Mode::RtJoint, error));
        const auto hold = robot.hold_command();
        for (size_t i = 0; i < 7; ++i) assert(near(hold[i], 0.175f));
        infrabot::JointCommand command;
        for (size_t i = 0; i < 7; ++i) command.position[i] = hold[i];
        command.torque[1] = 3.28f;
        assert(robot.servej(command, error));
        wait_for([&] { return board.legacy_count > 0; }, "servej");
        auto sent = board.last_joint_stream();
        for (size_t i = 0; i < 7; ++i) {
            assert(sent.target_position_mrad[i] == 175 && sent.target_velocity_mrad_s[i] == 0);
            assert(sent.velocity_limit_mrad_s[i] == 3000 && sent.torque_ratio_milli[i] == 1000);
        }
        assert(sent.torque_mNm[1] == 3280 && sent.kp_milli[0] == 300000 && sent.kd_milli[4] == 800);
        const unsigned before_legacy = board.legacy_count;
        std::array<float, 7> position{}, velocity{}, kp{}, kd{};
        position.fill(0.2f); kp.fill(30.f); kd.fill(1.f);
        assert(robot.servej(position, velocity, kp, kd, error));
        wait_for([&] { return board.legacy_count > before_legacy; }, "legacy servej");
        sent = board.last_joint_stream();
        assert(sent.velocity_limit_mrad_s[0] == 3000 && sent.target_velocity_mrad_s[0] == 0 && sent.kp_milli[0] == 30000);
        command.position[0] = NAN;
        assert(!robot.servej(command, error));
        assert(robot.switch_mode(Mode::Idle, error));
        assert(!robot.servej(position, velocity, kp, kd, error));

        // NRT joint: the session holds first (id 1), movej returns at
        // arrival, and the keepalive goes on afterwards.
        assert(robot.switch_mode(Mode::NrtJoint, error));
        infrabot::MoveOptions options;
        std::array<float, 7> target{};
        target.fill(0.3f);
        const auto result = robot.movej(target, options, error);
        if (result.result) std::cerr << "movej: " << error << "\n";
        assert(result.result == 0 && result.motion_id == 2 && board.motion_count == 2);
        assert(near(board.last_request().command.joint.target[3], 0.3f));
        const unsigned keepalives = board.keepalive_count;
        pause_ms(150);
        assert(board.keepalive_count >= keepalives + 8);
        assert(robot.motion().can_execute && board.enabled);

        // Controlled stop: the arm stays enabled, holding in IDLE.
        assert(robot.stop(error));
        assert(board.stop_count >= 1 && board.enabled && board.mode == INFRABOT_CONTROL_IDLE && board.disable_count == 0);
        wait_for([&] { return robot.state().mode == Mode::Idle; }, "IDLE after stop");

        // RT Cartesian: servep streams the pose and its limits in wire units.
        assert(robot.switch_mode(Mode::RtCartesian, error));
        infrabot::CartesianCommand pose;
        std::array<float, 7> joints{};
        joints.fill(0.175f);
        pose.pose = Robot::forward(joints);
        pose.limits.linear_velocity = 0.25f;
        pose.limits.angular_velocity = 1.047f;
        pose.limits.linear_acceleration = 9.0f; // clamps to the maximum
        assert(robot.servep(pose, error));
        wait_for([&] { return board.cartesian_count > 0; }, "servep");
        const std::array<uint16_t, 4> limits{250, 1047, 2000, 0};
        assert(board.last_cartesian_limits() == limits && board.last_cartesian_pose() == pose.pose);
        assert(robot.switch_mode(Mode::Idle, error));

        // NRT Cartesian: the hold movep goes to the forward kinematics of the
        // hold command, then movep moves on.
        assert(robot.switch_mode(Mode::NrtCartesian, error));
        wait_for([&] { return robot.motion().can_execute; }, "Cartesian hold");
        auto first = board.last_request();
        assert(first.op == INFRABOT_MOVE_OP_MOVEP && first.motion_id == 1);
        const auto held = robot.hold_command();
        std::array<float, 7> held_joints{};
        std::copy_n(held.begin(), 7, held_joints.begin());
        assert(near(held_joints[0], 0.325f)); // the movej above moved the arm to 0.3 (+ sag)
        const auto expected = Robot::forward(held_joints);
        for (size_t i = 0; i < 12; ++i) assert(near(first.command.cartesian.pose[i], expected[i]));
        auto goal = expected;
        goal[2] += 0.01f;
        infrabot::CartesianMoveOptions cartesian;
        cartesian.limits.linear_velocity = 0.1f;
        const auto moved = robot.movep(goal, cartesian, error);
        if (moved.result) std::cerr << "movep: " << error << "\n";
        assert(moved.result == 0 && board.last_request().command.cartesian.cartesian_limits[0] == 100);
        // A cancel stops the arm in a controlled way: still enabled.
        assert(robot.cancel_motion(error));
        wait_for([&] { return board.stop_count >= 2; }, "cancel -> controlled stop");
        assert(board.enabled && board.disable_count == 0);
        wait_for([&] { return robot.motion().phase == infrabot::MotionStatus::Phase::Cancelled; }, "cancel confirmed");
        wait_for([&] { return robot.state().mode == Mode::Idle; }, "IDLE after cancel");

        // Gripper group and command.
        assert(robot.servo_on(MotorGroup::Gripper, error) && board.enabled_mask == 0xff);
        assert(robot.gripper(0.6f, 2.0f, 0.4f, error));
        assert(board.gripper_position == 600 && board.gripper_speed == 2000 && board.gripper_force == 400);

        // Telemetry v3: commands, coil temperature, safety, MCU link view.
        board.telemetry_v3 = true;
        board.mcu_command_rx = 987;
        wait_for([&] { return robot.state().telemetry_version == 3; }, "telemetry v3");
        auto state = robot.state();
        assert(state.joints.has_command && near(state.joints.command_torque[3], 0.3f) &&
               state.joints.temperature_coil[0] == 35 && state.safety.valid);
        wait_for([&] { return robot.link_stats().mcu_valid && robot.link_stats().mcu.command_rx == 987; }, "MCU link");
        wait_for([&] { return robot.link_stats().rx_last_second > 500; }, "link window", 2500);

        // A drive fault: logged with its meaning, cleared once disabled.
        board.fault_joint = 2;
        board.fault_state = INFRABOT_DRIVE_OVERCURRENT;
        wait_for([&] {
            for (const auto &fault : robot.fault_log())
                if (fault.kind == infrabot::FaultEvent::Kind::DriveFault && fault.joint == 2) return true;
            return false;
        }, "fault log");
        bool explained = false;
        for (const auto &fault : robot.fault_log())
            if (fault.kind == infrabot::FaultEvent::Kind::DriveFault)
                explained = fault.message.find("J3 过流") != std::string::npos &&
                            fault.message.find("处理：") != std::string::npos;
        assert(explained);
        assert(!robot.clear_fault(error) && error.find("先失能") != std::string::npos);
        assert(robot.servo_off(MotorGroup::All, error));
        assert(robot.clear_fault(error) && board.clear_error_count == 1);
        bool cleared = false;
        for (const auto &fault : robot.fault_log())
            cleared = cleared || fault.kind == infrabot::FaultEvent::Kind::DriveCleared;
        assert(cleared);

        // The parameter self-check refuses a mismatching drive, and says which.
        board.selfcheck_fail_joint = 2;
        board.selfcheck_fail_rid = INFRABOT_RID_VMAX;
        assert(!robot.set_motor_profile(MotorProfile::Mit, error));
        assert(error.find("J3 VMAX 读回 8") != std::string::npos && error.find("期望 10") != std::string::npos);
        assert(!robot.servo_on(MotorGroup::Arm, error));
        board.selfcheck_fail_joint = INFRABOT_NO_JOINT;
        assert(robot.set_motor_profile(MotorProfile::Mit, error));

        // An observer sends no heartbeat.
        robot.set_heartbeat(false);
        pause_ms(30);
        const unsigned quiet = board.heartbeat_count;
        pause_ms(100);
        assert(board.heartbeat_count == quiet);

        robot.stop_recording();
        const auto stats = robot.recording();
        assert(!stats.active && stats.telemetry_rows > 500 && stats.command_rows > 10 && stats.dropped == 0 &&
               stats.error.empty());
        const std::filesystem::path recorded(stats.directory);
        assert(recorded.filename().string().find("sdk_test") != std::string::npos);
        std::ifstream telemetry(recorded / "telemetry_000.csv");
        std::string header, row;
        assert(std::getline(telemetry, header) && std::getline(telemetry, row));
        const auto columns = [](const std::string &line) { return std::count(line.begin(), line.end(), ',') + 1; };
        assert(columns(header) == 98 && columns(row) == 98);
        assert(header.rfind("host_time_s,mcu_time_s,sequence,lost", 0) == 0);
        std::string last_row = row;
        while (std::getline(telemetry, row)) last_row = row;
        assert(columns(last_row) == 98 && last_row.find(",987,") != std::string::npos); // a v3 row
        std::ifstream events(recorded / "events.csv");
        std::string all((std::istreambuf_iterator<char>(events)), std::istreambuf_iterator<char>());
        assert(all.find("\"开始 \"\"标记\"\"\"") != std::string::npos);
        assert(all.find("SET_ARM_MODE") != std::string::npos && all.find("J3 过流") != std::string::npos);
        std::ifstream commands(recorded / "commands_000.csv");
        std::string commands_all((std::istreambuf_iterator<char>(commands)), std::istreambuf_iterator<char>());
        assert(commands_all.find(",v5,") != std::string::npos && commands_all.find(",dmov,") != std::string::npos &&
               commands_all.find(",v7,") != std::string::npos);
    }
    {
        // Resume after a controlled stop: back to the interrupted mode from
        // where the arm stopped, in one call; it waits for the arm to settle
        // and retries a busy MCU.
        FakeMcu board;
        board.telemetry_v3 = true;
        board.hold_offset_mrad = 25;
        Robot robot("127.0.0.2", static_cast<uint16_t>(board.command_port), static_cast<uint16_t>(board.telemetry_port));
        std::string error;
        assert(robot.connect(error));
        wait_for([&] { return robot.state().telemetry_version == 3; }, "resume: telemetry v3");
        assert(!robot.resume(error) && error.find("没有需要恢复") != std::string::npos);
        assert(robot.set_motor_profile(MotorProfile::Mit, error) && robot.servo_on(error));
        assert(robot.switch_mode(Mode::RtJoint, error));
        assert(robot.stop(error));
        auto s = robot.state();
        assert(s.mode == Mode::Idle && s.safety.stop_reason == infrabot::StopReason::Requested &&
               s.safety.stop_mode == Mode::RtJoint);
        // A joint still moving (a position-velocity drive trailing its stopped
        // command): resume waits for it, the MCU would refuse busy.
        board.arm_velocity_mrad_s = 200;
        std::thread settle([&] { pause_ms(300); board.arm_velocity_mrad_s = 0; });
        const auto t0 = fake::now_us();
        Mode resumed = Mode::Unknown;
        const bool ok = robot.resume(error, &resumed);
        settle.join();
        if (!ok) std::cerr << "resume: " << error << "\n";
        assert(ok && resumed == Mode::RtJoint && fake::now_us() - t0 >= 280000);
        assert(board.mode == INFRABOT_CONTROL_RT_JOINT);
        wait_for([&] { return robot.state().safety.stop_reason == infrabot::StopReason::None; }, "stop acknowledged");
        for (size_t i = 0; i < 7; ++i) assert(near(robot.hold_command()[i], 0.175f)); // where it stopped
        // The MCU's newest sample still moving: busy, retried.
        assert(robot.stop(error));
        board.busy_replies = 2;
        const unsigned requests = board.mode_requests;
        assert(robot.resume(error, &resumed) && resumed == Mode::RtJoint && board.mode_requests >= requests + 3);
        // NRT Joint: a new session holding where it stopped, then moves again.
        assert(robot.switch_mode(Mode::Idle, error) && robot.switch_mode(Mode::NrtJoint, error));
        wait_for([&] { return robot.motion().can_execute; }, "resume: NRT session");
        assert(robot.cancel_motion(error));
        wait_for([&] { const auto st = robot.state();
                       return st.mode == Mode::Idle && st.safety.stop_reason != infrabot::StopReason::None; },
                 "resume: NRT stopped");
        assert(robot.state().safety.stop_mode == Mode::NrtJoint);
        const unsigned moves = board.motion_count;
        assert(robot.resume(error, &resumed) && resumed == Mode::NrtJoint);
        wait_for([&] { return robot.motion().can_execute; }, "resume: new session holds");
        assert(board.motion_count == moves + 1 && robot.motion().id == 1 && board.mode == INFRABOT_CONTROL_NRT_JOINT);
        std::array<float, 7> target{};
        target.fill(0.2f);
        assert(robot.movej(target, infrabot::MoveOptions{}, error).result == 0);
        // A stop in IDLE: nothing to re-enter, resume acknowledges it.
        assert(robot.switch_mode(Mode::Idle, error));
        board.stop_mode = INFRABOT_CONTROL_IDLE;
        board.stop_reason = INFRABOT_STOP_TORQUE_LIMIT;
        wait_for([&] { return robot.state().safety.stop_reason == infrabot::StopReason::TorqueLimit; }, "IDLE stop");
        assert(robot.resume(error, &resumed) && resumed == Mode::Idle);
        assert(board.stop_reason == INFRABOT_STOP_NONE && board.mode == INFRABOT_CONTROL_IDLE);
        // A faulted arm drive cannot be resumed: it says what to do.
        board.fault_joint = 3;
        board.fault_state = INFRABOT_DRIVE_OVERCURRENT;
        board.stop_mode = INFRABOT_CONTROL_RT_JOINT;
        board.stop_reason = INFRABOT_STOP_DRIVE_FAULT;
        wait_for([&] { const auto st = robot.state();
                       return st.joints.drive_state[3] == INFRABOT_DRIVE_OVERCURRENT &&
                              st.safety.stop_reason == infrabot::StopReason::DriveFault; }, "faulted stop");
        assert(!robot.resume(error) && error.find("J4 过流") != std::string::npos &&
               error.find("清除故障") != std::string::npos && board.mode == INFRABOT_CONTROL_IDLE);
    }
    {
        // Firmware without the resume capability: resume() says so.
        FakeMcu board;
        board.telemetry_v3 = true;
        board.resume_supported = false;
        Robot robot("127.0.0.2", static_cast<uint16_t>(board.command_port), static_cast<uint16_t>(board.telemetry_port));
        std::string error;
        assert(robot.connect(error));
        wait_for([&] { return robot.state().telemetry_version == 3; }, "no resume: telemetry");
        assert(robot.set_motor_profile(MotorProfile::Mit, error) && robot.servo_on(error));
        assert(robot.switch_mode(Mode::RtJoint, error) && robot.stop(error));
        assert(!robot.resume(error) && error.find("不支持恢复") != std::string::npos);
    }
    {
        // Older firmware: no controlled stop, so stop() refuses rather than
        // pretending; the arm stays as it was.
        FakeMcu board;
        board.controlled_stop = false;
        Robot robot("127.0.0.2", static_cast<uint16_t>(board.command_port), static_cast<uint16_t>(board.telemetry_port));
        std::string error;
        assert(robot.connect(error));
        wait_for([&] { return robot.state().telemetry_age_ms < 50; }, "old firmware telemetry");
        assert(robot.set_motor_profile(MotorProfile::PositionVelocity, error));
        assert(robot.servo_on(error) && board.enabled_mask == 0xff);
        assert(!robot.stop(error) && error.find("不支持受控停止") != std::string::npos && board.enabled);
        assert(robot.servo_off(error) && board.enabled_mask == 0);
    }
    std::filesystem::remove_all(directory);
    std::cout << "SDK loopback: servej v_des, groups, modes, NRT keepalive, controlled stop, heartbeat, "
                 "telemetry v3, faults, self-check, recorder, resume passed\n";
    return 0;
}
