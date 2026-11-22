/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#define UDP_CONTROL_GUI_TEST
#include "../tools/gui/qt_control_gui.cpp"
#include <cassert>
#include <functional>
#include <iostream>
#include <memory>
#include <iterator>

#include "fake_mcu.hpp"

// Only loopback addresses are used. MainWindow(false) disables auto-connect.
using fake::FakeMcu;

static void wait_for(const std::function<bool()> &predicate, const char *message, int timeout_ms = 3000) {
    const auto end = monotonic_us() + timeout_ms * 1000;
    while (!predicate() && monotonic_us() < end) {
        QCoreApplication::processEvents();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    if (!predicate()) { std::cerr << "Timeout: " << message << std::endl; std::abort(); }
}
template<class T> static T *widget(MainWindow &window, const char *name) {
    auto *result = window.findChild<T *>(QString::fromLatin1(name)); assert(result); return result;
}
static void execute_confirmed(QPushButton *button) {
    assert(button->isEnabled());
    QTimer::singleShot(0, [] {
        for (QWidget *w : QApplication::topLevelWidgets())
            if (auto *dialog = qobject_cast<QMessageBox *>(w)) {
                auto *yes = dialog->button(QMessageBox::Yes);
                assert(yes); yes->click();
            }
    });
    button->click();
}

/* Clicks a button whose handler may open a modal message box: closes it and
 * returns its text ("" when none opened). */
static QString click_dismissing(QPushButton *button) {
    QString text;
    QTimer timer;
    QObject::connect(&timer, &QTimer::timeout, [&] {
        for (QWidget *w : QApplication::topLevelWidgets())
            if (auto *box = qobject_cast<QMessageBox *>(w); box && box->isVisible()) {
                text = box->text();
                box->accept();
            }
    });
    timer.start(10);
    button->click();
    timer.stop();
    return text;
}

int main(int argc, char **argv) {
    QApplication app(argc, argv);
    {
        FakeMcu board;board.calibrate_for_test();
        MainWindow window(false);window.show();
        widget<QLineEdit>(window,"mcu_ip")->setText("127.0.0.2");
        widget<QSpinBox>(window,"command_port")->setValue(board.command_port);
        widget<QSpinBox>(window,"telemetry_port")->setValue(board.telemetry_port);
        widget<QPushButton>(window,"connect_button")->click();
        wait_for([&]{return widget<QPushButton>(window,"apply_motor_profile")->isEnabled();},"admittance connect");
        widget<QPushButton>(window,"apply_motor_profile")->click();
        wait_for([&]{return widget<QPushButton>(window,"arm_enable")->isEnabled();},"admittance profile");
        widget<QPushButton>(window,"arm_enable")->click();
        for(auto entry: {std::pair<const char*,uint8_t>{"enter_joint_admittance",9},{"enter_cartesian_admittance",10}}){
            auto *button=widget<QPushButton>(window,entry.first);
            wait_for([&]{return button->isEnabled();},"local admittance button");
            button->click();wait_for([&]{return board.mode==entry.second;},"GUI local mode");
            wait_for([&]{return widget<QLabel>(window,"control_mode_status")->text().contains(QStringLiteral("ADM"));},"GUI mode name");
            assert(board.legacy_count==0);
            widget<QPushButton>(window,"admittance_stop")->click();
            wait_for([&]{return board.mode==0 && button->isEnabled();},"GUI local stop");
        }
        assert(board.disable_count==0);
    }
    {
        FakeMcu board;
        MainWindow window(false);
        window.show();
        window.findChild<QTabWidget *>()->setCurrentIndex(1);
        widget<QLineEdit>(window, "mcu_ip")->setText("127.0.0.2");
        widget<QSpinBox>(window, "command_port")->setValue(board.command_port);
        widget<QSpinBox>(window, "telemetry_port")->setValue(board.telemetry_port);
        widget<QPushButton>(window, "connect_button")->click();
        auto *transport = widget<QComboBox>(window, "transport_mode");
        auto *enable = widget<QPushButton>(window, "arm_enable");
        auto *execute = widget<QPushButton>(window, "move_execute");
        auto *cancel = widget<QPushButton>(window, "move_cancel");
        auto *target = widget<QDoubleSpinBox>(window, "joint_target_1");
        auto *status = widget<QLabel>(window, "move_status");
        assert(!widget<QPushButton>(window, "servep_reserved")->isEnabled());
        assert(!widget<QPushButton>(window, "movep_reserved")->isEnabled());
        auto *profile_apply = widget<QPushButton>(window, "apply_motor_profile");
        auto *mode_apply = widget<QPushButton>(window, "apply_control_mode");
        wait_for([&] { return profile_apply->isEnabled(); }, "fresh telemetry");
#ifdef INFRABOT_HAS_ARM_VIEW
        // The 3D view loads the URDF and draws the measured joints (150 mrad).
        auto *arm_view = dynamic_cast<infrabot_ik::UrdfViewerWidget *>(widget<QWidget>(window, "arm_view"));
        auto *arm_view_status = widget<QLabel>(window, "arm_view_status");
        assert(arm_view);
        if (!qEnvironmentVariable("INFRABOT_DESCRIPTION_DIR", QStringLiteral(INFRABOT_DEFAULT_DESCRIPTION_DIR)).isEmpty()
            && !qEnvironmentVariableIsSet("INFRABOT_TEST_MISSING_MODEL")) {
            wait_for([&] {
                const auto joints = arm_view->jointValues();
                return arm_view_status->text().startsWith(QStringLiteral("实测")) &&
                       arm_view_status->text().contains(QStringLiteral("末端")) &&
                       std::all_of(joints.begin(), joints.end(), [](double q) { return std::fabs(q - 0.15) < 1e-9; });
            }, "all measured 3D joint angles and tool position");
            assert(arm_view->lastError().isEmpty());
            window.findChild<QTabWidget *>()->setCurrentIndex(0);
            assert(!arm_view->isVisible());
            window.findChild<QTabWidget *>()->setCurrentIndex(1);
            assert(arm_view->isVisible());
        } else {
            assert(!arm_view->isVisible());
            assert(arm_view_status->text().contains("INFRABOT_DESCRIPTION_DIR"));
        }
#endif
        assert(!enable->isEnabled());
        profile_apply->click();
        wait_for([&] { return enable->isEnabled(); }, "profile configured");
        target->setValue(.4); // This draft must NOT become the enabling hold target.
        enable->click();
        wait_for([&] { return board.enabled && !profile_apply->isEnabled(); }, "IDLE enabled");
        assert(board.mode == kControlIdle && board.motion_count == 0 && board.legacy_count == 0);
        transport->setCurrentIndex(transport->findData(kControlNrtJoint));
        assert(board.mode == kControlIdle); // Selecting is not applying.
        mode_apply->click();
        wait_for([&] { return execute->isEnabled(); }, "MCU hold DONE -> Ready");
        // NRT Joint shows the movej parameters in the joint table.
        wait_for([&] { return widget<QDoubleSpinBox>(window, "move_param_0_0")->isVisible(); }, "movej columns shown");
        assert(board.motion_count == 1 && board.last_id == 1 && board.legacy_count == 0);
        assert(std::fabs(board.last_request().command.joint.target[0] - .15f) < .00001f);
        assert(board.legacy_after_move == 0);
        target->setValue(.25);
        widget<QDoubleSpinBox>(window, "move_param_0_0")->setValue(.08);
        widget<QDoubleSpinBox>(window, "move_param_0_2")->setValue(30);
        widget<QDoubleSpinBox>(window, "move_param_0_3")->setValue(.8);
        unsigned timer_ticks = 0;
        QTimer timer;
        QObject::connect(&timer, &QTimer::timeout, [&] { ++timer_ticks; }); timer.start(5);
        wait_for([&] { return timer_ticks >= 10; }, "idle heartbeats");
        assert(board.motion_count == 1); // Slider edits don't transmit a new motion.
        execute_confirmed(execute);
        assert(!execute->isEnabled() && !target->isEnabled());
        wait_for([&] { return board.last_id == 2; }, "movej transmitted");
        const auto request = board.last_request();
        assert(std::fabs(request.command.joint.target[0] - .25f) < .00001f);
        assert(std::fabs(request.command.joint.velocity_max[0] - .08f) < .00001f);
        assert(request.command.joint.kp[0] == 30 && request.command.joint.kd[0] == .8f);
        const auto ticks_at_start = timer_ticks;
        execute->click(); // Disabled action cannot enqueue the next move.
        wait_for([&] { return execute->isEnabled(); }, "MCU actual DONE unlocks next move");
        assert(board.motion_count == 2 && timer_ticks > ticks_at_start + 5);
        assert(status->text().contains(QStringLiteral("已到位")));
        board.block_done = true;
        execute_confirmed(execute);
        wait_for([&] { return board.last_id == 3; }, "third move transmitted");
        board.drop_telemetry = true;
        wait_for([&] { return timer_ticks > ticks_at_start + 150; }, "stale telemetry");
        assert(cancel->isEnabled()); // Stop remains available even with stale telemetry.
        // A cancel is a controlled stop: the MCU brakes and holds the arm in
        // IDLE, still enabled; nothing asks for DISABLE.
        cancel->click();
        wait_for([&] { return board.stop_count > 0 && board.mode == kControlIdle; }, "cancel stops the arm");
        assert(board.enabled && board.disable_count == 0);
        board.drop_telemetry = false;
        wait_for([&] { return transport->isEnabled(); }, "cancel confirmed");
        assert(board.motion_count == 3 && board.legacy_after_move == 0);
        const auto old_legacy = board.legacy_count.load();
        wait_for([&] { return mode_apply->isEnabled() && !enable->isEnabled(); }, "IDLE hold after cancel");
        assert(board.enabled && board.legacy_count == old_legacy);
        transport->setCurrentIndex(transport->findData(kControlRtJoint));
        mode_apply->click(); // Explicit switch, not enable, starts realtime.
        wait_for([&] { return board.legacy_count > old_legacy + 10; }, "existing servej path");
        auto *idle_button = widget<QPushButton>(window, "enter_idle");
        wait_for([&] { return idle_button->isEnabled(); }, "RT mode UI acknowledgement");
        idle_button->click();
        wait_for([&] { return board.mode == kControlIdle; }, "RT -> IDLE");
        assert(board.enabled);
        const auto idle_legacy = board.legacy_count.load();
        const auto end_idle = monotonic_us() + 50000;
        wait_for([&] { return monotonic_us() > end_idle; }, "hold heartbeat only");
        assert(board.legacy_count == idle_legacy);
        widget<QPushButton>(window, "arm_disable")->click();
        wait_for([&] { return !board.enabled; }, "realtime disable");
        window.close();
    }
    {
        FakeMcu board;
        UdpController controller; std::string error;
        assert(controller.start("127.0.0.2", board.command_port, board.telemetry_port, error));
        wait_for([&] { return controller.snapshot().connected; }, "handover telemetry");
        assert(controller.set_arm_mode(kModeMit));
        wait_for([&] { return controller.snapshot().profile_ready; }, "MIT profile");
        assert(controller.enable());
        assert(controller.set_control_mode(kControlNrtJoint, error));
        wait_for([&] { return controller.snapshot().motion.can_execute; }, "NRT hold DONE");
        assert(!controller.set_control_mode(kControlRtJoint, error)); // Must pass IDLE.
        assert(controller.set_control_mode(kControlIdle, error));
        wait_for([&] { return controller.snapshot().control_mode == kControlIdle; }, "NRT -> IDLE");
        assert(board.enabled && board.disable_count == 0 && board.legacy_count == 0);
        const auto hold_end = monotonic_us() + 150000;
        wait_for([&] { return monotonic_us() > hold_end; }, "old session cannot fail new IDLE");
        assert(board.enabled && board.disable_count == 0);
        assert(controller.set_control_mode(kControlRtJoint, error));
        wait_for([&] { return board.legacy_count > 10; }, "NRT -> IDLE -> RT");
        controller.stop();
    }
    {
        FakeMcu board;
        board.drop_motion_replies = true;
        UdpController controller;
        std::string error;
        assert(controller.start("127.0.0.2", board.command_port, board.telemetry_port, error));
        wait_for([&] { return controller.snapshot().connected; }, "backend telemetry");
        assert(!controller.enable());
        assert(controller.set_arm_mode(kModePosVel));
        wait_for([&] { return controller.snapshot().profile_ready; }, "position/velocity profile");
        assert(controller.enable());
        assert(!controller.set_arm_mode(kModeMit)); // Enabled profile is immutable.
        assert(!controller.set_control_mode(kControlRtCartesian, error));
        assert(board.mode == kControlIdle);
        assert(controller.set_control_mode(kControlNrtJoint, error));
        wait_for([&] { return controller.snapshot().motion.phase == motion::Phase::Error; }, "missing DMOV timeout");
        // No DMOS: the session cancels (a controlled stop), it does not disable.
        wait_for([&] { return board.stop_count > 0 && board.mode == kControlIdle; }, "timeout stops the arm");
        assert(board.enabled && board.disable_count == 0);
        assert(!controller.snapshot().motion.can_execute);
        assert(!controller.movej(controller.move_defaults(), error));
        assert(board.legacy_after_move == 0);
        controller.stop();
    }
    {
        FakeMcu board;
        board.supports_modes = false;
        UdpController controller; std::string error;
        assert(controller.start("127.0.0.2", board.command_port, board.telemetry_port, error));
        wait_for([&] { return controller.snapshot().connected; }, "legacy firmware telemetry");
        assert(!controller.enable() && !controller.set_arm_mode(kModeMit));
        assert(!controller.set_control_mode(kControlRtJoint, error));
        assert(!board.enabled && board.legacy_count == 0 && board.motion_count == 0);
        controller.stop();
    }
    {
        // RT Cartesian must start streaming the measured FK pose, never the
        // controller's default identity-at-origin target.
        FakeMcu board;
        board.supports_cartesian = true;
        UdpController controller; std::string error;
        assert(controller.start("127.0.0.2", board.command_port, board.telemetry_port, error));
        wait_for([&] { return controller.snapshot().connected; }, "Cartesian telemetry");
        assert(controller.set_arm_mode(kModeMit));
        wait_for([&] { return controller.snapshot().profile_ready; }, "Cartesian profile");
        assert(controller.enable());
        wait_for([&] { return controller.snapshot().enabled_mask == 0xff; }, "Cartesian enabled");
        assert(controller.set_control_mode(kControlRtCartesian, error));
        wait_for([&] { return board.cartesian_count > 0; }, "first v6 pose");
        // Independent joint-frame FK: DMTL already carries J2/J4 direction, so
        // the fake's 0.15 rad on every joint must not be sign-flipped again.
        infrabot_ik::JointVector q{};
        q.fill(0.15);
        const auto expected = pose_to_wire(infrabot_ik::Kinematics{}.forward(q));
        const auto sent = board.first_cartesian_pose();
        for (size_t i = 0; i < 12; ++i) assert(std::fabs(sent[i] - expected[i]) < 1e-5f);
        assert(std::fabs(sent[0]) + std::fabs(sent[1]) + std::fabs(sent[2]) > 0.01f);
        controller.stop();
    }
    {
        // GUI path: startup shows live FK, mode entry keeps it, dragging the
        // X slider changes the streamed pose.
        FakeMcu board;
        board.supports_cartesian = true;
        MainWindow window(false);
        window.show();
        window.findChild<QTabWidget *>()->setCurrentIndex(1);
        widget<QLineEdit>(window, "mcu_ip")->setText("127.0.0.2");
        widget<QSpinBox>(window, "command_port")->setValue(board.command_port);
        widget<QSpinBox>(window, "telemetry_port")->setValue(board.telemetry_port);
        widget<QPushButton>(window, "connect_button")->click();
        auto *x = widget<QDoubleSpinBox>(window, "cartesian_target_0");
        auto *z = widget<QDoubleSpinBox>(window, "cartesian_target_2");
        auto *profile_apply = widget<QPushButton>(window, "apply_motor_profile");
        auto *enable = widget<QPushButton>(window, "arm_enable");
        wait_for([&] { return profile_apply->isEnabled(); }, "Cartesian GUI telemetry");
        wait_for([&] { return std::fabs(x->value()) > 1.0 && std::fabs(z->value()) > 1.0; }, "startup FK display");
        profile_apply->click();
        wait_for([&] { return enable->isEnabled(); }, "Cartesian GUI profile");
        enable->click();
        wait_for([&] { return board.enabled.load(); }, "Cartesian GUI enabled");
        auto *transport = widget<QComboBox>(window, "transport_mode");
        transport->setCurrentIndex(transport->findData(kControlRtCartesian));
        widget<QPushButton>(window, "apply_control_mode")->click();
        wait_for([&] { return board.cartesian_count > 10; }, "Cartesian GUI stream");
        wait_for([&] { return !widget<QDoubleSpinBox>(window, "move_param_0_0")->isVisible() &&
                              !widget<QPushButton>(window, "move_execute")->isVisible(); },
                 "movej rows hidden outside NRT Joint");
        const auto before = board.last_cartesian_pose();
        assert(std::fabs(before[0] * 1000.0 - x->value()) < 1.5);
        auto *slider = widget<QSlider>(window, "cartesian_slider_0");
        slider->setValue(slider->value() + 10);
        const unsigned seen = board.cartesian_count;
        wait_for([&] { return board.cartesian_count > seen + 10; }, "Cartesian GUI stream after drag");
        const auto after = board.last_cartesian_pose();
        assert(after[0] > before[0] + 0.005f && std::fabs(after[2] - before[2]) < 1e-5f);
        // Speed limits: defaults on the wire, a spinbox edit reaches the
        // stream on the next packets (mm/s, mrad/s, mm/s^2, mrad/s^2).
        const std::array<uint16_t, 4> defaults{500, 3142, 2000, 6283}; // = maxima
        assert(board.last_cartesian_limits() == defaults);
        widget<QDoubleSpinBox>(window, "cartesian_limit_0")->setValue(250.0);
        widget<QDoubleSpinBox>(window, "cartesian_limit_1")->setValue(60.0);
        const unsigned edited = board.cartesian_count;
        const std::array<uint16_t, 4> faster{250, 1047, 2000, 6283};
        wait_for([&] { return board.cartesian_count > edited + 10 && board.last_cartesian_limits() == faster; },
                 "all four Cartesian limits after speed edit");
        widget<QDoubleSpinBox>(window, "cartesian_limit_0")->setValue(1500.0); // clamps to 500
        widget<QDoubleSpinBox>(window, "cartesian_limit_1")->setValue(180.0);
        const unsigned topped = board.cartesian_count;
        wait_for([&] { return board.cartesian_count > topped + 10; }, "Cartesian stream at top speed");
        widget<QDoubleSpinBox>(window, "cartesian_limit_2")->setValue(5000.0); // clamps to 2000
        widget<QDoubleSpinBox>(window, "cartesian_limit_3")->setValue(859.0);  // clamps to 360
        const unsigned accelerated = board.cartesian_count;
        wait_for([&] { return board.cartesian_count > accelerated + 10 && board.last_cartesian_limits() == defaults; },
                 "all four Cartesian limits at top speed and acceleration");
        // An unreachable target (X far beyond reach) is not streamed: the MCU
        // keeps the last reachable one.
        const auto reachable = board.last_cartesian_pose();
        z->setValue(790.0); // reach is ~0.52 m
        const unsigned refused = board.cartesian_count;
        wait_for([&] { return board.cartesian_count > refused + 10; }, "Cartesian stream after unreachable target");
        assert(board.last_cartesian_pose() == reachable);
        assert(widget<QLabel>(window, "cartesian_ik_result")->text().contains(QStringLiteral("超出可达范围")));
    }
    {
        // movep carries the same limits; per-joint caps only bound them.
        UdpController controller;
        const std::array<uint16_t, 4> limits{250, 1047, 400, 1396};
        controller.set_cartesian_limits(limits);
        const auto p = controller.cartesian_defaults();
        assert(p.cartesian_limits == limits);
        assert(p.velocity_max == kCartesianJointVelocityCap &&
               p.acceleration_max[6] == kCartesianJointAccelerationCap);
        std::string error;
        assert(motion::valid(p, error));
        auto top = p;
        top.cartesian_limits = {500, 3142, 2000, 6283};
        assert(motion::valid(top, error));
        auto over_acceleration = p;
        over_acceleration.cartesian_limits[2] = 2001;
        assert(!motion::valid(over_acceleration, error));
        over_acceleration = p;
        over_acceleration.cartesian_limits[3] = 6284;
        assert(!motion::valid(over_acceleration, error));
        auto over = p;
        over.cartesian_limits[1] = 3143;
        assert(!motion::valid(over, error));
        over = p;
        over.cartesian_limits[0] = 501;
        assert(!motion::valid(over, error));
    }
    {
        // Position-velocity profile: movep's per-joint velocity_max is the
        // drives' v_des, so it carries the runtime template's velocity limit.
        FakeMcu board;
        UdpController controller; std::string error;
        assert(controller.start("127.0.0.2", board.command_port, board.telemetry_port, error));
        wait_for([&] { return controller.snapshot().connected; }, "position-velocity telemetry");
        assert(controller.set_arm_mode(kModePosVel));
        wait_for([&] { return controller.snapshot().motor_profile == kModePosVel; }, "position-velocity profile");
        const auto p = controller.cartesian_defaults();
        for (size_t i = 0; i < motion::axes; ++i)
            assert(p.velocity_max[i] == kBuiltinArmVelocityLimitMrad[i] / 1000.0f);
        assert(motion::valid(p, error));
        controller.stop();
    }
    for (const bool supported : {true, false}) {
        // Mode entry seeds RT targets from the MCU hold command (no drop by
        // the gravity sag); old firmware without the query falls back to
        // feedback.
        FakeMcu board;
        board.supports_cartesian = true;
        board.supports_hold_query = supported;
        board.hold_offset_mrad = 25;
        UdpController controller; std::string error;
        assert(controller.start("127.0.0.2", board.command_port, board.telemetry_port, error));
        wait_for([&] { return controller.snapshot().connected; }, "hold telemetry");
        assert(controller.set_arm_mode(kModeMit));
        wait_for([&] { return controller.snapshot().profile_ready; }, "hold profile");
        assert(controller.enable());
        wait_for([&] { return controller.snapshot().enabled_mask == 0xff; }, "hold enabled");
        const int32_t expected = supported ? 175 : 150;
        assert(controller.set_control_mode(kControlRtJoint, error));
        wait_for([&] { return board.legacy_count > 0; }, "first RT joint packet");
        const auto first = board.first_arm_targets();
        for (size_t i = 0; i < 7; ++i) assert(first[i] == expected);
        // An enabled gripper continues from its hold command (squeezing, it
        // sits past the measured angle); old firmware: its measured value.
        assert(first[7] == (supported ? 325 : 300));
        assert(controller.set_control_mode(kControlIdle, error));
        wait_for([&] { return controller.snapshot().control_mode == kControlIdle; }, "IDLE telemetry");
        const bool entered = controller.set_control_mode(kControlRtCartesian, error);
        if (!entered) std::cerr << "RT Cartesian entry: " << error << "\n";
        assert(entered);
        wait_for([&] { return board.cartesian_count > 0; }, "first hold-seeded v6 pose");
        std::array<int32_t, kMotorCount> hold{};
        hold.fill(expected);
        const auto want = pose_to_wire(forward_from_mrad(hold));
        const auto sent = board.first_cartesian_pose();
        for (size_t i = 0; i < 12; ++i) assert(std::fabs(sent[i] - want[i]) < 1e-5f);
        controller.stop();
    }
    {
        // Arm and gripper enabled separately: the arm alone runs a control
        // mode; the gripper comes and goes without touching it, and no group
        // operation sends the full DISABLE.  The gripper has its own command
        // (position, speed, force) in every mode.
        FakeMcu board;
        board.supports_cartesian = true;
        MainWindow window(false);
        window.show();
        window.findChild<QTabWidget *>()->setCurrentIndex(1);
        widget<QLineEdit>(window, "mcu_ip")->setText("127.0.0.2");
        widget<QSpinBox>(window, "command_port")->setValue(board.command_port);
        widget<QSpinBox>(window, "telemetry_port")->setValue(board.telemetry_port);
        widget<QPushButton>(window, "connect_button")->click();
        auto *profile_apply = widget<QPushButton>(window, "apply_motor_profile");
        auto *enable_all = widget<QPushButton>(window, "arm_enable");
        auto *enable_arm = widget<QPushButton>(window, "arm_enable_arm");
        auto *enable_gripper = widget<QPushButton>(window, "arm_enable_gripper");
        auto *disable_arm = widget<QPushButton>(window, "arm_disable_arm");
        auto *disable_gripper = widget<QPushButton>(window, "arm_disable_gripper");
        auto *transport = widget<QComboBox>(window, "transport_mode");
        auto *mode_apply = widget<QPushButton>(window, "apply_control_mode");
        auto *idle = widget<QPushButton>(window, "enter_idle");
        wait_for([&] { return profile_apply->isEnabled(); }, "group telemetry");
        profile_apply->click();
        wait_for([&] { return enable_arm->isEnabled() && enable_gripper->isEnabled() && enable_all->isEnabled(); }, "group profile");
        enable_arm->click();
        wait_for([&] { return board.enabled_mask == 0x7f && !enable_arm->isEnabled() &&
                              enable_gripper->isEnabled() && enable_all->isEnabled() &&
                              disable_arm->isEnabled() && !disable_gripper->isEnabled(); }, "arm alone enabled");
        transport->setCurrentIndex(transport->findData(kControlRtJoint));
        mode_apply->click();
        wait_for([&] { return board.mode == kControlRtJoint && board.legacy_count > 0 && idle->isEnabled(); },
                 "RT Joint, arm alone");
        // Slow home needs the arm only, and homes the arm only.
        auto *home = widget<QPushButton>(window, "arm_home_slow");
        wait_for([&] { return home->isEnabled(); }, "slow home available, arm alone");
        assert(!widget<QSlider>(window, "gripper_slider")->isEnabled()); // gripper disabled
        home->click();
        wait_for([&] { return board.last_arm_targets()[0] < 140; }, "arm homing");
        assert(board.last_arm_targets()[7] == 300); // the gripper's target untouched
        idle->click();
        wait_for([&] { return board.mode == kControlIdle && enable_gripper->isEnabled(); }, "back to IDLE");
        enable_gripper->click();
        wait_for([&] { return board.enabled_mask == 0xff && !enable_all->isEnabled(); }, "gripper added");
        // Enabling it applies the speed/force set here at the position it holds.
        wait_for([&] { return board.gripper_count > 0; }, "gripper settings applied");
        assert(board.gripper_position == 300);
        // RT Cartesian: the gripper is commanded with position, speed and force.
        transport->setCurrentIndex(transport->findData(kControlRtCartesian));
        mode_apply->click();
        auto *gripper_target = widget<QDoubleSpinBox>(window, "joint_target_8");
        // The Cartesian controls enable once the GUI shows the mode: then the
        // gripper's controls must be enabled too.
        wait_for([&] { return board.mode == kControlRtCartesian && board.cartesian_count > 0 &&
                              widget<QDoubleSpinBox>(window, "cartesian_target_0")->isEnabled() &&
                              gripper_target->isEnabled() && widget<QSlider>(window, "gripper_slider")->isEnabled() &&
                              disable_gripper->isEnabled(); }, "RT Cartesian, both");
        widget<QDoubleSpinBox>(window, "gripper_speed")->setValue(2.5);
        widget<QDoubleSpinBox>(window, "gripper_force")->setValue(0.4);
        gripper_target->setValue(0.6);
        wait_for([&] { return board.gripper_position == 600 && board.gripper_speed == 2500 &&
                              board.gripper_force == 400; }, "gripper command in RT Cartesian");
        // The whole travel (0..5.94 rad) and the force in steps of 0.01.
        auto *gripper_force = widget<QDoubleSpinBox>(window, "gripper_force");
        assert(std::fabs(gripper_target->maximum() - 5.94) < 1e-9 &&
               widget<QSlider>(window, "gripper_slider")->maximum() == 5940);
        assert(std::fabs(gripper_force->singleStep() - 0.01) < 1e-9 &&
               std::fabs(gripper_force->minimum() - 0.01) < 1e-9);
        gripper_force->setValue(0.37);
        gripper_target->setValue(5.94);
        wait_for([&] { return board.gripper_position == 5940 && board.gripper_force == 370; },
                 "gripper at the end of its travel, force 0.37");
        // Speed and force each have a slider (0.01 steps) under the gripper's
        // position slider, in step with their value boxes.
        auto *speed_slider = widget<QSlider>(window, "gripper_speed_slider");
        auto *force_slider = widget<QSlider>(window, "gripper_force_slider");
        assert(speed_slider->value() == 250 && force_slider->value() == 37);
        speed_slider->setValue(320);
        force_slider->setValue(55);
        assert(std::fabs(widget<QDoubleSpinBox>(window, "gripper_speed")->value() - 3.2) < 1e-9 &&
               std::fabs(gripper_force->value() - 0.55) < 1e-9);
        wait_for([&] { return board.gripper_speed == 3200 && board.gripper_force == 550; },
                 "gripper speed/force from their sliders");
        // Torque bars and values: each motor's measured torque, signed; amber
        // past the rated torque, red past 90 % of full scale.
        const std::array<int32_t, 8> torque = {1500, -3840, 0, 11000, 3600, -9500, 250, -420};
        std::array<QLabel *, 8> labels;
        std::array<TorqueBar *, 8> bars;
        for (size_t i = 0; i < torque.size(); ++i) {
            const auto suffix = std::to_string(i + 1);
            labels[i] = widget<QLabel>(window, ("torque_value_" + suffix).c_str());
            bars[i] = dynamic_cast<TorqueBar *>(widget<QWidget>(window, ("torque_bar_" + suffix).c_str()));
            assert(bars[i]);
            board.torques[i] = torque[i];
        }
        wait_for([&] {
            for (size_t i = 0; i < torque.size(); ++i) {
                const double expected = torque[i] / 1000.0;
                if (labels[i]->text() != QString::number(expected, 'f', 2) ||
                    !(std::fabs(bars[i]->value() - expected) < 1e-3)) return false;
            }
            return true;
        }, "all eight torque labels and bars");
        assert(bars[0]->level() == 0 && bars[3]->level() == 0);  // 11 N·m: within J4's rated 12
        assert(bars[4]->level() == 1);  // 3.6 N·m: past J5's rated 3.5
        assert(bars[5]->level() == 2);  // -9.5 N·m: past 90 % of J6's 10
        for (size_t i = 0; i < 8; ++i) board.torques[i] = 0;
        disable_gripper->click();
        wait_for([&] { return board.enabled_mask == 0x7f && !gripper_target->isEnabled(); }, "gripper off");
        const unsigned streamed = board.cartesian_count;
        wait_for([&] { return board.cartesian_count > streamed + 20; }, "the arm keeps streaming");
        assert(board.mode == kControlRtCartesian && board.gripper_disable_count > 0);
        // The arm disabled during an NRT session: the move ends with it, the
        // gripper stays (a cancel would ask for the full DISABLE).
        wait_for([&] { return idle->isEnabled(); }, "IDLE available");
        idle->click();
        wait_for([&] { return board.mode == kControlIdle && enable_gripper->isEnabled(); }, "IDLE again");
        enable_gripper->click();
        wait_for([&] { return board.enabled_mask == 0xff; }, "gripper back");
        transport->setCurrentIndex(transport->findData(kControlNrtJoint));
        mode_apply->click();
        wait_for([&] { return widget<QPushButton>(window, "move_execute")->isEnabled(); }, "NRT session ready");
        // The move owns the arm only: the gripper is commanded meanwhile.
        gripper_target->setValue(0.25);
        wait_for([&] { return board.gripper_position == 250; }, "gripper command during an NRT session");
        disable_arm->click();
        wait_for([&] { return board.enabled_mask == 0x80 && board.mode == kControlIdle; }, "arm off");
        const auto settle = monotonic_us() + 150000;
        wait_for([&] { return monotonic_us() >= settle; }, "settle");
        assert(board.enabled_mask == 0x80 && board.arm_disable_count > 0 && board.disable_count == 0);
        disable_gripper->click();
        wait_for([&] { return board.enabled_mask == 0; }, "gripper off");
        assert(board.disable_count == 0);
    }
    {
        // A gripper that is not connected: its enable fails and disables
        // only its own group again; the arm keeps holding.
        FakeMcu board;
        board.gripper_absent = true;
        UdpController controller; std::string error;
        assert(controller.start("127.0.0.2", board.command_port, board.telemetry_port, error));
        wait_for([&] { return controller.snapshot().connected; }, "absent-gripper telemetry");
        assert(controller.set_arm_mode(kModeMit));
        wait_for([&] { return controller.snapshot().profile_ready; }, "absent-gripper profile");
        assert(controller.enable_group(kArmMask) && board.enabled_mask == 0x7f);
        assert(!controller.enable_group(kGripperMask));
        wait_for([&] { return board.gripper_disable_count > 0; }, "gripper group disabled again");
        assert(board.enabled_mask == 0x7f && board.disable_count == 0);
        assert(controller.set_control_mode(kControlRtJoint, error));
        controller.stop();
    }
    {
        // Telemetry v3 through the GUI: a failed parameter self-check names
        // the drive and register, the controlled stop button holds the arm,
        // and the 诊断 page records, decodes each drive and logs faults.
        FakeMcu board;
        board.telemetry_v3 = true;
        MainWindow window(false);
        window.show();
        auto *tabs = window.findChild<QTabWidget *>();
        tabs->setCurrentIndex(1);
        widget<QLineEdit>(window, "mcu_ip")->setText("127.0.0.2");
        widget<QSpinBox>(window, "command_port")->setValue(board.command_port);
        widget<QSpinBox>(window, "telemetry_port")->setValue(board.telemetry_port);
        widget<QPushButton>(window, "connect_button")->click();
        auto *profile_apply = widget<QPushButton>(window, "apply_motor_profile");
        auto *enable = widget<QPushButton>(window, "arm_enable");
        wait_for([&] { return profile_apply->isEnabled(); }, "v3 telemetry");
        board.selfcheck_fail_joint = 2;
        const QString refused = click_dismissing(profile_apply);
        assert(refused.contains(QStringLiteral("J3 VMAX 读回 8")) && refused.contains(QStringLiteral("期望 10")));
        wait_for([&] { return widget<QLabel>(window, "control_mode_status")->text().contains(QStringLiteral("参数自检失败")); },
                 "self-check failure on the arm page");
        assert(!enable->isEnabled());
        board.selfcheck_fail_joint = INFRABOT_NO_JOINT;
        profile_apply->click();
        wait_for([&] { return enable->isEnabled(); }, "profile after the self-check passed");
        enable->click();
        wait_for([&] { return board.enabled.load(); }, "enabled");
        auto *transport = widget<QComboBox>(window, "transport_mode");
        transport->setCurrentIndex(transport->findData(kControlRtJoint));
        widget<QPushButton>(window, "apply_control_mode")->click();
        wait_for([&] { return board.legacy_count > 10; }, "RT joint stream");
        auto *stop = widget<QPushButton>(window, "arm_controlled_stop");
        wait_for([&] { return stop->isEnabled(); }, "controlled stop available");
        stop->click();
        wait_for([&] { return board.stop_count > 0 && board.mode == kControlIdle; }, "controlled stop");
        assert(board.enabled && board.disable_count == 0);
        const unsigned streamed = board.legacy_count;
        const auto quiet = monotonic_us() + 50000;
        wait_for([&] { return monotonic_us() > quiet; }, "no stream after the stop");
        assert(board.legacy_count == streamed);

        tabs->setCurrentIndex(2);
        auto *start = widget<QPushButton>(window, "record_start");
        wait_for([&] { return start->isEnabled(); }, "recording available");
        widget<QLineEdit>(window, "record_label")->setText(QStringLiteral("gui test"));
        start->click();
        auto *record_status = widget<QLabel>(window, "record_status");
        wait_for([&] { return record_status->text().startsWith(QStringLiteral("录制中")); }, "recording");
        const auto recorded_for = monotonic_us() + 200000;
        wait_for([&] { return monotonic_us() > recorded_for; }, "recording 200 ms");
        widget<QLineEdit>(window, "record_marker_text")->setText(QStringLiteral("标记一"));
        widget<QPushButton>(window, "record_marker")->click();
        // A drive fault, decoded in the table and in the log.
        board.fault_joint = 4;
        board.fault_state = INFRABOT_DRIVE_MOS_OVERTEMP;
        auto *table = window.findChild<QTableWidget *>("motor_table");
        assert(table);
        wait_for([&] { return table->item(4, 0)->text() == QStringLiteral("MOS 过温"); }, "decoded drive fault");
        assert(table->item(4, 0)->toolTip().contains(QStringLiteral("处理：")));
        auto *fault_log = window.findChild<QPlainTextEdit *>("fault_log");
        wait_for([&] {
            return fault_log->toPlainText().contains(QStringLiteral("J5 MOS 过温")) &&
                   table->item(0, 1)->text() == "30" && table->item(0, 2)->text() == "35" &&
                   table->item(3, 4)->text() == "0.30" && table->item(0, 5)->text() == QStringLiteral("MIT") &&
                   widget<QLabel>(window, "link_quality")->text().contains(QStringLiteral("MCU：收指令"));
        }, "drive diagnostics, fault log and link feedback");
        // Clearing needs the drive disabled.
        widget<QPushButton>(window, "arm_disable")->click();
        wait_for([&] { return !board.enabled; }, "disabled");
        const QString cleared = click_dismissing(widget<QPushButton>(window, "clear_faults"));
        assert(cleared.isEmpty());
        wait_for([&] { return table->item(4, 0)->text() == QStringLiteral("失能"); }, "fault cleared");
        assert(board.clear_error_count == 1);
        widget<QPushButton>(window, "record_stop")->click();
        wait_for([&] { return record_status->text().startsWith(QStringLiteral("已停止")); }, "recording stopped");
        const QString text = record_status->text();
        const int from = text.indexOf(QStringLiteral("：")) + 1;
        const std::filesystem::path recorded(text.mid(from, text.indexOf(QStringLiteral(" |")) - from).toStdString());
        assert(recorded.filename().string().find("gui_test") != std::string::npos);
        // At 1 kHz: at least the 200 ms waited above.
        const int rows_at = text.indexOf(QStringLiteral("遥测 ")) + 3;
        assert(text.mid(rows_at, text.indexOf(QStringLiteral(" 行"), rows_at) - rows_at).toInt() >= 150);
        assert(std::filesystem::file_size(recorded / "telemetry_000.csv") > 150 * 300);
        std::ifstream events(recorded / "events.csv");
        const std::string all((std::istreambuf_iterator<char>(events)), std::istreambuf_iterator<char>());
        assert(all.find("标记一") != std::string::npos && all.find("J5 MOS 过温") != std::string::npos);
        std::filesystem::remove_all(recorded);
        std::error_code ignored;
        std::filesystem::remove(recorded.parent_path(), ignored); // only when empty
    }
    for (const bool controlled : {true, false}) {
        // Closing the GUI or reconnecting never removes power: the MCU brakes
        // the arm and holds it enabled (firmware with the controlled stop).
        // Only firmware without it is disabled, as it would disable itself
        // once the stream stops anyway.
        FakeMcu board;
        board.controlled_stop = controlled;
        board.telemetry_v3 = controlled;
        auto window = std::make_unique<MainWindow>(false);
        window->show();
        window->findChild<QTabWidget *>()->setCurrentIndex(1);
        widget<QLineEdit>(*window, "mcu_ip")->setText("127.0.0.2");
        widget<QSpinBox>(*window, "command_port")->setValue(board.command_port);
        widget<QSpinBox>(*window, "telemetry_port")->setValue(board.telemetry_port);
        widget<QPushButton>(*window, "connect_button")->click();
        auto *profile_apply = widget<QPushButton>(*window, "apply_motor_profile");
        auto *enable = widget<QPushButton>(*window, "arm_enable");
        auto *transport = widget<QComboBox>(*window, "transport_mode");
        auto *mode_apply = widget<QPushButton>(*window, "apply_control_mode");
        wait_for([&] { return profile_apply->isEnabled(); }, "leave: telemetry");
        profile_apply->click();
        wait_for([&] { return enable->isEnabled(); }, "leave: profile");
        enable->click();
        wait_for([&] { return board.enabled_mask == 0xff; }, "leave: enabled");
        transport->setCurrentIndex(transport->findData(kControlRtJoint));
        mode_apply->click();
        wait_for([&] { return board.mode == kControlRtJoint && board.legacy_count > 10; }, "leave: RT joint stream");
        if (controlled) {
            // Reconnect in RT Joint: confirmed, a controlled stop, then the new
            // connection takes the holding arm over and can stream again.
            const unsigned stops = board.stop_count;
            execute_confirmed(widget<QPushButton>(*window, "connect_button"));
            wait_for([&] { return board.stop_count > stops && board.mode == kControlIdle; }, "reconnect stops the arm");
            assert(board.enabled && board.disable_count == 0 && board.arm_disable_count == 0);
            const unsigned beats = board.heartbeat_count;
            wait_for([&] { return board.heartbeat_count > beats + 10; }, "the new connection keeps it alive");
            wait_for([&] { return mode_apply->isEnabled(); }, "the new connection sees the hold");
            const unsigned streamed = board.legacy_count;
            mode_apply->click(); // RT Joint is still selected
            wait_for([&] { return board.mode == kControlRtJoint && board.legacy_count > streamed + 10; },
                     "RT joint again after the reconnect");
        }
        // Close the window in RT Joint.
        const unsigned stops = board.stop_count, disables = board.disable_count;
        window->close();
        window.reset();
        if (controlled) {
            wait_for([&] { return board.stop_count > stops && board.mode == kControlIdle; }, "close stops the arm");
            const auto settle = monotonic_us() + 150000;
            wait_for([&] { return monotonic_us() > settle; }, "after the close");
            assert(board.enabled && board.enabled_mask == 0xff && board.disable_count == disables);
        } else {
            wait_for([&] { return board.disable_count > disables && !board.enabled; }, "old firmware: disabled");
        }
    }
    {
        // A controlled stop is resumed in one click: back in the interrupted
        // mode from where the arm stopped, no manual IDLE switch.
        FakeMcu board;
        board.telemetry_v3 = true;
        board.hold_offset_mrad = 25;
        MainWindow window(false);
        window.show();
        window.findChild<QTabWidget *>()->setCurrentIndex(1);
        widget<QLineEdit>(window, "mcu_ip")->setText("127.0.0.2");
        widget<QSpinBox>(window, "command_port")->setValue(board.command_port);
        widget<QSpinBox>(window, "telemetry_port")->setValue(board.telemetry_port);
        widget<QPushButton>(window, "connect_button")->click();
        auto *profile_apply = widget<QPushButton>(window, "apply_motor_profile");
        auto *enable = widget<QPushButton>(window, "arm_enable");
        auto *transport = widget<QComboBox>(window, "transport_mode");
        auto *mode_apply = widget<QPushButton>(window, "apply_control_mode");
        auto *idle = widget<QPushButton>(window, "enter_idle");
        auto *stop = widget<QPushButton>(window, "arm_controlled_stop");
        auto *resume = widget<QPushButton>(window, "arm_resume");
        auto *status_label = widget<QLabel>(window, "control_mode_status");
        auto *j1 = widget<QDoubleSpinBox>(window, "joint_target_1");
        wait_for([&] { return profile_apply->isEnabled(); }, "resume: telemetry");
        profile_apply->click();
        wait_for([&] { return enable->isEnabled(); }, "resume: profile");
        enable->click();
        wait_for([&] { return board.enabled_mask == 0xff; }, "resume: enabled");
        assert(!resume->isEnabled());
        transport->setCurrentIndex(transport->findData(kControlRtJoint));
        mode_apply->click();
        wait_for([&] { return board.mode == kControlRtJoint && board.legacy_count > 10; }, "resume: RT joint");
        j1->setValue(0.3); // the stream carries the dragged target
        wait_for([&] { return board.last_arm_targets()[0] == 300; }, "resume: dragged target streamed");
        wait_for([&] { return stop->isEnabled(); }, "resume: stop available");
        stop->click();
        wait_for([&] { return board.stop_count > 0 && board.mode == kControlIdle; }, "resume: stopped");
        wait_for([&] { return resume->isEnabled() && status_label->text().contains(QStringLiteral("点“恢复”回到 RT Joint")); },
                 "resume offered for RT Joint");
        const unsigned before = board.legacy_count;
        resume->click();
        wait_for([&] { return board.mode == kControlRtJoint && board.legacy_count > before + 10 &&
                              board.last_arm_targets()[0] == 175 && std::fabs(j1->value() - 0.175) < 1e-9 &&
                              transport->currentData().toInt() == kControlRtJoint; }, "resumed stream and held UI target");
        wait_for([&] { return !resume->isEnabled() && !status_label->text().contains(QStringLiteral("受控停止")); },
                 "resume: stop cleared");
        // "进入所选控制模式" on the interrupted mode resumes too, waiting for
        // the arm to settle (a position-velocity drive trailing its command).
        const unsigned stops = board.stop_count;
        stop->click();
        wait_for([&] { return board.stop_count > stops && board.mode == kControlIdle; }, "resume: stopped again");
        board.arm_velocity_mrad_s = 200;
        std::thread settle([&] { std::this_thread::sleep_for(std::chrono::milliseconds(300)); board.arm_velocity_mrad_s = 0; });
        wait_for([&] { return mode_apply->isEnabled(); }, "resume: mode apply available");
        mode_apply->click(); // RT Joint is still selected
        settle.join();
        wait_for([&] { return board.mode == kControlRtJoint; }, "resume through the mode button");
        // movej: the interrupted target stays the draft; executing continues it.
        wait_for([&] { return idle->isEnabled(); }, "resume: IDLE available");
        idle->click();
        wait_for([&] { return board.mode == kControlIdle; }, "resume: IDLE");
        transport->setCurrentIndex(transport->findData(kControlNrtJoint));
        mode_apply->click();
        auto *execute = widget<QPushButton>(window, "move_execute");
        wait_for([&] { return execute->isEnabled(); }, "resume: movej ready");
        j1->setValue(0.25);
        board.block_done = true;
        execute_confirmed(execute);
        wait_for([&] { return board.last_id == 2; }, "resume: movej running");
        widget<QPushButton>(window, "move_cancel")->click();
        wait_for([&] { return board.mode == kControlIdle && board.stop_reason != INFRABOT_STOP_NONE; }, "resume: movej stopped");
        board.block_done = false;
        wait_for([&] { return resume->isEnabled(); }, "resume offered for movej");
        resume->click();
        wait_for([&] { return execute->isEnabled(); }, "resume: new session holds");
        assert(board.mode == kControlNrtJoint && board.last_request().motion_id == 1);
        assert(std::fabs(j1->value() - 0.25) < 1e-9); // the draft is kept
        execute_confirmed(execute);
        wait_for([&] { return board.last_id == 2 && std::fabs(board.last_request().command.joint.target[0] - 0.25f) < 1e-5f; },
                 "resume: the interrupted movej continues");
        window.close();
    }
    std::cout << "Qt loopback: profile interlock, IDLE enable, explicit RT/NRT, SI targets, DONE gating, controlled-stop cancel/timeout, Cartesian/old-firmware rejection, arm/gripper groups, self-check, controlled stop, diagnostics/recording, close/reconnect hold, one-click resume passed\n";
}
