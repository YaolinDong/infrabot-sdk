# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Yaolin Dong
"""Real pybind extension and C++ loopback MCU; never sends to a physical device."""
import math
import tempfile
import time
import unittest
import infrabot_native as n
from _infrabot_test_mcu import FakeMcu


def wait(predicate, seconds=2):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if predicate():
            return
        time.sleep(.002)
    raise AssertionError("timed out waiting for loopback state")


class RobotBindingTests(unittest.TestCase):
    def setUp(self):
        self.board = FakeMcu()
        self.board.telemetry_v3 = True
        self.board.gripper_absent = True
        self.board.calibrate()
        self.robot = n.Robot("127.0.0.2", self.board.command_port, self.board.telemetry_port)
        self.robot.set_heartbeat(False)
        self.robot.connect()
        wait(lambda: self.robot.state().joints.host_time_us > 0)

    def tearDown(self):
        self.robot.disconnect()
        del self.robot
        del self.board

    def start(self, allow_zero_kp=True):
        # Application sequencing is explicit; the SDK native pump never enables.
        self.robot.set_motor_profile(n.MotorProfile.Mit)
        self.robot.set_heartbeat(True)
        self.robot.servo_on(n.MotorGroup.Arm)
        self.robot.switch_mode(n.Mode.RtJoint)
        c = n.JointCommand(); c.position = self.robot.hold_command()[:7]
        options = n.JointStreamOptions(); options.allow_zero_kp = allow_zero_kp
        self.robot.start_joint_stream(c, options)

    def test_protocol_negotiation(self):
        info = self.robot.info()
        self.assertEqual((info.protocol_major, info.protocol_minor, info.firmware), (1, 1, "fixture-123"))
        self.robot.disconnect()
        self.board.protocol_major = 2
        with self.assertRaisesRegex(RuntimeError, r"2\.1 firmware.*1\.1"):
            self.robot.connect()
        self.assertFalse(self.robot.connected())
        self.board.v1_only = True
        self.robot.connect()
        info = self.robot.info()
        self.assertEqual((info.protocol_version, info.protocol_major, info.firmware), (1, 1, "unknown"))

    def test_read_only_and_timestamps(self):
        self.assertEqual(self.board.enabled_mask, 0)
        self.assertEqual(self.board.heartbeat_count, 0)
        for i in range(7):
            self.board.torque(i, -1000*(i+1))
        wait(lambda: self.robot.state().joints.torque[6] == -7.)
        s = self.robot.state().joints
        self.assertEqual(len(s.position), 8)
        self.assertEqual(s.torque[:7], [-1., -2., -3., -4., -5., -6., -7.])
        self.assertLess(abs(s.host_time_us*1e-6-time.perf_counter()), .05)
        self.board.drop_telemetry = True
        time.sleep(.025)
        a = self.robot.state().joints; b = self.robot.state().joints
        self.assertEqual(a.host_time_us, b.host_time_us)
        self.assertEqual(a.sequence, b.sequence)
        wait(lambda: self.robot.state().telemetry_age_ms > 100)

    def test_local_gravity_modes_use_existing_robot_api(self):
        self.robot.set_motor_profile(n.MotorProfile.Mit)
        self.robot.set_heartbeat(True)
        self.robot.servo_on(n.MotorGroup.Arm)
        self.assertTrue(self.robot.info().capabilities & n.CAP_LOCAL_GRAVITY)
        for mode in (n.Mode.GravityFree, n.Mode.GravityLock):
            self.robot.switch_mode(mode)
            self.assertEqual(self.robot.state().mode, mode)
            self.assertFalse(self.robot.joint_stream_stats().active)
            time.sleep(.15)  # no Python producer; local control is not an RT stream
            self.assertEqual(self.robot.state().mode, mode)
            stopped_at = time.monotonic()
            self.robot.stop()
            self.assertGreaterEqual(time.monotonic()-stopped_at, .45)
            self.assertEqual(self.board.enabled_mask, 127)
        self.assertEqual(self.board.disable_count, 0)

    def test_local_gravity_is_rejected_on_older_firmware(self):
        self.robot.disconnect()
        self.board.supports_gravity = False
        self.robot.connect()
        wait(lambda: self.robot.state().joints.host_time_us > 0)
        self.robot.set_motor_profile(n.MotorProfile.Mit)
        self.robot.servo_on(n.MotorGroup.Arm)
        with self.assertRaisesRegex(RuntimeError, "固件不支持"):
            self.robot.switch_mode(n.Mode.GravityFree)

    def test_independent_compliant_modes(self):
        self.robot.set_motor_profile(n.MotorProfile.Mit)
        self.robot.servo_on(n.MotorGroup.Arm)
        for mode,status in [(n.Mode.JointImpedance,n.OperationalStatus.JointImpedance),
                            (n.Mode.CartesianImpedance,n.OperationalStatus.CartesianImpedance),
                            (n.Mode.JointAdmittance,n.OperationalStatus.JointAdmittance),
                            (n.Mode.CartesianAdmittance,n.OperationalStatus.CartesianAdmittance)]:
            self.robot.switch_mode(mode)
            wait(lambda:self.robot.state().operational_status==status)
            self.assertEqual(self.robot.state().mode,mode)
            self.assertFalse(self.robot.joint_stream_stats().active)
            self.robot.stop()
            self.assertEqual(self.board.enabled_mask,127)
        self.assertEqual(self.board.disable_count,0)

    def test_admittance_requires_capability(self):
        self.robot.disconnect()
        self.board.supports_admittance=False
        self.robot.connect()
        wait(lambda:self.robot.state().joints.host_time_us>0)
        self.robot.set_motor_profile(n.MotorProfile.Mit)
        self.robot.servo_on(n.MotorGroup.Arm)
        with self.assertRaisesRegex(RuntimeError,"固件不支持"):
            self.robot.switch_mode(n.Mode.CartesianAdmittance)
        self.assertEqual(self.board.mode,0)

    def test_joint_speed_ceiling_in_stream_and_one_shot(self):
        self.start()
        c = n.JointCommand()
        c.position = self.robot.hold_command()[:7]
        c.velocity = [3.14]*7
        c.velocity_limit = [3.14]*7
        self.robot.servej(c)
        wait(lambda: self.board.last_command()["velocity"][:7] == [3140]*7)
        c.velocity = [3.15]*7
        with self.assertRaisesRegex(RuntimeError, "invalid joint stream command"):
            self.robot.servej(c)
        self.robot.stop_joint_stream()
        with self.assertRaisesRegex(RuntimeError, "3.14"):
            self.robot.servej(c)
        c.velocity = [3.14]*7
        self.robot.servej(c)
        self.robot.stop()

    def test_five_fields_and_arm_only(self):
        self.start()
        c = n.JointCommand()
        c.position = [.1,-.2,.3,-.4,.5,-.6,.7]
        c.velocity = [.01,-.02,.03,-.04,.05,-.06,.07]
        c.kp = [0]*7; c.kd = [.8]*7
        c.torque = [1,-2,3,-4,5,-6,7]
        self.robot.servej(c)
        wait(lambda: self.board.last_command()["torque"][6] == 7000)
        packet = self.board.last_command()
        self.assertEqual(packet["position"], [100,-200,300,-400,500,-600,700,0])
        self.assertEqual(packet["velocity"], [10,-20,30,-40,50,-60,70,0])
        self.assertEqual(packet["kp"], [0]*8)
        self.assertEqual(packet["kd"], [800]*7+[0])
        self.assertEqual(self.board.enabled_mask, 127)
        with self.assertRaises((TypeError, ValueError)):
            c.position = [0]*8
        c.torque = [math.nan]*7
        with self.assertRaises(RuntimeError):
            self.robot.servej(c)
        self.robot.stop()
        self.assertEqual(self.board.enabled_mask, 127)
        self.assertEqual(self.board.disable_count, 0)
        with self.assertRaises(RuntimeError):
            self.robot.servej(n.JointCommand())
        self.robot.servo_off(n.MotorGroup.Arm)
        self.assertEqual(self.board.enabled_mask, 0)

    def test_native_watchdog_works_while_python_busy(self):
        self.start()
        # Keep the GIL/Python producer busy, with no command updates.
        until = time.perf_counter() + .25
        while time.perf_counter() < until:
            pass
        wait(lambda: self.board.stop_count > 0)
        self.assertFalse(self.robot.joint_stream_stats().active)
        self.assertIn("producer timed out", self.robot.joint_stream_stats().fault)
        self.assertEqual(self.board.enabled_mask, 127)
        self.assertEqual(self.board.disable_count, 0)

    def test_per_joint_feedback_freeze(self):
        self.start()
        self.board.frozen_feedback_mask = 4
        end = time.monotonic() + 1
        while self.robot.joint_stream_stats().active and time.monotonic() < end:
            try:
                self.robot.servej(n.JointCommand())
            except RuntimeError:
                break
            time.sleep(.01)
        self.assertIn("feedback stale", self.robot.joint_stream_stats().fault)
        self.assertEqual(self.board.disable_count, 0)

    def test_disconnect_and_record(self):
        with tempfile.TemporaryDirectory() as directory:
            self.robot.start_recording(directory)
            self.start()
            wait(lambda: self.robot.recording().telemetry_rows > 2)
            self.robot.stop()
            self.robot.disconnect()
            wait(lambda: self.board.stop_count > 0)
            self.assertEqual(self.board.enabled_mask, 127)
            self.assertEqual(self.board.disable_count, 0)

    def test_zero_stiffness_requires_explicit_opt_in(self):
        self.start(allow_zero_kp=False)
        c = n.JointCommand(); c.kp = [0]*7
        with self.assertRaises(RuntimeError):
            self.robot.servej(c)
        self.assertEqual(self.board.enabled_mask, 127)

    def test_original_one_packet_mode_and_explicit_native_start(self):
        self.assertFalse(hasattr(n, "ImpedanceSession"))
        with self.assertRaises(RuntimeError):
            self.robot.start_joint_stream(n.JointCommand())
        self.assertEqual(self.board.enabled_mask, 0)
        self.assertEqual(self.board.mode, 0)
        self.robot.set_motor_profile(n.MotorProfile.Mit)
        self.robot.servo_on(n.MotorGroup.Arm)
        self.robot.switch_mode(n.Mode.RtJoint)
        before = self.robot.link_stats().tx_sent
        self.robot.servej(n.JointCommand())
        time.sleep(.02)
        self.assertEqual(self.robot.link_stats().tx_sent, before + 1)
        self.assertFalse(self.robot.joint_stream_stats().active)
        self.assertEqual(self.robot.state().mode, n.Mode.RtJoint)

    def test_cancel_on_mode_switch_disable_disconnect(self):
        self.start()
        self.robot.switch_mode(n.Mode.Idle)
        self.assertFalse(self.robot.joint_stream_stats().active)
        count = self.robot.joint_stream_stats().sent
        time.sleep(.02)
        self.assertEqual(self.robot.joint_stream_stats().sent, count)
        self.robot.switch_mode(n.Mode.RtJoint)
        self.robot.start_joint_stream(n.JointCommand())
        self.robot.servo_off(n.MotorGroup.Gripper)
        self.assertTrue(self.robot.joint_stream_stats().active)
        self.robot.servo_off(n.MotorGroup.Arm)
        self.assertFalse(self.robot.joint_stream_stats().active)
        self.start()
        stop_count = self.board.stop_count
        self.robot.disconnect()
        count = self.robot.joint_stream_stats().sent
        time.sleep(.02)
        self.assertFalse(self.robot.joint_stream_stats().active)
        self.assertEqual(self.robot.joint_stream_stats().sent, count)
        self.assertEqual(self.board.stop_count, stop_count)  # original close-only semantics

    def test_drive_fault_stops_stream(self):
        self.start()
        self.board.fault_joint = 2
        wait(lambda: not self.robot.joint_stream_stats().active)
        self.assertIn("drive fault", self.robot.joint_stream_stats().fault)
        self.assertEqual(self.board.disable_count, 0)

    def test_original_nrt_move_and_diagnostics(self):
        self.robot.set_motor_profile(n.MotorProfile.Mit)
        self.robot.set_heartbeat(True)
        self.robot.servo_on(n.MotorGroup.Arm)
        self.robot.switch_mode(n.Mode.NrtJoint)
        options = n.MoveOptions()
        result = self.robot.movej([.2]*7, options)
        self.assertEqual(result.result, 0)
        options.velocity_max = [10]*7
        result = self.robot.movej([.25]*7, options)
        self.assertEqual(result.result, 0)
        for value in self.board.last_move_velocity():
            self.assertAlmostEqual(value, 3.14, places=5)
        self.assertTrue(self.robot.motion().has_reply)
        self.assertEqual(self.robot.motion().reply.active_id, result.motion_id)
        self.assertTrue(self.robot.link_stats().mcu_valid)
        self.assertEqual(self.robot.link_stats().mcu.control_missed, 0)
        self.assertFalse(self.robot.joint_stream_stats().active)
        self.robot.cancel_motion()

    def test_parameter_service(self):
        self.assertTrue(self.robot.info().capabilities & n.CAP_PARAMETERS)
        params = self.robot.list_parameters()
        self.assertEqual([p.name for p in params],
                         ["gravity.damping", "imp.joint_k", "adm.bias_learning", "adm.fixed"])
        self.assertEqual(params[1].unit, "Nm/rad")
        self.assertTrue(params[1].writable and not params[1].boolean)
        self.assertTrue(params[2].boolean and not params[3].writable)
        self.assertEqual(params[0].group, n.PARAM_GROUP_GRAVITY)
        joint_k = params[1].id
        self.assertAlmostEqual(self.robot.get_parameter("imp.joint_k").requested, 8.0)
        # A write shows the requested value while the applied value has not moved yet.
        result = self.robot.set_parameter("imp.joint_k", 12.0)
        self.assertEqual((result.requested, result.applied), (12.0, 8.0))
        self.assertAlmostEqual(self.board.param_requested(joint_k), 12.0)
        self.assertAlmostEqual(self.robot.get_parameter(joint_k).requested, 12.0)
        self.assertAlmostEqual(self.robot.set_parameter(joint_k, 9.5).requested, 9.5)
        # Refusals raise and leave the stored value alone.
        with self.assertRaisesRegex(RuntimeError, "超出范围"):
            self.robot.set_parameter("imp.joint_k", 31.0)
        with self.assertRaisesRegex(RuntimeError, "只读"):
            self.robot.set_parameter("adm.fixed", 2.0)
        with self.assertRaisesRegex(RuntimeError, "nope"):
            self.robot.get_parameter("nope")
        self.assertAlmostEqual(self.board.param_requested(joint_k), 9.5)
        self.board.param_not_owner = True
        with self.assertRaisesRegex(RuntimeError, "另一台 PC"):
            self.robot.set_parameter("imp.joint_k", 10.0)
        self.assertAlmostEqual(self.robot.get_parameter("imp.joint_k").requested, 9.5)
        self.board.param_not_owner = False
        # Firmware without the service: refused before anything is sent.
        self.robot.disconnect()
        self.board.supports_parameters = False
        self.robot.connect()
        sent = self.board.param_requests
        with self.assertRaisesRegex(RuntimeError, "INFRABOT_CAP_PARAMETERS"):
            self.robot.list_parameters()
        self.assertEqual(self.board.param_requests, sent)

    def test_diagnostics(self):
        self.assertTrue(self.robot.info().capabilities & n.CAP_DIAGNOSTICS)
        d = self.robot.diagnostics()
        self.assertEqual((d.uptime_ms, d.control_hz, d.control_period_max_us), (123456, 1000, 1042))
        self.assertEqual((d.can_tec, d.can_rec, d.can_error_passive, d.can_bus_off), (9, 10, True, False))
        self.assertEqual((d.heap_free_bytes, d.heap_min_free_bytes), (20000, 15000))
        self.assertEqual(d.stack_free_ethernet_bytes, 600)
        self.assertEqual(len(d.drive_state), 8)
        # Read only: a PC that does not control the arm still gets the snapshot.
        self.board.param_not_owner = True
        self.assertEqual(self.robot.diagnostics().command_invalid_total, 7)
        self.board.param_not_owner = False
        self.robot.disconnect()
        self.board.supports_diagnostics = False
        self.robot.connect()
        sent = self.board.diag_requests
        with self.assertRaisesRegex(RuntimeError, "INFRABOT_CAP_DIAGNOSTICS"):
            self.robot.diagnostics()
        self.assertEqual(self.board.diag_requests, sent)


if __name__ == "__main__":
    unittest.main()
