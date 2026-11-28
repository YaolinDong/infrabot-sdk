/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include <pybind11/pybind11.h>
#include <pybind11/functional.h>
#include <pybind11/stl.h>
#include "infrabot/robot.hpp"
#include "infrabot/firmware_update.hpp"
#include "docs.hpp"
#include <stdexcept>
namespace py = pybind11;
using namespace infrabot;
namespace {
void require(bool ok, const std::string& error) { if (!ok) throw std::runtime_error(error); }
using Unlock = py::call_guard<py::gil_scoped_release>;
}
PYBIND11_MODULE(infrabot_native, m) {
    m.def("rollback_firmware", [](const std::string& ip,const std::string& key){infrabot::rollback_firmware(ip,key);},pydoc::rollback_firmware,py::arg("ip"),py::arg("public_key_file")="",Unlock{});
    m.def("firmware_status", [](const std::string& ip,const std::string& key){return infrabot::firmware_status(ip,key);},pydoc::firmware_status, py::arg("ip"),py::arg("public_key_file")="",Unlock{});
    m.def("update_firmware", [](const std::string& ip,const std::string& package,const std::string& key){
        return infrabot::update_firmware(ip,package,key);
    },pydoc::update_firmware,py::arg("ip"),py::arg("package"),py::arg("public_key_file")="", Unlock{});
    m.doc() = pydoc::module;
    py::enum_<Mode>(m, "Mode",pydoc::Mode).value("Idle",Mode::Idle).value("RtJoint",Mode::RtJoint)
        .value("RtCartesian",Mode::RtCartesian).value("NrtJoint",Mode::NrtJoint).value("NrtCartesian",Mode::NrtCartesian)
        .value("GravityFree",Mode::GravityFree).value("GravityLock",Mode::GravityLock)
        .value("JointImpedance",Mode::JointImpedance).value("CartesianImpedance",Mode::CartesianImpedance)
        .value("JointAdmittance",Mode::JointAdmittance).value("CartesianAdmittance",Mode::CartesianAdmittance).value("Unknown",Mode::Unknown);
    py::enum_<MotorProfile>(m,"MotorProfile",pydoc::MotorProfile).value("Mit",MotorProfile::Mit).value("PositionVelocity",MotorProfile::PositionVelocity);
    py::enum_<MotorGroup>(m,"MotorGroup",pydoc::MotorGroup).value("All",MotorGroup::All).value("Arm",MotorGroup::Arm).value("Gripper",MotorGroup::Gripper);
    py::enum_<OperationalStatus>(m,"OperationalStatus",pydoc::OperationalStatus)
        .value("Booting",OperationalStatus::Booting).value("Precheck",OperationalStatus::Precheck)
        .value("ProfileRequired",OperationalStatus::ProfileRequired).value("Disabled",OperationalStatus::Disabled)
        .value("Enabling",OperationalStatus::Enabling).value("IdleHold",OperationalStatus::IdleHold)
        .value("RtJoint",OperationalStatus::RtJoint).value("RtCartesian",OperationalStatus::RtCartesian)
        .value("NrtJoint",OperationalStatus::NrtJoint).value("NrtCartesian",OperationalStatus::NrtCartesian)
        .value("GravityFree",OperationalStatus::GravityFree).value("GravityLock",OperationalStatus::GravityLock)
        .value("JointImpedance",OperationalStatus::JointImpedance).value("CartesianImpedance",OperationalStatus::CartesianImpedance)
        .value("JointAdmittance",OperationalStatus::JointAdmittance).value("CartesianAdmittance",OperationalStatus::CartesianAdmittance)
        .value("Stopping",OperationalStatus::Stopping).value("Fault",OperationalStatus::Fault)
        .value("Recovery",OperationalStatus::Recovery).value("Estop",OperationalStatus::Estop).value("Unknown",OperationalStatus::Unknown);
    py::enum_<StopReason>(m,"StopReason",pydoc::StopReason).value("None_",StopReason::None).value("StreamLost",StopReason::StreamLost)
        .value("MoveLost",StopReason::MoveLost).value("MoveFailed",StopReason::MoveFailed).value("Requested",StopReason::Requested)
        .value("NotFollowing",StopReason::NotFollowing).value("InvalidTarget",StopReason::InvalidTarget)
        .value("FeedbackLost",StopReason::FeedbackLost).value("DriveFault",StopReason::DriveFault).value("TorqueLimit",StopReason::TorqueLimit);
    py::enum_<StopPhase>(m,"StopPhase",pydoc::StopPhase).value("None_",StopPhase::None).value("Braking",StopPhase::Braking).value("Holding",StopPhase::Holding);
    py::enum_<SelfCheck>(m,"SelfCheck",pydoc::SelfCheck).value("NotRun",SelfCheck::NotRun).value("Ok",SelfCheck::Ok)
        .value("Mismatch",SelfCheck::Mismatch).value("NoReply",SelfCheck::NoReply);
#define RW(T,F) .def_readwrite(#F,&T::F,pydoc::field::PY::F)
#define RO(T,F) .def_readonly(#F,&T::F,pydoc::field::PY::F)
#undef PY
#define PY JointStreamOptions
    py::class_<JointStreamOptions>(m,"JointStreamOptions",pydoc::JointStreamOptions).def(py::init<>())
        RW(JointStreamOptions,transmit_hz) RW(JointStreamOptions,command_timeout_ms)
        RW(JointStreamOptions,feedback_timeout_ms) RW(JointStreamOptions,allow_zero_kp);
#undef PY
#define PY JointStreamStats
    py::class_<JointStreamStats>(m,"JointStreamStats",pydoc::JointStreamStats) RO(JointStreamStats,active) RO(JointStreamStats,sent)
        RO(JointStreamStats,skipped_deadlines) RO(JointStreamStats,fault);
#undef PY
#define PY JointCommand
    py::class_<JointCommand>(m,"JointCommand",pydoc::JointCommand).def(py::init<>()) RW(JointCommand,position) RW(JointCommand,velocity)
        RW(JointCommand,kp) RW(JointCommand,kd) RW(JointCommand,torque) RW(JointCommand,velocity_limit);
#undef PY
#define PY JointState
    py::class_<JointState>(m,"JointState",pydoc::JointState) RO(JointState,position) RO(JointState,velocity) RO(JointState,torque)
        RO(JointState,temperature) RO(JointState,temperature_coil) RO(JointState,feedback_count) RO(JointState,feedback_age_ms)
        RO(JointState,drive_state) RO(JointState,motor_flags) RO(JointState,drive_mode) RO(JointState,has_command)
        RO(JointState,command_position) RO(JointState,command_velocity) RO(JointState,command_torque)
        RO(JointState,sequence) RO(JointState,timestamp_us) RO(JointState,host_time_us);
#undef PY
#define PY SafetyState
    py::class_<SafetyState>(m,"SafetyState",pydoc::SafetyState) RO(SafetyState,valid) RO(SafetyState,stop_reason) RO(SafetyState,stop_phase)
        RO(SafetyState,stop_joint) RO(SafetyState,stop_detail) RO(SafetyState,stop_mode) RO(SafetyState,selfcheck)
        RO(SafetyState,selfcheck_joint) RO(SafetyState,selfcheck_register) RO(SafetyState,selfcheck_value) RO(SafetyState,selfcheck_expected);
#undef PY
#define PY RobotState
    py::class_<RobotState>(m,"RobotState",pydoc::RobotState) RO(RobotState,connected) RO(RobotState,enabled_mask) RO(RobotState,enabled)
        RO(RobotState,mode) RO(RobotState,profile) RO(RobotState,profile_ready) RO(RobotState,modes_supported)
        RO(RobotState,zero_valid) RO(RobotState,telemetry_age_ms) RO(RobotState,telemetry_epoch) RO(RobotState,telemetry_version)
        RO(RobotState,joints) RO(RobotState,safety) RO(RobotState,operational_status) RO(RobotState,controller_state)
        RO(RobotState,command_active) RO(RobotState,command_applied_sequence);
#undef PY
#define PY RobotInfo
    py::class_<RobotInfo>(m,"RobotInfo",pydoc::RobotInfo) RO(RobotInfo,received) RO(RobotInfo,firmware) RO(RobotInfo,product)
        RO(RobotInfo,axes) RO(RobotInfo,motor_count) RO(RobotInfo,capabilities) RO(RobotInfo,protocol_version)
        RO(RobotInfo,protocol_major) RO(RobotInfo,protocol_minor) RO(RobotInfo,board) RO(RobotInfo,arm_model)
        RO(RobotInfo,torque_max) RO(RobotInfo,velocity_max) RO(RobotInfo,position_min) RO(RobotInfo,position_max)
        RO(RobotInfo,profile_ready) RO(RobotInfo,enabled_mask) RO(RobotInfo,control_mode) RO(RobotInfo,motor_profile)
        RO(RobotInfo,controller_status);
#define PACKED_RO(T,F) .def_property_readonly(#F,[](const T& x){return x.F;},pydoc::field::PY::F)
#undef PY
#define PY ControllerLinkStats
    py::class_<InfrabotLinkStats>(m,"ControllerLinkStats",pydoc::ControllerLinkStats)
        PACKED_RO(InfrabotLinkStats,command_rx) PACKED_RO(InfrabotLinkStats,command_invalid)
        PACKED_RO(InfrabotLinkStats,command_interval_max_us) PACKED_RO(InfrabotLinkStats,command_jitter_max_us)
        PACKED_RO(InfrabotLinkStats,control_missed) PACKED_RO(InfrabotLinkStats,control_period_max_us)
        PACKED_RO(InfrabotLinkStats,can_tx_errors) PACKED_RO(InfrabotLinkStats,telemetry_errors) PACKED_RO(InfrabotLinkStats,invalid_reason);
#undef PACKED_RO
#undef PY
#define PY LinkStats
    py::class_<LinkStats>(m,"LinkStats",pydoc::LinkStats) RO(LinkStats,tx_sent) RO(LinkStats,tx_errors) RO(LinkStats,rx_telemetry)
        RO(LinkStats,rx_lost) RO(LinkStats,rx_invalid) RO(LinkStats,rx_last_second) RO(LinkStats,lost_last_second)
        RO(LinkStats,loss_percent) RO(LinkStats,rx_interval_max_us) RO(LinkStats,mcu_valid) RO(LinkStats,mcu);
    py::enum_<FaultEvent::Kind>(m,"FaultKind",pydoc::FaultKind).value("DriveFault",FaultEvent::Kind::DriveFault)
        .value("DriveCleared",FaultEvent::Kind::DriveCleared).value("Stop",FaultEvent::Kind::Stop)
        .value("SelfCheck",FaultEvent::Kind::SelfCheck).value("Link",FaultEvent::Kind::Link);
#undef PY
#define PY FaultEvent
    py::class_<FaultEvent>(m,"FaultEvent",pydoc::FaultEvent) RO(FaultEvent,kind) RO(FaultEvent,host_time_us) RO(FaultEvent,mcu_time_us)
        RO(FaultEvent,joint) RO(FaultEvent,code) RO(FaultEvent,detail) RO(FaultEvent,message);
#undef PY
#define PY RecordOptions
    py::class_<RecordOptions>(m,"RecordOptions",pydoc::RecordOptions).def(py::init<>()) RW(RecordOptions,label) RW(RecordOptions,rotate_seconds) RW(RecordOptions,commands);
#undef PY
#define PY RecorderStats
    py::class_<RecorderStats>(m,"RecorderStats",pydoc::RecorderStats) RO(RecorderStats,active) RO(RecorderStats,directory)
        RO(RecorderStats,telemetry_rows) RO(RecorderStats,command_rows) RO(RecorderStats,events)
        RO(RecorderStats,dropped) RO(RecorderStats,file_index) RO(RecorderStats,error);
#undef PY
#define PY SerialLogOptions
    py::class_<SerialLogOptions>(m,"SerialLogOptions",pydoc::SerialLogOptions).def(py::init<>()) RW(SerialLogOptions,port) RW(SerialLogOptions,baud) RW(SerialLogOptions,directory);
#undef PY
#define PY SerialLogStats
    py::class_<SerialLogStats>(m,"SerialLogStats",pydoc::SerialLogStats) RO(SerialLogStats,active) RO(SerialLogStats,path)
        RO(SerialLogStats,lines) RO(SerialLogStats,bytes) RO(SerialLogStats,reconnects)
        RO(SerialLogStats,port_open) RO(SerialLogStats,error);
#undef PY
#define PY McuLogOptions
    py::class_<McuLogOptions>(m,"McuLogOptions",pydoc::McuLogOptions).def(py::init<>()) RW(McuLogOptions,directory) RW(McuLogOptions,label)
        RW(McuLogOptions,port) RW(McuLogOptions,timeout_ms) RW(McuLogOptions,retries) RW(McuLogOptions,progress);
#undef PY
#define PY McuLogResult
    py::class_<McuLogResult>(m,"McuLogResult",pydoc::McuLogResult) RO(McuLogResult,text_path) RO(McuLogResult,telemetry_path)
        RO(McuLogResult,boot_id) RO(McuLogResult,text_lines) RO(McuLogResult,telemetry_rows)
        RO(McuLogResult,lost_bytes) RO(McuLogResult,writer_max_us) RO(McuLogResult,fault_latched);
#undef PY
#define PY CartesianLimits
    py::class_<CartesianLimits>(m,"CartesianLimits",pydoc::CartesianLimits).def(py::init<>()) RW(CartesianLimits,linear_velocity)
        RW(CartesianLimits,angular_velocity) RW(CartesianLimits,linear_acceleration) RW(CartesianLimits,angular_acceleration);
#undef PY
#define PY CartesianCommand
    py::class_<CartesianCommand>(m,"CartesianCommand",pydoc::CartesianCommand).def(py::init<>()) RW(CartesianCommand,pose) RW(CartesianCommand,limits)
        RW(CartesianCommand,kp) RW(CartesianCommand,kd) RW(CartesianCommand,torque) RW(CartesianCommand,velocity_limit);
#undef PY
#define PY MoveOptions
    py::class_<MoveOptions>(m,"MoveOptions",pydoc::MoveOptions).def(py::init<>()) RW(MoveOptions,timeout_ms) RW(MoveOptions,dwell_ms)
        RW(MoveOptions,position_tolerance) RW(MoveOptions,velocity_tolerance) RW(MoveOptions,velocity_max)
        RW(MoveOptions,acceleration_max) RW(MoveOptions,kp) RW(MoveOptions,kd) RW(MoveOptions,torque);
#undef PY
#define PY CartesianMoveOptions
    py::class_<CartesianMoveOptions>(m,"CartesianMoveOptions",pydoc::CartesianMoveOptions).def(py::init<>()) RW(CartesianMoveOptions,timeout_ms)
        RW(CartesianMoveOptions,dwell_ms) RW(CartesianMoveOptions,position_tolerance) RW(CartesianMoveOptions,velocity_tolerance)
        RW(CartesianMoveOptions,limits) RW(CartesianMoveOptions,velocity_max) RW(CartesianMoveOptions,acceleration_max)
        RW(CartesianMoveOptions,kp) RW(CartesianMoveOptions,kd);
#undef PY
#define PY MotionResult
    py::class_<MotionResult>(m,"MotionResult",pydoc::MotionResult) RO(MotionResult,result) RO(MotionResult,state) RO(MotionResult,motion_id)
        RO(MotionResult,arrived_mask) RO(MotionResult,fresh_mask) RO(MotionResult,max_error_rad) RO(MotionResult,reason) RO(MotionResult,message);
    py::enum_<MotionStatus::Phase>(m,"MotionPhase",pydoc::MotionPhase).value("Off",MotionStatus::Phase::Off)
        .value("Preparing",MotionStatus::Phase::Preparing).value("Waiting",MotionStatus::Phase::Waiting)
        .value("Running",MotionStatus::Phase::Running).value("Settling",MotionStatus::Phase::Settling)
        .value("Ready",MotionStatus::Phase::Ready).value("Done",MotionStatus::Phase::Done)
        .value("Cancelling",MotionStatus::Phase::Cancelling).value("Cancelled",MotionStatus::Phase::Cancelled).value("Error",MotionStatus::Phase::Error);
#undef PY
#define PY MoveReply
    py::class_<InfrabotMoveReply>(m,"MoveReply",pydoc::MoveReply) RO(InfrabotMoveReply,result) RO(InfrabotMoveReply,state)
        RO(InfrabotMoveReply,session) RO(InfrabotMoveReply,request_id) RO(InfrabotMoveReply,active_id)
        RO(InfrabotMoveReply,elapsed_ms) RO(InfrabotMoveReply,duration_ms) RO(InfrabotMoveReply,arrived_mask)
        RO(InfrabotMoveReply,enabled_mask) RO(InfrabotMoveReply,fresh_mask) RO(InfrabotMoveReply,max_error_rad) RO(InfrabotMoveReply,reason);
#undef PY
#define PY MotionStatus
    py::class_<MotionStatus>(m,"MotionStatus",pydoc::MotionStatus) RO(MotionStatus,phase) RO(MotionStatus,id) RO(MotionStatus,can_execute)
        RO(MotionStatus,has_reply) RO(MotionStatus,reply_age_us) RO(MotionStatus,error) RO(MotionStatus,reply);
#undef PY
#define PY ParameterInfo
    py::class_<ParameterInfo>(m,"ParameterInfo",pydoc::ParameterInfo) RO(ParameterInfo,id) RO(ParameterInfo,group) RO(ParameterInfo,name) RO(ParameterInfo,unit)
        RO(ParameterInfo,requested) RO(ParameterInfo,applied) RO(ParameterInfo,minimum) RO(ParameterInfo,maximum)
        RO(ParameterInfo,default_value) RO(ParameterInfo,rate) RO(ParameterInfo,writable) RO(ParameterInfo,boolean);
#undef PY
#define PY DiagnosticsInfo
    py::class_<DiagnosticsInfo>(m,"DiagnosticsInfo",pydoc::DiagnosticsInfo) RO(DiagnosticsInfo,uptime_ms) RO(DiagnosticsInfo,snapshot_seconds)
        RO(DiagnosticsInfo,control_hz) RO(DiagnosticsInfo,control_missed) RO(DiagnosticsInfo,control_period_max_us)
        RO(DiagnosticsInfo,can_tx) RO(DiagnosticsInfo,can_rx) RO(DiagnosticsInfo,can_tx_errors)
        RO(DiagnosticsInfo,command_rx) RO(DiagnosticsInfo,command_invalid) RO(DiagnosticsInfo,telemetry_tx)
        RO(DiagnosticsInfo,can_error_events) RO(DiagnosticsInfo,control_missed_total) RO(DiagnosticsInfo,can_tx_errors_total)
        RO(DiagnosticsInfo,command_invalid_total) RO(DiagnosticsInfo,can_tec) RO(DiagnosticsInfo,can_rec)
        RO(DiagnosticsInfo,can_error_passive) RO(DiagnosticsInfo,can_bus_off) RO(DiagnosticsInfo,heap_free_bytes)
        RO(DiagnosticsInfo,heap_min_free_bytes) RO(DiagnosticsInfo,stack_free_control_bytes) RO(DiagnosticsInfo,stack_free_status_bytes)
        RO(DiagnosticsInfo,stack_free_ethernet_bytes) RO(DiagnosticsInfo,drive_state) RO(DiagnosticsInfo,controller_status)
        RO(DiagnosticsInfo,invalid_reason);
#undef RO
#undef RW
#undef PY
    py::class_<Robot>(m,"Robot",pydoc::Robot)
        .def(py::init<std::string,uint16_t,uint16_t>(),pydoc::robot::__init__,py::arg("address")="192.168.0.50",py::arg("command_port")=5001,py::arg("telemetry_port")=5000)
        .def("connect",[](Robot& r){std::string e;require(r.connect(e),e);},pydoc::robot::connect,Unlock())
        .def("disconnect",&Robot::disconnect,pydoc::robot::disconnect,Unlock()).def("connected",&Robot::connected,pydoc::robot::connected,Unlock())
        .def("info",&Robot::info,pydoc::robot::info,Unlock()).def("state",&Robot::state,pydoc::robot::state,Unlock())
        .def("link_stats",&Robot::link_stats,pydoc::robot::link_stats,Unlock()).def("fault_log",&Robot::fault_log,pydoc::robot::fault_log,Unlock())
        .def("set_heartbeat",&Robot::set_heartbeat,pydoc::robot::set_heartbeat,py::arg("enabled"),Unlock())
        .def("set_motor_profile",[](Robot& r,MotorProfile p){std::string e;require(r.set_motor_profile(p,e),e);},pydoc::robot::set_motor_profile,py::arg("profile"),Unlock())
        .def("set_zero",[](Robot& r,bool enable){std::string e;require(r.set_zero(enable,e),e);},pydoc::robot::set_zero,py::arg("enable_after")=false,Unlock())
        .def("servo_on",[](Robot& r,MotorGroup g){std::string e;require(r.servo_on(g,e),e);},pydoc::robot::servo_on,py::arg_v("group", MotorGroup::All, "MotorGroup.All"),Unlock())
        .def("servo_off",[](Robot& r,MotorGroup g,bool wait){std::string e;require(r.servo_off(g,e,wait),e);},pydoc::robot::servo_off,py::arg_v("group", MotorGroup::All, "MotorGroup.All"),py::arg("wait")=true,Unlock())
        .def("stop",[](Robot& r,bool wait){std::string e;require(r.stop(e,wait),e);},pydoc::robot::stop,py::arg("wait")=true,Unlock())
        .def("resume",[](Robot& r){std::string e;Mode mode;require(r.resume(e,&mode),e);return mode;},pydoc::robot::resume,Unlock())
        .def("clear_fault",[](Robot& r,uint32_t t){std::string e;require(r.clear_fault(e,t),e);},pydoc::robot::clear_fault,py::arg("timeout_ms")=1000,Unlock())
        .def("switch_mode",[](Robot& r,Mode m){std::string e;require(r.switch_mode(m,e),e);},pydoc::robot::switch_mode,py::arg("mode"),Unlock())
        .def("hold_command",&Robot::hold_command,pydoc::robot::hold_command,Unlock())
        .def("read_hold_command",[](Robot& r){std::string e;std::array<float,8> q;require(r.read_hold_command(q,e),e);return q;},pydoc::robot::read_hold_command,Unlock())
        .def("servej",[](Robot& r,const JointCommand& c){std::string e;require(r.servej(c,e),e);},pydoc::robot::servej,py::arg("command"),Unlock())
        .def("servep",[](Robot& r,const CartesianCommand& c){std::string e;require(r.servep(c,e),e);},pydoc::robot::servep,py::arg("command"),Unlock())
        .def("start_joint_stream",[](Robot& r,const JointCommand& c,const JointStreamOptions& o){std::string e;require(r.start_joint_stream(c,o,e),e);},pydoc::robot::start_joint_stream,py::arg("initial"),py::arg_v("options", JointStreamOptions{}, "JointStreamOptions()"),Unlock())
        .def("stop_joint_stream",&Robot::stop_joint_stream,pydoc::robot::stop_joint_stream,Unlock())
        .def("joint_stream_stats",&Robot::joint_stream_stats,pydoc::robot::joint_stream_stats,Unlock())
        .def("movej",[](Robot& r,const std::array<float,7>& q,const MoveOptions& o){std::string e;auto v=r.movej(q,o,e);if(!e.empty())throw std::runtime_error(e);return v;},pydoc::robot::movej,py::arg("position"),py::arg_v("options", MoveOptions{}, "MoveOptions()"),Unlock())
        .def("movep",[](Robot& r,const Pose& p,const CartesianMoveOptions& o){std::string e;auto v=r.movep(p,o,e);if(!e.empty())throw std::runtime_error(e);return v;},pydoc::robot::movep,py::arg("pose"),py::arg_v("options", CartesianMoveOptions{}, "CartesianMoveOptions()"),Unlock())
        .def("submit_movej",[](Robot& r,const std::array<float,7>& q,const MoveOptions& o){std::string e;require(r.submit_movej(q,o,e),e);},pydoc::robot::submit_movej,py::arg("position"),py::arg_v("options", MoveOptions{}, "MoveOptions()"),Unlock())
        .def("submit_movep",[](Robot& r,const Pose& p,const CartesianMoveOptions& o){std::string e;require(r.submit_movep(p,o,e),e);},pydoc::robot::submit_movep,py::arg("pose"),py::arg_v("options", CartesianMoveOptions{}, "CartesianMoveOptions()"),Unlock())
        .def("motion",&Robot::motion,pydoc::robot::motion,Unlock())
        .def("wait_motion",[](Robot& r,uint32_t t){std::string e;MotionResult v;require(r.wait_motion(t,v,e),e);return v;},pydoc::robot::wait_motion,py::arg("timeout_ms"),Unlock())
        .def("cancel_motion",[](Robot& r){std::string e;require(r.cancel_motion(e),e);},pydoc::robot::cancel_motion,Unlock())
        .def("set_default_move_options",&Robot::set_default_move_options,pydoc::robot::set_default_move_options,py::arg("options"),Unlock())
        .def("set_default_cartesian_options",&Robot::set_default_cartesian_options,pydoc::robot::set_default_cartesian_options,py::arg("options"),Unlock())
        .def("gripper",[](Robot& r,float p,float v,float f){std::string e;require(r.gripper(p,v,f,e),e);},pydoc::robot::gripper,py::arg("position_rad"),py::arg("speed_rad_s"),py::arg("force_ratio"),Unlock())
        .def("start_recording",[](Robot& r,const std::string& dir,const RecordOptions& o){std::string e;require(r.start_recording(dir,e,o),e);},pydoc::robot::start_recording,py::arg("directory"),py::arg_v("options", RecordOptions{}, "RecordOptions()"),Unlock())
        .def("stop_recording",&Robot::stop_recording,pydoc::robot::stop_recording,Unlock()).def("recording",&Robot::recording,pydoc::robot::recording,Unlock())
        .def("record_event",&Robot::record_event,pydoc::robot::record_event,py::arg("text"),Unlock())
        .def("start_serial_log",[](Robot& r,const SerialLogOptions& o){std::string e;require(r.start_serial_log(e,o),e);return r.serial_log().path;},pydoc::robot::start_serial_log,
             py::arg_v("options", SerialLogOptions{}, "SerialLogOptions()"),Unlock())
        .def("stop_serial_log",&Robot::stop_serial_log,pydoc::robot::stop_serial_log,Unlock()).def("serial_log",&Robot::serial_log,pydoc::robot::serial_log,Unlock())
        .def_static("log_directory",&Robot::log_directory,pydoc::robot::log_directory)
        .def("download_mcu_log",[](Robot& r,const McuLogOptions& o){std::string e;McuLogResult v;require(r.download_mcu_log(v,e,o),e);return v;},pydoc::robot::download_mcu_log,
             py::arg_v("options", McuLogOptions{}, "McuLogOptions()"),Unlock())
        .def("request_feedback",[](Robot& r,uint8_t i){std::string e;require(r.request_feedback(i,e),e);},pydoc::robot::request_feedback,py::arg("motor"),Unlock())
        .def("get_state",[](Robot& r,uint8_t i){std::string e;require(r.get_state(i,e),e);},pydoc::robot::get_state,py::arg("motor"),Unlock())
        .def("get_register",[](Robot& r,uint8_t i,uint8_t rid,uint8_t type){std::string e;uint32_t v;require(r.get_register(i,rid,type,v,e),e);return v;},pydoc::robot::get_register,py::arg("motor"),py::arg("register_id"),py::arg("value_type"),Unlock())
        .def("write_register",[](Robot& r,uint8_t i,uint8_t rid,uint32_t v,uint8_t type){std::string e;require(r.write_register(i,rid,v,type,e),e);},pydoc::robot::write_register,py::arg("motor"),py::arg("register_id"),py::arg("value"),py::arg("value_type"),Unlock())
        .def("enable",[](Robot& r,uint8_t i){std::string e;require(r.enable(i,e),e);},pydoc::robot::enable,py::arg("motor"),Unlock())
        .def("disable",[](Robot& r,uint8_t i){std::string e;require(r.disable(i,e),e);},pydoc::robot::disable,py::arg("motor"),Unlock())
        .def("clear_error",[](Robot& r,uint8_t i){std::string e;require(r.clear_error(i,e),e);},pydoc::robot::clear_error,py::arg("motor"),Unlock())
        .def("set_zero_position",[](Robot& r,uint8_t i){std::string e;require(r.set_zero_position(i,e),e);},pydoc::robot::set_zero_position,py::arg("motor"),Unlock())
        .def("ensure_mode",[](Robot& r,uint8_t i,uint8_t m){std::string e;require(r.ensure_mode(i,m,e),e);},pydoc::robot::ensure_mode,py::arg("motor"),py::arg("mode"),Unlock())
        .def("send_mit",[](Robot& r,uint8_t i,float p,float v,float kp,float kd,float t){std::string e;require(r.send_mit(i,p,v,kp,kd,t,e),e);},pydoc::robot::send_mit,py::arg("motor"),py::arg("position_rad"),py::arg("velocity_rad_s"),py::arg("kp"),py::arg("kd"),py::arg("torque_nm"),Unlock())
        .def("send_pos_vel",[](Robot& r,uint8_t i,float p,float v){std::string e;require(r.send_pos_vel(i,p,v,e),e);},pydoc::robot::send_pos_vel,py::arg("motor"),py::arg("position_rad"),py::arg("velocity_limit_rad_s"),Unlock())
        .def("send_vel",[](Robot& r,uint8_t i,float v){std::string e;require(r.send_vel(i,v,e),e);},pydoc::robot::send_vel,py::arg("motor"),py::arg("velocity_rad_s"),Unlock())
        .def("send_force_pos",[](Robot& r,uint8_t i,float p,float v,float f){std::string e;require(r.send_force_pos(i,p,v,f,e),e);},pydoc::robot::send_force_pos,py::arg("motor"),py::arg("position_rad"),py::arg("velocity_limit_rad_s"),py::arg("torque_ratio"),Unlock())
        .def("set_can_timeout_ms",[](Robot& r,uint8_t i,uint32_t t){std::string e;require(r.set_can_timeout_ms(i,t,e),e);},pydoc::robot::set_can_timeout_ms,py::arg("motor"),py::arg("timeout_ms"),Unlock())
        .def("store_parameters",[](Robot& r,uint8_t i){std::string e;require(r.store_parameters(i,e),e);},pydoc::robot::store_parameters,py::arg("motor"),Unlock())
        .def("list_parameters",[](Robot& r){std::string e;std::vector<ParameterInfo> v;require(r.list_parameters(v,e),e);return v;},pydoc::robot::list_parameters,Unlock())
        .def("get_parameter",[](Robot& r,uint16_t id){std::string e;ParameterInfo p;require(r.get_parameter(id,p,e),e);return p;},pydoc::robot::get_parameter_id,py::arg("id"),Unlock())
        .def("get_parameter",[](Robot& r,const std::string& name){std::string e;ParameterInfo p;require(r.get_parameter(name,p,e),e);return p;},pydoc::robot::get_parameter_name,py::arg("name"),Unlock())
        .def("set_parameter",[](Robot& r,uint16_t id,float value){std::string e;ParameterInfo p;require(r.set_parameter(id,value,p,e),e);return p;},pydoc::robot::set_parameter_id,py::arg("id"),py::arg("value"),Unlock())
        .def("set_parameter",[](Robot& r,const std::string& name,float value){std::string e;ParameterInfo p;require(r.set_parameter(name,value,p,e),e);return p;},pydoc::robot::set_parameter_name,py::arg("name"),py::arg("value"),Unlock())
        .def("diagnostics",[](Robot& r){std::string e;DiagnosticsInfo d;require(r.diagnostics(d,e),e);return d;},pydoc::robot::diagnostics,Unlock())
        .def_static("forward",&Robot::forward,pydoc::robot::forward,py::arg("joints"),Unlock());
    m.attr("ARM_MASK") = INFRABOT_ARM_MASK;
    m.attr("CAP_JOINT_RT") = INFRABOT_CAP_JOINT_RT;
    m.attr("CAP_GROUPS") = INFRABOT_CAP_GROUPS;
    m.attr("CAP_TELEMETRY_V3") = INFRABOT_CAP_TELEMETRY_V3;
    m.attr("CAP_CONTROLLED_STOP") = INFRABOT_CAP_CONTROLLED_STOP;
    m.attr("MOTOR_FEEDBACK_STALE") = INFRABOT_MOTOR_FEEDBACK_STALE;
    m.attr("CAP_ADMITTANCE")=py::int_(INFRABOT_CAP_ADMITTANCE);
    m.attr("CAP_LOCAL_GRAVITY") = INFRABOT_CAP_LOCAL_GRAVITY;
    m.attr("CAP_PARAMETERS") = INFRABOT_CAP_PARAMETERS;
    m.attr("CAP_DIAGNOSTICS") = INFRABOT_CAP_DIAGNOSTICS;
    m.attr("PARAM_GROUP_GRAVITY") = INFRABOT_PARAM_GROUP_GRAVITY;
    m.attr("PARAM_GROUP_IMPEDANCE") = INFRABOT_PARAM_GROUP_IMPEDANCE;
    m.attr("PARAM_GROUP_ADMITTANCE") = INFRABOT_PARAM_GROUP_ADMITTANCE;
    m.attr("ARM_MAX_JOINT_VELOCITY") = INFRABOT_ARM_MAX_JOINT_VELOCITY;
    m.attr("REGISTER_U32") = INFRABOT_REGISTER_U32;
    m.attr("REGISTER_F32") = INFRABOT_REGISTER_F32;
}
