/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include <pybind11/pybind11.h>
#include <pybind11/functional.h>
#include <pybind11/stl.h>
#include "infrabot/robot.hpp"
#include "infrabot/firmware_update.hpp"
#include <stdexcept>
namespace py = pybind11;
using namespace infrabot;
namespace {
void require(bool ok, const std::string& error) { if (!ok) throw std::runtime_error(error); }
using Unlock = py::call_guard<py::gil_scoped_release>;
}
PYBIND11_MODULE(infrabot_native, m) {
    m.def("rollback_firmware", [](const std::string& ip,const std::string& key){infrabot::rollback_firmware(ip,key);},py::arg("ip"),py::arg("public_key_file")="",Unlock{});
    m.def("firmware_status", [](const std::string& ip,const std::string& key){return infrabot::firmware_status(ip,key);}, py::arg("ip"),py::arg("public_key_file")="",Unlock{});
    m.def("update_firmware", [](const std::string& ip,const std::string& package,const std::string& key){
        return infrabot::update_firmware(ip,package,key);
    },py::arg("ip"),py::arg("package"),py::arg("public_key_file")="", Unlock{});
    m.doc() = "Bindings for the existing infrabot::Robot C++ SDK (joint SI units)";
    py::enum_<Mode>(m, "Mode").value("Idle",Mode::Idle).value("RtJoint",Mode::RtJoint)
        .value("RtCartesian",Mode::RtCartesian).value("NrtJoint",Mode::NrtJoint).value("NrtCartesian",Mode::NrtCartesian)
        .value("GravityFree",Mode::GravityFree).value("GravityLock",Mode::GravityLock)
        .value("JointImpedance",Mode::JointImpedance).value("CartesianImpedance",Mode::CartesianImpedance)
        .value("JointAdmittance",Mode::JointAdmittance).value("CartesianAdmittance",Mode::CartesianAdmittance).value("Unknown",Mode::Unknown);
    py::enum_<MotorProfile>(m,"MotorProfile").value("Mit",MotorProfile::Mit).value("PositionVelocity",MotorProfile::PositionVelocity);
    py::enum_<MotorGroup>(m,"MotorGroup").value("All",MotorGroup::All).value("Arm",MotorGroup::Arm).value("Gripper",MotorGroup::Gripper);
    py::enum_<OperationalStatus>(m,"OperationalStatus")
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
    py::enum_<StopReason>(m,"StopReason").value("None_",StopReason::None).value("StreamLost",StopReason::StreamLost)
        .value("MoveLost",StopReason::MoveLost).value("MoveFailed",StopReason::MoveFailed).value("Requested",StopReason::Requested)
        .value("NotFollowing",StopReason::NotFollowing).value("InvalidTarget",StopReason::InvalidTarget)
        .value("FeedbackLost",StopReason::FeedbackLost).value("DriveFault",StopReason::DriveFault).value("TorqueLimit",StopReason::TorqueLimit);
    py::enum_<StopPhase>(m,"StopPhase").value("None_",StopPhase::None).value("Braking",StopPhase::Braking).value("Holding",StopPhase::Holding);
    py::enum_<SelfCheck>(m,"SelfCheck").value("NotRun",SelfCheck::NotRun).value("Ok",SelfCheck::Ok)
        .value("Mismatch",SelfCheck::Mismatch).value("NoReply",SelfCheck::NoReply);
#define RW(T,F) .def_readwrite(#F,&T::F)
#define RO(T,F) .def_readonly(#F,&T::F)
    py::class_<JointStreamOptions>(m,"JointStreamOptions").def(py::init<>())
        RW(JointStreamOptions,transmit_hz) RW(JointStreamOptions,command_timeout_ms)
        RW(JointStreamOptions,feedback_timeout_ms) RW(JointStreamOptions,allow_zero_kp);
    py::class_<JointStreamStats>(m,"JointStreamStats") RO(JointStreamStats,active) RO(JointStreamStats,sent)
        RO(JointStreamStats,skipped_deadlines) RO(JointStreamStats,fault);
    py::class_<JointCommand>(m,"JointCommand").def(py::init<>()) RW(JointCommand,position) RW(JointCommand,velocity)
        RW(JointCommand,kp) RW(JointCommand,kd) RW(JointCommand,torque) RW(JointCommand,velocity_limit);
    py::class_<JointState>(m,"JointState") RO(JointState,position) RO(JointState,velocity) RO(JointState,torque)
        RO(JointState,temperature) RO(JointState,temperature_coil) RO(JointState,feedback_count) RO(JointState,feedback_age_ms)
        RO(JointState,drive_state) RO(JointState,motor_flags) RO(JointState,drive_mode) RO(JointState,has_command)
        RO(JointState,command_position) RO(JointState,command_velocity) RO(JointState,command_torque)
        RO(JointState,sequence) RO(JointState,timestamp_us) RO(JointState,host_time_us);
    py::class_<SafetyState>(m,"SafetyState") RO(SafetyState,valid) RO(SafetyState,stop_reason) RO(SafetyState,stop_phase)
        RO(SafetyState,stop_joint) RO(SafetyState,stop_detail) RO(SafetyState,stop_mode) RO(SafetyState,selfcheck)
        RO(SafetyState,selfcheck_joint) RO(SafetyState,selfcheck_register) RO(SafetyState,selfcheck_value) RO(SafetyState,selfcheck_expected);
    py::class_<RobotState>(m,"RobotState") RO(RobotState,connected) RO(RobotState,enabled_mask) RO(RobotState,enabled)
        RO(RobotState,mode) RO(RobotState,profile) RO(RobotState,profile_ready) RO(RobotState,modes_supported)
        RO(RobotState,zero_valid) RO(RobotState,telemetry_age_ms) RO(RobotState,telemetry_epoch) RO(RobotState,telemetry_version)
        RO(RobotState,joints) RO(RobotState,safety) RO(RobotState,operational_status) RO(RobotState,controller_state)
        RO(RobotState,command_active) RO(RobotState,command_applied_sequence);
    py::class_<RobotInfo>(m,"RobotInfo") RO(RobotInfo,received) RO(RobotInfo,firmware) RO(RobotInfo,product)
        RO(RobotInfo,axes) RO(RobotInfo,motor_count) RO(RobotInfo,capabilities) RO(RobotInfo,protocol_version)
        RO(RobotInfo,protocol_major) RO(RobotInfo,protocol_minor) RO(RobotInfo,board) RO(RobotInfo,arm_model)
        RO(RobotInfo,torque_max) RO(RobotInfo,velocity_max) RO(RobotInfo,position_min) RO(RobotInfo,position_max)
        RO(RobotInfo,profile_ready) RO(RobotInfo,enabled_mask) RO(RobotInfo,control_mode) RO(RobotInfo,motor_profile)
        RO(RobotInfo,controller_status);
#define PACKED_RO(T,F) .def_property_readonly(#F,[](const T& x){return x.F;})
    py::class_<InfrabotLinkStats>(m,"ControllerLinkStats")
        PACKED_RO(InfrabotLinkStats,command_rx) PACKED_RO(InfrabotLinkStats,command_invalid)
        PACKED_RO(InfrabotLinkStats,command_interval_max_us) PACKED_RO(InfrabotLinkStats,command_jitter_max_us)
        PACKED_RO(InfrabotLinkStats,control_missed) PACKED_RO(InfrabotLinkStats,control_period_max_us)
        PACKED_RO(InfrabotLinkStats,can_tx_errors) PACKED_RO(InfrabotLinkStats,telemetry_errors) PACKED_RO(InfrabotLinkStats,invalid_reason);
#undef PACKED_RO
    py::class_<LinkStats>(m,"LinkStats") RO(LinkStats,tx_sent) RO(LinkStats,tx_errors) RO(LinkStats,rx_telemetry)
        RO(LinkStats,rx_lost) RO(LinkStats,rx_invalid) RO(LinkStats,rx_last_second) RO(LinkStats,lost_last_second)
        RO(LinkStats,loss_percent) RO(LinkStats,rx_interval_max_us) RO(LinkStats,mcu_valid) RO(LinkStats,mcu);
    py::enum_<FaultEvent::Kind>(m,"FaultKind").value("DriveFault",FaultEvent::Kind::DriveFault)
        .value("DriveCleared",FaultEvent::Kind::DriveCleared).value("Stop",FaultEvent::Kind::Stop)
        .value("SelfCheck",FaultEvent::Kind::SelfCheck).value("Link",FaultEvent::Kind::Link);
    py::class_<FaultEvent>(m,"FaultEvent") RO(FaultEvent,kind) RO(FaultEvent,host_time_us) RO(FaultEvent,mcu_time_us)
        RO(FaultEvent,joint) RO(FaultEvent,code) RO(FaultEvent,detail) RO(FaultEvent,message);
    py::class_<RecordOptions>(m,"RecordOptions").def(py::init<>()) RW(RecordOptions,label) RW(RecordOptions,rotate_seconds) RW(RecordOptions,commands);
    py::class_<RecorderStats>(m,"RecorderStats") RO(RecorderStats,active) RO(RecorderStats,directory)
        RO(RecorderStats,telemetry_rows) RO(RecorderStats,command_rows) RO(RecorderStats,events)
        RO(RecorderStats,dropped) RO(RecorderStats,file_index) RO(RecorderStats,error);
    py::class_<SerialLogOptions>(m,"SerialLogOptions").def(py::init<>()) RW(SerialLogOptions,port) RW(SerialLogOptions,baud) RW(SerialLogOptions,directory);
    py::class_<SerialLogStats>(m,"SerialLogStats") RO(SerialLogStats,active) RO(SerialLogStats,path)
        RO(SerialLogStats,lines) RO(SerialLogStats,bytes) RO(SerialLogStats,reconnects)
        RO(SerialLogStats,port_open) RO(SerialLogStats,error);
    py::class_<McuLogOptions>(m,"McuLogOptions").def(py::init<>()) RW(McuLogOptions,directory) RW(McuLogOptions,label)
        RW(McuLogOptions,port) RW(McuLogOptions,timeout_ms) RW(McuLogOptions,retries) RW(McuLogOptions,progress);
    py::class_<McuLogResult>(m,"McuLogResult") RO(McuLogResult,text_path) RO(McuLogResult,telemetry_path)
        RO(McuLogResult,boot_id) RO(McuLogResult,text_lines) RO(McuLogResult,telemetry_rows)
        RO(McuLogResult,lost_bytes) RO(McuLogResult,writer_max_us) RO(McuLogResult,fault_latched);
    py::class_<CartesianLimits>(m,"CartesianLimits").def(py::init<>()) RW(CartesianLimits,linear_velocity)
        RW(CartesianLimits,angular_velocity) RW(CartesianLimits,linear_acceleration) RW(CartesianLimits,angular_acceleration);
    py::class_<CartesianCommand>(m,"CartesianCommand").def(py::init<>()) RW(CartesianCommand,pose) RW(CartesianCommand,limits)
        RW(CartesianCommand,kp) RW(CartesianCommand,kd) RW(CartesianCommand,torque) RW(CartesianCommand,velocity_limit);
    py::class_<MoveOptions>(m,"MoveOptions").def(py::init<>()) RW(MoveOptions,timeout_ms) RW(MoveOptions,dwell_ms)
        RW(MoveOptions,position_tolerance) RW(MoveOptions,velocity_tolerance) RW(MoveOptions,velocity_max)
        RW(MoveOptions,acceleration_max) RW(MoveOptions,kp) RW(MoveOptions,kd) RW(MoveOptions,torque);
    py::class_<CartesianMoveOptions>(m,"CartesianMoveOptions").def(py::init<>()) RW(CartesianMoveOptions,timeout_ms)
        RW(CartesianMoveOptions,dwell_ms) RW(CartesianMoveOptions,position_tolerance) RW(CartesianMoveOptions,velocity_tolerance)
        RW(CartesianMoveOptions,limits) RW(CartesianMoveOptions,velocity_max) RW(CartesianMoveOptions,acceleration_max)
        RW(CartesianMoveOptions,kp) RW(CartesianMoveOptions,kd);
    py::class_<MotionResult>(m,"MotionResult") RO(MotionResult,result) RO(MotionResult,state) RO(MotionResult,motion_id)
        RO(MotionResult,arrived_mask) RO(MotionResult,fresh_mask) RO(MotionResult,max_error_rad) RO(MotionResult,reason) RO(MotionResult,message);
    py::enum_<MotionStatus::Phase>(m,"MotionPhase").value("Off",MotionStatus::Phase::Off)
        .value("Preparing",MotionStatus::Phase::Preparing).value("Waiting",MotionStatus::Phase::Waiting)
        .value("Running",MotionStatus::Phase::Running).value("Settling",MotionStatus::Phase::Settling)
        .value("Ready",MotionStatus::Phase::Ready).value("Done",MotionStatus::Phase::Done)
        .value("Cancelling",MotionStatus::Phase::Cancelling).value("Cancelled",MotionStatus::Phase::Cancelled).value("Error",MotionStatus::Phase::Error);
    py::class_<InfrabotMoveReply>(m,"MoveReply") RO(InfrabotMoveReply,result) RO(InfrabotMoveReply,state)
        RO(InfrabotMoveReply,session) RO(InfrabotMoveReply,request_id) RO(InfrabotMoveReply,active_id)
        RO(InfrabotMoveReply,elapsed_ms) RO(InfrabotMoveReply,duration_ms) RO(InfrabotMoveReply,arrived_mask)
        RO(InfrabotMoveReply,enabled_mask) RO(InfrabotMoveReply,fresh_mask) RO(InfrabotMoveReply,max_error_rad) RO(InfrabotMoveReply,reason);
    py::class_<MotionStatus>(m,"MotionStatus") RO(MotionStatus,phase) RO(MotionStatus,id) RO(MotionStatus,can_execute)
        RO(MotionStatus,has_reply) RO(MotionStatus,reply_age_us) RO(MotionStatus,error) RO(MotionStatus,reply);
#undef RO
#undef RW
    py::class_<Robot>(m,"Robot")
        .def(py::init<std::string,uint16_t,uint16_t>(),py::arg("address")="192.168.0.50",py::arg("command_port")=5001,py::arg("telemetry_port")=5000)
        .def("connect",[](Robot& r){std::string e;require(r.connect(e),e);},Unlock())
        .def("disconnect",&Robot::disconnect,Unlock()).def("connected",&Robot::connected,Unlock())
        .def("info",&Robot::info,Unlock()).def("state",&Robot::state,Unlock())
        .def("link_stats",&Robot::link_stats,Unlock()).def("fault_log",&Robot::fault_log,Unlock())
        .def("set_heartbeat",&Robot::set_heartbeat,Unlock())
        .def("set_motor_profile",[](Robot& r,MotorProfile p){std::string e;require(r.set_motor_profile(p,e),e);},Unlock())
        .def("set_zero",[](Robot& r,bool enable){std::string e;require(r.set_zero(enable,e),e);},py::arg("enable_after")=false,Unlock())
        .def("servo_on",[](Robot& r,MotorGroup g){std::string e;require(r.servo_on(g,e),e);},py::arg_v("group", MotorGroup::All, "MotorGroup.All"),Unlock())
        .def("servo_off",[](Robot& r,MotorGroup g,bool wait){std::string e;require(r.servo_off(g,e,wait),e);},py::arg_v("group", MotorGroup::All, "MotorGroup.All"),py::arg("wait")=true,Unlock())
        .def("stop",[](Robot& r,bool wait){std::string e;require(r.stop(e,wait),e);},py::arg("wait")=true,Unlock())
        .def("resume",[](Robot& r){std::string e;Mode mode;require(r.resume(e,&mode),e);return mode;},Unlock())
        .def("clear_fault",[](Robot& r,uint32_t t){std::string e;require(r.clear_fault(e,t),e);},py::arg("timeout_ms")=1000,Unlock())
        .def("switch_mode",[](Robot& r,Mode m){std::string e;require(r.switch_mode(m,e),e);},Unlock())
        .def("hold_command",&Robot::hold_command,Unlock())
        .def("read_hold_command",[](Robot& r){std::string e;std::array<float,8> q;require(r.read_hold_command(q,e),e);return q;},Unlock())
        .def("servej",[](Robot& r,const JointCommand& c){std::string e;require(r.servej(c,e),e);},Unlock())
        .def("servep",[](Robot& r,const CartesianCommand& c){std::string e;require(r.servep(c,e),e);},Unlock())
        .def("start_joint_stream",[](Robot& r,const JointCommand& c,const JointStreamOptions& o){std::string e;require(r.start_joint_stream(c,o,e),e);},py::arg("initial"),py::arg_v("options", JointStreamOptions{}, "JointStreamOptions()"),Unlock())
        .def("stop_joint_stream",&Robot::stop_joint_stream,Unlock())
        .def("joint_stream_stats",&Robot::joint_stream_stats,Unlock())
        .def("movej",[](Robot& r,const std::array<float,7>& q,const MoveOptions& o){std::string e;auto v=r.movej(q,o,e);if(!e.empty())throw std::runtime_error(e);return v;},py::arg("position"),py::arg_v("options", MoveOptions{}, "MoveOptions()"),Unlock())
        .def("movep",[](Robot& r,const Pose& p,const CartesianMoveOptions& o){std::string e;auto v=r.movep(p,o,e);if(!e.empty())throw std::runtime_error(e);return v;},py::arg("pose"),py::arg_v("options", CartesianMoveOptions{}, "CartesianMoveOptions()"),Unlock())
        .def("submit_movej",[](Robot& r,const std::array<float,7>& q,const MoveOptions& o){std::string e;require(r.submit_movej(q,o,e),e);},py::arg("position"),py::arg_v("options", MoveOptions{}, "MoveOptions()"),Unlock())
        .def("submit_movep",[](Robot& r,const Pose& p,const CartesianMoveOptions& o){std::string e;require(r.submit_movep(p,o,e),e);},py::arg("pose"),py::arg_v("options", CartesianMoveOptions{}, "CartesianMoveOptions()"),Unlock())
        .def("motion",&Robot::motion,Unlock())
        .def("wait_motion",[](Robot& r,uint32_t t){std::string e;MotionResult v;require(r.wait_motion(t,v,e),e);return v;},Unlock())
        .def("cancel_motion",[](Robot& r){std::string e;require(r.cancel_motion(e),e);},Unlock())
        .def("set_default_move_options",&Robot::set_default_move_options,Unlock())
        .def("set_default_cartesian_options",&Robot::set_default_cartesian_options,Unlock())
        .def("gripper",[](Robot& r,float p,float v,float f){std::string e;require(r.gripper(p,v,f,e),e);},Unlock())
        .def("start_recording",[](Robot& r,const std::string& dir,const RecordOptions& o){std::string e;require(r.start_recording(dir,e,o),e);},py::arg("directory"),py::arg_v("options", RecordOptions{}, "RecordOptions()"),Unlock())
        .def("stop_recording",&Robot::stop_recording,Unlock()).def("recording",&Robot::recording,Unlock())
        .def("record_event",&Robot::record_event,Unlock())
        .def("start_serial_log",[](Robot& r,const SerialLogOptions& o){std::string e;require(r.start_serial_log(e,o),e);return r.serial_log().path;},
             py::arg_v("options", SerialLogOptions{}, "SerialLogOptions()"),Unlock())
        .def("stop_serial_log",&Robot::stop_serial_log,Unlock()).def("serial_log",&Robot::serial_log,Unlock())
        .def_static("log_directory",&Robot::log_directory)
        .def("download_mcu_log",[](Robot& r,const McuLogOptions& o){std::string e;McuLogResult v;require(r.download_mcu_log(v,e,o),e);return v;},
             py::arg_v("options", McuLogOptions{}, "McuLogOptions()"),Unlock())
        .def("request_feedback",[](Robot& r,uint8_t i){std::string e;require(r.request_feedback(i,e),e);},Unlock())
        .def("get_state",[](Robot& r,uint8_t i){std::string e;require(r.get_state(i,e),e);},Unlock())
        .def("get_register",[](Robot& r,uint8_t i,uint8_t rid,uint8_t type){std::string e;uint32_t v;require(r.get_register(i,rid,type,v,e),e);return v;},Unlock())
        .def("write_register",[](Robot& r,uint8_t i,uint8_t rid,uint32_t v,uint8_t type){std::string e;require(r.write_register(i,rid,v,type,e),e);},Unlock())
        .def("enable",[](Robot& r,uint8_t i){std::string e;require(r.enable(i,e),e);},Unlock())
        .def("disable",[](Robot& r,uint8_t i){std::string e;require(r.disable(i,e),e);},Unlock())
        .def("clear_error",[](Robot& r,uint8_t i){std::string e;require(r.clear_error(i,e),e);},Unlock())
        .def("set_zero_position",[](Robot& r,uint8_t i){std::string e;require(r.set_zero_position(i,e),e);},Unlock())
        .def("ensure_mode",[](Robot& r,uint8_t i,uint8_t m){std::string e;require(r.ensure_mode(i,m,e),e);},Unlock())
        .def("send_mit",[](Robot& r,uint8_t i,float p,float v,float kp,float kd,float t){std::string e;require(r.send_mit(i,p,v,kp,kd,t,e),e);},Unlock())
        .def("send_pos_vel",[](Robot& r,uint8_t i,float p,float v){std::string e;require(r.send_pos_vel(i,p,v,e),e);},Unlock())
        .def("send_vel",[](Robot& r,uint8_t i,float v){std::string e;require(r.send_vel(i,v,e),e);},Unlock())
        .def("send_force_pos",[](Robot& r,uint8_t i,float p,float v,float f){std::string e;require(r.send_force_pos(i,p,v,f,e),e);},Unlock())
        .def("set_can_timeout_ms",[](Robot& r,uint8_t i,uint32_t t){std::string e;require(r.set_can_timeout_ms(i,t,e),e);},Unlock())
        .def("store_parameters",[](Robot& r,uint8_t i){std::string e;require(r.store_parameters(i,e),e);},Unlock())
        .def_static("forward",&Robot::forward,Unlock());
    m.attr("ARM_MASK") = INFRABOT_ARM_MASK;
    m.attr("CAP_JOINT_RT") = INFRABOT_CAP_JOINT_RT;
    m.attr("CAP_GROUPS") = INFRABOT_CAP_GROUPS;
    m.attr("CAP_TELEMETRY_V3") = INFRABOT_CAP_TELEMETRY_V3;
    m.attr("CAP_CONTROLLED_STOP") = INFRABOT_CAP_CONTROLLED_STOP;
    m.attr("MOTOR_FEEDBACK_STALE") = INFRABOT_MOTOR_FEEDBACK_STALE;
    m.attr("CAP_ADMITTANCE")=py::int_(INFRABOT_CAP_ADMITTANCE);
    m.attr("CAP_LOCAL_GRAVITY") = INFRABOT_CAP_LOCAL_GRAVITY;
    m.attr("ARM_MAX_JOINT_VELOCITY") = INFRABOT_ARM_MAX_JOINT_VELOCITY;
    m.attr("REGISTER_U32") = INFRABOT_REGISTER_U32;
    m.attr("REGISTER_F32") = INFRABOT_REGISTER_F32;
}
