/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "fake_mcu.hpp"
#include "fake_log_board.hpp"
namespace py = pybind11;
PYBIND11_MODULE(_infrabot_test_mcu, m) {
    py::class_<fake_log::Board>(m, "FakeLogBoard")
        .def(py::init([] {
            auto board = std::make_unique<fake_log::Board>();
            std::lock_guard<std::mutex> lock(board->mutex);
            board->model.init(8, 8);
            board->model.add_text(100, "loopback example");
            return board;
        }))
        .def_readonly("port", &fake_log::Board::port);
    using F = fake::FakeMcu;
    py::class_<F>(m, "FakeMcu")
        .def(py::init<>()).def("calibrate", &F::calibrate_for_test)
        .def("shift_arm", &F::shift_arm_for_test)
        .def_property("protocol_major", [](F& f){return f.protocol_major.load();}, [](F& f,uint16_t v){f.protocol_major=v;})
        .def_property("v1_only", [](F& f){return f.v1_only.load();}, [](F& f,bool v){f.v1_only=v;})
        .def_readonly("command_port", &F::command_port).def_readonly("telemetry_port", &F::telemetry_port)
        .def_property("telemetry_v3", [](F& f){return f.telemetry_v3.load();}, [](F& f,bool v){f.telemetry_v3=v;})
        .def_property("supports_admittance", [](F& f){return f.supports_admittance.load();}, [](F& f,bool v){f.supports_admittance=v;})
        .def_property("supports_gravity", [](F& f){return f.supports_gravity.load();}, [](F& f,bool v){f.supports_gravity=v;})
        .def_property("supports_parameters", [](F& f){return f.supports_parameters.load();}, [](F& f,bool v){f.supports_parameters=v;})
        .def_property("param_not_owner", [](F& f){return f.param_not_owner.load();}, [](F& f,bool v){f.param_not_owner=v;})
        .def_property_readonly("param_requests", [](F& f){return f.param_requests.load();})
        .def("param_requested", [](F& f, uint16_t id){return f.param_requested(id);})
        .def_property("supports_diagnostics", [](F& f){return f.supports_diagnostics.load();}, [](F& f,bool v){f.supports_diagnostics=v;})
        .def_property_readonly("diag_requests", [](F& f){return f.diag_requests.load();})
        .def_property("drop_telemetry", [](F& f){return f.drop_telemetry.load();}, [](F& f,bool v){f.drop_telemetry=v;})
        .def_property("gripper_absent", [](F& f){return f.gripper_absent.load();}, [](F& f,bool v){f.gripper_absent=v;})
        .def_property("frozen_feedback_mask", [](F& f){return f.frozen_feedback_mask.load();}, [](F& f,uint8_t v){f.frozen_feedback_mask=v;})
        .def_property("follow_joint_commands", [](F& f){return f.follow_joint_commands.load();}, [](F& f,bool v){f.follow_joint_commands=v;})
        .def_property("hold_offset_mrad", [](F& f){return f.hold_offset_mrad.load();}, [](F& f,int32_t v){f.hold_offset_mrad=v;})
        .def_property("idle_heartbeat_timeout_ms", [](F& f){return f.idle_heartbeat_timeout_ms.load();}, [](F& f,uint32_t v){f.idle_heartbeat_timeout_ms=v;})
        .def_property("fault_joint", [](F& f){return f.fault_joint.load();}, [](F& f,uint8_t v){f.fault_joint=v;})
        .def_property_readonly("enabled_mask", [](F& f){return f.enabled_mask.load();})
        .def_property_readonly("disable_count", [](F& f){return f.disable_count.load();})
        .def_property_readonly("stop_count", [](F& f){return f.stop_count.load();})
        .def_property_readonly("stop_reason", [](F& f){return f.stop_reason.load();})
        .def_property_readonly("stop_phase", [](F& f){return f.stop_phase.load();})
        .def_property_readonly("mode", [](F& f){return f.mode.load();})
        .def_property_readonly("info_count", [](F& f){return f.info_count.load();})
        .def_property_readonly("heartbeat_count", [](F& f){return f.heartbeat_count.load();})
        .def("last_move_velocity", [](F& f){
            const auto r=f.last_request();
            const float *v=r.op==motion::MoveP ? r.command.cartesian.velocity_max : r.command.joint.velocity_max;
            return std::vector<float>(v,v+7);
        })
        .def("torque", [](F& f, unsigned i, int32_t value){if(i>=8)throw std::out_of_range("joint");f.torques[i]=value;})
        .def("last_command", [](F& f){
            const auto c=f.last_joint_stream(); py::dict d;
            for (auto name : {"position", "velocity", "kp", "kd", "torque"}) {
                std::vector<int32_t> values(8);
                for (size_t i=0;i<8;++i) values[i] = std::string(name)=="position" ? c.target_position_mrad[i] :
                    std::string(name)=="velocity" ? c.target_velocity_mrad_s[i] :
                    std::string(name)=="kp" ? c.kp_milli[i] : std::string(name)=="kd" ? c.kd_milli[i] : c.torque_mNm[i];
                d[name]=values;
            } return d;
        });
}
