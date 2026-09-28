#include "native_controller.hpp"
#include <Elite/DashboardClient.hpp>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py=pybind11;
using namespace libpyelite;
using Release=py::call_guard<py::gil_scoped_release>;
PYBIND11_MODULE(_native,m) {
  m.attr("IK_BACKEND")=PinkIKModel::backendName();
  py::class_<PinkIKOptions>(m,"PinkIKOptions").def(py::init<>())
#define OPT(name) .def_readwrite(#name,&PinkIKOptions::name)
    OPT(max_iterations) OPT(dt) OPT(damping) OPT(step_size) OPT(max_step_norm)
    OPT(position_tolerance) OPT(orientation_tolerance) OPT(position_cost)
    OPT(orientation_cost) OPT(frame_gain) OPT(posture_weight) OPT(posture_gain);
#undef OPT
  py::class_<PinkIKDiagnostics>(m,"IKDiagnostics")
    .def_readonly("success",&PinkIKDiagnostics::success)
    .def_readonly("iterations",&PinkIKDiagnostics::iterations)
    .def_readonly("position_error",&PinkIKDiagnostics::position_error)
    .def_readonly("orientation_error",&PinkIKDiagnostics::orientation_error);
  py::class_<PinkIKModel>(m,"PinkIKModel")
    .def(py::init<const std::string&,const PinkPose&,PinkIKOptions>(),
      py::arg("urdf"),py::arg("tool")=kIdentityPose,py::arg("options")=PinkIKOptions{},Release())
    .def("fk",[](PinkIKModel &s,const PinkArmVector &q){return s.forwardKinematics(0,q);},Release())
    .def("solve",[](PinkIKModel &s,const PinkPose &p,const PinkArmVector &q,const PinkArmVector &step) {
      PinkArmVector result=q; PinkIKDiagnostics d; s.solve(0,p,q,step,result,d);
      return std::make_pair(result,d);
    },Release())
    .def_property_readonly("lower_limits",&PinkIKModel::lowerLimits)
    .def_property_readonly("upper_limits",&PinkIKModel::upperLimits)
    .def_property_readonly("velocity_limits",&PinkIKModel::velocityLimits);
  py::class_<PlannerConfig>(m,"PlannerConfig").def(py::init<>())
#define PC(name) .def_readwrite(#name,&PlannerConfig::name)
    PC(period) PC(stale_timeout) PC(max_linear_velocity) PC(max_angular_velocity) PC(velocity_scale);
#undef PC
  py::class_<PlannerStatus>(m,"PlannerStatus")
#define PS(name) .def_readonly(#name,&PlannerStatus::name)
    PS(sequence) PS(consumed_sequence) PS(ik_failures) PS(moving) PS(stale) PS(idle) PS(solve_time_s) PS(ik);
#undef PS
  py::class_<Planner>(m,"Planner")
    .def(py::init<const std::string&,const PinkPose&,PinkIKOptions,PlannerConfig>())
    .def("seed",&Planner::seed).def("submit",&Planner::submit,
      py::arg("target"),py::arg("arrival"),py::arg("sequence"),py::arg("duration")=0)
    .def("submit_joints",&Planner::submitJoints).def("tick",&Planner::tick)
    .def("cancel",&Planner::cancel).def("status",&Planner::status)
    .def("pose",&Planner::pose).def("q",&Planner::q);
  m.def("interpolate_pose",&interpolatePose);
  m.def("pose_from_rotvec",&poseFromRotvec);
  m.def("pose_from_controller_tcp",&poseFromControllerTcp);
  m.def("pose_to_rotvec",&poseToRotvec);
  py::class_<DriverConfig>(m,"EliteDriverConfig").def(py::init<>())
#define DC(name) .def_readwrite(#name,&DriverConfig::name)
    DC(robot_ip) DC(local_ip) DC(script_file_path) DC(headless_mode)
    DC(reverse_port) DC(script_sender_port) DC(trajectory_port) DC(script_command_port)
    DC(servoj_time) DC(servoj_lookahead_time) DC(servoj_gain) DC(stopj_acc)
    DC(model_path) DC(tool) DC(ik) DC(planner) DC(command_timeout_ms)
    DC(feedback_timeout) DC(planner_timeout) DC(fk_position_tolerance) DC(fk_orientation_tolerance);
#undef DC
  py::class_<NativeStatus>(m,"NativeStatus")
#define NS(name) .def_readonly(#name,&NativeStatus::name)
    NS(planner) NS(running) NS(reset_active) NS(error) NS(planner_overruns)
    NS(commanded_q) NS(commanded_pose);
#undef NS
  py::class_<ELITE::RtsiIOInterface,std::shared_ptr<ELITE::RtsiIOInterface>>(m,"RtsiIOInterface")
    .def(py::init<const std::string&,const std::string&,double>(),Release())
    .def("connect",&ELITE::RtsiIOInterface::connect,Release())
    .def("disconnect",&ELITE::RtsiIOInterface::disconnect,Release())
    .def("isConnected",&ELITE::RtsiIOInterface::isConnected,Release())
    .def("getTimestamp",&ELITE::RtsiIOInterface::getTimestamp,Release())
    .def("getActualTCPPose",&ELITE::RtsiIOInterface::getActualTCPPose,Release())
    .def("getActualJointPositions",&ELITE::RtsiIOInterface::getActualJointPositions,Release())
    .def("getActualJointVelocity",&ELITE::RtsiIOInterface::getActualJointVelocity,Release())
    .def("getActualJointTorques",&ELITE::RtsiIOInterface::getActualJointTorques,Release());
  py::class_<ELITE::DashboardClient>(m,"DashboardClientInterface")
    .def(py::init<>())
    .def("connect",&ELITE::DashboardClient::connect,py::arg("ip"),py::arg("port")=29999,Release())
    .def("disconnect",&ELITE::DashboardClient::disconnect,Release())
    .def("powerOn",&ELITE::DashboardClient::powerOn,Release())
    .def("brakeRelease",&ELITE::DashboardClient::brakeRelease,Release());
  py::class_<NativeDriver>(m,"NativeDriver")
    .def(py::init<DriverConfig>(),Release())
    .def("isRobotConnected",&NativeDriver::isRobotConnected,Release())
    .def("sendExternalControlScript",&NativeDriver::sendExternalControlScript,Release())
    .def("startServo",&NativeDriver::startServo,Release())
    .def("stopServo",&NativeDriver::stopServo,Release())
    .def("stopControl",&NativeDriver::stopControl,Release())
    .def("writeIdle",&NativeDriver::writeIdle,Release())
    .def("moveJ",&NativeDriver::moveJ,Release())
    .def("cancel",&NativeDriver::cancel,Release())
    .def("submit",&NativeDriver::submit,py::arg("pose"),py::arg("duration")=0,Release())
    .def("submit_joints",&NativeDriver::submitJoints,Release())
    .def("status",&NativeDriver::status,Release());
}
