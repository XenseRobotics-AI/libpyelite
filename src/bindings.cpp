#include "safety.hpp"
#include "native_controller.hpp"
#include "station_collision.hpp"
#include "station_coordinator.hpp"
#include <Elite/DashboardClient.hpp>
#include <Elite/RobotConfPackage.hpp>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <fstream>
#include <set>
#include <sstream>

namespace py=pybind11;
using namespace libpyelite;
using Release=py::call_guard<py::gil_scoped_release>;
PYBIND11_MODULE(_native,m) {
  py::class_<ELITE::PrimaryPackage,std::shared_ptr<ELITE::PrimaryPackage>>(m,"PrimaryPackage");
  py::class_<ELITE::KinematicsInfo,ELITE::PrimaryPackage,std::shared_ptr<ELITE::KinematicsInfo>>(m,"KinematicsInfo")
    .def(py::init<>())
    .def_readonly("dh_alpha_",&ELITE::KinematicsInfo::dh_alpha_)
    .def_readonly("dh_a_",&ELITE::KinematicsInfo::dh_a_)
    .def_readonly("dh_d_",&ELITE::KinematicsInfo::dh_d_);
  m.attr("IK_BACKEND")=PinkIKModel::backendName();
  py::class_<PinkIKOptions>(m,"PinkIKOptions").def(py::init<>())
    .def_property_readonly("streaming_iteration_budget", &PinkIKOptions::streamingIterationBudget)
#define OPT(name) .def_readwrite(#name,&PinkIKOptions::name)
    OPT(max_iterations) OPT(dt) OPT(damping) OPT(step_size) OPT(max_step_norm)
    OPT(position_tolerance) OPT(orientation_tolerance) OPT(streaming_position_tolerance)
    OPT(streaming_orientation_tolerance) OPT(position_cost)
    OPT(orientation_cost) OPT(frame_gain) OPT(posture_weight) OPT(posture_gain);
#undef OPT
  py::class_<PinkIKDiagnostics>(m,"IKDiagnostics")
    .def_readonly("success",&PinkIKDiagnostics::success)
    .def_readonly("progress",&PinkIKDiagnostics::progress)
    .def_readonly("qp_failed",&PinkIKDiagnostics::qp_failed)
    .def_readonly("configuration_limited",&PinkIKDiagnostics::configuration_limited)
    .def_readonly("orientation_limited",&PinkIKDiagnostics::orientation_limited)
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
    .def_property_readonly("command_lower_limits",&PinkIKModel::commandLowerLimits)
    .def_property_readonly("command_upper_limits",&PinkIKModel::commandUpperLimits)
    .def_property_readonly("velocity_limits",&PinkIKModel::velocityLimits);
  py::class_<PlannerConfig>(m,"PlannerConfig").def(py::init<>())
#define PC(name) .def_readwrite(#name,&PlannerConfig::name)
    PC(period) PC(stale_timeout) PC(max_linear_velocity) PC(max_angular_velocity) PC(velocity_scale) PC(joint_motion_limits) PC(joint_target_horizon) PC(max_joint_velocity) PC(max_joint_acceleration) PC(max_joint_jerk) PC(joint_tracking_gain) PC(cartesian_motion_limits) PC(max_linear_acceleration) PC(max_angular_acceleration) PC(max_linear_jerk) PC(max_angular_jerk) PC(cartesian_tracking_gain)
    PC(max_target_translation_jump) PC(max_target_rotation_jump) PC(max_target_joint_jump) PC(arm_name)
    PC(preserve_configuration) PC(configuration_margin) PC(configuration_joint_span) PC(boundary_orientation_tolerance);
#undef PC
  py::class_<PlannerStatus>(m,"PlannerStatus")
#define PS(name) .def_readonly(#name,&PlannerStatus::name)
    PS(step_scale) PS(waypoint_fraction)
    PS(boundary_limited) PS(boundary_braking) PS(boundary_holds)
    PS(reference_position_error) PS(reference_orientation_error)
    PS(sequence) PS(consumed_sequence) PS(ik_failures) PS(moving) PS(stale) PS(idle) PS(solve_time_s) PS(ik) PS(joint_velocity) PS(joint_acceleration) PS(braking);
#undef PS
  py::class_<Planner>(m,"Planner")
    .def(py::init<const std::string&,const PinkPose&,PinkIKOptions,PlannerConfig>())
    .def("reanchor",&Planner::reanchor).def("reference",&Planner::reference).def("set_reference",&Planner::setReference).def("plan_reference",&Planner::planReference).def("advance",&Planner::advance).def("seed",&Planner::seed).def("submit",&Planner::submit,
      py::arg("target"),py::arg("arrival"),py::arg("sequence"),py::arg("duration")=0)
    .def("submit_joints",&Planner::submitJoints).def("tick",&Planner::tick)
    .def("cancel",&Planner::cancel).def("status",&Planner::status)
    .def("command_lower_limits",[](const Planner &s){return s.model().commandLowerLimits();})
    .def("command_upper_limits",[](const Planner &s){return s.model().commandUpperLimits();})
    .def("configuration_valid",[](const Planner &s,const PinkArmVector &q){return s.model().configurationValid(q);})
    .def("make_sender_limiter",&Planner::makeMotionLimiter)
    .def("pose",&Planner::pose).def("q",&Planner::q);
  py::class_<MotionLimiter>(m,"MotionLimiter")
    .def(py::init<double,PinkArmVector,PinkArmVector,PinkArmVector,PinkArmVector,PinkArmVector>())
    .def("reanchor",&MotionLimiter::reanchor).def("seed",&MotionLimiter::seed).def("step",&MotionLimiter::step,py::arg("target"),py::arg("stop")=false)
    .def("track",&MotionLimiter::track).def("gate",&MotionLimiter::gate,py::arg("target"),py::arg("velocity"),py::arg("acceleration"),py::arg("hard_only")=false).def("follow",&MotionLimiter::follow).def("passed",&MotionLimiter::passed)
    .def("velocity",&MotionLimiter::velocity).def("acceleration",&MotionLimiter::acceleration)
    .def("finished",&MotionLimiter::finished);
  m.def("validate_robot_feedback",&validateRobotFeedback,py::arg("robot_mode"),py::arg("safety_mode"),py::arg("wrench"),py::arg("limits"),py::arg("check_mode")=true);
  py::class_<ComplianceConfig>(m,"ComplianceConfig").def(py::init<>())
#define CC(name) .def_readwrite(#name,&ComplianceConfig::name)
    CC(enabled) CC(gravity) CC(payload_mass) CC(payload_cog) CC(friction_coulomb) CC(friction_viscous)
    CC(friction_speed) CC(torque_band) CC(transient_factor) CC(transient_window) CC(torque_filter_hz)
    CC(accel_filter_hz) CC(bias_time) CC(mass) CC(damping) CC(stiffness) CC(force_deadband) CC(max_force)
    CC(max_offset) CC(max_velocity) CC(collision_threshold);
#undef CC
  py::class_<ComplianceStatus>(m,"ComplianceStatus")
#define CS(name) .def_readonly(#name,&ComplianceStatus::name)
    CS(enabled) CS(biased) CS(in_contact) CS(tau_ext) CS(force) CS(offset) CS(velocity) CS(collision);
#undef CS
  py::class_<ComplianceController>(m,"ComplianceController")
    .def(py::init<const std::string&,const PinkPose&,ComplianceConfig>(),
      py::arg("urdf"),py::arg("tool")=kIdentityPose,py::arg("config")=ComplianceConfig{})
    .def("update",&ComplianceController::update).def("rebias",&ComplianceController::rebias)
    .def("reset_motion",&ComplianceController::resetMotion)
    .def("status",&ComplianceController::status)
    .def("model_torque",&ComplianceController::modelTorque)
    .def("tcp_jacobian",&ComplianceController::tcpJacobian);
  py::class_<CollisionShape>(m,"CollisionShape").def(py::init<>())
#define CSH(name) .def_readwrite(#name,&CollisionShape::name)
    CSH(frame) CSH(type) CSH(size) CSH(radius) CSH(length) CSH(xyz) CSH(rpy) CSH(vertices) CSH(faces);
#undef CSH
  py::class_<ClearanceResult>(m,"ClearanceResult")
    .def_readonly("safe",&ClearanceResult::safe).def_readonly("minimum_distance",&ClearanceResult::minimum_distance)
    .def_readonly("pair",&ClearanceResult::pair).def_readonly("clearance",&ClearanceResult::clearance);
  py::class_<StationCollisionModel,std::shared_ptr<StationCollisionModel>>(m,"StationCollisionModel")
    .def(py::init<const std::string&,std::vector<CollisionShape>,std::vector<std::array<int,2>>,std::vector<double>,double,double,double>(),
      py::arg("urdf"),py::arg("shapes"),py::arg("pairs"),py::arg("margins"),py::arg("detection_distance"),py::arg("gain"),py::arg("qp_buffer"))
    .def_property_readonly("nq",&StationCollisionModel::nq)
    .def("check",&StationCollisionModel::check,py::arg("q"),py::arg("cutoff")=-1.0)
    .def("linearize",[](StationCollisionModel &self,const Eigen::VectorXd &q,const std::vector<int> &indices) {
      Eigen::MatrixXd rows; Eigen::VectorXd bounds; ClearanceResult closest;
      self.linearize(q,indices,rows,bounds,closest);
      std::vector<std::tuple<int,double,Eigen::Vector3d,Eigen::Vector3d>> contacts;
      for(const auto &c:self.contacts()) contacts.emplace_back(c.pair,c.distance,c.point1,c.point2);
      return py::make_tuple(rows,bounds,closest,contacts);
    })
    .def("verify_step",&StationCollisionModel::verifyStep,py::arg("q0"),py::arg("q1"),py::arg("resolution")=0.002)
    .def("placements",&StationCollisionModel::placements)
    .def_property_readonly("last_narrow_phase_calls",&StationCollisionModel::lastNarrowPhaseCalls)
    .def_property_readonly("last_sampled_pairs",&StationCollisionModel::lastSampledPairs);
  py::class_<GuardResult>(m,"GuardResult")
    .def_readonly("q",&GuardResult::q).def_readonly("held",&GuardResult::held)
    .def_readonly("limited",&GuardResult::limited).def_readonly("clearance",&GuardResult::clearance)
    .def_readonly("constraints",&GuardResult::constraints).def_readonly("solve_time_s",&GuardResult::solve_time_s)
    .def_readonly("reference_iterations",&GuardResult::reference_iterations)
    .def_readonly("output_iterations",&GuardResult::output_iterations)
    .def_readonly("sampled_pairs",&GuardResult::sampled_pairs)
    .def_readonly("gripper_age_s",&GuardResult::gripper_age_s)
    .def_readonly("gripper_sequence",&GuardResult::gripper_sequence)
    .def_readonly("measured_geometry_hold",&GuardResult::measured_geometry_hold);
  py::class_<StationGuard,std::shared_ptr<StationGuard>>(m,"StationGuard")
    .def(py::init<std::shared_ptr<StationCollisionModel>,const std::string&,const Eigen::VectorXd&,const std::array<PinkPose,2>&>(),
      py::arg("collision"),py::arg("station_urdf"),py::arg("default_q"),py::arg("tools"))
    .def("tick",&StationGuard::tick,py::arg("left").none(true),py::arg("right").none(true),
      py::arg("left_fixed"),py::arg("right_fixed"),py::arg("now"))
    .def("reset",&StationGuard::reset)
    .def("configure_grippers",&StationGuard::configureGrippers,py::arg("joints"),py::arg("sides"),py::arg("max_age_s"))
    .def("update_grippers",&StationGuard::updateGrippers,py::arg("positions"),py::arg("sample_times"),
      py::arg("sequences"),py::arg("valid"))
    .def("station_q",static_cast<Eigen::VectorXd (StationGuard::*)(const std::array<PinkArmVector,2>&) const>(
      &StationGuard::stationQ));
  py::class_<ProjectionQP>(m,"ProjectionQP").def(py::init<>())
    .def("solve",&ProjectionQP::solve,py::arg("metric"),py::arg("proposed"),py::arg("lower"),py::arg("upper"),
      py::arg("soft_lower"),py::arg("soft_upper"),py::arg("g"),py::arg("h"))
    .def_property_readonly("last_iterations",&ProjectionQP::lastIterations)
    .def_property_readonly("last_converged",&ProjectionQP::lastConverged)
    .def_property_readonly("last_repair",&ProjectionQP::lastRepair);
  py::class_<CoordinatorStatus>(m,"CoordinatorStatus")
    .def_readonly("running",&CoordinatorStatus::running).def_readonly("error",&CoordinatorStatus::error)
    .def_readonly("last",&CoordinatorStatus::last).def_readonly("ticks",&CoordinatorStatus::ticks)
    .def_readonly("overruns",&CoordinatorStatus::overruns).def_readonly("max_tick_s",&CoordinatorStatus::max_tick_s);
  m.def("interpolate_pose",&interpolatePose);
  m.def("pose_from_rotvec",&poseFromRotvec);
  m.def("pose_from_controller_tcp",&poseFromControllerTcp);
  m.def("pose_to_rotvec",&poseToRotvec);
  py::class_<DriverConfig>(m,"EliteDriverConfig").def(py::init<>())
#define DC(name) .def_readwrite(#name,&DriverConfig::name)
    DC(robot_ip) DC(local_ip) DC(script_file_path) DC(headless_mode)
    DC(reverse_port) DC(script_sender_port) DC(trajectory_port) DC(script_command_port)
    DC(servoj_time) DC(servoj_lookahead_time) DC(servoj_gain) DC(stopj_acc)
    DC(model_path) DC(tool) DC(ik) DC(planner) DC(command_timeout_ms) DC(max_consecutive_write_failures)
    DC(feedback_timeout) DC(planner_timeout) DC(fk_position_tolerance) DC(fk_orientation_tolerance)
    DC(check_robot_safety) DC(max_contact_wrench) DC(max_start_joint_speed) DC(contact_monitor);
#undef DC
  py::class_<NativeStatus>(m,"NativeStatus")
#define NS(name) .def_readonly(#name,&NativeStatus::name)
    NS(planner) NS(running) NS(reset_active) NS(error) NS(planner_overruns) NS(command_sequence) NS(sent_sequence)
    NS(commanded_q) NS(commanded_pose) NS(sent_q) NS(sent_velocity) NS(sent_acceleration) NS(contact) NS(max_sender_gap_s) NS(max_write_s);
#undef NS
  py::class_<ELITE::RtsiIOInterface,std::shared_ptr<ELITE::RtsiIOInterface>>(m,"RtsiIOInterface")
    .def(py::init([](const std::string &output,const std::string &input,double frequency) {
      std::ifstream file(output);
      if(!file) throw std::invalid_argument("Cannot read RTSI output recipe");
      std::set<std::string> fields;
      std::string line;
      while(std::getline(file,line)) {
        std::istringstream stream(line); std::string field; stream>>field; fields.insert(field);
      }
      for(const char *field:{"timestamp","actual_TCP_pose","actual_joint_positions","actual_joint_speeds",
                           "actual_joint_torques","robot_mode","safety_status","actual_TCP_force"})
        if(!fields.count(field)) throw std::invalid_argument(std::string("Missing required RTSI field: ")+field);
      if(!std::isfinite(frequency)||frequency<=0) throw std::invalid_argument("Invalid RTSI frequency");
      return std::make_shared<ELITE::RtsiIOInterface>(output,input,frequency);
    }),Release())
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
    .def("rebiasContact",&NativeDriver::rebiasContact,Release())
    .def("setExternalPlanning",&NativeDriver::setExternalPlanning,Release())
    .def("externalPlanning",&NativeDriver::externalPlanning)
    .def("setPayload",&NativeDriver::setPayload,Release())
    .def("getPrimaryPackage",&NativeDriver::getPrimaryPackage,Release())
    .def("moveJ",&NativeDriver::moveJ,Release())
    .def("cancel",&NativeDriver::cancel,Release())
    .def("submit",&NativeDriver::submit,py::arg("pose"),py::arg("duration")=0,Release())
    .def("submit_joints",&NativeDriver::submitJoints,Release())
    .def("status",&NativeDriver::status,Release());
  py::class_<StationCoordinator>(m,"StationCoordinator")
    .def(py::init<std::shared_ptr<StationGuard>,NativeDriver&,NativeDriver&,
                  std::shared_ptr<ELITE::RtsiIOInterface>,std::shared_ptr<ELITE::RtsiIOInterface>>(),
      py::arg("guard"),py::arg("left"),py::arg("right"),py::arg("left_rtsi"),py::arg("right_rtsi"),
      py::keep_alive<1,3>(),py::keep_alive<1,4>())
    .def("start",&StationCoordinator::start,Release())
    .def("stop",&StationCoordinator::stop,Release())
    .def("status",&StationCoordinator::status);
}
