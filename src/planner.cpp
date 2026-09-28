#include "planner.hpp"
#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace libpyelite {
namespace {
using Matrix4 = Eigen::Matrix<double,4,4,Eigen::RowMajor>;
Matrix4 matrix(const PinkPose &p) { return Eigen::Map<const Matrix4>(p.data()); }
PinkPose array(const Matrix4 &m) {
  PinkPose p; Eigen::Map<Matrix4>(p.data()) = m; return p;
}
double minJerk(double x) { return x*x*x*(10+x*(-15+6*x)); }
}
bool validPose(const PinkPose &p) {
  auto m=matrix(p); const Eigen::Matrix3d r=m.block<3,3>(0,0);
  return m.allFinite() && (m.row(3)-Eigen::RowVector4d(0,0,0,1)).norm()<1e-8 &&
    std::abs(r.determinant()-1)<1e-6 &&
    (r.transpose()*r-Eigen::Matrix3d::Identity()).norm()<1e-6;
}
PinkPose poseFromRotvec(const PinkArmVector &p) {
  Matrix4 m=Matrix4::Identity();
  Eigen::Vector3d v(p[3],p[4],p[5]); double angle=v.norm();
  if(angle>1e-12) m.block<3,3>(0,0)=Eigen::AngleAxisd(angle,v/angle).toRotationMatrix();
  m.block<3,1>(0,3)=Eigen::Vector3d(p[0],p[1],p[2]);
  return array(m);
}
PinkPose poseFromControllerTcp(const PinkArmVector &p) {
  Matrix4 m=Matrix4::Identity();
  const double roll=p[3], pitch=p[4], yaw=p[5];
  m.block<3,3>(0,0) = Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ()).toRotationMatrix()
                    * Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY()).toRotationMatrix()
                    * Eigen::AngleAxisd(roll,Eigen::Vector3d::UnitX()).toRotationMatrix();
  m.block<3,1>(0,3)=Eigen::Vector3d(p[0],p[1],p[2]);
  return array(m);
}
PinkArmVector poseToRotvec(const PinkPose &p) {
  auto m=matrix(p); Eigen::AngleAxisd aa(m.block<3,3>(0,0));
  Eigen::Vector3d v=aa.angle()*aa.axis(); return {m(0,3),m(1,3),m(2,3),v[0],v[1],v[2]};
}
PinkPose interpolatePose(const PinkPose &a,const PinkPose &b,double alpha) {
  const auto am=matrix(a),bm=matrix(b); Matrix4 out=Matrix4::Identity();
  alpha=std::clamp(alpha,0.0,1.0);
  out.block<3,1>(0,3)=(1-alpha)*am.block<3,1>(0,3)+alpha*bm.block<3,1>(0,3);
  out.block<3,3>(0,0)=Eigen::Quaterniond(am.block<3,3>(0,0)).slerp(
    alpha,Eigen::Quaterniond(bm.block<3,3>(0,0))).normalized().toRotationMatrix();
  return array(out);
}
Planner::Planner(const std::string &urdf,const PinkPose &tool,PinkIKOptions ik,PlannerConfig c)
  : model_(urdf,tool,ik),config_(c) {
  for(double v:{c.period,c.stale_timeout,c.max_linear_velocity,c.max_angular_velocity,c.velocity_scale})
    if(!std::isfinite(v)||v<=0) throw std::invalid_argument("Planner configuration must be positive and finite");
  if(c.period<0.004 || c.period>0.01 || c.velocity_scale>1)
    throw std::invalid_argument("Elite period must be 4..10ms; velocity_scale must be <=1");
  auto velocity=model_.velocityLimits();
  for(int i=0;i<6;++i) max_step_[i]=velocity[i]*c.velocity_scale*c.period;
}
void Planner::seed(const PinkArmVector &q) {
  pose_=model_.forwardKinematics(0,q); q_=q; seeded_=true; cancel();
}
void Planner::cancel() {
  status_.moving=false; status_.idle=true; status_.stale=false;
  status_.consumed_sequence=status_.sequence; arrival_=-1; reset_=false;
}
void Planner::submit(const PinkPose &target,double arrival,std::uint64_t seq,double duration) {
  if(!seeded_) throw std::logic_error("Planner must be feedback-seeded");
  if(!validPose(target)||!std::isfinite(arrival)||!std::isfinite(duration)||duration<0)
    throw std::invalid_argument("Invalid Cartesian target or time");
  if(seq<=status_.sequence) throw std::invalid_argument("Target sequence must increase");
  double interval=arrival_<0 ? config_.period : std::max(config_.period,arrival-arrival_);
  start_=pose_; target_=target;
  auto a=matrix(start_),b=matrix(target_);
  const double distance=(a.block<3,1>(0,3)-b.block<3,1>(0,3)).norm();
  const double angle=Eigen::Quaterniond(a.block<3,3>(0,0)).angularDistance(Eigen::Quaterniond(b.block<3,3>(0,0)));
  reset_=duration>0; joint_mode_=false;
  // Tianji arrival-time interpolation, rebased at the current planned pose.
  // Preserve the old Elite timed-reset min-jerk curve (peak derivative 1.875).
  double peak=reset_?1.875:1.0;
  duration_=std::max({config_.period,reset_?duration:interval,
    peak*distance/config_.max_linear_velocity,peak*angle/config_.max_angular_velocity});
  arrival_=arrival; first_tick_=true; status_.sequence=seq;
  status_.moving=true; status_.idle=false; status_.stale=false;
}
void Planner::submitJoints(const PinkArmVector &q,double arrival,std::uint64_t seq) {
  model_.forwardKinematics(0,q);
  submit(pose_,arrival,seq); joint_mode_=true; joint_target_=q;
}
PinkArmVector Planner::tick(double now) {
  if(!seeded_||!std::isfinite(now)) throw std::logic_error("Invalid planner tick");
  if(arrival_>=0 && !reset_ && now-arrival_>config_.stale_timeout) {
    cancel(); status_.stale=true; return q_;
  }
  if(!status_.moving) return q_;
  if(first_tick_) { start_time_=now; first_tick_=false; }
  const double alpha=std::clamp((now-start_time_+config_.period)/duration_,0.0,1.0);
  PinkArmVector candidate=q_;
  if(joint_mode_) candidate=joint_target_;
  else {
    const auto begin=std::chrono::steady_clock::now();
    const bool ok=model_.solve(0,interpolatePose(start_,target_,reset_?minJerk(alpha):alpha),
      q_,max_step_,candidate,status_.ik);
    status_.solve_time_s=std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();
    if(!ok) { ++status_.ik_failures; status_.moving=false; reset_=false; return q_; }
  }
  bool reached=true;
  for(int i=0;i<6;++i) {
    const double delta=candidate[i]-q_[i];
    if(std::abs(delta)>max_step_[i]) reached=false;
    q_[i]+=std::clamp(delta,-max_step_[i],max_step_[i]);
  }
  pose_=model_.forwardKinematics(0,q_);
  if((joint_mode_||alpha>=1.0) && reached) {
    status_.moving=false; status_.consumed_sequence=status_.sequence;
    if(reset_) { arrival_=now; reset_=false; }
  }
  return q_;
}
} // namespace libpyelite
