#include "planner.hpp"
#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>

namespace libpyelite {
namespace {
using Matrix4 = Eigen::Matrix<double,4,4,Eigen::RowMajor>;
Matrix4 matrix(const PinkPose &p) { return Eigen::Map<const Matrix4>(p.data()); }
PinkPose array(const Matrix4 &m) {
  PinkPose p; Eigen::Map<Matrix4>(p.data()) = m; return p;
}
double minJerk(double x) { return x*x*x*(10+x*(-15+6*x)); }
std::ostringstream jumpError(const char *kind,const PlannerConfig &config,
                             std::uint64_t sequence,std::uint64_t last_sequence) {
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::setprecision(std::numeric_limits<double>::max_digits10)
      << kind << " target jump rejected: arm=" << (config.arm_name.empty()?"unspecified":config.arm_name)
      << " sequence=" << sequence << " last_accepted_sequence=" << last_sequence;
  return out;
}
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
  for(double value:{c.max_target_translation_jump,c.max_target_rotation_jump,c.max_target_joint_jump,c.joint_target_horizon})
    if(!std::isfinite(value)||value<0) throw std::invalid_argument("Invalid target jump limit");
  if(!std::isfinite(c.joint_tracking_gain)||c.joint_tracking_gain<=0)
    throw std::invalid_argument("joint_tracking_gain must be positive and finite");
  auto velocity=model_.velocityLimits();
  if(c.joint_motion_limits) {
    for(int i=0;i<6;++i) {
      if(c.max_joint_velocity[i]>velocity[i]*c.velocity_scale)
        throw std::invalid_argument("Joint velocity exceeds scaled URDF bound");
      velocity[i]=c.max_joint_velocity[i]/c.velocity_scale;
    }
    limiter_=std::make_unique<MotionLimiter>(c.period,c.max_joint_velocity,c.max_joint_acceleration,
      c.max_joint_jerk,model_.lowerLimits(),model_.upperLimits());
  }
  for(int i=0;i<6;++i) max_step_[i]=velocity[i]*c.velocity_scale*c.period;
  if(c.cartesian_motion_limits)
    smoother_=std::make_unique<CartesianSmoother>(CartesianLimits{c.max_linear_velocity,c.max_angular_velocity,
      c.max_linear_acceleration,c.max_angular_acceleration,c.max_linear_jerk,c.max_angular_jerk,
      c.cartesian_tracking_gain});
}
PinkArmVector Planner::scaleStep(const PinkArmVector &from,PinkArmVector to,double &factor) const {
  factor=1.0;
  for(int i=0;i<6;++i) {
    const double delta=std::abs(to[i]-from[i]);
    if(delta>max_step_[i]) factor=std::min(factor,max_step_[i]/delta);
  }
  if(factor<1.0) for(int i=0;i<6;++i) to[i]=from[i]+factor*(to[i]-from[i]);
  return to;
}
void Planner::seed(const PinkArmVector &q) {
  pose_=model_.forwardKinematics(0,q); q_=q; seeded_=true; cancel();
  if(limiter_) limiter_->seed(q);
  stopping_=false; status_.braking=false; status_.moving=false; status_.idle=true;
  status_.joint_velocity.fill(0); status_.joint_acceleration.fill(0);
  ref_q_=q; ref_pose_=pose_; ref_velocity_.fill(0);
  goal_=pose_; cartesian_stop_=false;
  if(smoother_) smoother_->reset(pose_);
}
void Planner::reanchor(const PinkArmVector &q,const PinkArmVector &v,const PinkArmVector &a) {
  if(!limiter_) { seed(q); return; }
  // An external gate (collision QP) trimmed the output. Only the output state
  // moves: an active target and its reference keep going, so tracking does not
  // restart. Pulling the reference here would turn the next IK step into a 1/dt
  // position gain and limit-cycle against the acceleration limit; an external
  // layer that must constrain the reference uses setReference() instead.
  pose_=model_.forwardKinematics(0,q); q_=q;
  limiter_->reanchor(q,v,a);
  status_.joint_velocity=v; status_.joint_acceleration=a;
  if(!status_.moving) {
    ref_q_=q; ref_pose_=pose_; ref_velocity_.fill(0);
    bool still=true;
    for(int i=0;i<6;++i) still=still && v[i]==0 && a[i]==0;
    stopping_=!still; status_.braking=stopping_; status_.moving=stopping_; status_.idle=still;
  }
}
void Planner::cancel() {
  // A Cartesian-smoothed motion brakes along its own TCP path; joint braking
  // would bend it. Joint-mode motions and the unsmoothed path brake per joint.
  cartesian_stop_=smoother_ && !joint_mode_ && status_.moving && !stopping_;
  if(cartesian_stop_) {
    status_.braking=true; status_.idle=false; status_.stale=false;
    status_.consumed_sequence=status_.sequence; arrival_=-1; reset_=false;
    return;
  }
  stopping_=bool(limiter_); status_.braking=stopping_;
  status_.moving=stopping_; status_.idle=!stopping_; status_.stale=false;
  status_.consumed_sequence=status_.sequence; arrival_=-1; reset_=false;
}
void Planner::submit(const PinkPose &target,double arrival,std::uint64_t seq,double duration) {
  if(!seeded_) throw std::logic_error("Planner must be feedback-seeded");
  if(!validPose(target)||!std::isfinite(arrival)||!std::isfinite(duration)||duration<0)
    throw std::invalid_argument("Invalid Cartesian target or time");
  if(seq<=status_.sequence) throw std::invalid_argument("Target sequence must increase");
  // Reject a target that jumps away from the previous target AND from where the
  // arm is: a jump back onto the arm (teleop re-anchoring to the measured TCP
  // after a collision hold left the old target behind the obstacle) is harmless.
  const auto requested=matrix(target);
  const auto jumps=[&](const PinkPose &reference) {
    const auto from=matrix(reference);
    return (config_.max_target_translation_jump>0 &&
        (from.block<3,1>(0,3)-requested.block<3,1>(0,3)).norm()>config_.max_target_translation_jump) ||
      (config_.max_target_rotation_jump>0 && Eigen::Quaterniond(from.block<3,3>(0,0)).angularDistance(
        Eigen::Quaterniond(requested.block<3,3>(0,0)))>config_.max_target_rotation_jump);
  };
  if(duration==0 && jumps(arrival_<0?pose_:target_) && jumps(pose_)) {
    // Format only on rejection; keep the streaming checks and their short-circuit
    // behavior unchanged. Report both references: they can trip different limits.
    auto error=jumpError("Cartesian",config_,seq,status_.sequence);
    error << "; limits={native_max_target_translation_jump=" << config_.max_target_translation_jump
          << " m" << (config_.max_target_translation_jump==0?" (disabled)":"")
          << ", native_max_target_rotation_jump=" << config_.max_target_rotation_jump
          << " rad" << (config_.max_target_rotation_jump==0?" (disabled)":"") << "}"
          << "; previous_reference=" << (arrival_<0?"planned_pose":"previous_target");
    const auto describe=[&](const char *name,const PinkPose &reference) {
      const auto from=matrix(reference);
      const double translation=(from.block<3,1>(0,3)-requested.block<3,1>(0,3)).norm();
      const double rotation=Eigen::Quaterniond(from.block<3,3>(0,0)).angularDistance(
        Eigen::Quaterniond(requested.block<3,3>(0,0)));
      const bool translation_exceeded=config_.max_target_translation_jump>0 && translation>config_.max_target_translation_jump;
      const bool rotation_exceeded=config_.max_target_rotation_jump>0 && rotation>config_.max_target_rotation_jump;
      error << "; " << name << "={translation=" << translation << " m, rotation=" << rotation << " rad, exceeded=[";
      if(translation_exceeded) error << "native_max_target_translation_jump";
      if(translation_exceeded && rotation_exceeded) error << ", ";
      if(rotation_exceeded) error << "native_max_target_rotation_jump";
      error << "]}";
    };
    describe("previous",arrival_<0?pose_:target_);
    describe("planned_pose",pose_);
    throw std::invalid_argument(error.str());
  }
  const bool resume=stopping_ || !status_.moving || joint_mode_;
  if(limiter_ && resume) { ref_q_=q_; ref_pose_=pose_; ref_velocity_.fill(0); }
  if(smoother_ && (resume || duration>0)) {
    // A fresh Cartesian stream starts from rest at the reference; a timed reset
    // has its own min-jerk profile and leaves the smoother idle.
    smoother_->reset(limiter_?ref_pose_:pose_); goal_=smoother_->pose();
  }
  const bool continuing=smoother_ && !resume && !cartesian_stop_ && duration==0;
  stopping_=false; cartesian_stop_=false; status_.braking=false;
  double interval=arrival_<0 ? config_.period : std::max(config_.period,arrival-arrival_);
  // A retarget continues from the reference, not from the (lagging) limited
  // output. With smoothing the waypoint stream itself stays continuous instead.
  start_=continuing?goal_:(limiter_?ref_pose_:pose_); target_=target;
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
  if(config_.max_target_joint_jump>0) {
    const bool previous=joint_mode_ && arrival_>=0;
    const auto &reference=previous?joint_target_:q_;
    for(int i=0;i<6;++i) if(std::abs(q[i]-reference[i])>config_.max_target_joint_jump) {
      auto error=jumpError("Joint",config_,seq,status_.sequence);
      error << "; native_max_target_joint_jump=" << config_.max_target_joint_jump << " rad"
            << "; reference=" << (previous?"previous_joint_target":"planned_q") << "; exceeded=[";
      bool first=true;
      for(int j=0;j<6;++j) {
        const double delta=std::abs(q[j]-reference[j]);
        if(delta<=config_.max_target_joint_jump) continue;
        if(!first) error << ", ";
        first=false;
        error << "J" << j+1 << "{delta=" << delta << " rad, target=" << q[j]
              << " rad, reference=" << reference[j] << " rad}";
      }
      error << "]";
      throw std::invalid_argument(error.str());
    }
  }
  submit(pose_,arrival,seq); joint_mode_=true; joint_target_=q;
}
PinkArmVector Planner::tick(double now) {
  if(planReference(now)) return advance();
  return q_;
}
bool Planner::planReference(double now) {
  if(!seeded_||!std::isfinite(now)) throw std::logic_error("Invalid planner tick");
  pending_=false;
  if(arrival_>=0 && !reset_ && now-arrival_>config_.stale_timeout) {
    cancel(); status_.stale=true;
  }
  if(stopping_) {
    q_=limiter_->step(q_,true); pose_=model_.forwardKinematics(0,q_);
    ref_q_=q_; ref_pose_=pose_; ref_velocity_.fill(0);
    status_.joint_velocity=limiter_->velocity(); status_.joint_acceleration=limiter_->acceleration();
    if(limiter_->finished()) { stopping_=false; status_.braking=false; status_.moving=false; status_.idle=true; }
    return false;
  }
  if(!status_.moving) return false;
  if(first_tick_) { start_time_=now; first_tick_=false; }
  pending_now_=now;
  pending_alpha_=std::clamp((now-start_time_+config_.period)/duration_,0.0,1.0);
  pending_candidate_=q_;
  pending_converged_=true;
  ref_before_=ref_q_;
  if(joint_mode_) pending_candidate_=joint_target_;
  else {
    const auto begin=std::chrono::steady_clock::now();
    // The IK follows the reference pose (the limited output when there is no
    // limiter). Bound the waypoint to one period of Cartesian speed from it, so
    // elapsed interpolation time cannot build up a catch-up jump while IK is blocked.
    const PinkPose &anchor=limiter_?ref_pose_:pose_;
    const PinkArmVector &seed=limiter_?ref_q_:q_;
    const bool smoothing=smoother_ && !reset_;
    PinkPose waypoint;
    if(smoothing) {
      // The interpolated waypoint is the target stream; its twist is the feed-forward.
      CartesianSmoother::Vector6 feedforward=CartesianSmoother::Vector6::Zero();
      if(cartesian_stop_) waypoint=smoother_->pose();
      else {
        waypoint=interpolatePose(start_,target_,pending_alpha_);
        feedforward=CartesianSmoother::difference(goal_,waypoint)/config_.period;
        goal_=waypoint;
      }
      waypoint=smoother_->step(waypoint,feedforward,config_.period);
    } else waypoint=interpolatePose(start_,target_,reset_?minJerk(pending_alpha_):pending_alpha_);
    const auto current=matrix(anchor), goal=matrix(waypoint);
    const double distance=(goal.block<3,1>(0,3)-current.block<3,1>(0,3)).norm();
    const double angle=Eigen::Quaterniond(current.block<3,3>(0,0)).angularDistance(
      Eigen::Quaterniond(goal.block<3,3>(0,0)));
    double fraction=1.0;
    // The smoother already holds the speed limit; capping again at exactly that
    // speed lets the reference fall behind it and later catch up in a jump.
    // Keep only a jump guard then.
    const double headroom=smoothing?kSmoothedStepHeadroom:1.0;
    if(distance>0) fraction=std::min(fraction,headroom*config_.max_linear_velocity*config_.period/distance);
    if(angle>0) fraction=std::min(fraction,headroom*config_.max_angular_velocity*config_.period/angle);
    waypoint=interpolatePose(anchor,waypoint,fraction);
    PinkArmVector candidate=q_, box=max_step_;
    for(double &step:box) step*=kTrustFactor;
    const bool ok=model_.solve(0,waypoint,seed,box,candidate,status_.ik,!reset_);
    double factor=1.0;
    candidate=scaleStep(seed,candidate,factor);
    pending_converged_=ok && fraction>=1.0 && factor>=1.0;
    status_.step_scale=factor; status_.waypoint_fraction=fraction;
    // Joint speed limited the step: slow the Cartesian path by the same factor
    // (ros2_control-style time scaling) instead of letting it run ahead.
    if(smoothing && factor<1.0 && (ok || status_.ik.progress))
      smoother_->retime(model_.forwardKinematics(0,candidate),factor);
    status_.solve_time_s=std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();
    if(!ok && reset_) {
      ++status_.ik_failures;
      auto consumed=status_.consumed_sequence;
      cancel(); status_.consumed_sequence=consumed;
      return limiter_?planReference(now):false;
    }
    const bool advanced=ok || status_.ik.progress;
    if(!advanced) ++status_.ik_failures;
    if(limiter_) {
      // Keep tracking a held reference until cancellation/staleness; the output
      // settles on it smoothly and a fresh target can recover.
      if(advanced) { ref_q_=candidate; ref_pose_=model_.forwardKinematics(0,ref_q_); }
      for(int i=0;i<6;++i) ref_velocity_[i]=(ref_q_[i]-ref_before_[i])/config_.period;
      pending_candidate_=ref_q_;
    } else if(!advanced) return false;  // hold this tick; never accept an unconverged global IK
    else pending_candidate_=candidate;
  }
  pending_=true;
  return true;
}
void Planner::setReference(const PinkArmVector &q) {
  if(!limiter_) throw std::logic_error("A joint reference exists only with joint motion limits");
  if(!pending_ || joint_mode_) throw std::logic_error("set_reference belongs between plan_reference and advance");
  // ref_pose_ already holds this tick's IK solution; the motion is from the previous tick.
  const PinkPose before=model_.forwardKinematics(0,ref_before_);
  ref_pose_=model_.forwardKinematics(0,q); ref_q_=q; pending_candidate_=q;
  // The constrained reference is where the smoothed TCP now is, moving as the
  // reference actually moved; otherwise the smoother winds up behind an obstacle.
  if(smoother_ && !reset_)
    smoother_->reset(ref_pose_,CartesianSmoother::difference(before,ref_pose_)/config_.period);
  // The feed-forward is the motion the reference actually makes, not what the
  // IK asked for; otherwise a blocked reference keeps pushing the output.
  for(int i=0;i<6;++i) ref_velocity_[i]=(q[i]-ref_before_[i])/config_.period;
}
PinkArmVector Planner::advance() {
  if(!pending_) throw std::logic_error("advance() needs a successful plan_reference()");
  pending_=false;
  const double alpha=pending_alpha_;
  const auto &candidate=pending_candidate_;
  bool reached=true;
  if(limiter_) {
    bool reference_still=true;
    for(double v:ref_velocity_) reference_still=reference_still && std::abs(v)<=kStillVelocity;
    // A still reference at the end of the segment is a true goal: stop on it
    // time-optimally. A moving one is followed with its velocity as feed-forward;
    // a zero-velocity position target would keep a braking distance behind it.
    // With Cartesian smoothing the reference is already feasible and the
    // follower reproduces it (its feed-forward is the reference velocity); it
    // only departs from it, smoothly, when the reference itself steps (held by
    // a collision layer, or near a singularity).
    if(joint_mode_ || (alpha>=1.0 && reference_still)) q_=limiter_->step(candidate);
    else q_=limiter_->follow(candidate,ref_velocity_,config_.joint_tracking_gain);
    status_.joint_velocity=limiter_->velocity(); status_.joint_acceleration=limiter_->acceleration();
    reached=limiter_->finished() && (joint_mode_ || reference_still);
    if(joint_mode_) { ref_q_=q_; ref_velocity_.fill(0); }
  } else for(int i=0;i<6;++i) {
    const double delta=candidate[i]-q_[i];
    if(std::abs(delta)>max_step_[i]) reached=false;
    q_[i]+=std::clamp(delta,-max_step_[i],max_step_[i]);
  }
  pose_=model_.forwardKinematics(0,q_);
  if(limiter_ && joint_mode_) ref_pose_=pose_;
  if(smoother_ && !joint_mode_ && !reset_) {
    if(cartesian_stop_) {
      if(smoother_->still() && reached) {
        cartesian_stop_=false; status_.braking=false; status_.moving=false; status_.idle=true;
      }
      return q_;
    }
    reached=reached && smoother_->settled(goal_);
  }
  if((joint_mode_||alpha>=1.0) && reached && pending_converged_) {
    status_.moving=false; status_.consumed_sequence=status_.sequence;
    if(reset_) { arrival_=pending_now_; reset_=false; }
  }
  return q_;
}
} // namespace libpyelite
