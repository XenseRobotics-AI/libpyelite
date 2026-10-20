#pragma once
#include "pink_ik.hpp"
#include "motion_limiter.hpp"
#include "cartesian_smoother.hpp"
#include <cstdint>
#include <string>

namespace libpyelite {
struct PlannerConfig {
  double period{0.004};
  double stale_timeout{0.5};
  double max_linear_velocity{0.5};
  double max_angular_velocity{1.0};
  double velocity_scale{1.0};
  double joint_target_horizon{0.0}; // deprecated, ignored: limits track a joint reference instead
  bool joint_motion_limits{false}; // opt-in: explicit six-axis limits required
  PinkArmVector max_joint_velocity{}, max_joint_acceleration{}, max_joint_jerk{};
  double max_target_translation_jump{0}; // 0 disables, metres
  double max_target_rotation_jump{0}; // radians
  double max_target_joint_jump{0}; // radians
  double joint_tracking_gain{20.0}; // 1/s, position correction while following a moving reference
  // Opt-in Cartesian smoothing before IK; magnitudes of the linear/angular twist parts.
  // Velocity limits are max_linear_velocity / max_angular_velocity above.
  bool cartesian_motion_limits{false};
  double max_linear_acceleration{2.0}, max_angular_acceleration{5.0};  // m/s², rad/s² (Flexiv defaults)
  double max_linear_jerk{1000.0}, max_angular_jerk{2500.0};           // m/s³, rad/s³ (MoveIt fallback 1000)
  double cartesian_tracking_gain{20.0};                                 // 1/s
  std::string arm_name; // optional diagnostic identity, e.g. left/right; does not affect planning
  // CS66-only session policy. Each arm captures its own controller coordinates;
  // mount rotations never change the meaning of these joint angles.
  bool preserve_configuration{false};
  double configuration_margin{0.05235987755982989};  // 3 degrees, elbow/wrist
  double configuration_joint_span{3.140592653589793}; // unwrapped half-span < pi
  double boundary_orientation_tolerance{0.01};       // rad, nonlinear acceptance
};
struct PlannerStatus {
  std::uint64_t sequence{0}, consumed_sequence{0}, ik_failures{0};
  bool moving{false}, stale{false}, idle{true};
  double solve_time_s{0};
  PinkIKDiagnostics ik;
  PinkArmVector joint_velocity{}, joint_acceleration{};
  bool braking{false};
  bool boundary_limited{false};
  bool boundary_braking{false};
  std::uint64_t boundary_holds{0};
  double reference_position_error{0}, reference_orientation_error{0};
  // Last streaming step: joint-speed scale applied to the IK step (1 = none) and
  // the fraction of the Cartesian waypoint step kept by the TCP speed cap.
  double step_scale{1.0}, waypoint_fraction{1.0};
};
PinkPose poseFromRotvec(const PinkArmVector &pose);
PinkPose poseFromControllerTcp(const PinkArmVector &pose);
PinkArmVector poseToRotvec(const PinkPose &pose);
PinkPose interpolatePose(const PinkPose &a, const PinkPose &b, double alpha);
bool validPose(const PinkPose &pose);

// Hardware-free single-arm planner. Targets are already in controller base axes.
// Latest-target mailbox and thread synchronization are owned by NativeDriver.
class Planner {
 public:
  Planner(const std::string &urdf, const PinkPose &tool, PinkIKOptions ik, PlannerConfig config);
  void seed(const PinkArmVector &q);
  void reanchor(const PinkArmVector &q,const PinkArmVector &v,const PinkArmVector &a);
  // Joint reference the limited output follows (equals q() without joint motion limits).
  PinkArmVector reference() const { return limiter_?ref_q_:q_; }
  // tick() == planReference() then advance(). A layer that must constrain the
  // joint reference (the coupled collision projection) calls the two halves and
  // setReference() in between, so the output never follows an unconstrained one.
  bool planReference(double now);  // false: no output step pending this tick
  void setReference(const PinkArmVector &q);
  PinkArmVector advance();
  void submit(const PinkPose &target, double arrival, std::uint64_t sequence, double duration = 0.0);
  void submitJoints(const PinkArmVector &target, double arrival, std::uint64_t sequence);
  void cancel();
  PinkArmVector tick(double now);
  PinkArmVector q() const { return q_; }
  PinkPose pose() const { return pose_; }
  PlannerStatus status() const { return status_; }
  PinkIKModel &model() { return model_; }
  const PinkIKModel &model() const { return model_; }
  const PlannerConfig &config() const { return config_; }
  std::unique_ptr<MotionLimiter> makeMotionLimiter(const PinkArmVector &seed) const;
  bool pending() const { return pending_; }   // planReference() left an output step to advance()
  bool jointMode() const { return joint_mode_; }
  // The goal the arm is being steered to, base axes: the latest Cartesian
  // target, or the FK of the joint target in joint mode.
  PinkPose goal() const { return joint_mode_ ? model_.forwardKinematics(0,joint_target_) : target_; }
 private:
  PinkIKModel model_;
  PlannerConfig config_;
  PinkArmVector q_{}, joint_target_{}, max_step_{};
  // With joint motion limits the IK runs on an unlimited reference (ref_q_), and
  // the Ruckig output tracks its position *and* velocity instead of stopping at it.
  PinkArmVector ref_q_{}, ref_velocity_{};
  PinkPose ref_pose_{kIdentityPose};
  PinkArmVector ref_before_{}, pending_candidate_{};
  double pending_now_{0}, pending_alpha_{0};
  bool pending_{false}, pending_converged_{true};
  static constexpr double kStillVelocity=1e-6;  // rad/s, a projected reference is never exactly still
  PinkPose pose_{kIdentityPose}, start_{kIdentityPose}, target_{kIdentityPose};
  double arrival_{-1}, start_time_{0}, duration_{0};
  bool seeded_{false}, reset_{false}, joint_mode_{false}, first_tick_{false};
  PlannerStatus status_;
  std::unique_ptr<MotionLimiter> limiter_;
  bool stopping_{false};
  // Cartesian smoothing: goal_ is the last interpolated waypoint (the target
  // stream, continuous across retargets); the smoother's pose is what IK tracks.
  std::unique_ptr<CartesianSmoother> smoother_;
  PinkPose goal_{kIdentityPose};
  bool cartesian_stop_{false};
  // IK solves inside a wider per-joint box, then the whole step is scaled by one
  // factor to max_step_, keeping its direction (MoveIt Servo, Drake diff-IK).
  static constexpr double kTrustFactor=4.0;
  static constexpr double kSmoothedStepHeadroom=2.0;
  PinkArmVector scaleStep(const PinkArmVector &from, PinkArmVector to, double &factor) const;
  void feedbackReference(const PinkPose &before);
};
} // namespace libpyelite
