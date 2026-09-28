#pragma once
#include "pink_ik.hpp"
#include <cstdint>

namespace libpyelite {
struct PlannerConfig {
  double period{0.004};
  double stale_timeout{0.5};
  double max_linear_velocity{0.5};
  double max_angular_velocity{1.0};
  double velocity_scale{1.0};
};
struct PlannerStatus {
  std::uint64_t sequence{0}, consumed_sequence{0}, ik_failures{0};
  bool moving{false}, stale{false}, idle{true};
  double solve_time_s{0};
  PinkIKDiagnostics ik;
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
  void submit(const PinkPose &target, double arrival, std::uint64_t sequence, double duration = 0.0);
  void submitJoints(const PinkArmVector &target, double arrival, std::uint64_t sequence);
  void cancel();
  PinkArmVector tick(double now);
  PinkArmVector q() const { return q_; }
  PinkPose pose() const { return pose_; }
  PlannerStatus status() const { return status_; }
  PinkIKModel &model() { return model_; }
 private:
  PinkIKModel model_;
  PlannerConfig config_;
  PinkArmVector q_{}, joint_target_{}, max_step_{};
  PinkPose pose_{kIdentityPose}, start_{kIdentityPose}, target_{kIdentityPose};
  double arrival_{-1}, start_time_{0}, duration_{0};
  bool seeded_{false}, reset_{false}, joint_mode_{false}, first_tick_{false};
  PlannerStatus status_;
};
} // namespace libpyelite
