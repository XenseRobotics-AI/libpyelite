#pragma once
#include "planner.hpp"
#include "projection_qp.hpp"
#include "station_collision.hpp"

#include <array>
#include <memory>
#include <mutex>
#include <string>

namespace libpyelite {

struct GuardResult {
  std::array<PinkArmVector, 2> q{};     // published joints per arm (left, right)
  std::array<bool, 2> held{};           // reference held: target is behind an obstacle
  std::array<bool, 2> limited{};        // output changed by the collision gate
  ClearanceResult clearance;            // closest pair this step (within the detection band)
  int constraints{0};                   // barrier rows this step
  double solve_time_s{0};
  // Diagnostics: ADMM iterations of the two projections (0 = not needed), and
  // pairs the step verification had to sample (0 = motion bound sufficed).
  int reference_iterations{0}, output_iterations{0}, sampled_pairs{0};
  std::array<double, 2> gripper_age_s{{-1, -1}};
  std::array<std::uint64_t, 2> gripper_sequence{};
  bool measured_geometry_hold{false};  // fresh free-moving gripper geometry violates clearance
};

// One control step of the two arms under the station collision layer.
// The same object runs in libpyelite.sim and in the hardware StationCoordinator,
// so what the simulator shows is what the robot executes:
//  1. each active planner plans its joint reference (Cartesian smoothing + IK);
//  2. distances and barrier rows are linearised once, at the published joints;
//  3. the references are projected (TCP metric); an arm whose blocked step
//     would not bring its TCP closer to the goal is held instead of sliding;
//  4. the joint limiters advance; the outputs pass a joint QP gate (hard:
//     velocity, position, barrier; soft: acceleration/jerk), joint-target arms
//     only along their commanded joint step;
//  5. the step is verified on the real geometry (motion bound, then samples),
//     shortened if needed; a corrected arm's planner is re-anchored.
// An arm given without a planner (servo stopped, MoveJ) is a fixed obstacle
// at the given joints. Anything inconsistent throws; the caller stops both arms.
class StationGuard {
 public:
  StationGuard(std::shared_ptr<StationCollisionModel> collision, const std::string &station_urdf,
               const Eigen::VectorXd &default_q, const std::array<PinkPose, 2> &tools);
  ~StationGuard();
  StationGuard(const StationGuard &) = delete;
  StationGuard &operator=(const StationGuard &) = delete;

  GuardResult tick(Planner *left, Planner *right, const PinkArmVector &left_fixed,
                   const PinkArmVector &right_fixed, double now);
  // Forget the command derivatives (after a reset/seed of the arms).
  void reset();
  // Station configuration (every URDF joint) for the two arms' joints.
  Eigen::VectorXd stationQ(const std::array<PinkArmVector, 2> &q) const;
  // Configure once before starting the coordinator. A measured snapshot is
  // mandatory for every mapped side; simulation keeps its static defaults.
  void configureGrippers(const std::vector<std::string> &joints, const std::vector<int> &sides,
                         double max_age_s);
  void updateGrippers(const std::vector<double> &positions, const std::array<double, 2> &sample_times,
                      const std::array<std::uint64_t, 2> &sequences, const std::array<bool, 2> &valid);
  std::shared_ptr<StationCollisionModel> collision() const { return collision_; }

  // Orientation rows weighted as a point this far from the TCP; a small joint
  // term keeps the metric definite at singularities.
  static constexpr double kMetricRotationRadius = 0.1, kMetricRegularization = 1e-3;
  // Per-step slack on "no worse": round-off, not a budget a slide may spend.
  static constexpr double kHoldPosition = 1e-7, kHoldOrientation = 1e-6;

 private:
  struct Model;
  struct Grippers {
    std::vector<double> positions;
    std::array<double, 2> sample_times{};
    std::array<std::uint64_t, 2> sequences{};
    std::array<bool, 2> valid{};
  };
  Eigen::VectorXd gripperDefaults(GuardResult *result = nullptr) const;
  Eigen::VectorXd stationQ(const std::array<PinkArmVector, 2> &q, const Eigen::VectorXd &defaults) const;
  Eigen::MatrixXd taskMetric(const Eigen::VectorXd &station_q);
  Eigen::VectorXd project(const Eigen::VectorXd &proposed, const Eigen::VectorXd &lower,
                          const Eigen::VectorXd &upper, const Eigen::VectorXd &soft_lower,
                          const Eigen::VectorXd &soft_upper, const Eigen::MatrixXd &metric,
                          const Eigen::MatrixXd &g, const Eigen::VectorXd &h, ProjectionQP &solver);

  std::shared_ptr<StationCollisionModel> collision_;
  std::unique_ptr<Model> model_;
  std::array<PinkPose, 2> tools_;
  std::vector<int> velocity_indices_;
  ProjectionQP reference_qp_, output_qp_;
  Eigen::VectorXd command_velocity_{Eigen::VectorXd::Zero(12)}, command_acceleration_{Eigen::VectorXd::Zero(12)};
  std::array<bool, 2> was_active_{};
  int last_iterations_{0};
  std::vector<int> gripper_indices_, gripper_sides_;
  std::array<bool, 2> gripper_required_{};
  double gripper_max_age_s_{0};
  std::shared_ptr<const Grippers> grippers_;
  Grippers gripper_watermark_;  // last accepted sequence survives invalid publications
  Eigen::VectorXd gripper_checked_defaults_;
  bool measured_geometry_hold_{false};
  std::mutex gripper_publish_mutex_;  // producer only; the planner never takes this lock
};
}  // namespace libpyelite
