#include "station_guard.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>

// Pinocchio's joint collection has 25 alternatives; see pink_ik.cpp.
#include <pinocchio/deprecated/pinocchio/container/boost-container-limits.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/skew.hpp>

namespace libpyelite {
namespace {
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kChanged = 1e-9;  // rad: a projection that moved a joint by less is "unchanged"
const std::array<const char *, 6> kJoints{{"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                           "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"}};
const std::array<const char *, 2> kSides{{"left_", "right_"}};

Eigen::Matrix4d matrix(const PinkPose &p) {
  return Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(p.data());
}

// Position and rotation-angle error from pose to goal (both base axes).
std::pair<double, double> poseError(const PinkPose &pose, const PinkPose &goal) {
  const Eigen::Matrix4d a = matrix(pose), b = matrix(goal);
  const Eigen::Matrix3d relative = a.topLeftCorner<3, 3>().transpose() * b.topLeftCorner<3, 3>();
  return {(b.topRightCorner<3, 1>() - a.topRightCorner<3, 1>()).norm(), Eigen::AngleAxisd(relative).angle()};
}

// May a blocked arm slide from q0 to q1? Only if its TCP is no farther from the
// goal in position or orientation, and closer in one.
bool closer(const Planner &planner, const PinkArmVector &q0, const PinkArmVector &q1) {
  const PinkPose goal = planner.goal();
  const auto [p0, r0] = poseError(planner.model().forwardKinematics(0, q0), goal);
  const auto [p1, r1] = poseError(planner.model().forwardKinematics(0, q1), goal);
  if(p1 > p0 + StationGuard::kHoldPosition || r1 > r0 + StationGuard::kHoldOrientation) return false;
  return p1 < p0 - StationGuard::kHoldPosition || r1 < r0 - StationGuard::kHoldOrientation;
}

Eigen::VectorXd stack(const std::array<PinkArmVector, 2> &q) {
  Eigen::VectorXd out(12);
  for(int a = 0; a < 2; ++a)
    for(int i = 0; i < 6; ++i) out[6 * a + i] = q[static_cast<std::size_t>(a)][static_cast<std::size_t>(i)];
  return out;
}
std::array<PinkArmVector, 2> split(const Eigen::VectorXd &q) {
  std::array<PinkArmVector, 2> out{};
  for(int a = 0; a < 2; ++a)
    for(int i = 0; i < 6; ++i) out[static_cast<std::size_t>(a)][static_cast<std::size_t>(i)] = q[6 * a + i];
  return out;
}
}  // namespace

struct StationGuard::Model {
  pinocchio::Model model;
  pinocchio::Data data;
  std::array<std::array<int, 6>, 2> q_index{}, v_index{};
  std::array<pinocchio::FrameIndex, 2> tool0{};
  Eigen::VectorXd defaults;
};

StationGuard::StationGuard(std::shared_ptr<StationCollisionModel> collision, const std::string &station_urdf,
                           const Eigen::VectorXd &default_q, const std::array<PinkPose, 2> &tools)
    : collision_(std::move(collision)), model_(std::make_unique<Model>()), tools_(tools) {
  if(!collision_) throw std::invalid_argument("StationGuard needs a collision model");
  auto &m = *model_;
  pinocchio::urdf::buildModel(station_urdf, m.model);
  m.data = pinocchio::Data(m.model);
  if(m.model.nq != collision_->nq() || default_q.size() != m.model.nq || !default_q.allFinite())
    throw std::invalid_argument("Station URDF, collision model and default configuration disagree");
  m.defaults = default_q;
  for(int a = 0; a < 2; ++a) {
    const std::string side = kSides[static_cast<std::size_t>(a)];
    for(int i = 0; i < 6; ++i) {
      const std::string name = side + kJoints[static_cast<std::size_t>(i)];
      if(!m.model.existJointName(name)) throw std::invalid_argument("Station URDF has no joint " + name);
      const auto &joint = m.model.joints[m.model.getJointId(name)];
      m.q_index[static_cast<std::size_t>(a)][static_cast<std::size_t>(i)] = joint.idx_q();
      m.v_index[static_cast<std::size_t>(a)][static_cast<std::size_t>(i)] = joint.idx_v();
      velocity_indices_.push_back(joint.idx_v());
    }
    if(!m.model.existFrame(side + "tool0")) throw std::invalid_argument("Station URDF has no " + side + "tool0");
    m.tool0[static_cast<std::size_t>(a)] = m.model.getFrameId(side + "tool0");
  }
}

StationGuard::~StationGuard() = default;

void StationGuard::reset() {
  command_velocity_.setZero();
  command_acceleration_.setZero();
}

Eigen::VectorXd StationGuard::stationQ(const std::array<PinkArmVector, 2> &q) const {
  Eigen::VectorXd out = model_->defaults;
  for(std::size_t a = 0; a < 2; ++a)
    for(std::size_t i = 0; i < 6; ++i) out[model_->q_index[a][i]] = q[a][i];
  return out;
}

Eigen::MatrixXd StationGuard::taskMetric(const std::array<PinkArmVector, 2> &q) {
  auto &m = *model_;
  pinocchio::computeJointJacobians(m.model, m.data, stationQ(q));
  pinocchio::updateFramePlacements(m.model, m.data);
  Eigen::MatrixXd metric = Eigen::MatrixXd::Zero(12, 12);
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian(6, m.model.nv);
  for(std::size_t a = 0; a < 2; ++a) {
    jacobian.setZero();
    pinocchio::getFrameJacobian(m.model, m.data, m.tool0[a], pinocchio::LOCAL_WORLD_ALIGNED, jacobian);
    const Eigen::Matrix4d tool = matrix(tools_[a]);
    const Eigen::Vector3d offset = m.data.oMf[m.tool0[a]].rotation() * tool.topRightCorner<3, 1>();
    Eigen::Matrix<double, 6, 6> task;
    for(std::size_t i = 0; i < 6; ++i) {
      const auto column = jacobian.col(m.v_index[a][i]);
      task.col(static_cast<Eigen::Index>(i)) << column.head<3>() - pinocchio::skew(offset) * column.tail<3>(),
          kMetricRotationRadius * column.tail<3>();
    }
    Eigen::Matrix<double, 6, 6> block = task.transpose() * task;
    block.diagonal().array() += kMetricRegularization * block.trace() / 6;
    metric.block<6, 6>(static_cast<Eigen::Index>(6 * a), static_cast<Eigen::Index>(6 * a)) = block;
  }
  return metric;
}

Eigen::VectorXd StationGuard::project(const Eigen::VectorXd &proposed, const Eigen::VectorXd &lower,
                                      const Eigen::VectorXd &upper, const Eigen::VectorXd &soft_lower,
                                      const Eigen::VectorXd &soft_upper, const Eigen::MatrixXd &metric,
                                      const Eigen::MatrixXd &g, const Eigen::VectorXd &h, ProjectionQP &solver) {
  // A step that already satisfies everything is taken as is (round-off on the
  // finite-difference bounds must not cost a solve).
  const double slack = 1e-9 * std::max(1.0, (upper - lower).cwiseAbs().maxCoeff());
  const Eigen::VectorXd floor = lower.cwiseMax((soft_lower.array() - slack).matrix());
  const Eigen::VectorXd ceiling = upper.cwiseMin((soft_upper.array() + slack).matrix());
  const bool within = (proposed.array() >= floor.array()).all() && (proposed.array() <= ceiling.array()).all() &&
                      (g.rows() == 0 || ((g * proposed).array() <= h.array() + 1e-10).all());
  if(within) { last_iterations_ = 0; return proposed; }
  auto x = solver.solve(metric, proposed, lower, upper, soft_lower, soft_upper, g, h);
  last_iterations_ = solver.lastIterations();
  return x;
}

GuardResult StationGuard::tick(Planner *left, Planner *right, const PinkArmVector &left_fixed,
                               const PinkArmVector &right_fixed, double now) {
  const auto begin = std::chrono::steady_clock::now();
  std::array<Planner *, 2> planners{{left, right}};
  const std::array<PinkArmVector, 2> fixed{{left_fixed, right_fixed}};
  std::array<bool, 2> active{};
  std::array<PinkArmVector, 2> previous{}, reference{};
  double period = 0;
  for(std::size_t a = 0; a < 2; ++a) {
    active[a] = planners[a] != nullptr;
    previous[a] = active[a] ? planners[a]->q() : fixed[a];
    reference[a] = active[a] ? planners[a]->reference() : fixed[a];
    if(active[a]) {
      const double p = planners[a]->config().period;
      if(period > 0 && std::abs(p - period) > 1e-12) throw std::invalid_argument("Coupled arms need one control period");
      period = p;
    }
    if(active[a] != was_active_[a]) {  // a (re)started arm starts from rest
      command_velocity_.segment<6>(static_cast<Eigen::Index>(6 * a)).setZero();
      command_acceleration_.segment<6>(static_cast<Eigen::Index>(6 * a)).setZero();
    }
    was_active_[a] = active[a];
  }
  GuardResult result;
  if(period <= 0) {  // nothing to drive
    result.q = previous;
    return result;
  }
  for(std::size_t a = 0; a < 2; ++a)
    if(active[a]) planners[a]->planReference(now);

  // 2. One linearisation per step, at the published joints.
  const Eigen::VectorXd q0 = stack(previous);
  const Eigen::VectorXd station0 = stationQ(previous);
  Eigen::MatrixXd g;
  Eigen::VectorXd h;
  ClearanceResult closest;
  collision_->linearize(station0, velocity_indices_, g, h, closest);
  const Eigen::MatrixXd metric = taskMetric(previous);
  result.constraints = static_cast<int>(h.size());

  // Joint position limits and the per-step velocity box, per arm.
  Eigen::VectorXd joint_lower(12), joint_upper(12), vmax(12);
  for(std::size_t a = 0; a < 2; ++a) {
    const Eigen::Index o = static_cast<Eigen::Index>(6 * a);
    if(!active[a]) { joint_lower.segment<6>(o) = q0.segment<6>(o); joint_upper.segment<6>(o) = q0.segment<6>(o); vmax.segment<6>(o).setZero(); continue; }
    const auto &model = planners[a]->model();
    const auto &config = planners[a]->config();
    const auto lo = model.lowerLimits(), hi = model.upperLimits(), velocity = model.velocityLimits();
    for(std::size_t i = 0; i < 6; ++i) {
      joint_lower[o + static_cast<Eigen::Index>(i)] = lo[i];
      joint_upper[o + static_cast<Eigen::Index>(i)] = hi[i];
      vmax[o + static_cast<Eigen::Index>(i)] =
          config.joint_motion_limits ? config.max_joint_velocity[i] : velocity[i] * config.velocity_scale;
    }
  }
  const Eigen::VectorXd no_lower = Eigen::VectorXd::Constant(12, -kInf), no_upper = Eigen::VectorXd::Constant(12, kInf);

  // 3. Reference projection (needs the joint reference, i.e. joint motion limits).
  bool reference_layer = true;
  for(std::size_t a = 0; a < 2; ++a)
    if(active[a] && !planners[a]->config().joint_motion_limits) reference_layer = false;
  if(reference_layer) {
    std::array<bool, 2> free{};
    for(std::size_t a = 0; a < 2; ++a) free[a] = active[a] && planners[a]->pending() && !planners[a]->jointMode();
    if(free[0] || free[1]) {
      const Eigen::VectorXd r0 = stack(reference);
      Eigen::VectorXd current(12);
      for(std::size_t a = 0; a < 2; ++a)
        for(std::size_t i = 0; i < 6; ++i)
          current[static_cast<Eigen::Index>(6 * a + i)] = active[a] ? planners[a]->reference()[i] : fixed[a][i];
      // Shift the rows to the reference (G (r0 + d - q0) <= h), never below 0 so holding stays feasible.
      const Eigen::VectorXd shifted = (h - g * (r0 - q0)).cwiseMax(0.0);
      Eigen::VectorXd lower = (-vmax * period * (1 + 1e-6)).cwiseMax(joint_lower - r0);
      Eigen::VectorXd upper = (vmax * period * (1 + 1e-6)).cwiseMin(joint_upper - r0);
      Eigen::VectorXd proposed = (current - r0).cwiseMax(lower).cwiseMin(upper);
      for(std::size_t a = 0; a < 2; ++a)
        if(!free[a]) {
          const Eigen::Index o = static_cast<Eigen::Index>(6 * a);
          lower.segment<6>(o).setZero(); upper.segment<6>(o).setZero(); proposed.segment<6>(o).setZero();
        }
      Eigen::VectorXd displacement;
      while(true) {
        displacement = project(proposed, lower, upper, no_lower, no_upper, metric, g, shifted, reference_qp_);
        result.reference_iterations += last_iterations_;
        bool any = false;
        for(std::size_t a = 0; a < 2; ++a) {
          const Eigen::Index o = static_cast<Eigen::Index>(6 * a);
          if(!free[a] || (displacement.segment<6>(o) - proposed.segment<6>(o)).cwiseAbs().maxCoeff() <= kChanged) continue;
          const auto from = split(r0)[a], to = split(r0 + displacement)[a];
          if(closer(*planners[a], from, to)) continue;
          lower.segment<6>(o).setZero(); upper.segment<6>(o).setZero(); proposed.segment<6>(o).setZero();
          free[a] = false; result.held[a] = true; any = true;
        }
        if(!any) break;
      }
      const auto projected = split(r0 + displacement);
      for(std::size_t a = 0; a < 2; ++a) {
        if(!active[a] || !planners[a]->pending() || planners[a]->jointMode()) continue;
        double change = 0;
        for(std::size_t i = 0; i < 6; ++i) change = std::max(change, std::abs(projected[a][i] - planners[a]->reference()[i]));
        if(change > kChanged) planners[a]->setReference(projected[a]);
      }
    }
  }

  // 4. Limiters, then the joint QP gate on the published step.
  std::array<PinkArmVector, 2> advanced = previous;
  for(std::size_t a = 0; a < 2; ++a)
    if(active[a]) advanced[a] = planners[a]->pending() ? planners[a]->advance() : planners[a]->q();
  const Eigen::VectorXd q1 = stack(advanced);
  const Eigen::VectorXd proposed = q1 - q0;
  const Eigen::VectorXd lower = (-vmax * period).cwiseMax(joint_lower - q0);
  const Eigen::VectorXd upper = (vmax * period).cwiseMin(joint_upper - q0);
  Eigen::VectorXd soft_lower = no_lower, soft_upper = no_upper;
  for(std::size_t a = 0; a < 2; ++a) {
    if(!active[a] || !planners[a]->config().joint_motion_limits) continue;
    const auto &config = planners[a]->config();
    for(std::size_t i = 0; i < 6; ++i) {
      const Eigen::Index k = static_cast<Eigen::Index>(6 * a + i);
      // Bounds on finite differences of the published joints. Soft: the barrier
      // may brake harder than they allow near an obstacle.
      const double amin = std::max(-config.max_joint_acceleration[i], command_acceleration_[k] - config.max_joint_jerk[i] * period);
      const double amax = std::min(config.max_joint_acceleration[i], command_acceleration_[k] + config.max_joint_jerk[i] * period);
      soft_lower[k] = (command_velocity_[k] + amin * period) * period;
      soft_upper[k] = (command_velocity_[k] + amax * period) * period;
    }
  }
  Eigen::VectorXd displacement = project(proposed, lower, upper, soft_lower, soft_upper, metric, g, h, output_qp_);
  result.output_iterations = last_iterations_;
  // Joint-target arms keep their commanded joint direction, only shortened.
  for(std::size_t a = 0; a < 2; ++a) {
    const Eigen::Index o = static_cast<Eigen::Index>(6 * a);
    if(!active[a] || !planners[a]->jointMode() ||
       (displacement.segment<6>(o) - proposed.segment<6>(o)).cwiseAbs().maxCoeff() <= kChanged) continue;
    const Eigen::VectorXd own = proposed.segment<6>(o).cwiseMax(lower.segment<6>(o)).cwiseMin(upper.segment<6>(o));
    Eigen::VectorXd others = displacement;
    others.segment<6>(o).setZero();
    const Eigen::VectorXd demand = g.middleCols<6>(o) * own;
    const Eigen::VectorXd room = h - g * others;
    double fraction = 1.0;
    for(Eigen::Index r = 0; r < demand.size(); ++r)
      if(demand[r] > 1e-12) fraction = std::min(fraction, room[r] / demand[r]);
    displacement.segment<6>(o) = std::clamp(fraction, 0.0, 1.0) * own;
  }

  // 5. Verify on the real geometry; shorten (finally hold) rather than fault on a first-order miss.
  Eigen::VectorXd accepted = q0;
  ClearanceResult verified = closest;
  for(double fraction : {1.0, 0.5, 0.25, 0.0}) {
    accepted = q0 + fraction * displacement;
    if(fraction == 0.0) { verified = closest; break; }
    verified = collision_->verifyStep(station0, stationQ(split(accepted)));
    result.sampled_pairs += collision_->lastSampledPairs();
    if(verified.safe) break;
  }
  if(!verified.safe) throw std::runtime_error("Current configuration violates collision clearance");
  const Eigen::VectorXd correction = accepted - q1;
  const bool changed = correction.norm() > 1e-7;
  const Eigen::VectorXd step = accepted - q0;
  const auto out = split(accepted);
  for(std::size_t a = 0; a < 2; ++a) {
    if(!active[a]) continue;
    const Eigen::Index o = static_cast<Eigen::Index>(6 * a);
    result.limited[a] = changed;
    if(changed) {
      // Keep the limiter's own continuous derivatives plus the velocity this
      // correction removed: finite-difference derivatives lag by a period and,
      // fed back every tick, erase the deceleration being built.
      const auto status = planners[a]->status();
      PinkArmVector velocity{}, acceleration = status.joint_acceleration;
      for(std::size_t i = 0; i < 6; ++i)
        velocity[i] = status.joint_velocity[i] + correction[o + static_cast<Eigen::Index>(i)] / period;
      planners[a]->reanchor(out[a], velocity, acceleration);
    }
  }
  const Eigen::VectorXd velocity = step / period;
  command_acceleration_ = (velocity - command_velocity_) / period;
  command_velocity_ = velocity;
  result.q = out;
  result.clearance = verified;
  result.solve_time_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();
  return result;
}
}  // namespace libpyelite
