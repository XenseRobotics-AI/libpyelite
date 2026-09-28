#include "pink_ik.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

// Pinocchio's joint collection has 25 alternatives.  Its compatibility
// header must be included before any Boost.MPL header (multibody.hpp includes
// boost::variant internally); otherwise Boost selects its 20-type generated
// list and Pinocchio 4 fails during template instantiation.
#include <pinocchio/deprecated/pinocchio/container/boost-container-limits.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody.hpp>
#include <pinocchio/spatial.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

namespace libpyelite {
namespace {

using SE3 = pinocchio::SE3;
using Model = pinocchio::Model;
using Data = pinocchio::Data;
using JointVector = Eigen::Matrix<double, 6, 1>;
using JointJacobian = Eigen::Matrix<double, 6, 6>;
using Vector6 = Eigen::Matrix<double, 6, 1>;
using Matrix6 = Eigen::Matrix<double, 6, 6>;
using JointMatrix = Eigen::Matrix<double, 6, 6>;

// Pink hands a small strictly-convex velocity QP to qpsolvers.  The RT
// package cannot call a Python backend from its planner thread, so solve the
// same box-constrained quadratic locally.  There are only six variables per
// Elite arm; this deterministic active-set implementation is allocation-free
// and uses Eigen's LDLT for each free-set solve.  The objective is
//   1/2 vᵀ H v - rhsᵀ v,  lower <= v <= upper.
bool solveBoxQP(const JointMatrix &hessian, const JointVector &rhs,
                const JointVector &lower, const JointVector &upper,
                JointVector &solution) noexcept {
  constexpr double kTolerance = 1.0e-10;
  constexpr int kMaxActiveSetIterations = 64;
  std::array<int, 6> active{};  // -1: lower, 0: free, +1: upper
  JointVector state = JointVector::Zero();

  if (!hessian.allFinite() || !rhs.allFinite() || !lower.allFinite() ||
      !upper.allFinite()) {
    return false;
  }
  for (int index = 0; index < 6; ++index) {
    if (!(lower[index] < upper[index])) {
      return false;
    }
  }

  for (int iteration = 0; iteration < kMaxActiveSetIterations; ++iteration) {
    // Eliminate fixed variables from the free-variable system while retaining
    // a full 7x7 matrix.  This avoids dynamic Eigen allocations in the RT
    // loop and keeps the active values exact at their bounds.
    JointMatrix system = hessian;
    JointVector vector = rhs;
    for (int index = 0; index < 6; ++index) {
      if (active[static_cast<std::size_t>(index)] == 0) {
        continue;
      }
      const double value = active[static_cast<std::size_t>(index)] < 0
                               ? lower[index]
                               : upper[index];
      state[index] = value;
      for (int row = 0; row < 6; ++row) {
        if (active[static_cast<std::size_t>(row)] == 0) {
          vector[row] -= hessian(row, index) * value;
        }
        if (row != index) {
          system(row, index) = 0.0;
          system(index, row) = 0.0;
        }
      }
      system(index, index) = 1.0;
      vector[index] = value;
    }

    Eigen::LDLT<JointMatrix> factor(system);
    if (factor.info() != Eigen::Success) {
      return false;
    }
    state = factor.solve(vector);
    if (!state.allFinite()) {
      return false;
    }

    // Add the most-violated free variable to the active set.
    double largest_violation = kTolerance;
    int violated_index = -1;
    int violated_side = 0;
    for (int index = 0; index < 6; ++index) {
      if (active[static_cast<std::size_t>(index)] != 0) {
        continue;
      }
      if (state[index] < lower[index] - kTolerance) {
        const double violation = lower[index] - state[index];
        if (violation > largest_violation) {
          largest_violation = violation;
          violated_index = index;
          violated_side = -1;
        }
      } else if (state[index] > upper[index] + kTolerance) {
        const double violation = state[index] - upper[index];
        if (violation > largest_violation) {
          largest_violation = violation;
          violated_index = index;
          violated_side = 1;
        }
      }
    }
    if (violated_index >= 0) {
      active[static_cast<std::size_t>(violated_index)] = violated_side;
      continue;
    }

    // Check KKT multipliers for variables currently fixed at a bound.  At a
    // lower bound the gradient must be non-negative; at an upper bound it
    // must be non-positive.  Release the most-violated bound and resolve.
    const JointVector gradient = hessian * state - rhs;
    double largest_multiplier_violation = kTolerance;
    int release_index = -1;
    for (int index = 0; index < 6; ++index) {
      const int side = active[static_cast<std::size_t>(index)];
      const double violation =
          side < 0 ? -gradient[index] : side > 0 ? gradient[index] : 0.0;
      if (violation > largest_multiplier_violation) {
        largest_multiplier_violation = violation;
        release_index = index;
      }
    }
    if (release_index >= 0) {
      active[static_cast<std::size_t>(release_index)] = 0;
      continue;
    }

    solution = state;
    return true;
  }

  // A positive-definite Hessian should converge well within the bound above.
  // If round-off causes an active-set tie, return a projected finite solution
  // rather than allowing a transient QP bookkeeping issue to stop the arm.
  const Eigen::LDLT<JointMatrix> factor(hessian);
  if (factor.info() != Eigen::Success) {
    return false;
  }
  solution = factor.solve(rhs);
  if (!solution.allFinite()) {
    return false;
  }
  for (int index = 0; index < 6; ++index) {
    solution[index] = std::clamp(solution[index], lower[index], upper[index]);
  }
  return true;
}

bool finite(double value) noexcept { return std::isfinite(value); }

SE3 poseFromArray(const PinkPose &values) {
  Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
  for (int row = 0; row < 4; ++row) {
    for (int column = 0; column < 4; ++column) {
      matrix(row, column) = values[static_cast<std::size_t>(row * 4 + column)];
    }
  }
  return SE3(matrix.block<3, 3>(0, 0), matrix.block<3, 1>(0, 3));
}

PinkPose poseToArray(const SE3 &value) {
  PinkPose result{};
  const Eigen::Matrix3d rotation = value.rotation();
  const Eigen::Vector3d translation = value.translation();
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      result[static_cast<std::size_t>(row * 4 + column)] =
          rotation(row, column);
    }
    result[static_cast<std::size_t>(row * 4 + 3)] = translation[row];
  }
  result[12] = 0.0;
  result[13] = 0.0;
  result[14] = 0.0;
  result[15] = 1.0;
  return result;
}

JointVector toEigen(const PinkArmVector &values) {
  JointVector result;
  for (int index = 0; index < 6; ++index) {
    result[index] = values[static_cast<std::size_t>(index)];
  }
  return result;
}

PinkArmVector fromEigen(const JointVector &values) {
  PinkArmVector result{};
  for (int index = 0; index < 6; ++index) {
    result[static_cast<std::size_t>(index)] = values[index];
  }
  return result;
}

Eigen::Vector3d rotationError(const Eigen::Matrix3d &current,
                              const Eigen::Matrix3d &target) {
  // log3 is expressed in the current frame.  LOCAL_WORLD_ALIGNED Jacobians
  // use world axes, so rotate the error into that same frame.
  return current * pinocchio::log3(current.transpose() * target);
}

void update(Model &model, Data &data, const JointVector &q) {
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::computeJointJacobians(model, data, q);
  pinocchio::updateFramePlacements(model, data);
}

}  // namespace

struct PinkIKModel::ArmModel {
  Model model;
  Data data;
  pinocchio::FrameIndex tcp_frame{0};
  std::array<int, 6> q_index{};
  PinkArmVector lower{};
  PinkArmVector upper{};

  ArmModel() : data(model) {}
};

std::unique_ptr<PinkIKModel::ArmModel> PinkIKModel::buildArmModel(
    const std::string &urdf_path, const PinkPose &tool) {
  auto result = std::make_unique<ArmModel>();
  auto &model = result->model;
  pinocchio::urdf::buildModel(urdf_path, model);
  if (model.nq != 6 || model.nv != 6)
    throw std::invalid_argument("CS66 requires six bounded revolute joints");
  const std::array<std::string, 6> names{{
      "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
      "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"}};
  for (int i = 0; i < 6; ++i) {
    auto id = model.getJointId(names[i]);
    if (id >= model.joints.size() || model.joints[id].idx_q() != i ||
        model.joints[id].nq() != 1)
      throw std::invalid_argument("Unexpected CS66 joint order");
    result->q_index[i] = i;
    result->lower[i] = model.lowerPositionLimit[i];
    result->upper[i] = model.upperPositionLimit[i];
    if (!finite(result->lower[i]) || !finite(result->upper[i]) ||
        result->lower[i] >= result->upper[i] ||
        !finite(model.velocityLimit[i]) || model.velocityLimit[i] <= 0)
      throw std::invalid_argument("Invalid URDF joint limits");
  }
  if (!model.existFrame("tool0"))
    throw std::invalid_argument("URDF has no tool0 frame");
  const auto &flange = model.frames[model.getFrameId("tool0")];
  result->tcp_frame = model.addFrame(pinocchio::Frame(
      "libpyelite_tcp", flange.parentJoint, flange.placement * poseFromArray(tool),
      pinocchio::OP_FRAME));
  result->data = Data(model);
  return result;
}

PinkArmVector PinkIKModel::lowerLimits() const { return arms_[0]->lower; }
PinkArmVector PinkIKModel::upperLimits() const { return arms_[0]->upper; }
PinkArmVector PinkIKModel::velocityLimits() const {
  return fromEigen(arms_[0]->model.velocityLimit);
}

void PinkIKModel::validateOptions(const PinkIKOptions &options) {
  if (options.max_iterations < 1) {
    throw std::invalid_argument("Pink IK max_iterations must be positive");
  }
  const std::array<double, 11> nonnegative{{
      options.dt,
      options.damping,
      options.step_size,
      options.max_step_norm,
      options.position_tolerance,
      options.orientation_tolerance,
      options.position_cost,
      options.orientation_cost,
      options.frame_gain,
      options.posture_weight,
      options.posture_gain,
  }};
  for (double value : nonnegative) {
    if (!finite(value) || value < 0.0) {
      throw std::invalid_argument(
          "Pink IK options must be finite and non-negative");
    }
  }
  if (options.dt <= 0.0 || options.damping <= 0.0 ||
      options.step_size <= 0.0 || options.max_step_norm <= 0.0 ||
      options.position_tolerance <= 0.0 ||
      options.orientation_tolerance <= 0.0 ||
      options.position_cost <= 0.0 || options.orientation_cost <= 0.0 ||
      options.frame_gain <= 0.0 || options.frame_gain > 1.0 ||
      options.posture_gain > 1.0) {
    throw std::invalid_argument("Pink IK options contain an invalid range");
  }
}

PinkIKModel::PinkIKModel(const std::string &urdf_path, const PinkPose &tool,
                         PinkIKOptions options) : options_(options) {
  validateOptions(options_);
  if (!validPose(tool)) throw std::invalid_argument("Invalid tool SE(3)");
  arms_[0] = buildArmModel(urdf_path, tool);
}

PinkIKModel::~PinkIKModel() = default;

bool PinkIKModel::finiteVector(const PinkArmVector &values) noexcept {
  for (double value : values) {
    if (!finite(value)) {
      return false;
    }
  }
  return true;
}

bool PinkIKModel::validPose(const PinkPose &values) noexcept {
  if (!std::all_of(values.begin(), values.end(), finite)) {
    return false;
  }
  if (std::abs(values[12]) > 1.0e-6 || std::abs(values[13]) > 1.0e-6 ||
      std::abs(values[14]) > 1.0e-6 || std::abs(values[15] - 1.0) > 1.0e-6) {
    return false;
  }
  Eigen::Matrix3d rotation;
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      rotation(row, column) = values[static_cast<std::size_t>(row * 4 + column)];
    }
  }
  const double determinant = rotation.determinant();
  return finite(determinant) && std::abs(determinant - 1.0) < 2.0e-3 &&
         (rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm() <
             2.0e-3;
}

PinkPose PinkIKModel::forwardKinematicsUnlocked(
    std::size_t arm, const PinkArmVector &q) const {
  if (arm >= arms_.size() || !arms_[arm] || !finiteVector(q)) {
    throw std::invalid_argument("invalid arm or joint vector for Pink FK");
  }
  auto &model = arms_[arm]->model;
  auto &data = arms_[arm]->data;
  JointVector values = toEigen(q);
  for (int index = 0; index < 6; ++index) {
    if (values[index] < arms_[arm]->lower[index] ||
        values[index] > arms_[arm]->upper[index])
      throw std::invalid_argument("FK joint outside URDF limits");
  }
  update(model, data, values);
  return poseToArray(data.oMf[arms_[arm]->tcp_frame]);
}

PinkPose PinkIKModel::forwardKinematics(std::size_t arm,
                                        const PinkArmVector &q) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return forwardKinematicsUnlocked(arm, q);
}

bool PinkIKModel::solveUnlocked(std::size_t arm, const PinkPose &target_values,
                                const PinkArmVector &reference,
                                const PinkArmVector &max_step_rad,
                                PinkArmVector &solution,
                                PinkIKDiagnostics &diagnostics) const {
  diagnostics = PinkIKDiagnostics{};
  if (arm >= arms_.size() || !arms_[arm] || !validPose(target_values) ||
      !finiteVector(reference) || !finiteVector(max_step_rad)) {
    return false;
  }
  for (double value : max_step_rad) {
    if (value <= 0.0) {
      return false;
    }
  }

  auto &arm_model = *arms_[arm];
  auto &model = arm_model.model;
  auto &data = arm_model.data;
  const SE3 target = poseFromArray(target_values);
  JointVector q = toEigen(reference);
  JointVector seed = q;
  // Match Pink's native migration semantics: Pinocchio joint limits constrain
  // the numerical iteration and the velocity box bounds each integrated task
  // step.  The caller still checks/clips the final candidate relative to its
  // last published command, because one solve intentionally runs several
  // numerical iterations and can therefore span multiple RT cycles.
  JointVector trust_lower;
  JointVector trust_upper;
  for (int index = 0; index < 6; ++index) {
    const auto i = static_cast<std::size_t>(index);
    q[index] = std::clamp(q[index], arm_model.lower[i], arm_model.upper[i]);
    seed[index] = q[index];
    trust_lower[index] = arm_model.lower[i];
    trust_upper[index] = arm_model.upper[i];
  }
  // Match Pink's failure contract: even when the requested pose cannot be
  // reached, return the best finite warm-start state reached so far.  The
  // native controller will hold it (and report -1) rather than consuming an
  // uninitialized/zero candidate.
  solution = fromEigen(q);

  const auto evaluate = [&](const JointVector &state, Vector6 *error,
                            double &position_error,
                            double &orientation_error) {
    update(model, data, state);
    const SE3 current = data.oMf[arm_model.tcp_frame];
    const Eigen::Vector3d position = target.translation() - current.translation();
    const Eigen::Vector3d orientation =
        rotationError(current.rotation(), target.rotation());
    position_error = position.norm();
    orientation_error = orientation.norm();
    if (error != nullptr) {
      error->head<3>() = position;
      error->tail<3>() = orientation;
    }
  };

  double position_error = 0.0;
  double orientation_error = 0.0;
  Vector6 error;
  evaluate(q, &error, position_error, orientation_error);
  auto converged = [&]() {
    return position_error <= options_.position_tolerance &&
           orientation_error <= options_.orientation_tolerance;
  };
  if (converged()) {
    solution = fromEigen(q);
    diagnostics.success = true;
    diagnostics.position_error = position_error;
    diagnostics.orientation_error = orientation_error;
    return true;
  }

  Matrix6 weight = Matrix6::Zero();
  weight.block<3, 3>(0, 0).diagonal().setConstant(options_.position_cost);
  weight.block<3, 3>(3, 3).diagonal().setConstant(options_.orientation_cost);

  for (int iteration = 1; iteration <= options_.max_iterations; ++iteration) {
    // ``computeJointJacobians`` was called by evaluate above; refresh it after
    // every accepted step as Pink does for a warm-started configuration.
    JointJacobian jacobian;
    jacobian.setZero();
    pinocchio::getFrameJacobian(model, data, arm_model.tcp_frame,
                                pinocchio::LOCAL_WORLD_ALIGNED, jacobian);

    JointMatrix hessian = jacobian.transpose() * weight * jacobian;
    // Pink's global ``damping`` parameter is a positive diagonal term of the
    // QP Hessian.  Keep it linear here rather than squaring it as a physical
    // viscosity coefficient; otherwise the streaming default 1e-4 would be
    // reduced to 1e-8 and provide almost no singularity conditioning.
    hessian.diagonal().array() += options_.damping;
    // Pink's FrameTask target is a desired task *velocity*: the pose error is
    // divided by dt before the QP is assembled.  Keeping this scaling here is
    // important when the streaming dt differs from the host planner period;
    // the subsequent manifold integration multiplies the solved velocity by
    // the same dt and therefore preserves Pink's per-step behavior.
    const Vector6 desired_task_velocity =
        options_.frame_gain * error / options_.dt;
    JointVector rhs = jacobian.transpose() * weight * desired_task_velocity;
    if (options_.posture_weight > 0.0) {
      hessian.diagonal().array() += options_.posture_weight;
      rhs += options_.posture_weight * options_.posture_gain *
             (seed - q) / options_.dt;
    }
    JointVector velocity;
    JointVector velocity_lower;
    JointVector velocity_upper;
    for (int index = 0; index < 6; ++index) {
      const auto i = static_cast<std::size_t>(index);
      const double limit = max_step_rad[i] / options_.dt;
      // Pink's configuration-limit task is expressed in velocity space for
      // each integrated step.  Intersect that position-derived interval with
      // the caller's per-cycle trust region before solving the QP; the later
      // clamp is then only a round-off guard, not the primary limit mechanism.
      velocity_lower[index] =
          std::max(-limit, (trust_lower[index] - q[index]) / options_.dt);
      velocity_upper[index] =
          std::min(limit, (trust_upper[index] - q[index]) / options_.dt);
      if (velocity_lower[index] > velocity_upper[index]) {
        diagnostics.iterations = iteration;
        diagnostics.position_error = position_error;
        diagnostics.orientation_error = orientation_error;
        return false;
      }
    }
    if (!solveBoxQP(hessian, rhs, velocity_lower, velocity_upper, velocity)) {
      solution = fromEigen(q);
      diagnostics.iterations = iteration;
      diagnostics.position_error = position_error;
      diagnostics.orientation_error = orientation_error;
      return false;
    }

    // Keep each Newton iteration bounded.  Joint limits are enforced after
    // integrating on Pinocchio's configuration manifold, just as Pink does;
    // the outer controller separately limits the final candidate relative to
    // its last published command.
    JointVector step = options_.step_size * options_.dt * velocity;
    const double norm = step.norm();
    const double max_norm = options_.max_step_norm;
    if (norm > max_norm) {
      step *= max_norm / norm;
    }
    q = pinocchio::integrate(model, q, step);
    for (int index = 0; index < 6; ++index) {
      q[index] = std::clamp(q[index], trust_lower[index], trust_upper[index]);
    }

    evaluate(q, &error, position_error, orientation_error);
    diagnostics.iterations = iteration;
    diagnostics.position_error = position_error;
    diagnostics.orientation_error = orientation_error;
    if (converged()) {
      diagnostics.success = true;
      solution = fromEigen(q);
      return true;
    }
  }

  solution = fromEigen(q);
  return false;
}

bool PinkIKModel::solve(std::size_t arm, const PinkPose &target,
                        const PinkArmVector &reference,
                        const PinkArmVector &max_step_rad,
                        PinkArmVector &solution,
                        PinkIKDiagnostics &diagnostics) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return solveUnlocked(arm, target, reference, max_step_rad, solution,
                       diagnostics);
}

}  // namespace libpyelite
