#include "compliance.hpp"
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace libpyelite {
namespace {
using Matrix4 = Eigen::Matrix<double, 4, 4, Eigen::RowMajor>;
using Vector6 = Eigen::Matrix<double, 6, 1>;
Vector6 eigen(const PinkArmVector &v) { return Eigen::Map<const Vector6>(v.data()); }
PinkArmVector array(const Vector6 &v) { PinkArmVector out; Eigen::Map<Vector6>(out.data()) = v; return out; }
Vector3 array(const Eigen::Vector3d &v) { return {v.x(), v.y(), v.z()}; }
double lowpassAlpha(double dt, double hz) { return dt / (dt + 1.0 / (2.0 * M_PI * hz)); }
bool finite(const PinkArmVector &v) { return std::all_of(v.begin(), v.end(), [](double x) { return std::isfinite(x); }); }
}  // namespace

struct ComplianceController::Model {
  pinocchio::Model model;
  mutable pinocchio::Data data{model};
  pinocchio::FrameIndex tcp{0};
};

ComplianceController::ComplianceController(const std::string &urdf, const PinkPose &tool, ComplianceConfig c)
    : model_(std::make_unique<Model>()), config_(c) {
  for (double v : {c.friction_speed, c.transient_factor, c.torque_filter_hz, c.accel_filter_hz, c.bias_time,
                   c.mass, c.max_force, c.max_offset, c.max_velocity})
    if (!std::isfinite(v) || v <= 0) throw std::invalid_argument("Compliance parameters must be positive");
  for (double v : {c.damping, c.stiffness, c.force_deadband, c.transient_window, c.payload_mass})
    if (!std::isfinite(v) || v < 0) throw std::invalid_argument("Compliance parameters must be non-negative");
  for (const auto *v : {&c.friction_coulomb, &c.friction_viscous, &c.torque_band})
    for (double x : *v)
      if (!std::isfinite(x) || x < 0) throw std::invalid_argument("Friction and torque bands must be non-negative");
  if (std::hypot(c.gravity[0], c.gravity[1], c.gravity[2]) < 1.0)
    throw std::invalid_argument("Compliance gravity vector is missing");

  auto &model = model_->model;
  pinocchio::urdf::buildModel(urdf, model);
  if (model.nq != 6 || model.nv != 6 || !model.existFrame("tool0"))
    throw std::invalid_argument("Compliance needs the six-joint CS66 URDF with tool0");
  const auto &tool0 = model.frames[model.getFrameId("tool0")];
  if (c.payload_mass > 0) {
    const pinocchio::Inertia point(c.payload_mass, Eigen::Vector3d(c.payload_cog[0], c.payload_cog[1], c.payload_cog[2]),
                                   Eigen::Matrix3d::Zero());
    model.inertias[tool0.parentJoint] += tool0.placement.act(point);
  }
  model.gravity.linear(Eigen::Vector3d(c.gravity[0], c.gravity[1], c.gravity[2]));
  const Matrix4 t = Eigen::Map<const Matrix4>(tool.data());
  model_->tcp = model.addFrame(pinocchio::Frame("compliance_tcp", tool0.parentJoint,
      tool0.placement * pinocchio::SE3(Eigen::Matrix3d(t.block<3, 3>(0, 0)), Eigen::Vector3d(t.block<3, 1>(0, 3))),
      pinocchio::OP_FRAME));
  model_->data = pinocchio::Data(model);
  status_.enabled = c.enabled;
}

ComplianceController::~ComplianceController() = default;

PinkArmVector ComplianceController::modelTorque(const PinkArmVector &q, const PinkArmVector &qd,
                                                const PinkArmVector &qdd) const {
  // rnea with the fitted gravity vector: M(q) qdd + C(q, qd) qd + g(q).
  Vector6 tau = pinocchio::rnea(model_->model, model_->data, eigen(q), eigen(qd), eigen(qdd));
  for (int j = 0; j < 6; ++j)
    tau[j] += (config_.friction_coulomb[j] + config_.friction_viscous[j] * std::abs(qd[j])) *
              std::tanh(qd[j] / config_.friction_speed);
  return array(tau);
}

std::array<double, 18> ComplianceController::tcpJacobian(const PinkArmVector &q) const {
  Eigen::Matrix<double, 6, 6> jacobian = Eigen::Matrix<double, 6, 6>::Zero();
  pinocchio::computeFrameJacobian(model_->model, model_->data, eigen(q), model_->tcp,
                                  pinocchio::LOCAL_WORLD_ALIGNED, jacobian);
  std::array<double, 18> out;
  Eigen::Map<Eigen::Matrix<double, 3, 6, Eigen::RowMajor>>(out.data()) = jacobian.topRows<3>();
  return out;
}

void ComplianceController::rebias() {
  status_.biased = false;
  bias_elapsed_ = 0;
  bias_samples_ = 0;
  bias_sum_ = {};
  resetMotion();
}

void ComplianceController::resetMotion() {
  status_.offset = {};
  status_.velocity = {};
  status_.force = {};
  status_.in_contact = false;
}

Vector3 ComplianceController::update(double dt, const PinkArmVector &q, const PinkArmVector &qd,
                                     const PinkArmVector &tau) {
  if (!config_.enabled || !status_.collision.empty()) return status_.offset;
  if (!std::isfinite(dt) || dt <= 0 || !finite(q) || !finite(qd) || !finite(tau))
    throw std::invalid_argument("Compliance update needs finite feedback and a positive period");

  // Acceleration for the inertial term: low-passed velocity, differentiated.
  const double accel_alpha = lowpassAlpha(dt, config_.accel_filter_hz);
  for (int j = 0; j < 6; ++j) {
    const double smooth = primed_ ? filtered_qd_[j] + accel_alpha * (qd[j] - filtered_qd_[j]) : qd[j];
    qdd_[j] = primed_ ? (smooth - filtered_qd_[j]) / dt : 0.0;
    filtered_qd_[j] = smooth;
  }
  const auto model = modelTorque(q, filtered_qd_, qdd_);
  const double torque_alpha = lowpassAlpha(dt, config_.torque_filter_hz);
  for (int j = 0; j < 6; ++j) {
    const double raw = tau[j] - model[j];
    residual_[j] = primed_ ? residual_[j] + torque_alpha * (raw - residual_[j]) : raw;
  }
  primed_ = true;

  if (!status_.biased) {
    // Average once the filter has settled; the first half of the window is discarded.
    bias_elapsed_ += dt;
    if (bias_elapsed_ > 0.5 * config_.bias_time) {
      for (int j = 0; j < 6; ++j) bias_sum_[j] += residual_[j];
      ++bias_samples_;
    }
    if (bias_elapsed_ >= config_.bias_time && bias_samples_ > 0) {
      for (int j = 0; j < 6; ++j) bias_[j] = bias_sum_[j] / bias_samples_;
      status_.biased = true;
    }
    return status_.offset;
  }

  // Start, stop and reversal break friction loose; the model is worst for a short window.
  PinkArmVector band{}, limit{}, deadbanded{};
  for (int j = 0; j < 6; ++j) {
    const int direction = std::abs(qd[j]) > config_.friction_speed ? (qd[j] > 0 ? 1 : -1) : 0;
    if (direction != direction_[j]) transient_timer_[j] = config_.transient_window;
    direction_[j] = direction;
    transient_timer_[j] = std::max(0.0, transient_timer_[j] - dt);
    const double widen = transient_timer_[j] > 0 ? config_.transient_factor : 1.0;
    const double ext = residual_[j] - bias_[j];
    status_.tau_ext[j] = ext;
    band[j] = config_.torque_band[j] * widen;
    limit[j] = config_.collision_threshold[j] * widen;
    deadbanded[j] = std::copysign(std::max(0.0, std::abs(ext) - band[j]), ext);
  }
  for (int j = 0; j < 6; ++j)
    if (config_.collision_threshold[j] > 0 && std::abs(status_.tau_ext[j]) > limit[j]) {
      status_.collision = "J" + std::to_string(j) + " external torque " + std::to_string(status_.tau_ext[j]) +
                          " Nm exceeds " + std::to_string(limit[j]) + " Nm";
      resetMotion();
      return status_.offset;
    }

  // Motor torque resists the contact: tau_meas - model = -J^T F. Weighted least squares
  // over all six joints, each weighted by the inverse of its own uncertainty.
  const auto j_data = tcpJacobian(q);
  const Eigen::Map<const Eigen::Matrix<double, 3, 6, Eigen::RowMajor>> jacobian(j_data.data());
  Vector6 weight;
  for (int j = 0; j < 6; ++j) weight[j] = 1.0 / std::pow(std::max(config_.torque_band[j], 0.2), 2);
  Eigen::Matrix3d normal = jacobian * weight.asDiagonal() * jacobian.transpose();
  normal += 1e-9 * normal.trace() * Eigen::Matrix3d::Identity();
  Eigen::Vector3d force = -normal.ldlt().solve(jacobian * weight.asDiagonal() * eigen(deadbanded));
  status_.force = array(force);

  const double magnitude = force.norm();
  status_.in_contact = magnitude > config_.force_deadband;
  Eigen::Vector3d drive = Eigen::Vector3d::Zero();
  if (status_.in_contact)
    drive = force * (std::min(magnitude, config_.max_force) - config_.force_deadband) / magnitude;

  Eigen::Vector3d x(status_.offset[0], status_.offset[1], status_.offset[2]);
  Eigen::Vector3d v(status_.velocity[0], status_.velocity[1], status_.velocity[2]);
  const Eigen::Vector3d a = (drive - config_.damping * v - config_.stiffness * x) / config_.mass;
  v += a * dt;
  if (v.norm() > config_.max_velocity) v *= config_.max_velocity / v.norm();
  x += v * dt;
  if (x.norm() > config_.max_offset) {
    x *= config_.max_offset / x.norm();
    const Eigen::Vector3d outward = x.normalized();
    v -= std::max(0.0, v.dot(outward)) * outward;  // stop pushing against the limit
  }
  status_.offset = array(x);
  status_.velocity = array(v);
  return status_.offset;
}
}  // namespace libpyelite
