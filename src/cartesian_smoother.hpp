#pragma once
#include "pink_ik.hpp"
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace libpyelite {
// Velocity, acceleration and jerk limits for the TCP, as magnitudes of the
// linear (m) and angular (rad) parts of a twist, never per axis.
struct CartesianLimits {
  double linear_velocity{0.5}, angular_velocity{1.0};
  double linear_acceleration{2.0}, angular_acceleration{5.0};
  double linear_jerk{1000.0}, angular_jerk{2500.0};
  double gain{20.0};  // 1/s, pose-error correction on top of the target's own twist
};

// Smooths a streamed TCP target in Cartesian space, before IK, as Flexiv's
// controller, libfranka's rate limiter and UMI's interpolator do. Joint-space
// per-axis limiting desynchronises the joints and bends the TCP path.
//
// Each period the commanded twist is the target's twist (feed-forward) plus a
// correction along the 6-D pose error, slowed so the remaining error can still
// be braked away. Velocity, acceleration and jerk are then limited by scaling
// the whole 6-D vector with one factor, the smallest the linear and angular
// limits require (KDL/UMI's shared path parameter). Direction is kept, so a
// target moving on a line stays on it and a pure rotation turns about the TCP,
// and translation and rotation finish together.
class CartesianSmoother {
 public:
  using Vector6 = Eigen::Matrix<double, 6, 1>;  // [linear; angular], base axes

  explicit CartesianSmoother(CartesianLimits limits) : limits_(limits) {
    for(double value:{limits.linear_velocity,limits.angular_velocity,limits.linear_acceleration,
        limits.angular_acceleration,limits.linear_jerk,limits.angular_jerk,limits.gain})
      if(!std::isfinite(value)||value<=0) throw std::invalid_argument("Cartesian limits must be positive and finite");
  }

  // Error from pose a to pose b: [translation; rotation vector], base axes.
  static Vector6 difference(const PinkPose &a, const PinkPose &b) {
    Vector6 result;
    result.head<3>() = translation(b) - translation(a);
    const Eigen::AngleAxisd turn(rotation(b) * rotation(a).transpose());
    result.tail<3>() = turn.angle() * turn.axis();
    return result;
  }

  void reset(const PinkPose &pose, const Vector6 &velocity = Vector6::Zero(),
             const Vector6 &acceleration = Vector6::Zero()) {
    position_ = translation(pose);
    rotation_ = rotation(pose);
    velocity_ = velocity;
    acceleration_ = acceleration;
  }

  // Path time scaling: a later stage achieved only a fraction of this step.
  // Continue from where it got, with the twist scaled by the same fraction.
  void retime(const PinkPose &achieved, double fraction) {
    reset(achieved, fraction * velocity_, fraction * acceleration_);
  }

  PinkPose step(const PinkPose &goal, const Vector6 &goal_velocity, double dt) {
    if(!std::isfinite(dt)||dt<=0||!goal_velocity.allFinite())
      throw std::invalid_argument("Invalid Cartesian smoother input");
    const Vector6 error = difference(pose(), goal);
    double gain = limits_.gain;
    // Approach no faster than can still be braked within the remaining error,
    // counting the jerk ramp of the deceleration. Without the ramp term a low
    // jerk limit makes the arm arrive too fast and ring about the target.
    const double linear = error.head<3>().norm(), angular = error.tail<3>().norm();
    if(linear > 0) gain = std::min(gain, brakingSpeed(linear, limits_.linear_acceleration, limits_.linear_jerk) / linear);
    if(angular > 0) gain = std::min(gain, brakingSpeed(angular, limits_.angular_acceleration, limits_.angular_jerk) / angular);
    Vector6 velocity = goal_velocity + gain * error;
    velocity *= scale(velocity, limits_.linear_velocity, limits_.angular_velocity);
    // Accelerate toward it, no harder than jerk can ramp back to zero before
    // the velocity arrives (otherwise the velocity overshoots).
    const Vector6 change = velocity - velocity_;
    Vector6 acceleration = change / dt;
    acceleration *= scale(acceleration,
        std::min(limits_.linear_acceleration, std::sqrt(2 * limits_.linear_jerk * change.head<3>().norm())),
        std::min(limits_.angular_acceleration, std::sqrt(2 * limits_.angular_jerk * change.tail<3>().norm())));
    Vector6 jerk = acceleration - acceleration_;
    jerk *= scale(jerk, limits_.linear_jerk * dt, limits_.angular_jerk * dt);
    acceleration_ += jerk;
    velocity_ += acceleration_ * dt;
    velocity_ *= scale(velocity_, limits_.linear_velocity, limits_.angular_velocity);
    position_ += velocity_.head<3>() * dt;
    const Eigen::Vector3d turn = velocity_.tail<3>() * dt;
    if(turn.norm() > 0) rotation_ = Eigen::AngleAxisd(turn.norm(), turn.normalized()).toRotationMatrix() * rotation_;
    rotation_ = Eigen::Quaterniond(rotation_).normalized().toRotationMatrix();
    return pose();
  }

  PinkPose pose() const {
    PinkPose result{kIdentityPose};
    for(int row=0; row<3; ++row) {
      for(int column=0; column<3; ++column) result[4*row+column] = rotation_(row, column);
      result[4*row+3] = position_[row];
    }
    return result;
  }
  const Vector6 &velocity() const { return velocity_; }
  const Vector6 &acceleration() const { return acceleration_; }
  bool still() const { return velocity_.norm() < kStillVelocity && acceleration_.norm() < kStillAcceleration; }
  bool settled(const PinkPose &goal) const {
    const Vector6 error = difference(pose(), goal);
    return still() && error.head<3>().norm() < kSettledPosition && error.tail<3>().norm() < kSettledRotation;
  }

 private:
  static constexpr double kStillVelocity = 1e-5, kStillAcceleration = 1e-3;     // m|rad /s, /s^2
  static constexpr double kBrakingShare = 0.5;
  static constexpr double kSettledPosition = 1e-6, kSettledRotation = 1e-5;     // m, rad

  // Largest speed v that stops within distance d at deceleration a reached and
  // released at jerk j: d = v^2/(2a) + v*a/(2j) (the two ramps add a/j of travel
  // at about v/2), solved for v; kBrakingShare of a is used, the rest is margin
  // for a deceleration that starts from a nonzero acceleration.
  static double brakingSpeed(double distance, double acceleration, double jerk) {
    const double a = kBrakingShare * acceleration, ramp = a * a / (2 * jerk);
    return std::sqrt(ramp * ramp + 2 * a * distance) - ramp;
  }

  // Largest factor <= 1 that brings both parts within their magnitude limits.
  static double scale(const Vector6 &value, double linear, double angular) {
    double result = 1.0;
    const double a = value.head<3>().norm(), b = value.tail<3>().norm();
    if(a > linear) result = std::min(result, linear / a);
    if(b > angular) result = std::min(result, angular / b);
    return result;
  }
  static Eigen::Vector3d translation(const PinkPose &p) { return {p[3], p[7], p[11]}; }
  static Eigen::Matrix3d rotation(const PinkPose &p) {
    Eigen::Matrix3d r;
    for(int row=0; row<3; ++row) for(int column=0; column<3; ++column) r(row, column) = p[4*row+column];
    return r;
  }

  CartesianLimits limits_;
  Eigen::Vector3d position_{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d rotation_{Eigen::Matrix3d::Identity()};
  Vector6 velocity_{Vector6::Zero()}, acceleration_{Vector6::Zero()};
};
}  // namespace libpyelite
