#pragma once
#include "pink_ik.hpp"
#include <array>
#include <memory>
#include <string>

namespace libpyelite {
using Vector3 = std::array<double, 3>;

// Sensorless Cartesian admittance for the CS66, which has no joint torque sensors:
// external joint torque is reconstructed from motor current (RTSI actual_joint_torques)
// minus a rigid-body + friction model, mapped to a translational force at the TCP and
// fed through M*a + D*v + K*x = F to offset the commanded TCP position.
// Model parameters come from scripts/probe_force_noise.py analyze --export.
struct ComplianceConfig {
  bool enabled{false};
  // Static model. gravity is the fitted base-frame vector (tilted mounts, model scale);
  // the payload is a point mass at payload_cog in the tool0 (flange) frame.
  Vector3 gravity{{0.0, 0.0, -9.81}};
  double payload_mass{0.0};
  Vector3 payload_cog{{0.0, 0.0, 0.0}};
  // tau_f = (coulomb + viscous*|qd|) * tanh(qd / friction_speed)
  PinkArmVector friction_coulomb{};
  PinkArmVector friction_viscous{};
  double friction_speed{0.01};
  // Per-joint soft deadband on the external torque: what the model cannot explain at
  // rest (stiction hysteresis) plus model error. Widened by transient_factor for
  // transient_window seconds after a joint starts, stops or reverses.
  PinkArmVector torque_band{{4.0, 4.0, 4.0, 1.0, 1.0, 1.0}};
  double transient_factor{2.0};
  double transient_window{0.25};
  double torque_filter_hz{10.0};
  double accel_filter_hz{15.0};
  // Seconds of still, untouched averaging that zero the residual on start / rebias().
  double bias_time{0.3};
  // Translational admittance at the TCP, base axes.
  double mass{8.0};          // kg
  double damping{100.0};     // N s/m
  double stiffness{300.0};   // N/m
  double force_deadband{8.0};  // N, soft
  double max_force{60.0};      // N, clamp before the admittance
  double max_offset{0.05};     // m
  double max_velocity{0.1};    // m/s
  // Per-joint |external torque| that stops the arm (any link, not only the tool);
  // also widened by transient_factor. <= 0 disables that joint.
  PinkArmVector collision_threshold{{35.0, 35.0, 30.0, 8.0, 8.0, 8.0}};
};

struct ComplianceStatus {
  bool enabled{false}, biased{false}, in_contact{false};
  PinkArmVector tau_ext{};  // filtered, bias-removed external torque, Nm
  Vector3 force{};          // estimated force applied to the TCP, N, base axes
  Vector3 offset{};         // admittance displacement of the TCP target, m
  Vector3 velocity{};       // m/s
  std::string collision;    // non-empty once a collision tripped
};

class ComplianceController {
 public:
  ComplianceController(const std::string &urdf, const PinkPose &tool, ComplianceConfig config);
  ~ComplianceController();
  ComplianceController(const ComplianceController &) = delete;
  ComplianceController &operator=(const ComplianceController &) = delete;

  // One control period. Returns the TCP offset to add to the nominal target.
  // Until the bias is acquired the offset stays zero and collision checks are off.
  Vector3 update(double dt, const PinkArmVector &q, const PinkArmVector &qd, const PinkArmVector &tau);
  // Re-acquire the bias (arm still and untouched) and drop the admittance state.
  void rebias();
  // Drop the admittance state but keep the bias, e.g. when the target is re-anchored.
  void resetMotion();
  const ComplianceStatus &status() const { return status_; }
  const ComplianceConfig &config() const { return config_; }

  // Model terms, exposed for diagnostics and tests.
  PinkArmVector modelTorque(const PinkArmVector &q, const PinkArmVector &qd, const PinkArmVector &qdd) const;
  std::array<double, 18> tcpJacobian(const PinkArmVector &q) const;  // 3x6 row-major, linear part

 private:
  struct Model;
  std::unique_ptr<Model> model_;
  ComplianceConfig config_;
  ComplianceStatus status_;
  PinkArmVector filtered_qd_{}, previous_qd_{}, qdd_{}, residual_{}, bias_{}, bias_sum_{};
  PinkArmVector transient_timer_{};
  std::array<int, 6> direction_{};
  double bias_elapsed_{0};
  int bias_samples_{0};
  bool primed_{false};
};
}  // namespace libpyelite
