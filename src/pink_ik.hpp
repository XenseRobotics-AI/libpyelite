#pragma once

#include <array>
#include <cstddef>
#include <memory>
#include <mutex>
#include <string>

namespace libpyelite {

// The native planner uses the same 4x4 row-major pose convention as the
// Python adapter.  These aliases intentionally stay independent of the
// vendor SDK types: no SDK object is ever passed to the Pinocchio solver.
using PinkPose = std::array<double, 16>;
using PinkArmVector = std::array<double, 6>;
inline constexpr PinkPose kIdentityPose{{1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1}};

struct PinkIKDiagnostics {
  bool success{false};
  bool progress{false};
  bool qp_failed{false};
  int iterations{0};
  double position_error{0.0};
  double orientation_error{0.0};
};

// Streaming defaults for the Elite (adapted from Tianji) model. Damping is the Hessian diagonal
// coefficient (lambda squared in damped-least-squares notation). A reduced
// task gain moderates each linearized correction near wrist singularities.
struct PinkIKOptions {
  int max_iterations{500};
  int streamingIterationBudget() const { return max_iterations < 8 ? max_iterations : 8; }
  double dt{0.1};
  double damping{1.0e-3};
  double step_size{1.0};
  double max_step_norm{0.15};
  double position_tolerance{1.0e-4};
  double orientation_tolerance{1.0e-3};
  // Streaming (bounded) solves stop at these instead. Stopping anywhere inside
  // the 0.1 mm offline tolerance makes the residual jump between ticks, which
  // differentiates into joint acceleration/jerk spikes (38 rad/s², 1.5e4 rad/s³
  // on a 2 m/s² Cartesian move); the iteration budget still bounds the cost.
  double streaming_position_tolerance{1.0e-7};
  double streaming_orientation_tolerance{1.0e-6};
  double position_cost{1.0};
  double orientation_cost{1.0};
  double frame_gain{0.5};
  // Pink/tron2-ik leave the posture task disabled by default.  The previous
  // 1e-3 regularizer biased small Cartesian updates enough to miss the
  // 1e-4 m streaming tolerance; continuity comes from the warm-start seed and
  // the native safety gate unless a caller explicitly enables posture bias.
  double posture_weight{0.0};
  double posture_gain{1.0};
};

// A small, allocation-free-at-solve-time Pinocchio implementation of Pink's
// task semantics.  This is an independent Elite (adapted from Tianji) implementation: tron2-ik is
// used as a design/reference implementation only; no tron2-ik object or
// source file is linked into this package.  The Python Pink package uses
// qpsolvers for an interactive/non-real-time path.  libpyelite cannot invoke
// Python from its real-time planner worker, so it embeds the same warm-started Cartesian
// quadratic objective and constrained numerical solve natively.
class PinkIKModel {
 public:
  explicit PinkIKModel(const std::string &urdf_path,
                       const PinkPose &tool = kIdentityPose,
                       PinkIKOptions options = {});
  PinkArmVector lowerLimits() const;
  PinkArmVector upperLimits() const;
  PinkArmVector velocityLimits() const;
  ~PinkIKModel();

  PinkIKModel(const PinkIKModel &) = delete;
  PinkIKModel &operator=(const PinkIKModel &) = delete;

  PinkPose forwardKinematics(std::size_t arm,
                            const PinkArmVector &q) const;

  // Offline solve: max_step_rad bounds each numerical iteration.
  // Streaming (bounded=true): also bound TOTAL displacement from reference,
  // using at most eight iterations. success means pose tolerances were met;
  // progress means a finite, bounded state reduced the nonlinear task error.
  // The controller retains a final velocity gate in either mode.
  bool solve(std::size_t arm, const PinkPose &target,
             const PinkArmVector &reference,
             const PinkArmVector &max_step_rad, PinkArmVector &solution,
             PinkIKDiagnostics &diagnostics, bool bounded = false) const;

  static const char *backendName() noexcept { return "pink-pinocchio-native"; }

 private:
  struct ArmModel;

  static std::unique_ptr<ArmModel> buildArmModel(
      const std::string &urdf_path, const PinkPose &tool);
  static void validateOptions(const PinkIKOptions &options);
  static bool validPose(const PinkPose &pose) noexcept;
  static bool finiteVector(const PinkArmVector &values) noexcept;

  PinkPose forwardKinematicsUnlocked(std::size_t arm,
                                     const PinkArmVector &q) const;
  bool solveUnlocked(std::size_t arm, const PinkPose &target,
                     const PinkArmVector &reference,
                     const PinkArmVector &max_step_rad,
                     PinkArmVector &solution,
                     PinkIKDiagnostics &diagnostics, bool bounded = false) const;

  mutable std::array<std::unique_ptr<ArmModel>, 1> arms_;
  PinkIKOptions options_{};
  mutable std::mutex mutex_;
};

}  // namespace libpyelite
