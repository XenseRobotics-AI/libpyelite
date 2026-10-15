#pragma once
#include <Eigen/Core>

namespace libpyelite {

// Closest feasible displacement in a metric:
//   min 1/2 (x - p)^T M (x - p) + w/2 |s|^2
//   lower <= x <= upper, G x <= h                        (hard)
//   soft_lower - s <= x <= soft_upper + s, s >= 0        (soft, optional)
//
// Primal active-set method (Nocedal & Wright, Alg. 16.3), the family used for
// small dense QPs (qpOASES, quadprog). Zero displacement is always feasible for
// a step from the last accepted configuration (lower <= 0 <= upper, h >= 0;
// otherwise solve() throws), so the method starts there and every iterate stays
// feasible: an iteration cap only costs optimality, never a hard constraint.
// Iterations are about the number of constraints that become active.
class ProjectionQP {
 public:
  static constexpr double kSlackWeight = 1e4;
  static constexpr int kMaxIterations = 200;

  Eigen::VectorXd solve(const Eigen::MatrixXd &metric, const Eigen::VectorXd &proposed,
                        const Eigen::VectorXd &lower, const Eigen::VectorXd &upper,
                        const Eigen::VectorXd &soft_lower, const Eigen::VectorXd &soft_upper,
                        const Eigen::MatrixXd &g, const Eigen::VectorXd &h);
  int lastIterations() const { return iterations_; }
  bool lastConverged() const { return converged_; }
  // Largest hard-row round-off the final clip/scale removed.
  double lastRepair() const { return repair_; }

 private:
  int iterations_{0};
  bool converged_{false};
  double repair_{0};
};
}  // namespace libpyelite
