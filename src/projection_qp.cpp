#include "projection_qp.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace libpyelite {
namespace {
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kStepTolerance = 1e-12;     // scaled units: a step below this is "no step"
constexpr double kMultiplierTolerance = 1e-10;
constexpr double kDirectionTolerance = 1e-14;
}  // namespace

Eigen::VectorXd ProjectionQP::solve(const Eigen::MatrixXd &metric, const Eigen::VectorXd &proposed,
                                    const Eigen::VectorXd &lower, const Eigen::VectorXd &upper,
                                    const Eigen::VectorXd &soft_lower, const Eigen::VectorXd &soft_upper,
                                    const Eigen::MatrixXd &g, const Eigen::VectorXd &h) {
  const Eigen::Index n = proposed.size(), rows = h.size();
  if(metric.rows() != n || metric.cols() != n || lower.size() != n || upper.size() != n ||
     soft_lower.size() != n || soft_upper.size() != n || g.rows() != rows || (rows && g.cols() != n))
    throw std::invalid_argument("Projection QP dimensions do not match");
  if(!proposed.allFinite() || !lower.allFinite() || !upper.allFinite() || !g.allFinite() || !h.allFinite() ||
     !metric.allFinite() || (lower.array() > upper.array()).any())
    throw std::runtime_error("Collision/motion QP: infeasible or nonfinite bounds");
  if((lower.array() > 0).any() || (upper.array() < 0).any() || (h.array() < 0).any())
    throw std::runtime_error("Collision/motion QP: holding still is not feasible");
  const bool soft = (soft_lower.array() > -kInf).any() || (soft_upper.array() < kInf).any();

  // Scaled variables y = [x / scale; s / scale], constraints a_i^T y >= b_i.
  const double scale = std::max((upper - lower).maxCoeff(), 1e-9);
  const Eigen::Index vars = soft ? 2 * n : n;
  std::vector<Eigen::VectorXd> a;
  std::vector<double> b;
  auto add = [&](Eigen::VectorXd row, double bound) { a.push_back(std::move(row)); b.push_back(bound); };
  for(Eigen::Index i = 0; i < n; ++i) {
    Eigen::VectorXd e = Eigen::VectorXd::Zero(vars);
    e[i] = 1; add(e, lower[i] / scale);
    e[i] = -1; add(e, -upper[i] / scale);
    if(!soft) continue;
    Eigen::VectorXd f = Eigen::VectorXd::Zero(vars);
    if(soft_lower[i] > -kInf) { f[i] = 1; f[n + i] = 1; add(f, soft_lower[i] / scale); }
    if(soft_upper[i] < kInf) { f.setZero(); f[i] = -1; f[n + i] = 1; add(f, -soft_upper[i] / scale); }
    f.setZero(); f[n + i] = 1; add(f, 0.0);
  }
  for(Eigen::Index r = 0; r < rows; ++r) {
    Eigen::VectorXd row = Eigen::VectorXd::Zero(vars);
    row.head(n) = -g.row(r).transpose() * scale;
    add(row, -h[r]);
  }
  const std::size_t constraints = a.size();

  const Eigen::MatrixXd mm = (metric + metric.transpose()) / 2 / std::max(metric.trace() / n, 1e-12);
  Eigen::MatrixXd p = Eigen::MatrixXd::Zero(vars, vars);
  p.topLeftCorner(n, n) = mm;
  if(soft) p.bottomRightCorner(n, n).diagonal().setConstant(kSlackWeight);
  Eigen::VectorXd q = Eigen::VectorXd::Zero(vars);
  q.head(n) = -mm * proposed / scale;

  // Feasible start: no motion, slacks just covering the soft bounds.
  Eigen::VectorXd y = Eigen::VectorXd::Zero(vars);
  if(soft)
    for(Eigen::Index i = 0; i < n; ++i)
      y[n + i] = std::max({0.0, soft_lower[i] / scale, -soft_upper[i] / scale});

  std::vector<std::size_t> working;
  std::vector<bool> in_working(constraints, false);
  converged_ = false;
  int it = 0;
  for(; it < kMaxIterations; ++it) {
    // Equality-constrained step: min 1/2 d^T P d + (P y + q)^T d, a_i^T d = 0 for i in W.
    const Eigen::Index w = static_cast<Eigen::Index>(working.size());
    Eigen::MatrixXd kkt = Eigen::MatrixXd::Zero(vars + w, vars + w);
    kkt.topLeftCorner(vars, vars) = p;
    for(Eigen::Index k = 0; k < w; ++k) {
      kkt.block(0, vars + k, vars, 1) = -a[working[static_cast<std::size_t>(k)]];
      kkt.block(vars + k, 0, 1, vars) = a[working[static_cast<std::size_t>(k)]].transpose();
    }
    Eigen::VectorXd rhs = Eigen::VectorXd::Zero(vars + w);
    rhs.head(vars) = -(p * y + q);
    // Rank-revealing: a dependent working set (degenerate vertex) stays solvable.
    const Eigen::VectorXd solution = kkt.completeOrthogonalDecomposition().solve(rhs);
    const Eigen::VectorXd d = solution.head(vars);
    if(!d.allFinite()) break;
    if(d.lpNorm<Eigen::Infinity>() <= kStepTolerance) {
      // Stationary on W: optimal if every multiplier is nonnegative, else drop the most negative.
      Eigen::Index worst = -1;
      double most = -kMultiplierTolerance;
      for(Eigen::Index k = 0; k < w; ++k)
        if(solution[vars + k] < most) { most = solution[vars + k]; worst = k; }
      if(worst < 0) { converged_ = true; ++it; break; }
      in_working[working[static_cast<std::size_t>(worst)]] = false;
      working.erase(working.begin() + worst);
      continue;
    }
    // Longest feasible step along d; the first constraint it reaches joins W.
    double alpha = 1.0;
    std::size_t blocking = constraints;
    for(std::size_t i = 0; i < constraints; ++i) {
      if(in_working[i]) continue;
      const double rate = a[i].dot(d);
      if(rate >= -kDirectionTolerance) continue;
      const double reach = std::max(0.0, (b[i] - a[i].dot(y)) / rate);
      if(reach < alpha) { alpha = reach; blocking = i; }
    }
    y += alpha * d;
    if(blocking < constraints) { working.push_back(blocking); in_working[blocking] = true; }
  }
  iterations_ = it;

  // Round-off guard on the hard rows (a no-op unless the iterates drifted).
  Eigen::VectorXd x = (y.head(n) * scale).cwiseMax(lower).cwiseMin(upper);
  repair_ = 0;
  if(rows) {
    const Eigen::VectorXd demand = g * x;
    double fraction = 1.0;
    for(Eigen::Index r = 0; r < rows; ++r)
      if(demand[r] > h[r]) {
        repair_ = std::max(repair_, demand[r] - h[r]);
        fraction = std::min(fraction, demand[r] > 0 ? h[r] / demand[r] : 0.0);
      }
    x *= std::max(0.0, fraction);
  }
  if(!x.allFinite()) throw std::runtime_error("Collision/motion QP: nonfinite solution");
  return x;
}
}  // namespace libpyelite
