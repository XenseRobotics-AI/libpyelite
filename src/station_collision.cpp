#include "station_collision.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <numeric>
#include <stdexcept>

// Pinocchio's joint collection has 25 alternatives; see pink_ik.cpp.
#include <pinocchio/deprecated/pinocchio/container/boost-container-limits.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/math/rpy.hpp>
#include <pinocchio/multibody.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/skew.hpp>

#include <coal/collision_data.h>
#include <coal/distance.h>
#include <coal/shape/geometric_shapes.h>

namespace libpyelite {
namespace {
using Matrix6x = Eigen::Matrix<double, 6, Eigen::Dynamic>;
constexpr double kMotionBoundSafety = 1.05;  // second-order slack of the first-order motion bound
constexpr double kMotionBoundFloor = 1e-6;   // m
}  // namespace

struct StationCollisionModel::Impl {
  pinocchio::Model model;
  pinocchio::Data data;
  std::vector<CollisionShape> shapes;
  std::vector<std::shared_ptr<coal::CollisionGeometry>> geometry;
  std::vector<pinocchio::FrameIndex> frame;
  std::vector<pinocchio::SE3> local;
  std::vector<Eigen::Vector3d> half;      // local AABB half extents
  std::vector<double> bounding_radius;    // bounding sphere about the shape centre
  std::vector<bool> moving;
  std::vector<int> moving_shapes;
  std::vector<pinocchio::SE3> world;
  std::vector<Eigen::Vector3d> lower, upper;
  // Joints (velocity columns) that move each shape; empty for static shapes.
  std::vector<std::vector<int>> chain;
  // Per pair: cached shape-pair dispatch and GJK solver, and a warm-started request.
  std::vector<coal::ComputeDistance> distance;
  std::vector<coal::DistanceRequest> request;
  // Broad-phase level 1: a moving frame's shapes form one group, each static
  // shape is its own. A group's AABB encloses its members', so a group-pair gap
  // bounds every member pair's AABB gap, hence its distance, from below; member
  // pairs are only looked at when their group pair passes. A finely fitted part
  // (26 fingertip boxes) then costs one test per obstacle, not 26.
  struct GroupPair {
    int first, second;
    double widest_margin;
    std::vector<int> pairs;  // ascending pair indices
  };
  std::vector<int> group;  // per shape
  std::vector<std::vector<int>> group_shapes;
  std::vector<int> moving_groups;
  std::vector<Eigen::Vector3d> group_lower, group_upper;
  std::vector<GroupPair> group_pairs;
  Eigen::VectorXd q;

  Impl(const std::string &urdf) {
    pinocchio::urdf::buildModel(urdf, model);
    data = pinocchio::Data(model);
  }

  void place(const Eigen::VectorXd &configuration, bool everything) {
    if(configuration.size() != model.nq || !configuration.allFinite())
      throw std::invalid_argument("Invalid collision configuration");
    pinocchio::framesForwardKinematics(model, data, configuration);
    q = configuration;
    for(std::size_t k = 0; k < shapes.size(); ++k) {
      if(!everything && !moving[k]) continue;
      world[k] = data.oMf[frame[k]] * local[k];
      const Eigen::Vector3d extent = world[k].rotation().cwiseAbs() * half[k];
      lower[k] = world[k].translation() - extent;
      upper[k] = world[k].translation() + extent;
    }
    auto enclose = [&](int g) {
      group_lower[g] = lower[group_shapes[g].front()];
      group_upper[g] = upper[group_shapes[g].front()];
      for(int k : group_shapes[g]) {
        group_lower[g] = group_lower[g].cwiseMin(lower[k]);
        group_upper[g] = group_upper[g].cwiseMax(upper[k]);
      }
    };
    if(everything)
      for(std::size_t g = 0; g < group_shapes.size(); ++g) enclose(static_cast<int>(g));
    else
      for(int g : moving_groups) enclose(g);
  }

  static double gap(const Eigen::Vector3d &lower_a, const Eigen::Vector3d &upper_a, const Eigen::Vector3d &lower_b,
                    const Eigen::Vector3d &upper_b) {
    return (lower_a - upper_b).cwiseMax(lower_b - upper_a).cwiseMax(0.0).norm();
  }

  double aabbGap(int a, int b) const { return gap(lower[a], upper[a], lower[b], upper[b]); }

  double groupGap(const GroupPair &p) const {
    return gap(group_lower[p.first], group_upper[p.first], group_lower[p.second], group_upper[p.second]);
  }

  double measure(int index, const std::array<int, 2> &pair, coal::DistanceResult &result) {
    result.clear();
    const auto &a = world[pair[0]], &b = world[pair[1]];
    const double d = distance[index](coal::Transform3s(a.rotation(), a.translation()),
                                     coal::Transform3s(b.rotation(), b.translation()), request[index], result);
    if(!std::isfinite(d)) throw std::runtime_error("Nonfinite collision distance");
    request[index].updateGuess(result);
    return d;
  }
};

StationCollisionModel::StationCollisionModel(const std::string &urdf, std::vector<CollisionShape> shapes,
                                             std::vector<std::array<int, 2>> pairs, std::vector<double> margins,
                                             double detection_distance, double gain, double qp_buffer)
    : impl_(std::make_unique<Impl>(urdf)), pairs_(std::move(pairs)), margins_(std::move(margins)),
      detection_(detection_distance), gain_(gain), buffer_(qp_buffer) {
  auto &m = *impl_;
  if(margins_.size() != pairs_.size()) throw std::invalid_argument("One margin per collision pair");
  for(double value : {detection_, gain_, buffer_})
    if(!std::isfinite(value) || value < 0) throw std::invalid_argument("Invalid collision barrier parameters");
  m.shapes = std::move(shapes);
  const std::size_t n = m.shapes.size();
  m.world.resize(n); m.lower.resize(n); m.upper.resize(n); m.chain.resize(n);
  for(const auto &s : m.shapes) {
    if(!m.model.existFrame(s.frame)) throw std::invalid_argument("Missing collision frame: " + s.frame);
    const auto frame = m.model.getFrameId(s.frame);
    m.frame.push_back(frame);
    m.local.emplace_back(pinocchio::rpy::rpyToMatrix(s.rpy[0], s.rpy[1], s.rpy[2]),
                         Eigen::Vector3d(s.xyz[0], s.xyz[1], s.xyz[2]));
    if(s.type == 0) {
      m.geometry.push_back(std::make_shared<coal::Box>(s.size[0], s.size[1], s.size[2]));
      m.half.emplace_back(s.size[0] / 2, s.size[1] / 2, s.size[2] / 2);
    } else if(s.type == 1) {
      m.geometry.push_back(std::make_shared<coal::Capsule>(s.radius, s.length));
      m.half.emplace_back(s.radius, s.radius, s.length / 2 + s.radius);
    } else if(s.type == 2) {
      m.geometry.push_back(std::make_shared<coal::Cylinder>(s.radius, s.length));
      m.half.emplace_back(s.radius, s.radius, s.length / 2);
    } else throw std::invalid_argument("Unknown collision primitive type");
    if(!(m.half.back().array() > 0).all()) throw std::invalid_argument("Invalid primitive dimensions");
    m.bounding_radius.push_back(s.type == 1 ? s.length / 2 + s.radius : m.half.back().norm());
    const auto joint = m.model.frames[frame].parentJoint;
    m.moving.push_back(joint != 0);
  }
  for(std::size_t k = 0; k < n; ++k) {
    if(!m.moving[k]) continue;
    m.moving_shapes.push_back(static_cast<int>(k));
    for(auto joint = m.model.frames[m.frame[k]].parentJoint; joint != 0; joint = m.model.parents[joint]) {
      if(m.model.joints[joint].nv() != 1) throw std::invalid_argument("Collision chains need one-DoF joints");
      m.chain[k].push_back(m.model.joints[joint].idx_v());
    }
  }
  m.distance.reserve(pairs_.size());
  m.request.resize(pairs_.size());
  for(std::size_t index = 0; index < pairs_.size(); ++index) {
    const auto &p = pairs_[index];
    if(p[0] < 0 || p[1] < 0 || p[0] >= static_cast<int>(n) || p[1] >= static_cast<int>(n) || p[0] == p[1])
      throw std::invalid_argument("Invalid collision pair");
    if(!std::isfinite(margins_[index]) || margins_[index] < 0) throw std::invalid_argument("Invalid pair margin");
    m.distance.emplace_back(m.geometry[p[0]].get(), m.geometry[p[1]].get());
    auto &request = m.request[index];
    request.enable_signed_distance = true;
    request.gjk_initial_guess = coal::GJKInitialGuess::CachedGuess;
  }
  std::map<pinocchio::FrameIndex, int> frame_group;
  m.group.resize(n);
  for(std::size_t k = 0; k < n; ++k) {
    int g = static_cast<int>(m.group_shapes.size());
    if(m.moving[k]) {
      const auto [it, added] = frame_group.emplace(m.frame[k], g);
      if(added) m.moving_groups.push_back(g);
      g = it->second;
    }
    if(g == static_cast<int>(m.group_shapes.size())) m.group_shapes.emplace_back();
    m.group[k] = g;
    m.group_shapes[static_cast<std::size_t>(g)].push_back(static_cast<int>(k));
  }
  m.group_lower.resize(m.group_shapes.size());
  m.group_upper.resize(m.group_shapes.size());
  std::map<std::pair<int, int>, std::size_t> group_pair;
  for(std::size_t index = 0; index < pairs_.size(); ++index) {
    const int a = m.group[pairs_[index][0]], b = m.group[pairs_[index][1]];
    const auto [it, added] = group_pair.emplace(std::minmax(a, b), m.group_pairs.size());
    if(added) m.group_pairs.push_back({a, b, 0.0, {}});
    auto &p = m.group_pairs[it->second];
    p.widest_margin = std::max(p.widest_margin, margins_[index]);
    p.pairs.push_back(static_cast<int>(index));
  }
  m.place(pinocchio::neutral(m.model), true);  // static shapes once
}

StationCollisionModel::~StationCollisionModel() = default;

int StationCollisionModel::nq() const { return impl_->model.nq; }

ClearanceResult StationCollisionModel::check(const Eigen::VectorXd &q, double cutoff) {
  auto &m = *impl_;
  m.place(q, false);
  narrow_calls_ = 0;
  // The AABB gap never exceeds the true distance, so gap - margin bounds each
  // pair's clearance from below, and a group pair's gap - widest margin bounds
  // all its members'. Measure a promising pair first; only pairs whose bound can
  // still beat that need a narrow-phase query.
  constexpr double kInf = std::numeric_limits<double>::infinity();
  auto pair_bound = [&](int index) {
    const auto i = static_cast<std::size_t>(index);
    const double gap = m.aabbGap(pairs_[i][0], pairs_[i][1]);
    return (cutoff >= 0 && gap > cutoff) ? kInf : gap - margins_[i];
  };
  std::vector<double> group_bound(m.group_pairs.size());
  for(std::size_t g = 0; g < m.group_pairs.size(); ++g) {
    const double gap = m.groupGap(m.group_pairs[g]);
    group_bound[g] = (cutoff >= 0 && gap > cutoff) ? kInf : gap - m.group_pairs[g].widest_margin;
  }
  // Seed: the best member of the most promising group pair (a group pair can
  // pass the cutoff while none of its members does; then try the next one).
  int first = -1;
  std::vector<double> seed_bound = group_bound;
  while(first < 0 && !seed_bound.empty()) {
    const auto g = std::min_element(seed_bound.begin(), seed_bound.end()) - seed_bound.begin();
    if(!std::isfinite(seed_bound[g])) break;
    double lowest = kInf;
    for(int index : m.group_pairs[g].pairs) {
      const double b = pair_bound(index);
      if(b < lowest) lowest = b, first = index;
    }
    seed_bound[g] = kInf;
  }
  ClearanceResult best;
  if(first < 0) return best;
  coal::DistanceResult result;
  auto visit = [&](int index) {
    const double d = m.measure(index, pairs_[static_cast<std::size_t>(index)], result);
    ++narrow_calls_;
    const double clearance = d - margins_[static_cast<std::size_t>(index)];
    if(clearance < best.clearance) best = {clearance >= -1e-9, d, index, clearance};
  };
  visit(first);
  std::vector<std::pair<double, int>> order;
  const double limit = std::max(best.clearance, 0.0);
  for(std::size_t g = 0; g < m.group_pairs.size(); ++g) {
    if(group_bound[g] > limit) continue;
    for(int index : m.group_pairs[g].pairs) {
      if(index == first) continue;
      const double b = pair_bound(index);
      if(b <= limit) order.emplace_back(b, index);
    }
  }
  std::sort(order.begin(), order.end());
  for(const auto &[lower, index] : order) {
    if(lower > std::max(best.clearance, 0.0)) break;
    visit(index);
  }
  return best;
}

void StationCollisionModel::linearize(const Eigen::VectorXd &q, const std::vector<int> &velocity_indices,
                                      Eigen::MatrixXd &rows, Eigen::VectorXd &bounds, ClearanceResult &closest) {
  auto &m = *impl_;
  for(int column : velocity_indices)
    if(column < 0 || column >= m.model.nv) throw std::invalid_argument("Invalid velocity index");
  m.place(q, false);
  narrow_calls_ = 0;
  contacts_.clear();
  closest = ClearanceResult{};
  coal::DistanceResult result;
  // Same candidates, in the same ascending order, as a flat scan of every pair.
  std::vector<int> candidates;
  for(const auto &g : m.group_pairs) {
    if(m.groupGap(g) > detection_) continue;
    for(int index : g.pairs)
      if(m.aabbGap(pairs_[static_cast<std::size_t>(index)][0], pairs_[static_cast<std::size_t>(index)][1]) <= detection_)
        candidates.push_back(index);
  }
  std::sort(candidates.begin(), candidates.end());
  for(int candidate : candidates) {
    const auto index = static_cast<std::size_t>(candidate);
    const auto &p = pairs_[index];
    const double d = m.measure(static_cast<int>(index), p, result);
    ++narrow_calls_;
    const double clearance = d - margins_[index];
    if(clearance < closest.clearance) closest = {clearance >= -1e-9, d, static_cast<int>(index), clearance};
    if(d > detection_) continue;
    contacts_.push_back({static_cast<int>(index), d, result.nearest_points[0], result.nearest_points[1]});
  }
  linearized_q_ = q;
  rows.setZero(static_cast<Eigen::Index>(contacts_.size()), static_cast<Eigen::Index>(velocity_indices.size()));
  bounds.resize(static_cast<Eigen::Index>(contacts_.size()));
  if(contacts_.empty()) return;
  // One Jacobian pass for every row; joint columns are world twists at the origin.
  pinocchio::computeJointJacobians(m.model, m.data, q);
  for(std::size_t r = 0; r < contacts_.size(); ++r) {
    const auto &c = contacts_[r];
    if(c.distance <= 1e-9) throw std::runtime_error("Already touching/penetrating; reset to a checked configuration");
    const Eigen::Vector3d normal = (c.point1 - c.point2).normalized();
    for(std::size_t k = 0; k < velocity_indices.size(); ++k) {
      const int v = velocity_indices[k];
      double gradient = 0;
      for(int side = 0; side < 2; ++side) {
        const int shape = pairs_[c.pair][side];
        const auto &chain = m.chain[shape];
        if(std::find(chain.begin(), chain.end(), v) == chain.end()) continue;
        const Eigen::Vector3d point = side == 0 ? c.point1 : c.point2;
        const Eigen::Vector3d velocity = m.data.J.col(v).head<3>() + m.data.J.col(v).tail<3>().cross(point);
        gradient += (side == 0 ? 1.0 : -1.0) * normal.dot(velocity);
      }
      rows(static_cast<Eigen::Index>(r), static_cast<Eigen::Index>(k)) = -gradient;
    }
    bounds[static_cast<Eigen::Index>(r)] = gain_ * std::max(0.0, c.distance - margins_[c.pair] - buffer_);
  }
}

ClearanceResult StationCollisionModel::verifyStep(const Eigen::VectorXd &q0, const Eigen::VectorXd &q1,
                                                  double resolution) {
  auto &m = *impl_;
  if(q0.size() != m.model.nq || q1.size() != m.model.nq || !q1.allFinite() || !(resolution > 0))
    throw std::invalid_argument("Invalid collision segment");
  if(linearized_q_.size() != q0.size() || (linearized_q_ - q0).cwiseAbs().maxCoeff() > 1e-12)
    throw std::logic_error("verifyStep needs the configuration of the last linearize()");
  sampled_pairs_ = 0;
  narrow_calls_ = 0;
  const Eigen::VectorXd dq = q1 - q0;
  // Motion bound per moving shape at q0: sum_j |dq_j| (|v_j + w_j x c| + |w_j| r).
  pinocchio::computeJointJacobians(m.model, m.data, q0);
  pinocchio::framesForwardKinematics(m.model, m.data, q0);
  std::vector<double> bound(m.shapes.size(), 0.0);
  double largest = 0;
  for(int k : m.moving_shapes) {
    const Eigen::Vector3d centre = (m.data.oMf[m.frame[k]] * m.local[k]).translation();
    double sum = 0;
    for(int v : m.chain[k]) {
      const auto column = m.data.J.col(v);
      sum += std::abs(dq[v]) * ((column.head<3>() + column.tail<3>().cross(centre)).norm() +
                                column.tail<3>().norm() * m.bounding_radius[k]);
    }
    bound[k] = kMotionBoundSafety * sum + kMotionBoundFloor;
    largest = std::max(largest, bound[k]);
  }
  double widest_margin = 0;
  for(double margin : margins_) widest_margin = std::max(widest_margin, margin);
  if(2 * largest > detection_ - widest_margin) {
    // A pair outside the band could close in on its margin within this step:
    // the band argument does not hold, check the end configuration exhaustively.
    return check(q1);
  }
  ClearanceResult worst;
  std::vector<int> unproven;
  for(const auto &c : contacts_) {
    const auto &p = pairs_[c.pair];
    const double clearance = c.distance - bound[p[0]] - bound[p[1]] - margins_[c.pair];
    if(clearance >= 0) {
      if(clearance < worst.clearance) worst = {true, c.distance - bound[p[0]] - bound[p[1]], c.pair, clearance};
    } else unproven.push_back(c.pair);
  }
  if(unproven.empty()) return worst;
  // Sample only the pairs the bound could not clear, including the end point.
  const int samples = std::max(1, static_cast<int>(std::ceil(dq.cwiseAbs().maxCoeff() / resolution)));
  sampled_pairs_ = static_cast<int>(unproven.size());
  coal::DistanceResult result;
  for(int s = 1; s <= samples; ++s) {
    m.place(q0 + dq * (static_cast<double>(s) / samples), false);
    for(int index : unproven) {
      const double d = m.measure(index, pairs_[index], result);
      ++narrow_calls_;
      const double clearance = d - margins_[index];
      if(clearance < worst.clearance) worst = {clearance >= -1e-9, d, index, clearance};
      if(clearance < -1e-9) {
        m.place(q0, false);
        return worst;
      }
    }
  }
  m.place(q0, false);
  return worst;
}

std::vector<std::array<double, 16>> StationCollisionModel::placements(const Eigen::VectorXd &q) {
  auto &m = *impl_;
  m.place(q, false);
  std::vector<std::array<double, 16>> result(m.shapes.size());
  for(std::size_t k = 0; k < m.shapes.size(); ++k) {
    const Eigen::Matrix4d h = m.world[k].toHomogeneousMatrix();
    for(int r = 0; r < 4; ++r)
      for(int c = 0; c < 4; ++c) result[k][static_cast<std::size_t>(4 * r + c)] = h(r, c);
  }
  return result;
}
}  // namespace libpyelite
