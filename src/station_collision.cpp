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
#include <coal/shape/convex.h>

namespace libpyelite {
namespace {
using Matrix6x = Eigen::Matrix<double, 6, Eigen::Dynamic>;
// Coal trusts the supplied topology. Validate before constructing it, including
// callers that bypass the JSON/Python profile reader.
std::shared_ptr<coal::CollisionGeometry> convexGeometry(const CollisionShape &shape) {
  const auto n = shape.vertices.size();
  if(n < 4 || n > 256 || shape.faces.size() < 4 || shape.faces.size() > 2 * n - 4)
    throw std::invalid_argument("Invalid convex vertex/face count");
  auto points = std::make_shared<std::vector<coal::Vec3s>>();
  Eigen::Vector3d lo = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity()), hi = -lo;
  Eigen::Vector3d centre = Eigen::Vector3d::Zero();
  for(const auto &v : shape.vertices) {
    points->emplace_back(v[0], v[1], v[2]);
    if(!points->back().allFinite()) throw std::invalid_argument("Nonfinite convex vertex");
    lo = lo.cwiseMin(points->back()); hi = hi.cwiseMax(points->back());
    centre += points->back() / static_cast<double>(n);
  }
  const double scale = (hi - lo).maxCoeff(), tolerance = scale * 1e-9;
  if(!std::isfinite(scale) || !(scale > 0) || !centre.allFinite())
    throw std::invalid_argument("Degenerate convex extent");
  for(std::size_t i = 0; i < n; ++i)
    for(std::size_t j = 0; j < i; ++j)
      if(((*points)[i] - (*points)[j]).norm() <= tolerance)
        throw std::invalid_argument("Duplicate convex vertices");
  auto polygons = std::make_shared<std::vector<coal::Triangle>>();
  std::map<std::pair<int, int>, int> edges;
  std::vector<bool> used(n, false);
  double volume6 = 0;
  for(const auto &face : shape.faces) {
    for(int i : face)
      if(i < 0 || i >= static_cast<int>(n)) throw std::invalid_argument("Invalid convex face index");
    const Eigen::Vector3d a = (*points)[face[0]], b = (*points)[face[1]], c = (*points)[face[2]];
    const Eigen::Vector3d normal = (b - a).cross(c - a);
    const double area2 = normal.norm();
    if(!std::isfinite(area2) || area2 <= scale * scale * 1e-12)
      throw std::invalid_argument("Degenerate convex face");
    const Eigen::Vector3d unit = normal / area2;
    if(unit.dot(centre - a) >= -tolerance) throw std::invalid_argument("Convex faces must point outward");
    for(const auto &v : *points)
      if(unit.dot(v - a) > tolerance) throw std::invalid_argument("Nonconvex exterior");
    volume6 += (a - centre).dot((b - centre).cross(c - centre));
    for(int k = 0; k < 3; ++k) {
      used[face[k]] = true;
      if(++edges[{face[k], face[(k + 1) % 3]}] != 1)
        throw std::invalid_argument("Duplicate convex edge");
    }
    polygons->emplace_back(face[0], face[1], face[2]);
  }
  for(const auto &[edge, count] : edges)
    if(edges.find({edge.second, edge.first}) == edges.end())
      throw std::invalid_argument("Open convex exterior");
  if(std::find(used.begin(), used.end(), false) != used.end() ||
     static_cast<int>(n) - static_cast<int>(edges.size() / 2) + static_cast<int>(shape.faces.size()) != 2 ||
     !std::isfinite(volume6) || volume6 <= scale * scale * scale * 1e-12)
    throw std::invalid_argument("Invalid convex exterior topology or volume");
  return std::make_shared<coal::Convex<coal::Triangle>>(points, n, polygons, polygons->size());
}
constexpr double kMotionSlack = 1e-6;  // m; numerical reserve, not a finite-rotation approximation
constexpr int kMaxSegmentIntervals = 512;
constexpr int kMaxSegmentQueries = 4096;
constexpr int kMaxSegmentPairVisits = 131072;
constexpr int kMaxSegmentDepth = 32;
}  // namespace

struct StationCollisionModel::Impl {
  pinocchio::Model model;
  pinocchio::Data data;
  std::vector<CollisionShape> shapes;
  std::vector<std::shared_ptr<coal::CollisionGeometry>> geometry;
  std::vector<pinocchio::FrameIndex> frame;
  std::vector<pinocchio::SE3> local;
  std::vector<Eigen::Vector3d> half;      // local AABB half extents
  std::vector<Eigen::Vector3d> aabb_center;  // convex coordinates need not be centred
  std::vector<double> bounding_radius;    // bounding sphere about the placement origin
  std::vector<bool> moving;
  std::vector<int> moving_shapes;
  std::vector<pinocchio::SE3> world;
  std::vector<Eigen::Vector3d> lower, upper;
  // Joints (velocity columns) that move each shape; empty for static shapes.
  std::vector<std::vector<int>> chain;
  // Separate configuration indices: idx_q is not generally the velocity index.
  // Built lazily so check()/linearize() retain their existing joint support.
  struct MotionLink {
    int q_index;
    int v_index;
    bool prismatic;
    double offset;
  };
  struct MotionChain {
    double radius;
    std::vector<MotionLink> links;  // shape's parent joint towards the root
  };
  std::vector<MotionChain> motion_chains;
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

  std::vector<double> motionBounds(const Eigen::VectorXd &q0, const Eigen::VectorXd &q1) {
    if(motion_chains.empty()) {
      std::vector<MotionChain> chains(shapes.size());
      for(int k : moving_shapes) {
        const auto &f = model.frames[frame[k]];
        auto &c = chains[k];
        c.radius = (f.placement * local[k]).translation().norm() + bounding_radius[k];
        for(auto j = f.parentJoint; j != 0; j = model.parents[j]) {
          const auto &joint = model.joints[j];
          const auto name = joint.shortname();
          const bool prismatic = name == "JointModelPX" || name == "JointModelPY" ||
                                 name == "JointModelPZ" || name == "JointModelPrismaticUnaligned";
          const bool revolute = name == "JointModelRX" || name == "JointModelRY" ||
                                name == "JointModelRZ" || name == "JointModelRevoluteUnaligned";
          if(joint.nq() != 1 || joint.nv() != 1 || !(prismatic || revolute))
            throw std::invalid_argument("verifyStep supports scalar revolute/prismatic collision chains only");
          c.links.push_back({joint.idx_q(), joint.idx_v(), prismatic, model.jointPlacements[j].translation().norm()});
        }
      }
      motion_chains = std::move(chains);
    }
    std::vector<double> bound(shapes.size(), 0.0);
    for(int k : moving_shapes) {
      const auto &c = motion_chains[k];
      double reach = c.radius, travel = 0;
      for(const auto &j : c.links) {
        const double delta = std::abs(q1[j.q_index] - q0[j.q_index]);
        travel += delta * (j.prismatic ? 1.0 : reach);
        // Rotations preserve lengths. The triangle inequality bounds the
        // lever arm at EVERY configuration, including simultaneous rotations
        // and downstream prismatic travel, not just the Jacobian at q0.
        reach += j.offset;
        if(j.prismatic) reach += std::max(std::abs(q0[j.q_index]), std::abs(q1[j.q_index]));
      }
      if(!std::isfinite(travel) || !std::isfinite(reach))
        throw std::invalid_argument("Nonfinite collision motion bound");
      bound[k] = travel;
    }
    return bound;
  }

  struct MotionDerivativeBound {
    double speed{0}, acceleration{0};
  };

  std::vector<MotionDerivativeBound> derivativeBounds(const Eigen::VectorXd &q0,
                                                     const Eigen::VectorXd &q1) {
    motionBounds(q0, q1);  // Build and validate the same scalar-joint chains.
    std::vector<MotionDerivativeBound> bounds(shapes.size());
    for(int k : moving_shapes) {
      double reach = motion_chains[k].radius;
      auto &b = bounds[k];
      for(const auto &j : motion_chains[k].links) {
        const double delta = std::abs(q1[j.q_index] - q0[j.q_index]);
        if(j.prismatic) {
          b.speed += delta;
          reach += std::max(std::abs(q0[j.q_index]), std::abs(q1[j.q_index]));
        } else {
          // Differentiate R(t)x(t): the cross term must use the OLD speed.
          b.acceleration += 2 * delta * b.speed + delta * delta * reach;
          b.speed += delta * reach;
        }
        reach += j.offset;
      }
      const double reserve = 1 + 128 * std::numeric_limits<double>::epsilon();
      b.speed *= reserve;
      b.acceleration *= reserve;
      if(!std::isfinite(b.speed) || !std::isfinite(b.acceleration))
        throw std::invalid_argument("Nonfinite collision derivative bound");
    }
    return bounds;
  }

  double support(int k, const Eigen::Vector3d &direction) const {
    const auto &s = shapes[k];
    if(s.type == 0) return half[k].dot(direction.cwiseAbs());
    if(s.type == 1) return s.length * 0.5 * std::abs(direction.z()) + s.radius * direction.norm();
    if(s.type == 2) return s.length * 0.5 * std::abs(direction.z()) + s.radius * direction.head<2>().norm();
    double largest = -std::numeric_limits<double>::infinity();
    for(const auto &v : s.vertices)
      largest = std::max(largest, direction.dot(Eigen::Vector3d(v[0], v[1], v[2])));
    return largest;
  }

  // Construct a material point instead of trusting a distance solver's rounded
  // nearest point to lie in the primitive. Convex-face candidates are explicit
  // convex combinations of validated vertices, including edges and vertices.
  Eigen::Vector3d witness(int k, const Eigen::Vector3d &world_point) const {
    const Eigen::Vector3d p = world[k].actInv(world_point);
    const auto &s = shapes[k];
    Eigen::Vector3d local_point = p;
    if(s.type == 0) {
      local_point = p.cwiseMax(-half[k]).cwiseMin(half[k]);
    } else if(s.type == 1) {
      const Eigen::Vector3d axis(0, 0, std::clamp(p.z(), -s.length * 0.5, s.length * 0.5));
      const Eigen::Vector3d radial = p - axis;
      if(radial.norm() > s.radius) local_point = axis + radial * (s.radius / radial.norm());
    } else if(s.type == 2) {
      local_point.z() = std::clamp(p.z(), -s.length * 0.5, s.length * 0.5);
      const double radial = p.head<2>().norm();
      if(radial > s.radius) local_point.head<2>() *= s.radius / radial;
    } else {
      double best = std::numeric_limits<double>::infinity();
      auto consider = [&](const Eigen::Vector3d &candidate) {
        const double squared = (p - candidate).squaredNorm();
        if(squared < best) { best = squared; local_point = candidate; }
      };
      for(const auto &face : s.faces) {
        std::array<Eigen::Vector3d, 3> v;
        for(int i = 0; i < 3; ++i) {
          const auto &vertex = s.vertices[face[i]];
          v[i] = Eigen::Vector3d(vertex[0], vertex[1], vertex[2]);
        }
        for(int i = 0; i < 3; ++i) {
          const Eigen::Vector3d edge = v[(i + 1) % 3] - v[i];
          const double t = std::clamp((p - v[i]).dot(edge) / edge.squaredNorm(), 0.0, 1.0);
          consider((1 - t) * v[i] + t * v[(i + 1) % 3]);
        }
        const Eigen::Vector3d ab = v[1] - v[0], ac = v[2] - v[0], ap = p - v[0];
        const double aa = ab.squaredNorm(), cc = ac.squaredNorm(), cross = ab.dot(ac);
        const double determinant = aa * cc - cross * cross;
        if(determinant <= 0) continue;
        const double b = (cc * ap.dot(ab) - cross * ap.dot(ac)) / determinant;
        const double c = (aa * ap.dot(ac) - cross * ap.dot(ab)) / determinant;
        if(b >= 0 && c >= 0 && b + c <= 1) consider((1 - b - c) * v[0] + b * v[1] + c * v[2]);
      }
    }
    return world[k].act(local_point);
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
      const Eigen::Vector3d centre = world[k].act(aabb_center[k]);
      lower[k] = centre - extent;
      upper[k] = centre + extent;
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
    for(double value : s.xyz)
      if(!std::isfinite(value)) throw std::invalid_argument("Nonfinite collision transform");
    for(double value : s.rpy)
      if(!std::isfinite(value)) throw std::invalid_argument("Nonfinite collision transform");
    const auto frame = m.model.getFrameId(s.frame);
    m.frame.push_back(frame);
    m.local.emplace_back(pinocchio::rpy::rpyToMatrix(s.rpy[0], s.rpy[1], s.rpy[2]),
                         Eigen::Vector3d(s.xyz[0], s.xyz[1], s.xyz[2]));
    m.aabb_center.emplace_back(Eigen::Vector3d::Zero());
    if(s.type == 0) {
      m.geometry.push_back(std::make_shared<coal::Box>(s.size[0], s.size[1], s.size[2]));
      m.half.emplace_back(s.size[0] / 2, s.size[1] / 2, s.size[2] / 2);
    } else if(s.type == 1) {
      m.geometry.push_back(std::make_shared<coal::Capsule>(s.radius, s.length));
      m.half.emplace_back(s.radius, s.radius, s.length / 2 + s.radius);
    } else if(s.type == 2) {
      m.geometry.push_back(std::make_shared<coal::Cylinder>(s.radius, s.length));
      m.half.emplace_back(s.radius, s.radius, s.length / 2);
    } else if(s.type == 3) {
      m.geometry.push_back(convexGeometry(s));
      auto &geometry = *m.geometry.back();
      geometry.computeLocalAABB();
      m.aabb_center.back() = (geometry.aabb_local.min_ + geometry.aabb_local.max_) / 2;
      m.half.push_back((geometry.aabb_local.max_ - geometry.aabb_local.min_) / 2);
    } else throw std::invalid_argument("Unknown collision primitive type");
    if(!m.half.back().allFinite() || !(m.half.back().array() > 0).all() ||
       ((s.type == 1 || s.type == 2) && (!std::isfinite(s.length) || s.length <= 0)))
      throw std::invalid_argument("Invalid primitive dimensions");
    double radius = s.type == 1 ? s.length / 2 + s.radius : m.half.back().norm();
    if(s.type == 3) {
      radius = 0;
      for(const auto &v : s.vertices)
        radius = std::max(radius, Eigen::Vector3d(v[0], v[1], v[2]).norm());
    }
    if(!std::isfinite(radius)) throw std::invalid_argument("Invalid collision bounding radius");
    m.bounding_radius.push_back(radius);
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
  linearizeImpl(q, velocity_indices, rows, bounds, closest, false);
}

bool StationCollisionModel::linearizeRetreat(const Eigen::VectorXd &q, const std::vector<int> &velocity_indices,
                                           Eigen::MatrixXd &rows, Eigen::VectorXd &bounds,
                                           ClearanceResult &closest) {
  return linearizeImpl(q, velocity_indices, rows, bounds, closest, true);
}

bool StationCollisionModel::linearizeImpl(const Eigen::VectorXd &q, const std::vector<int> &velocity_indices,
                                        Eigen::MatrixXd &rows, Eigen::VectorXd &bounds,
                                        ClearanceResult &closest, bool retreat) {
  auto &m = *impl_;
  for(int column : velocity_indices)
    if(column < 0 || column >= m.model.nv) throw std::invalid_argument("Invalid velocity index");
  m.place(q, false);
  narrow_calls_ = 0;
  contacts_.clear();
  closest = ClearanceResult{};
  coal::DistanceResult result;
  double band = detection_;
  if(retreat)
    for(double margin : margins_) band = std::max(band, margin + kMotionSlack);
  // Same candidates, in the same ascending order, as a flat scan of every pair.
  std::vector<int> candidates;
  for(const auto &g : m.group_pairs) {
    if(m.groupGap(g) > band) continue;
    for(int index : g.pairs)
      if(m.aabbGap(pairs_[static_cast<std::size_t>(index)][0], pairs_[static_cast<std::size_t>(index)][1]) <= band)
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
    if(d > band) continue;
    contacts_.push_back({static_cast<int>(index), d, result.nearest_points[0], result.nearest_points[1]});
  }
  linearized_q_ = q;
  rows.setZero(static_cast<Eigen::Index>(contacts_.size()), static_cast<Eigen::Index>(velocity_indices.size()));
  bounds.resize(static_cast<Eigen::Index>(contacts_.size()));
  if(contacts_.empty()) return true;
  // One Jacobian pass for every row; joint columns are world twists at the origin.
  pinocchio::computeJointJacobians(m.model, m.data, q);
  for(std::size_t r = 0; r < contacts_.size(); ++r) {
    const auto &c = contacts_[r];
    if(c.distance <= 1e-9 || !c.point1.allFinite() || !c.point2.allFinite() ||
       (c.point1 - c.point2).norm() <= 1e-9) {
      if(retreat) { rows.resize(0, rows.cols()); bounds.resize(0); return false; }
      throw std::runtime_error("Already touching/penetrating; reset to a checked configuration");
    }
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
  return true;
}

bool StationCollisionModel::verifyRetreat(const Eigen::VectorXd &q0, const Eigen::VectorXd &q1) {
  auto &m = *impl_;
  if(q0.size() != m.model.nq || q1.size() != m.model.nq || !q0.allFinite() || !q1.allFinite())
    throw std::invalid_argument("Invalid collision segment");
  if(linearized_q_.size() != q0.size() || (linearized_q_.array() != q0.array()).any())
    throw std::logic_error("verifyRetreat needs the configuration of the last linearize()");
  const Eigen::VectorXd dq = q1 - q0;
  if(!dq.allFinite()) throw std::invalid_argument("Nonfinite collision displacement");
  sampled_pairs_ = narrow_calls_ = 0;
  SegmentWork work;
  m.place(q0, false);
  struct RestorePlacement {
    Impl &model;
    const Eigen::VectorXd &q;
    ~RestorePlacement() { model.place(q, false); }
  } restore{m, q0};

  std::vector<double> start_distance(pairs_.size(), std::numeric_limits<double>::infinity());
  for(const auto &contact : contacts_) start_distance[contact.pair] = contact.distance;
  std::vector<int> deficient;
  coal::DistanceResult query;
  for(std::size_t i = 0; i < pairs_.size(); ++i) {
    if(++work.pair_visits > kMaxSegmentPairVisits) return false;
    double d = start_distance[i];
    if(!std::isfinite(d)) {
      const auto &p = pairs_[i];
      // This also handles a caller that used ordinary linearize with a
      // detection band smaller than a pair's margin.
      if(m.aabbGap(p[0], p[1]) > margins_[i] + kMotionSlack) continue;
      if(narrow_calls_ >= kMaxSegmentQueries) return false;
      d = m.measure(static_cast<int>(i), p, query);
      ++narrow_calls_;
    }
    if(d <= 1e-9) return false;
    if(d < margins_[i]) deficient.push_back(static_cast<int>(i));
  }
  if(dq.isZero(0)) return true;
  if(deficient.empty()) return verifySegment(q0, q1, 0.002, {}, work).safe;

  const auto motion = m.derivativeBounds(q0, q1);
  Eigen::VectorXd joint_step = Eigen::VectorXd::Zero(m.model.nv);
  for(int k : m.moving_shapes)
    for(const auto &j : m.motion_chains[k].links) joint_step[j.v_index] = dq[j.q_index];
  std::vector<Eigen::Vector3d> velocity(m.shapes.size(), Eigen::Vector3d::Zero());
  std::vector<Eigen::Vector3d> angular(m.shapes.size(), Eigen::Vector3d::Zero());
  struct Interval {
    double lo, hi;
    int depth;
    std::vector<int> pairs;
  };
  std::vector<Interval> pending{{0, 1, 0, deficient}};
  sampled_pairs_ = static_cast<int>(deficient.size());
  while(!pending.empty()) {
    Interval interval = std::move(pending.back());
    pending.pop_back();
    if(++work.intervals > kMaxSegmentIntervals) return false;
    const double half_width = (interval.hi - interval.lo) * 0.5;
    const double mid = interval.lo + half_width;
    const Eigen::VectorXd midpoint = q0 + mid * dq;
    m.place(midpoint, false);
    pinocchio::computeJointJacobians(m.model, m.data, midpoint);
    for(int k : m.moving_shapes) {
      Eigen::Matrix<double, 6, 1> twist = Eigen::Matrix<double, 6, 1>::Zero();
      for(int v : m.chain[k]) twist += m.data.J.col(v) * joint_step[v];
      angular[k] = twist.tail<3>();
      velocity[k] = twist.head<3>() + angular[k].cross(m.world[k].translation());
    }
    std::vector<int> unproven;
    for(int index : interval.pairs) {
      if(++work.pair_visits > kMaxSegmentPairVisits) return false;
      const auto &p = pairs_[index];
      const int a = p[0], b = p[1];
      const double speed = motion[a].speed + motion[b].speed;
      if(speed == 0) continue;  // Both shapes are exactly stationary.
      if(narrow_calls_ >= kMaxSegmentQueries) return false;
      const double distance = m.measure(index, p, query);
      ++narrow_calls_;
      if(distance <= 1e-9 || !query.nearest_points[0].allFinite() || !query.nearest_points[1].allFinite())
        return false;
      const Eigen::Vector3d difference = m.witness(a, query.nearest_points[0]) - m.witness(b, query.nearest_points[1]);
      const double upper_distance = difference.norm();
      if(!std::isfinite(upper_distance) || upper_distance <= 1e-9) return false;
      const Eigen::Vector3d normal = difference / upper_distance;
      const double arithmetic_scale = m.world[a].translation().norm() + m.world[b].translation().norm() +
                                      m.bounding_radius[a] + m.bounding_radius[b];
      const double distance_reserve = kMotionSlack + 128 * std::numeric_limits<double>::epsilon() * arithmetic_scale;
      const double gap = normal.dot(m.world[a].translation() - m.world[b].translation()) -
                         m.support(a, -m.world[a].rotation().transpose() * normal) -
                         m.support(b, m.world[b].rotation().transpose() * normal) - distance_reserve;
      const double travel = speed * half_width;
      const double lower_projection = gap - travel;
      bool certified = false;
      if(lower_projection > 0) {
        // The closest difference vector has projection >= lower_projection
        // and norm <= this transported material witness. This encloses ALL
        // closest normals, including a change of closest feature midinterval.
        const double cosine = std::min(1.0, lower_projection / (upper_distance + distance_reserve + travel));
        const double normal_radius = std::sqrt(2 * (1 - cosine));
        const double projected_rate = normal.dot(velocity[a] - velocity[b]) -
          m.bounding_radius[a] * normal.cross(angular[a]).norm() -
          m.bounding_radius[b] * normal.cross(angular[b]).norm();
        const double acceleration = motion[a].acceleration + motion[b].acceleration;
        const double rate_scale = speed + std::abs(projected_rate) + acceleration * half_width +
          velocity[a].norm() + velocity[b].norm() +
          arithmetic_scale * (angular[a].norm() + angular[b].norm());
        const double rate_reserve = 128 * std::numeric_limits<double>::epsilon() * rate_scale;
        const double lower_rate = projected_rate - acceleration * half_width - speed * normal_radius - rate_reserve;
        // No negative tolerance: repeatedly approving tiny losses is a drift
        // budget. An unresolved zero-rate motion is conservatively held.
        certified = std::isfinite(lower_rate) && lower_rate >= 0;
      }
      if(!certified) unproven.push_back(index);
    }
    if(unproven.empty()) continue;
    if(interval.depth >= kMaxSegmentDepth || mid == interval.lo || mid == interval.hi) return false;
    pending.push_back({mid, interval.hi, interval.depth + 1, unproven});
    pending.push_back({interval.lo, mid, interval.depth + 1, std::move(unproven)});
  }
  std::vector<bool> certified_retreat(pairs_.size(), false);
  for(int index : deficient) certified_retreat[index] = true;
  m.place(q0, false);
  // Only derivative-certified initial violations are exempt from the normal
  // margin predicate. All other pairs, including out-of-band ones, use it.
  return verifySegment(q0, q1, 0.002, certified_retreat, work).safe;
}

ClearanceResult StationCollisionModel::verifyStep(const Eigen::VectorXd &q0, const Eigen::VectorXd &q1,
                                                  double resolution) {
  sampled_pairs_ = 0;
  narrow_calls_ = 0;
  SegmentWork work;
  return verifySegment(q0, q1, resolution, {}, work);
}

ClearanceResult StationCollisionModel::verifySegment(const Eigen::VectorXd &q0, const Eigen::VectorXd &q1,
                                                     double resolution, const std::vector<bool> &certified_retreat,
                                                     SegmentWork &work) {
  auto &m = *impl_;
  if(q0.size() != m.model.nq || q1.size() != m.model.nq || !q0.allFinite() || !q1.allFinite() ||
     !std::isfinite(resolution) || !(resolution > 0))
    throw std::invalid_argument("Invalid collision segment");
  if(linearized_q_.size() != q0.size() || (linearized_q_.array() != q0.array()).any())
    throw std::logic_error("verifyStep needs the configuration of the last linearize()");
  const Eigen::VectorXd dq = q1 - q0;
  if(!dq.allFinite()) throw std::invalid_argument("Nonfinite collision displacement");
  if(pairs_.empty()) return {};
  if(dq.isZero(0)) return check(q0);
  const auto motion = m.motionBounds(q0, q1);
  // q(t) = q0 + t*dq. motion[k]*|t-s| bounds the displacement of EVERY
  // material point of shape k. Distance between two sets is 1-Lipschitz in
  // each shape's point displacement, so d(s) - pair_motion*|t-s| is a
  // clearance certificate over an interval, including between query points.
  std::vector<double> pair_motion(pairs_.size()), start_distance(pairs_.size(), detection_);
  for(const auto &c : contacts_) {
    if(!certified_retreat.empty() && certified_retreat[c.pair]) continue;
    start_distance[c.pair] = c.distance;
    const double clearance = c.distance - margins_[c.pair];
    if(clearance < -1e-9) return {false, c.distance, c.pair, clearance};
  }
  ClearanceResult worst;
  auto record = [&](int index, double clearance) {
    if(clearance < worst.clearance)
      worst = {true, clearance + margins_[index], index, clearance};
  };
  std::vector<int> candidates;
  double widest_margin = 0;
  for(std::size_t i = 0; i < pairs_.size(); ++i) {
    if(!certified_retreat.empty() && certified_retreat[i]) continue;
    pair_motion[i] = motion[pairs_[i][0]] + motion[pairs_[i][1]];
    if(!std::isfinite(pair_motion[i])) throw std::invalid_argument("Nonfinite collision pair motion");
    widest_margin = std::max(widest_margin, margins_[i]);
    // linearize() measured all pairs in the band; every other pair has
    // distance >= detection_, even when broadphase omitted it entirely.
    const double clearance = start_distance[i] - margins_[i] - pair_motion[i] - kMotionSlack;
    if(clearance >= 0) record(static_cast<int>(i), clearance);
    else candidates.push_back(static_cast<int>(i));
  }
  if(candidates.empty()) return worst;  // common small-step path: no FK or distance queries
  sampled_pairs_ += static_cast<int>(candidates.size());
  // Reserve the QP buffer and numerical slack, then spend only half of the
  // remaining detection band per substep. If there is no usable band, the
  // exhaustive clearance certificate below remains valid without this hint.
  const double spatial_budget = 0.5 * (detection_ - widest_margin - buffer_ - kMotionSlack);
  struct Interval {
    double lo, hi;
    int depth;
    std::vector<int> pairs;
  };
  std::vector<Interval> pending;
  pending.push_back({0, 1, 0, std::move(candidates)});
  coal::DistanceResult result;
  auto finish = [&](ClearanceResult answer) {
    m.place(q0, false);
    return answer;
  };
  auto unresolved = [&](int index, double lower) {
    // Negative clearance is a conservative lower bound, NOT an assertion of
    // measured penetration. The caller must shrink/hold an uncertified step.
    const double clearance = std::min(lower, -kMotionSlack);
    return finish({false, margins_[index] + clearance, index, clearance});
  };
  while(!pending.empty()) {
    Interval interval = std::move(pending.back());
    pending.pop_back();
    if(++work.intervals > kMaxSegmentIntervals) return unresolved(interval.pairs.front(), -kMotionSlack);
    const double width = interval.hi - interval.lo;
    const double mid = interval.lo + 0.5 * width;
    m.place(q0 + mid * dq, false);
    std::vector<int> unproven;
    double lowest = 0;
    for(int index : interval.pairs) {
      if(++work.pair_visits > kMaxSegmentPairVisits) return unresolved(index, -kMotionSlack);
      const auto &p = pairs_[index];
      const double radius = 0.5 * width * pair_motion[index];
      double lower = m.aabbGap(p[0], p[1]) - margins_[index] - radius - kMotionSlack;
      if(lower < 0 && (spatial_budget <= 0 || width * pair_motion[index] <= spatial_budget)) {
        if(narrow_calls_ >= kMaxSegmentQueries) return unresolved(index, lower);
        const double d = m.measure(index, p, result);
        ++narrow_calls_;
        const double clearance = d - margins_[index];
        if(clearance < -1e-9) return finish({false, d, index, clearance});
        lower = clearance - radius - kMotionSlack;
      }
      if(lower >= 0) record(index, lower);
      else {
        unproven.push_back(index);
        lowest = std::min(lowest, lower);
      }
    }
    if(unproven.empty()) continue;
    // resolution is retained only for API compatibility; reaching a sample
    // spacing NEVER proves safety. Only certified intervals leave this stack.
    if(interval.depth >= kMaxSegmentDepth || mid == interval.lo || mid == interval.hi)
      return unresolved(unproven.front(), lowest);
    pending.push_back({mid, interval.hi, interval.depth + 1, unproven});
    pending.push_back({interval.lo, mid, interval.depth + 1, std::move(unproven)});
  }
  return finish(worst);
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
