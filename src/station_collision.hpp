#pragma once
#include <Eigen/Core>
#include <array>
#include <memory>
#include <string>
#include <vector>

namespace libpyelite {

// One primitive of the collision profile, attached to a named URDF frame.
// Dimensions are full lengths (box sides, capsule/cylinder axial length).
struct CollisionShape {
  std::string frame;
  int type{0};  // 0 box, 1 capsule, 2 cylinder, 3 convex polytope
  std::array<double, 3> size{};
  double radius{0}, length{0};
  std::array<double, 3> xyz{}, rpy{};
  // Local coordinates and outward-facing triangles of a closed convex exterior.
  std::vector<std::array<double, 3>> vertices;
  std::vector<std::array<int, 3>> faces;
};

struct CollisionContact {
  int pair{-1};
  double distance{0};
  Eigen::Vector3d point1{Eigen::Vector3d::Zero()}, point2{Eigen::Vector3d::Zero()};
};

struct ClearanceResult {
  bool safe{true};
  double minimum_distance{std::numeric_limits<double>::infinity()};
  int pair{-1};
  double clearance{std::numeric_limits<double>::infinity()};
};

// Native station collision layer: the same primitives, pairs, margins and
// barrier as libpyelite.collision, computed once per control step in C++.
//
// Following what Pinocchio/Coal, OCS2, Tesseract and Mink do:
// - a flat precomputed pair list, each pair with its own cached Coal
//   ComputeDistance functor and a GJK warm start from the previous step;
// - a two-level broad phase on world AABBs: per group pair (a moving frame's
//   shapes together, each static shape alone), then per shape pair only where
//   the group pair passed. The static environment's are computed once, only
//   the moving shapes' are refreshed per configuration;
// - one linearisation per step (distances + barrier rows) shared by every QP
//   pass of that step;
// - a conservative motion bound instead of sampling every step's joint segment.
class StationCollisionModel {
 public:
  StationCollisionModel(const std::string &urdf, std::vector<CollisionShape> shapes,
                        std::vector<std::array<int, 2>> pairs, std::vector<double> margins,
                        double detection_distance, double gain, double qp_buffer);
  ~StationCollisionModel();
  StationCollisionModel(const StationCollisionModel &) = delete;
  StationCollisionModel &operator=(const StationCollisionModel &) = delete;

  int nq() const;
  int pairCount() const { return static_cast<int>(pairs_.size()); }

  // Smallest clearance (distance - margin) over all pairs; with a finite
  // cutoff only pairs whose AABBs are within it are measured.
  ClearanceResult check(const Eigen::VectorXd &q, double cutoff = -1);

  // Barrier rows at q for the given velocity columns:
  //   -n^T (J_a - J_b) dq <= gain * max(0, d - margin - qp_buffer)
  // for every pair within the detection distance. Contacts are kept for verifyStep.
  void linearize(const Eigen::VectorXd &q, const std::vector<int> &velocity_indices,
                 Eigen::MatrixXd &rows, Eigen::VectorXd &bounds, ClearanceResult &closest);
  const std::vector<CollisionContact> &contacts() const { return contacts_; }

  // Certify the complete linear joint segment q0 -> q1, starting at exactly
  // the last linearize() configuration. Scalar revolute/prismatic chains use
  // configuration-indexed serial-chain reach bounds valid over the full path.
  // Clearance minus pair motion certifies intervals; unresolved intervals are
  // subdivided, including out-of-band pairs. A work/depth limit returns unsafe,
  // never endpoint-only approval. Returned distances may be conservative lower
  // bounds; unsafe can mean uncertified rather than measured penetration.
  // resolution remains accepted (positive, finite) for API compatibility;
  // a sampling spacing is never a safety certificate.
  ClearanceResult verifyStep(const Eigen::VectorXd &q0, const Eigen::VectorXd &q1, double resolution = 0.002);

  // World placements (row-major 4x4) of every shape, for viewers.
  std::vector<std::array<double, 16>> placements(const Eigen::VectorXd &q);

  // Diagnostics of the last call: narrow-phase queries and distinct pairs
  // requiring the interval-verification fallback.
  int lastNarrowPhaseCalls() const { return narrow_calls_; }
  int lastSampledPairs() const { return sampled_pairs_; }

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  std::vector<std::array<int, 2>> pairs_;
  std::vector<double> margins_;
  double detection_, gain_, buffer_;
  std::vector<CollisionContact> contacts_;
  Eigen::VectorXd linearized_q_;
  int narrow_calls_{0}, sampled_pairs_{0};
};
}  // namespace libpyelite
