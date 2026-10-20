#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

namespace libpyelite {

// A reference-seeded, joint-space configuration envelope for a recognized CS66
// chain. This preserves three local singular-surface signs and joint winding;
// it does NOT assert global uniqueness of an inverse-kinematics solution.
//
// Joint numbering in the formulas below is one-based. For the supplied URDFs:
//   E = sin(q3), W = sin(q5),
//   S = -L2*cos(q2) - L3*cos(q2+q3) + h*sin(q2+q3+q4).
// With shoulder twist alpha, wrist twist beta, and last twist -beta:
//   h = d5 - d6*cos(beta),
//   det(J_LWA) = L2*L3*sin(alpha)*sin(beta)^2 * E*W*S.
// Canonical and diagonal-07/08 control chains share these coordinates. Their
// proper fixed base/tool transforms do not change the surfaces. In particular,
// do not negate right-arm joint values to account for a mirrored mount.
//
// The caller MUST recognize the loaded model's topology, local joint axes,
// signs, offsets and twists before setting Geometry::recognized. Merely having
// six revolute joints, matching names, or a CS66 filename is insufficient.
class ConfigurationPolicy {
 public:
  using Vector = std::array<double, 6>;
  static constexpr double kPi = 3.141592653589793238462643383279502884;
  static constexpr double kDefaultAngularMargin = kPi / 60.0;
  static constexpr double kDefaultWindingHalfSpan = kPi - 1.0e-3;
  static constexpr double kDefaultShoulderMargin = 1.0e-4;  // metres

  struct Geometry {
    bool recognized{false};
    double upper_arm{0.427};
    double forearm{0.3905};
    // Effective d5-d6*cos(beta), not just the joint-5 translation magnitude.
    // The source URDF uses beta=1.570796326589793, slightly short of pi/2.
    double wrist_offset{0.09649999998051517};
  };

  ConfigurationPolicy() noexcept = default;

  // Invalid options, unrecognized geometry, nonfinite/inverted physical limits,
  // or a reference outside the permitted nonsingular envelope leave the policy
  // uninitialized. All predicates then fail closed. A new policy should be
  // created only for a deliberate new session, not after each motion command.
  ConfigurationPolicy(
      const Vector &reference, const Vector &physical_lower,
      const Vector &physical_upper, Geometry geometry,
      double angular_margin = kDefaultAngularMargin,
      double winding_half_span = kDefaultWindingHalfSpan,
      double shoulder_margin = kDefaultShoulderMargin) noexcept
      : reference_(reference),
        lower_(physical_lower),
        upper_(physical_upper),
        geometry_(geometry),
        angular_margin_(angular_margin),
        shoulder_margin_(shoulder_margin) {
    if (!geometry_.recognized || !finite(geometry_.upper_arm) ||
        !finite(geometry_.forearm) || !finite(geometry_.wrist_offset) ||
        geometry_.upper_arm <= 0.0 || geometry_.forearm <= 0.0 ||
        !finite(angular_margin) || angular_margin <= kAngleSlack ||
        angular_margin >= kPi / 2.0 || !finite(winding_half_span) ||
        winding_half_span <= kAngleSlack || winding_half_span >= kPi ||
        !finite(shoulder_margin) || shoulder_margin <= 0.0) {
      return;
    }
    for (std::size_t i = 0; i < reference.size(); ++i) {
      if (!finite(reference[i]) || !finite(physical_lower[i]) ||
          !finite(physical_upper[i]) ||
          physical_lower[i] >= physical_upper[i] ||
          reference[i] < physical_lower[i] ||
          reference[i] > physical_upper[i]) {
        return;
      }
      // Each joint gets its OWN reference-seeded winding interval.
      lower_[i] = std::max(physical_lower[i], reference[i] - winding_half_span);
      upper_[i] = std::min(physical_upper[i], reference[i] + winding_half_span);
      if (!finite(lower_[i]) || !finite(upper_[i]) || lower_[i] >= upper_[i]) {
        return;
      }
    }
    // Intersect elbow and wrist with the particular open half-turn containing
    // the reference, inset by the margin. Endpoint membership consequently
    // certifies these factors throughout any segment or rectangle.
    for (const std::size_t i : {std::size_t{2}, std::size_t{4}}) {
      const double half_turn = std::floor(reference[i] / kPi);
      lower_[i] = std::max(lower_[i], half_turn * kPi + angular_margin);
      upper_[i] = std::min(upper_[i], (half_turn + 1.0) * kPi - angular_margin);
      if (!finite(lower_[i]) || !finite(upper_[i]) || lower_[i] >= upper_[i]) {
        return;
      }
    }
    elbow_sign_ = sign(std::sin(reference[2]));
    wrist_sign_ = sign(std::sin(reference[4]));
    const double shoulder = shoulderMeasure(reference);
    if (!finite(shoulder)) {
      return;
    }
    shoulder_sign_ = sign(shoulder);
    initialized_ = true;
    initialized_ = valid(reference);
  }

  bool initialized() const noexcept { return initialized_; }
  const Vector &reference() const noexcept { return reference_; }
  const Vector &lower() const noexcept { return lower_; }
  const Vector &upper() const noexcept { return upper_; }
  const Geometry &geometry() const noexcept { return geometry_; }
  int shoulderSign() const noexcept { return shoulder_sign_; }
  int elbowSign() const noexcept { return elbow_sign_; }
  int wristSign() const noexcept { return wrist_sign_; }
  double angularMargin() const noexcept { return angular_margin_; }
  double shoulderMargin() const noexcept { return shoulder_margin_; }

  double shoulderMeasure(const Vector &q) const noexcept {
    return -geometry_.upper_arm * std::cos(q[1]) -
           geometry_.forearm * std::cos(q[1] + q[2]) +
           geometry_.wrist_offset * std::sin(q[1] + q[2] + q[3]);
  }

  Vector shoulderGradient(const Vector &q) const noexcept {
    const double wrist = geometry_.wrist_offset *
                         std::cos(q[1] + q[2] + q[3]);
    const double forearm = geometry_.forearm * std::sin(q[1] + q[2]);
    return {{0.0, geometry_.upper_arm * std::sin(q[1]) + forearm + wrist,
             forearm + wrist, wrist, 0.0, 0.0}};
  }

  bool valid(const Vector &q) const noexcept {
    if (!initialized_ || !insideBounds(q)) {
      return false;
    }
    // Explicit signs guard against roundoff in half-turn index construction.
    // A tiny angular-boundary slack permits an exactly represented 3-degree
    // boundary without allowing a zero or opposite sign.
    if (elbow_sign_ * std::sin(q[2]) <= 0.0 ||
        wrist_sign_ * std::sin(q[4]) <= 0.0) {
      return false;
    }
    return shoulder_sign_ * shoulderMeasure(q) >=
           shoulder_margin_ + shoulderRoundoff(q);
  }

  // Certifies EVERY point in an axis-aligned joint rectangle. This is useful
  // for a stopping trajectory's per-joint extrema: different extrema need not
  // occur at the same time. A failed certificate is conservative and does not
  // imply that the actual trajectory crosses a singular surface.
  bool boxValid(const Vector &lo, const Vector &hi) const noexcept {
    if (!initialized_ || !insideBounds(lo) || !insideBounds(hi)) {
      return false;
    }
    for (std::size_t i = 0; i < lo.size(); ++i) {
      if (lo[i] > hi[i]) {
        return false;
      }
    }
    Vector middle{};
    Vector radius{};
    Vector magnitude{};
    for (std::size_t i = 0; i < lo.size(); ++i) {
      middle[i] = lo[i] + (hi[i] - lo[i]) / 2.0;
      radius[i] = (hi[i] - lo[i]) / 2.0;
      magnitude[i] = std::max(std::abs(lo[i]), std::abs(hi[i]));
    }
    const double roundoff = shoulderRoundoff(magnitude);
    const Interval a = trigRange(lo[1], hi[1], false);
    const Interval b = trigRange(lo[1] + lo[2], hi[1] + hi[2], false);
    const Interval c = trigRange(lo[1] + lo[2] + lo[3],
                                hi[1] + hi[2] + hi[3], true);
    const double natural_lower =
        productLower(-shoulder_sign_ * geometry_.upper_arm, a) +
        productLower(-shoulder_sign_ * geometry_.forearm, b) +
        productLower(shoulder_sign_ * geometry_.wrist_offset, c);
    if (natural_lower >= shoulder_margin_ + roundoff) {
      return true;
    }

    // A second, independent Taylor enclosure retains cancellation between the
    // linear derivatives of the three terms. |sin''|, |cos''| <= 1 bounds the
    // remainder for every displacement in the rectangle.
    const Vector gradient = shoulderGradient(middle);
    const double b_radius = radius[1] + radius[2];
    const double c_radius = b_radius + radius[3];
    const double remainder =
        0.5 * (geometry_.upper_arm * radius[1] * radius[1] +
               geometry_.forearm * b_radius * b_radius +
               std::abs(geometry_.wrist_offset) * c_radius * c_radius);
    const double linear_radius = std::abs(gradient[1]) * radius[1] +
                                 std::abs(gradient[2]) * radius[2] +
                                 std::abs(gradient[3]) * radius[3];
    return shoulder_sign_ * shoulderMeasure(middle) - linear_radius -
               remainder >=
           shoulder_margin_ + roundoff;
  }

  // Certifies the ENTIRE straight joint-space segment. Equal endpoint signs
  // alone are insufficient: the shoulder factor can cross zero twice.
  // Adaptive midpoint certificates use an analytic Lipschitz bound, not a
  // sampling assumption. Depth is capped at 10; an unresolved leaf rejects.
  bool segmentValid(const Vector &from, const Vector &to) const noexcept {
    if (!valid(from) || !valid(to)) {
      return false;
    }
    Vector lo{};
    Vector hi{};
    for (std::size_t i = 0; i < lo.size(); ++i) {
      lo[i] = std::min(from[i], to[i]);
      hi[i] = std::max(from[i], to[i]);
    }
    if (boxValid(lo, hi)) {
      return true;
    }
    return segmentCertificate(from, to, 10);
  }

 private:
  struct Interval {
    double lo;
    double hi;
  };
  static constexpr double kAngleSlack = 1.0e-12;

  static bool finite(double value) noexcept { return std::isfinite(value); }
  static int sign(double value) noexcept { return value < 0.0 ? -1 : 1; }

  bool insideBounds(const Vector &q) const noexcept {
    for (std::size_t i = 0; i < q.size(); ++i) {
      if (!finite(q[i]) || q[i] < lower_[i] - kAngleSlack ||
          q[i] > upper_[i] + kAngleSlack) {
        return false;
      }
    }
    return true;
  }

  double shoulderRoundoff(const Vector &q) const noexcept {
    // Deliberately larger than floating-point roundoff for the supplied radian
    // ranges; also accounts for angle-sum and interval-endpoint arithmetic.
    return 1.0e-12 *
           (1.0 + geometry_.upper_arm + geometry_.forearm +
            std::abs(geometry_.wrist_offset)) *
           (1.0 + std::abs(q[1]) + std::abs(q[2]) + std::abs(q[3]));
  }

  static bool containsPhase(double lo, double hi, double phase) noexcept {
    const double period = 2.0 * kPi;
    const double first = std::ceil((lo - phase) / period);
    return first <= (hi - phase) / period;
  }

  static Interval trigRange(double lo, double hi, bool sine) noexcept {
    const double infinity = std::numeric_limits<double>::infinity();
    lo = std::nextafter(lo, -infinity);
    hi = std::nextafter(hi, infinity);
    if (!finite(lo) || !finite(hi) || hi - lo >= 2.0 * kPi) {
      return {-1.0, 1.0};
    }
    const double first = sine ? std::sin(lo) : std::cos(lo);
    const double last = sine ? std::sin(hi) : std::cos(hi);
    Interval result{std::min(first, last), std::max(first, last)};
    if (containsPhase(lo, hi, sine ? kPi / 2.0 : 0.0)) {
      result.hi = 1.0;
    }
    if (containsPhase(lo, hi, sine ? -kPi / 2.0 : kPi)) {
      result.lo = -1.0;
    }
    return result;
  }

  static double productLower(double coefficient,
                             const Interval &range) noexcept {
    return coefficient >= 0.0 ? coefficient * range.lo
                              : coefficient * range.hi;
  }

  bool segmentCertificate(const Vector &from, const Vector &to,
                          unsigned depth) const noexcept {
    Vector middle{};
    Vector delta{};
    for (std::size_t i = 0; i < from.size(); ++i) {
      delta[i] = to[i] - from[i];
      middle[i] = from[i] + delta[i] / 2.0;
    }
    const double signed_middle = shoulder_sign_ * shoulderMeasure(middle);
    const double roundoff = std::max(shoulderRoundoff(from),
                                    shoulderRoundoff(to));
    if (signed_middle < shoulder_margin_ + roundoff) {
      return false;
    }
    // For q(t)=from+t*delta, |dS/dt| never exceeds this expression.
    const double lipschitz =
        geometry_.upper_arm * std::abs(delta[1]) +
        geometry_.forearm * std::abs(delta[1] + delta[2]) +
        std::abs(geometry_.wrist_offset) *
            std::abs(delta[1] + delta[2] + delta[3]);
    if (signed_middle - lipschitz / 2.0 >= shoulder_margin_ + roundoff) {
      return true;
    }
    if (depth == 0) {
      return false;
    }
    return segmentCertificate(from, middle, depth - 1) &&
           segmentCertificate(middle, to, depth - 1);
  }

  Vector reference_{};
  Vector lower_{};
  Vector upper_{};
  Geometry geometry_{};
  double angular_margin_{kDefaultAngularMargin};
  double shoulder_margin_{kDefaultShoulderMargin};
  int shoulder_sign_{0};
  int elbow_sign_{0};
  int wrist_sign_{0};
  bool initialized_{false};
};

}  // namespace libpyelite
