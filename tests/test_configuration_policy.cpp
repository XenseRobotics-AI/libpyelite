// Standalone, hardware-free regression:
// Compile this file with c++ -std=c++17 -O2 -Wall -Wextra -pedantic;
// no native library, Pinocchio, or vendor SDK linkage is needed.
#include "../src/configuration_policy.hpp"

#include <cstdlib>
#include <iostream>
#include <limits>
#include <random>

namespace {
using Policy = libpyelite::ConfigurationPolicy;
using Vector = Policy::Vector;
constexpr double pi = Policy::kPi;
const Vector physical_lo{{-2 * pi, -2 * pi, -pi, -2 * pi, -2 * pi, -2 * pi}};
const Vector physical_hi{{2 * pi, 2 * pi, pi, 2 * pi, 2 * pi, 2 * pi}};
const Vector left{{-1.3963, -0.5236, 1.5708, -0.5236, 1.5708, -0.7854}};
const Vector right{{-1.7453, -2.618, -1.5708, -2.618, -1.5708, -0.7854}};

void require(bool condition, const char *message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
  }
}

Policy make(const Vector &reference = left) {
  return Policy(reference, physical_lo, physical_hi, {true});
}

Vector interpolate(const Vector &a, const Vector &b, double t) {
  Vector q{};
  for (std::size_t i = 0; i < q.size(); ++i) {
    q[i] = a[i] + (b[i] - a[i]) * t;
  }
  return q;
}

void testReferenceAndMirroring() {
  const Policy l = make();
  const Policy r = make(right);
  require(l.initialized() && r.initialized(), "station defaults initialize");
  require(l.valid(left) && r.valid(right), "reference accepted");
  require(l.shoulderSign() == -1 && l.elbowSign() == 1 && l.wristSign() == 1,
          "left arm signs");
  require(r.shoulderSign() == 1 && r.elbowSign() == -1 && r.wristSign() == -1,
          "right arm signs derived from its own reference");
  require(std::abs(l.shoulderMeasure(left) + 0.5167916555465952) < 1e-10,
          "left shoulder factor");
  require(std::abs(r.shoulderMeasure(right) - 0.5167895116650321) < 1e-10,
          "right shoulder factor");
  require(!l.valid(right) && !r.valid(left), "opposite branches rejected");
  require(l.boxValid(left, left) && l.segmentValid(left, left),
          "zero-size certificates");

  for (std::size_t i : {std::size_t{0}, std::size_t{1}, std::size_t{3},
                        std::size_t{5}}) {
    require(std::abs(l.lower()[i] -
                     std::max(physical_lo[i], left[i] -
                              Policy::kDefaultWindingHalfSpan)) < 1e-14,
            "independent lower winding interval");
    require(std::abs(l.upper()[i] -
                     std::min(physical_hi[i], left[i] +
                              Policy::kDefaultWindingHalfSpan)) < 1e-14,
            "independent upper winding interval");
  }
  require(std::abs(l.lower()[2] - pi / 60) < 1e-14 &&
              std::abs(l.upper()[2] - (pi - pi / 60)) < 1e-14,
          "elbow interval inset by three degrees");
  require(std::abs(r.lower()[4] - (-pi + pi / 60)) < 1e-14 &&
              std::abs(r.upper()[4] + pi / 60) < 1e-14,
          "negative wrist branch interval");
  Vector q = left;
  q[5] += Policy::kDefaultWindingHalfSpan;
  require(l.valid(q), "winding boundary");
  q[5] += 1e-7;
  require(!l.valid(q), "winding boundary violation");
  q = left;
  q[2] = pi / 60;
  require(l.valid(q) && make(q).initialized(), "exact angular margin");
  q[2] -= 1e-8;
  require(!l.valid(q) && !make(q).initialized(), "inside elbow margin");
  q = left;
  q[4] = pi - pi / 60;
  require(l.valid(q), "upper angular margin");
  q[4] += 1e-8;
  require(!l.valid(q), "inside wrist margin");
}

void testInvalidInputs() {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  require(!Policy{}.valid(left), "default invalid");
  require(!Policy(left, physical_lo, physical_hi, {}).initialized(),
          "unrecognized geometry fails closed");
  require(!make(Vector{}).initialized(), "singular zero initialization");
  for (std::size_t joint : {std::size_t{2}, std::size_t{4}}) {
    for (double angle : {0.0, pi, -pi, pi / 180, -pi / 180}) {
      Vector q = left;
      q[joint] = angle;
      require(!make(q).initialized(), "singular or near-singular reference");
    }
  }
  // q2=pi/2, q3=pi/2 gives L3 + h*sin(pi+q4); use a different
  // nonsingular elbow and solve A*cos(q2)+B*sin(q2)=0 analytically.
  Vector shoulder_singular = left;
  const double q3 = shoulder_singular[2];
  const double q4 = shoulder_singular[3];
  const double h = Policy::Geometry{}.wrist_offset;
  const double a = -0.427 - 0.3905 * std::cos(q3) + h * std::sin(q3 + q4);
  const double b = 0.3905 * std::sin(q3) + h * std::cos(q3 + q4);
  shoulder_singular[1] = std::atan2(-a, b);
  require(std::abs(make().shoulderMeasure(shoulder_singular)) < 1e-15,
          "constructed shoulder zero");
  require(!make(shoulder_singular).initialized(), "shoulder zero rejected");

  for (std::size_t i = 0; i < left.size(); ++i) {
    for (double invalid : {nan, inf, -inf}) {
      Vector q = left;
      q[i] = invalid;
      require(!make(q).initialized() && !make().valid(q),
              "nonfinite reference/candidate");
      Vector lo = physical_lo;
      Vector hi = physical_hi;
      lo[i] = invalid;
      require(!Policy(left, lo, hi, {true}).initialized(),
              "nonfinite physical lower");
      lo = physical_lo;
      hi[i] = invalid;
      require(!Policy(left, lo, hi, {true}).initialized(),
              "nonfinite physical upper");
    }
    Vector lo = physical_lo;
    Vector hi = physical_hi;
    lo[i] = hi[i];
    require(!Policy(left, lo, hi, {true}).initialized(), "equal physical bounds");
    lo[i] = hi[i] + 1;
    require(!Policy(left, lo, hi, {true}).initialized(), "inverted bounds");
    Vector q = left;
    q[i] = physical_hi[i] + 0.01;
    require(!make(q).initialized(), "reference outside physical bounds");
  }
  for (double margin : {nan, inf, -1.0, 0.0, pi / 2, pi}) {
    require(!Policy(left, physical_lo, physical_hi, {true}, margin).initialized(),
            "invalid angular margin");
  }
  for (double span : {nan, inf, -1.0, 0.0, pi, 2 * pi}) {
    require(!Policy(left, physical_lo, physical_hi, {true}, pi / 60, span)
                 .initialized(), "invalid winding span");
  }
  for (double margin : {nan, inf, -1.0, 0.0}) {
    require(!Policy(left, physical_lo, physical_hi, {true}, pi / 60,
                    pi - .001, margin).initialized(), "invalid shoulder margin");
  }
  for (double length : {nan, inf, -1.0, 0.0}) {
    Policy::Geometry geometry{true};
    geometry.upper_arm = length;
    require(!Policy(left, physical_lo, physical_hi, geometry).initialized(),
            "invalid geometry");
  }
}

void testHiddenShoulderCrossing() {
  const Policy policy = make();
  const Vector a{{0.7378438407785932, 2.403556919565743, 3.085198099054266,
                  -1.0107926238345524, 2.774264110583451, -0.1545697342876724}};
  const Vector b{{-2.226991431697133, -1.5743014371319184, 2.8751711726944773,
                  -1.4193042790761545, 0.5580905279171627, -3.630936681462359}};
  require(policy.valid(a) && policy.valid(b), "both branch endpoints valid");
  require(!policy.valid(interpolate(a, b, 0.5)), "interior leaves shoulder branch");
  require(!policy.segmentValid(a, b), "double shoulder crossing rejected");
  Vector lo{};
  Vector hi{};
  for (std::size_t i = 0; i < lo.size(); ++i) {
    lo[i] = std::min(a[i], b[i]);
    hi[i] = std::max(a[i], b[i]);
  }
  require(!policy.boxValid(lo, hi), "unsafe braking rectangle rejected");
  require(!policy.boxValid(hi, lo), "inverted rectangle rejected");
}

void testGradientAndCertificates() {
  const Policy policy = make();
  const Vector gradient = policy.shoulderGradient(left);
  for (std::size_t i = 0; i < left.size(); ++i) {
    Vector minus = left;
    Vector plus = left;
    minus[i] -= 1e-6;
    plus[i] += 1e-6;
    const double numerical =
        (policy.shoulderMeasure(plus) - policy.shoulderMeasure(minus)) / 2e-6;
    require(std::abs(gradient[i] - numerical) < 1e-9, "shoulder gradient");
  }
  std::mt19937_64 rng(93271);
  std::uniform_real_distribution<double> uniform(0.0, 1.0);
  unsigned boxes = 0;
  unsigned segments = 0;
  for (unsigned trial = 0; trial < 5000; ++trial) {
    Vector a{};
    Vector b{};
    Vector lo{};
    Vector hi{};
    for (std::size_t i = 0; i < a.size(); ++i) {
      const double range = policy.upper()[i] - policy.lower()[i];
      a[i] = policy.lower()[i] + range * uniform(rng);
      b[i] = trial % 2 ? policy.lower()[i] + range * uniform(rng)
                       : std::clamp(a[i] + .05 * (uniform(rng) - .5),
                                    policy.lower()[i], policy.upper()[i]);
      lo[i] = std::min(a[i], b[i]);
      hi[i] = std::max(a[i], b[i]);
    }
    if (policy.boxValid(lo, hi)) {
      ++boxes;
      for (unsigned sample = 0; sample < 40; ++sample) {
        Vector q{};
        for (std::size_t i = 0; i < q.size(); ++i) {
          q[i] = lo[i] + (hi[i] - lo[i]) * uniform(rng);
        }
        require(policy.valid(q), "accepted rectangle contains invalid point");
      }
      for (unsigned corner = 0; corner < 8; ++corner) {
        Vector q = lo;
        for (unsigned i = 1; i <= 3; ++i) {
          q[i] = (corner & (1u << (i - 1))) ? hi[i] : lo[i];
        }
        require(policy.valid(q), "accepted rectangle contains invalid corner");
      }
    }
    if (policy.segmentValid(a, b)) {
      ++segments;
      for (unsigned sample = 0; sample <= 200; ++sample) {
        require(policy.valid(interpolate(a, b, sample / 200.0)),
                "accepted segment contains invalid point");
      }
    }
  }
  require(boxes > 500 && segments > 500, "certificates accept useful motions");
  std::cout << "Certified randomized boxes: " << boxes
            << "; segments: " << segments << '\n';
}
}  // namespace

int main() {
  testReferenceAndMirroring();
  testInvalidInputs();
  testHiddenShoulderCrossing();
  testGradientAndCertificates();
  std::cout << "ConfigurationPolicy regressions passed\n";
}
