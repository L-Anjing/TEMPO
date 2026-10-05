#pragma once

#include <data_structure/base/trajectory.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace minco_planner {
struct DynamicsLimits {
  double max_vel{10.0}, max_acc{3.0}, max_jerk{10.0};
  // 规划器实际使用的巡航速度上限。max_vel 是硬件/校验硬上限，planning_vel 是
  // MINCO 与 A* 种子时间分配用的速度；两者分开，避免用软件上限生成短时贴边轨迹。
  double planning_vel{2.0};
  double optimizer_margin{0.85}, validation_tolerance{0.05};
  void validate() const {
    if (!std::isfinite(max_vel) || !std::isfinite(max_acc) || !std::isfinite(max_jerk) ||
        max_vel <= 0 || max_acc <= 0 || max_jerk <= 0 ||
        !std::isfinite(planning_vel) || planning_vel <= 0 ||
        !std::isfinite(optimizer_margin) || optimizer_margin <= 0 || optimizer_margin >= 1 ||
        !std::isfinite(validation_tolerance) || validation_tolerance < 0 || validation_tolerance > .2) {
      throw std::invalid_argument("Invalid dynamics limits, optimizer margin or validation tolerance");
    }
  }
};
struct DynamicsExtrema {
  double velocity{0}, acceleration{0}, jerk{0};
  bool finite{true};
};

// Derivative norms are planar for an omnidirectional chassis. Isolate every
// stationary point of the squared polynomial norm, including piece boundaries.
inline double maxPlanarDerivative(const geometry_utils::Piece & piece, int derivative) {
  const auto & coefficients = piece.getCoeffMat();
  const int degree = coefficients.cols() - 1 - derivative;
  if (degree < 0) { return 0; }
  Eigen::MatrixXd c = Eigen::MatrixXd::Zero(2, degree + 1);
  for (int k = 0; k <= degree; ++k) {
    const int power = degree + derivative - k;
    double factor = 1;
    for (int j = 0; j < derivative; ++j) { factor *= power - j; }
    c.col(k) = coefficients.col(k).head<2>() * factor * std::pow(piece.getDuration(), degree - k);
  }
  Eigen::VectorXd squared = Eigen::VectorXd::Zero(2 * degree + 1);
  for (int i = 0; i <= degree; ++i) for (int j = 0; j <= degree; ++j) {
    squared(i + j) += c.col(i).dot(c.col(j));
  }
  std::set<double> times{0.0, 1.0};
  if (degree > 0) {
    Eigen::VectorXd stationary(2 * degree);
    for (int i = 0; i < stationary.size(); ++i) { stationary(i) = squared(i) * (2 * degree - i); }
    if (stationary.cwiseAbs().maxCoeff() > 1e-14) {
      const auto roots = math_utils::RootFinder::solvePolynomial(stationary, 0.0, 1.0, 1e-9);
      times.insert(roots.begin(), roots.end());
    }
  }
  double maximum = 0;
  for (double time : times) {
    Eigen::Vector2d value = Eigen::Vector2d::Zero();
    for (int k = 0; k <= degree; ++k) { value = value * time + c.col(k); }
    if (!value.allFinite()) { return std::numeric_limits<double>::quiet_NaN(); }
    maximum = std::max(maximum, value.norm());
  }
  return maximum;
}
inline DynamicsExtrema evaluateDynamics(const geometry_utils::Trajectory & trajectory) {
  DynamicsExtrema result;
  for (const auto & piece : trajectory) {
    const double v = maxPlanarDerivative(piece, 1), a = maxPlanarDerivative(piece, 2),
      j = maxPlanarDerivative(piece, 3);
    result.finite = result.finite && std::isfinite(v) && std::isfinite(a) && std::isfinite(j);
    result.velocity = std::max(result.velocity, v);
    result.acceleration = std::max(result.acceleration, a);
    result.jerk = std::max(result.jerk, j);
  }
  return result;
}
inline double requiredTimeScale(const DynamicsExtrema & values, const DynamicsLimits & limits) {
  return std::max({1.0, values.velocity / limits.max_vel,
    std::sqrt(values.acceleration / limits.max_acc), std::cbrt(values.jerk / limits.max_jerk)});
}
inline geometry_utils::Trajectory scaleTrajectoryTime(const geometry_utils::Trajectory & input, double scale) {
  if (!std::isfinite(scale) || scale < 1) { throw std::invalid_argument("Invalid trajectory time scale"); }
  geometry_utils::Trajectory output;
  output.start_WT = input.start_WT;
  for (const auto & piece : input) {
    auto coefficients = piece.getCoeffMat();
    const int degree = coefficients.cols() - 1;
    for (int k = 0; k <= degree; ++k) { coefficients.col(k) /= std::pow(scale, degree - k); }
    output.emplace_back(piece.getDuration() * scale, coefficients);
  }
  return output;
}
}  // namespace minco_planner
