#pragma once
#include <Eigen/Dense>
#include <stdexcept>

namespace point_lio {
// h contains only the observed leading state columns. variance is R, not R^-1.
inline Eigen::MatrixXd lidarKalmanGain(const Eigen::MatrixXd & covariance,
  const Eigen::MatrixXd & h, double variance)
{
  if (!std::isfinite(variance) || variance <= 0.0) {
    throw std::invalid_argument("LiDAR measurement variance must be positive");
  }
  const Eigen::Index n = covariance.rows(), observed = h.cols();
  if (h.rows() < n) {
    const Eigen::MatrixXd pht = covariance.leftCols(observed) * h.transpose();
    Eigen::MatrixXd innovation = h * pht.topRows(observed);
    innovation.diagonal().array() += variance;
    return innovation.ldlt().solve(pht.transpose()).transpose();
  }
  Eigen::MatrixXd information = covariance.inverse();
  information.topLeftCorner(observed, observed).noalias() += h.transpose() * h / variance;
  Eigen::MatrixXd rhs = Eigen::MatrixXd::Zero(n, h.rows());
  rhs.topRows(observed) = h.transpose() / variance;
  return information.ldlt().solve(rhs);
}
}  // namespace point_lio
