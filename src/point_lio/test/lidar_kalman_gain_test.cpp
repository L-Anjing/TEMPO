#include <gtest/gtest.h>
#include "lidar_kalman_gain.h"

TEST(LidarKalmanGain, BothBatchSizesMatchDirectKalmanFormula)
{
  for (int n : {24, 30}) {
    const Eigen::MatrixXd a = Eigen::MatrixXd::Random(n, n);
    const Eigen::MatrixXd p = a * a.transpose() + .1 * Eigen::MatrixXd::Identity(n, n);
    for (int m : {1, n - 1, n, n + 1, 90}) {
      const Eigen::MatrixXd h = Eigen::MatrixXd::Random(m, 12);
      Eigen::MatrixXd full = Eigen::MatrixXd::Zero(m, n);
      full.leftCols(12) = h;
      for (double variance : {.01, 1., 100.}) {
        const Eigen::MatrixXd expected = p * full.transpose() *
          (full * p * full.transpose() + variance * Eigen::MatrixXd::Identity(m, m)).inverse();
        const Eigen::MatrixXd actual = point_lio::lidarKalmanGain(p, h, variance);
        EXPECT_LT((actual - expected).norm(), 1.e-8);
      }
    }
  }
}
