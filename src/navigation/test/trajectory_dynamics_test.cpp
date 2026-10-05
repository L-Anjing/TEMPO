#include <gtest/gtest.h>
#include <minco_core/components/trajectory_dynamics.hpp>

namespace {
geometry_utils::Trajectory restToRest() {
  Eigen::Matrix<double, 3, 6> coefficients = Eigen::Matrix<double, 3, 6>::Zero();
  coefficients.row(0) << 3.6, -9, 6, 0, 0, 0;
  coefficients.row(1) << 4.8, -12, 8, 0, 0, 0;
  // Terrain height is independent of the planar chassis limits.
  coefficients.row(2) << 600, -1500, 1000, 0, 0, 0;
  geometry_utils::Trajectory trajectory;
  trajectory.emplace_back(1.0, coefficients); return trajectory;
}
}
TEST(TrajectoryDynamics, FindsInteriorPeaksAndOmnidirectionalNorm) {
  const auto values = minco_planner::evaluateDynamics(restToRest());
  ASSERT_TRUE(values.finite);
  EXPECT_NEAR(values.velocity, 1.875, 1e-7);
  EXPECT_NEAR(values.acceleration, 10 / std::sqrt(3.0), 1e-7);
  EXPECT_NEAR(values.jerk, 60.0, 1e-7);
}
TEST(TrajectoryDynamics, TimeScalingPreservesGeometryAndReducesDerivatives) {
  const auto original = restToRest();
  minco_planner::DynamicsLimits limits;
  limits.max_vel = 1; limits.max_acc = .6; limits.max_jerk = 10;
  const auto before = minco_planner::evaluateDynamics(original);
  const double scale = minco_planner::requiredTimeScale(before, limits) * 1.01;
  const auto slower = minco_planner::scaleTrajectoryTime(original, scale);
  for (int i = 0; i <= 100; ++i) {
    const double time = i / 100.0;
    EXPECT_LT((original.getPos(time) - slower.getPos(time * scale)).norm(), 1e-9);
    EXPECT_LT((original.getVel(time) / scale - slower.getVel(time * scale)).norm(), 1e-9);
  }
  const auto after = minco_planner::evaluateDynamics(slower);
  EXPECT_LE(after.velocity, limits.max_vel);
  EXPECT_LE(after.acceleration, limits.max_acc);
  EXPECT_LE(after.jerk, limits.max_jerk);
  EXPECT_LT(slower.getVel(0).norm(), 1e-9);
  EXPECT_LT(slower.getVel(slower.getTotalDuration()).norm(), 1e-9);
}
TEST(TrajectoryDynamics, SmallOvershootRequiresScalingRatherThanToleranceAcceptance) {
  minco_planner::DynamicsLimits limits;
  limits.max_acc = .6;
  const minco_planner::DynamicsExtrema values{.28, .600355, 1, true};
  EXPECT_GT(minco_planner::requiredTimeScale(values, limits), 1);
  EXPECT_LT(values.acceleration, limits.max_acc * (1 + limits.validation_tolerance));
}
