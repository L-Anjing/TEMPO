#include <gtest/gtest.h>
#include "imu_odometry_predictor.h"
using Predictor = point_lio::ImuOdometryPredictor;

TEST(ImuOdometryPredictor, StationaryBiasAndGravityCancel)
{
  Predictor p;
  Predictor::State seed, out;
  seed.bg = Eigen::Vector3d(.001, -.002, .015);
  seed.ba = Eigen::Vector3d(.01, .02, -.03);
  for (int i = 0; i <= 40; ++i) {
    p.push({i * .005, (Eigen::Vector3d(0, 0, 9.81) + seed.ba) / 9.81, seed.bg});
  }
  p.seed(seed);
  ASSERT_TRUE(p.predict(out, .3));
  EXPECT_NEAR(out.pos.norm(), 0.0, 1.e-12);
  EXPECT_NEAR(out.vel.norm(), 0.0, 1.e-12);
  EXPECT_NEAR(out.omega.norm(), 0.0, 1.e-12);
  EXPECT_FALSE(p.predict(out, .3));
}

TEST(ImuOdometryPredictor, AccelerationAndPartialFirstInterval)
{
  Predictor p;
  Predictor::State seed, out;
  seed.stamp = .0025;
  seed.acc_scale = 1.0;
  for (int i = 0; i <= 40; ++i) { p.push({i * .005, {2, 0, 9.81}, {0, 0, 0}}); }
  p.seed(seed);
  ASSERT_TRUE(p.predict(out, .3));
  EXPECT_NEAR(out.vel.x(), 2 * .1975, 1.e-9);
  EXPECT_NEAR(out.pos.x(), .1975 * .1975, 1.e-9);
}

TEST(ImuOdometryPredictor, StaleCorrectionOrMissingImuStopsOutput)
{
  Predictor p;
  Predictor::State seed, out;
  p.push({0, {0, 0, 1}, {0, 0, 0}});
  p.push({.1, {0, 0, 1}, {0, 0, 0}});
  p.seed(seed);
  EXPECT_FALSE(p.predict(out, .05));
  EXPECT_FALSE(p.predict(out, .3));
}

TEST(ImuOdometryPredictor, ReseedReplaysHistoryWithoutDuplicateTime)
{
  Predictor p;
  Predictor::State seed, out;
  for (int i = 0; i <= 20; ++i) { p.push({i * .005, {0, 0, 1}, {0, 0, 0}}); }
  p.seed(seed);
  ASSERT_TRUE(p.predict(out, .3));
  seed.stamp = .05;
  seed.pos.x() = 1.;
  p.seed(seed);
  EXPECT_FALSE(p.predict(out, .3));
  p.push({.105, {0, 0, 1}, {0, 0, 0}});
  ASSERT_TRUE(p.predict(out, .3));
  EXPECT_NEAR(out.pos.x(), 1., 1.e-12);
  EXPECT_NEAR(out.stamp, .105, 1.e-12);
  p.push({0., {0, 0, 1}, {0, 0, 0}});
  EXPECT_FALSE(p.predict(out, .3));
}
