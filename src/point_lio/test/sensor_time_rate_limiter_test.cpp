#include <gtest/gtest.h>
#include "sensor_time_rate_limiter.h"

TEST(SensorTimeRateLimiter, TwoMillisecondStateUpdatesMaintainTwoHundredHertz)
{
  point_lio::SensorTimeRateLimiter limiter;
  int count = 0;
  // Real Unix sensor timestamps exercise precision as well as the 2 ms / 5 ms schedule.
  for (int update = 0; update < 5000; ++update) {
    count += limiter.should_publish(1791000000.0 + update * 0.002, 200.0);
  }
  EXPECT_EQ(count, 2000);
}

TEST(SensorTimeRateLimiter, MissingUpdatesDoNotProduceCatchupBursts)
{
  point_lio::SensorTimeRateLimiter limiter;
  EXPECT_TRUE(limiter.should_publish(10., 200.));
  EXPECT_TRUE(limiter.should_publish(11., 200.));
  EXPECT_FALSE(limiter.should_publish(11., 200.));
  EXPECT_FALSE(limiter.should_publish(11.001, 200.));
  EXPECT_TRUE(limiter.should_publish(11.005, 200.));
}

TEST(SensorTimeRateLimiter, TimestampResetAndFrequencyChangeRestartSchedule)
{
  point_lio::SensorTimeRateLimiter limiter;
  EXPECT_TRUE(limiter.should_publish(10., 200.));
  EXPECT_FALSE(limiter.should_publish(10.001, 200.));
  EXPECT_TRUE(limiter.should_publish(1., 200.));
  EXPECT_FALSE(limiter.should_publish(1.001, 200.));
  EXPECT_TRUE(limiter.should_publish(1.001, 100.));
  EXPECT_FALSE(limiter.should_publish(1.005, 100.));
  EXPECT_TRUE(limiter.should_publish(1.011, 100.));
}

TEST(SensorTimeRateLimiter, InvalidInputsAreRejected)
{
  point_lio::SensorTimeRateLimiter limiter;
  EXPECT_FALSE(limiter.should_publish(1., 0.));
  EXPECT_FALSE(limiter.should_publish(1., -1.));
  EXPECT_FALSE(limiter.should_publish(std::numeric_limits<double>::quiet_NaN(), 200.));
  EXPECT_FALSE(limiter.should_publish(1., std::numeric_limits<double>::infinity()));
}
