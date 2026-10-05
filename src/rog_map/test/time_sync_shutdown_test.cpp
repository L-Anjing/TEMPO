#include <gtest/gtest.h>
#include <rog_map_ros/odometry_buffer.hpp>
#include <rog_map_ros/callback_gate.hpp>
#include <future>
#include <thread>

TEST(OdometryTimeSync, InterpolatesPositionAndQuaternionAtExactCloudTime) {
  rog_map::OdometryBuffer buffer;
  const rog_map::Pose first{super_utils::Vec3f::Zero(), super_utils::Quatf::Identity()};
  const rog_map::Pose second{super_utils::Vec3f(2, 4, 6),
    super_utils::Quatf(Eigen::AngleAxisd(1.0, Eigen::Vector3d::UnitZ()))};
  ASSERT_TRUE(buffer.insert(10.0, first)); ASSERT_TRUE(buffer.insert(10.01, second));
  rog_map::Pose actual;
  ASSERT_TRUE(buffer.interpolate(10.005, actual, .05));
  EXPECT_LT((actual.first - super_utils::Vec3f(1, 2, 3)).norm(), 1e-9);
  EXPECT_NEAR(actual.second.angularDistance(first.second), .5, 1e-9);
  EXPECT_FALSE(buffer.interpolate(9.99, actual, .05));
  EXPECT_FALSE(buffer.interpolate(10.02, actual, .05));
  EXPECT_FALSE(buffer.interpolate(10.005, actual, .001));
}
TEST(OdometryTimeSync, AcceptsLateBracketWithoutLatestPoseFallback) {
  rog_map::OdometryBuffer buffer;
  rog_map::Pose actual;
  const rog_map::Pose pose{super_utils::Vec3f::Zero(), super_utils::Quatf::Identity()};
  buffer.insert(10, pose);
  EXPECT_FALSE(buffer.interpolate(10.004, actual, .05));
  buffer.insert(10.005, pose);
  EXPECT_TRUE(buffer.interpolate(10.004, actual, .05));
  EXPECT_FALSE(buffer.insert(0, pose));
}
TEST(CallbackShutdown, DrainsRunningAndRejectsQueuedCallbacks) {
  auto gate = std::make_shared<rog_map::CallbackGate>();
  std::promise<void> entered, release;
  auto release_future = release.get_future();
  std::thread worker([&] { gate->run([&] { entered.set_value(); release_future.wait(); }); });
  entered.get_future().wait();
  auto stop = std::async(std::launch::async, [&] { gate->stop(); });
  EXPECT_EQ(stop.wait_for(std::chrono::milliseconds(20)), std::future_status::timeout);
  release.set_value(); worker.join(); stop.get();
  bool touched = false;
  EXPECT_FALSE(gate->run([&] { touched = true; }));
  EXPECT_FALSE(touched);
}
