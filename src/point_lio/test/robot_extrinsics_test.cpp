#include <gtest/gtest.h>
#include "robot_extrinsics.h"
#include "output_frame.h"
#include <cmath>

namespace {
// Point-LIO 的 PointType 来自 common_lib.h（会拉进 PCL）；这里用一个满足同样接口的最小点类型。
struct TestPoint
{
  float x{0.f}, y{0.f}, z{0.f};
  Eigen::Vector3f getVector3fMap() const { return Eigen::Vector3f(x, y, z); }
};

point_lio::RobotExtrinsics calibration()
{
  point_lio::RobotExtrinsics result;
  result.t_imu_in_lidar = Eigen::Vector3d(0.011, 0.02329, -0.04412);
  result.t_lidar_in_robot = Eigen::Vector3d(-0.1025, -0.303, 0.2634);
  result.initializeTransforms();
  return result;
}
}

TEST(RobotExtrinsics, Mid360ConventionAndRobotOrigin)
{
  const auto ex = calibration();
  // Independent reference: IMU origin in base_link = T_lidar^base * T_imu^lidar.
  const Eigen::Vector3d imu_in_robot(-0.0915, -0.27971, 0.21928);
  EXPECT_TRUE(ex.lidarOriginInImu().isApprox(Eigen::Vector3d(-0.011, -0.02329, 0.04412)));
  EXPECT_TRUE(ex.robotOriginInImu().isApprox(-imu_in_robot));
  const auto state = point_lio::robotKinematics(ex, imu_in_robot,
    Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  EXPECT_NEAR(state.position.norm(), 0., 1.e-12);
}

TEST(RobotExtrinsics, RotatingInPlaceHasNoRobotTranslationVelocity)
{
  auto ex = calibration();
  ex.R_lidar_in_robot = Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitY()).toRotationMatrix();
  ex.R_imu_in_lidar = Eigen::AngleAxisd(-0.2, Eigen::Vector3d::UnitX()).toRotationMatrix();
  ex.initializeTransforms();
  const Eigen::Matrix3d R_world_robot =
    Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Matrix3d R_imu_robot = ex.R_lidar_in_robot * ex.R_imu_in_lidar;
  const Eigen::Matrix3d R_world_imu = R_world_robot * R_imu_robot;
  const Eigen::Vector3d t_imu_robot = ex.t_lidar_in_robot +
    ex.R_lidar_in_robot * ex.t_imu_in_lidar;
  const Eigen::Vector3d omega_robot(0.1, -0.2, 1.0);
  const Eigen::Vector3d omega_imu = R_imu_robot.transpose() * omega_robot;
  const Eigen::Vector3d imu_velocity = R_world_robot * omega_robot.cross(t_imu_robot);
  const auto state = point_lio::robotKinematics(ex, R_world_robot * t_imu_robot,
    R_world_imu, imu_velocity, omega_imu);
  EXPECT_NEAR(state.position.norm(), 0., 1.e-12);
  EXPECT_NEAR(state.linear_velocity.norm(), 0., 1.e-12);
  EXPECT_TRUE(state.rotation.toRotationMatrix().isApprox(R_world_robot));
  EXPECT_TRUE(state.angular_velocity.isApprox(omega_robot));
}

TEST(RobotExtrinsics, TiltedInstallationPreservesWorldCloudCoordinates)
{
  auto ex = calibration();
  ex.R_lidar_in_robot = Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitY()).toRotationMatrix();
  ex.R_imu_in_lidar = Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitX()).toRotationMatrix();
  ex.initializeTransforms();
  const Eigen::Matrix3d R_world_imu =
    Eigen::AngleAxisd(-0.5, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Vector3d p_world_imu(3., 4., 1.);
  const auto state = point_lio::robotKinematics(ex, p_world_imu, R_world_imu,
    Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  const Eigen::Vector3d p_lidar(2., -1., 0.5);
  const Eigen::Vector3d via_imu = p_world_imu + R_world_imu *
    (ex.lidarRotationInImu() * p_lidar + ex.lidarOriginInImu());
  const Eigen::Vector3d via_robot = state.position + state.rotation *
    (ex.R_lidar_in_robot * p_lidar + ex.t_lidar_in_robot);
  EXPECT_TRUE(via_imu.isApprox(via_robot));
  const Eigen::Vector3d laser_origin = state.position + state.rotation * ex.t_lidar_in_robot;
  EXPECT_TRUE(laser_origin.isApprox(p_world_imu - R_world_imu *
    ex.R_imu_in_lidar.transpose() * ex.t_imu_in_lidar));
}

TEST(RobotExtrinsics, InvalidCalibrationIsRejected)
{
  EXPECT_THROW(point_lio::extrinsicTranslation({1., 2.}, "translation"), std::invalid_argument);
  EXPECT_THROW(point_lio::extrinsicRotation({1., 0., 0., 0., 1., 0., 0., 0., -1.},
    "reflection"), std::invalid_argument);
}

TEST(OutputFrame, DefaultFrameIsIdentity)
{
  const point_lio::OutputFrame frame;
  const Eigen::Vector3d point(1.5, -2.5, 0.75);
  EXPECT_TRUE(frame.position(point).isApprox(point));
  auto robot = point_lio::robotKinematics(calibration(), Eigen::Vector3d(0.3, -0.2, 1.1),
    Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  const auto before = robot;
  frame.apply(robot);
  EXPECT_TRUE(robot.position.isApprox(before.position));
  // 旋转用矩阵比较：四元数 q 与 -q 表示同一旋转。
  EXPECT_TRUE(robot.rotation.toRotationMatrix().isApprox(before.rotation.toRotationMatrix()));
}

TEST(OutputFrame, FrozenPoseIsIdentityAndCloudMatchesRobotTransform)
{
  auto ex = calibration();
  ex.R_lidar_in_robot = Eigen::AngleAxisd(0.35, Eigen::Vector3d::UnitY()).toRotationMatrix();
  ex.R_imu_in_lidar = Eigen::AngleAxisd(-0.2, Eigen::Vector3d::UnitX()).toRotationMatrix();
  ex.initializeTransforms();
  // 冻结用的状态是 LiDAR 收敛之后的位姿，不是 IMU 刚初始化时的位姿。
  const Eigen::Matrix3d frozen_rotation =
    Eigen::AngleAxisd(0.7, Eigen::Vector3d(1., 2., 3.).normalized()).toRotationMatrix();
  const Eigen::Vector3d frozen_position(3., 4., 5.);
  // 冻结之前，内层状态带着 IMU 初始化残留的偏置（yaw 偏移 + 平移）。
  const Eigen::Matrix3d bias_rotation =
    Eigen::AngleAxisd(-0.15, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Matrix3d initial_rotation = frozen_rotation * bias_rotation;
  const Eigen::Vector3d initial_position = frozen_position + Eigen::Vector3d(0.4, -0.25, 0.1);

  const auto frozen_robot = point_lio::robotKinematics(ex, frozen_position,
    frozen_rotation, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  const point_lio::OutputFrame frame(frozen_robot);
  // 输出系的旋转必须是正交阵：position() 和 applyPoint() 都直接用它做刚体变换。
  EXPECT_TRUE((frame.rotation.transpose() * frame.rotation).isApprox(Eigen::Matrix3d::Identity()));
  EXPECT_NEAR(frame.rotation.determinant(), 1.0, 1.e-12);

  // 冻结时刻的位姿映射到输出系原点。
  const auto zero_robot = point_lio::robotKinematics(ex, frame.position(frozen_position),
    frame.rotation * frozen_rotation, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  EXPECT_NEAR(zero_robot.position.norm(), 0., 1.e-12);
  EXPECT_TRUE(zero_robot.rotation.toRotationMatrix().isApprox(Eigen::Matrix3d::Identity()));

  // 冻结之前的位姿在输出系里就是那次没收敛的偏置，之后不再变化。
  const auto initial_robot = point_lio::robotKinematics(ex, frame.position(initial_position),
    frame.rotation * initial_rotation, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  EXPECT_GT(initial_robot.position.norm(), 0.1);
  EXPECT_GT(std::abs(Eigen::AngleAxisd(initial_robot.rotation).angle()), 0.1);

  // 世界点、TF 与 odom 必须落在同一个输出系。
  const Eigen::Vector3d next_position = initial_position + Eigen::Vector3d(1., 2., -0.3);
  const Eigen::Matrix3d next_rotation = initial_rotation *
    Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const auto next_robot = point_lio::robotKinematics(ex, frame.position(next_position),
    frame.rotation * next_rotation, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  const Eigen::Vector3d lidar_point(2., -1., 0.5);
  const Eigen::Vector3d old_world_point = next_position + next_rotation *
    (ex.lidarRotationInImu() * lidar_point + ex.lidarOriginInImu());
  const Eigen::Vector3d point_via_robot = next_robot.position + next_robot.rotation *
    (ex.R_lidar_in_robot * lidar_point + ex.t_lidar_in_robot);
  TestPoint output_point;
  output_point.x = static_cast<float>(old_world_point.x());
  output_point.y = static_cast<float>(old_world_point.y());
  output_point.z = static_cast<float>(old_world_point.z());
  frame.applyPoint(output_point);
  EXPECT_NEAR(output_point.x, point_via_robot.x(), 1.e-5);
  EXPECT_NEAR(output_point.y, point_via_robot.y(), 1.e-5);
  EXPECT_NEAR(output_point.z, point_via_robot.z(), 1.e-5);
}

TEST(OutputFrame, RobotTwistIsBodyFrameAndSurvivesTheTransform)
{
  const auto ex = calibration();
  const Eigen::Vector3d position(1., 2., 3.);
  const Eigen::Matrix3d rotation =
    Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitY()).toRotationMatrix();
  const Eigen::Vector3d velocity(0.2, 0.3, -0.1);
  const Eigen::Vector3d omega(0.1, -0.2, 0.3);
  const auto before = point_lio::robotKinematics(ex, position, rotation, velocity, omega);
  const point_lio::OutputFrame frame(before);
  auto after = before;
  frame.apply(after);
  EXPECT_TRUE(after.linear_velocity.isApprox(before.linear_velocity));
  EXPECT_TRUE(after.angular_velocity.isApprox(before.angular_velocity));
  // 发布出去的四元数必须是单位四元数（Nav2/TF 直接消费）。
  EXPECT_NEAR(after.rotation.norm(), 1.0, 1.e-12);
  EXPECT_NEAR(after.position.norm(), 0., 1.e-12);
  EXPECT_TRUE(after.rotation.toRotationMatrix().isApprox(Eigen::Matrix3d::Identity()));
}
