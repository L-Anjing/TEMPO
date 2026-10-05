#pragma once

#include <Eigen/Geometry>
#include <stdexcept>
#include <string>
#include <vector>

namespace point_lio {

inline Eigen::Vector3d extrinsicTranslation(const std::vector<double> & values,
  const std::string & name)
{
  if (values.size() != 3) { throw std::invalid_argument(name + " requires 3 values"); }
  const Eigen::Vector3d translation(values[0], values[1], values[2]);
  if (!translation.allFinite()) { throw std::invalid_argument(name + " must be finite"); }
  return translation;
}

inline Eigen::Matrix3d extrinsicRotation(const std::vector<double> & values,
  const std::string & name)
{
  if (values.size() != 9) { throw std::invalid_argument(name + " requires 9 row-major values"); }
  Eigen::Matrix3d rotation;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) { rotation(row, col) = values[3 * row + col]; }
  }
  if (!rotation.allFinite() ||
      (rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm() > 1.e-3 ||
      std::abs(rotation.determinant() - 1.0) > 1.e-3) {
    throw std::invalid_argument(name + " must be a proper rotation matrix");
  }
  return Eigen::Quaterniond(rotation).normalized().toRotationMatrix();
}

// p_lidar = R_imu_in_lidar * p_imu + t_imu_in_lidar;
// p_robot = R_lidar_in_robot * p_lidar + t_lidar_in_robot.
struct RobotExtrinsics
{
  Eigen::Matrix3d R_imu_in_lidar{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d t_imu_in_lidar{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d R_lidar_in_robot{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d t_lidar_in_robot{Eigen::Vector3d::Zero()};

  // Call once after loading and validating fixed calibration parameters.
  void initializeTransforms()
  {
    R_lidar_in_imu_ = R_imu_in_lidar.transpose();
    t_lidar_in_imu_ = -R_lidar_in_imu_ * t_imu_in_lidar;
    R_robot_in_imu_ = (R_lidar_in_robot * R_imu_in_lidar).transpose();
    R_imu_in_robot_ = R_robot_in_imu_.transpose();
    const Eigen::Vector3d imu_in_robot =
      t_lidar_in_robot + R_lidar_in_robot * t_imu_in_lidar;
    t_robot_in_imu_ = -R_robot_in_imu_ * imu_in_robot;
  }

  const Eigen::Matrix3d & robotRotationInImu() const { return R_robot_in_imu_; }
  const Eigen::Matrix3d & imuRotationInRobot() const { return R_imu_in_robot_; }
  const Eigen::Vector3d & robotOriginInImu() const { return t_robot_in_imu_; }
  const Eigen::Matrix3d & lidarRotationInImu() const { return R_lidar_in_imu_; }
  const Eigen::Vector3d & lidarOriginInImu() const { return t_lidar_in_imu_; }

private:
  Eigen::Matrix3d R_lidar_in_imu_{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d t_lidar_in_imu_{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d R_robot_in_imu_{Eigen::Matrix3d::Identity()};
  Eigen::Matrix3d R_imu_in_robot_{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d t_robot_in_imu_{Eigen::Vector3d::Zero()};
};

struct RobotKinematics
{
  Eigen::Vector3d position;
  Eigen::Quaterniond rotation;
  Eigen::Vector3d linear_velocity;
  Eigen::Vector3d angular_velocity;
};

inline RobotKinematics robotKinematics(const RobotExtrinsics & extrinsics,
  const Eigen::Vector3d & imu_position, const Eigen::Matrix3d & imu_rotation,
  const Eigen::Vector3d & imu_velocity_world, const Eigen::Vector3d & angular_velocity_imu)
{
  const auto & R_robot_in_imu = extrinsics.robotRotationInImu();
  const auto & robot_in_imu = extrinsics.robotOriginInImu();
  RobotKinematics out;
  out.position = imu_position + imu_rotation * robot_in_imu;
  out.rotation = Eigen::Quaterniond(imu_rotation * R_robot_in_imu).normalized();
  // Derivative of the rotating lever arm; Odometry twist is in child_frame_id.
  out.linear_velocity = out.rotation.conjugate() *
    (imu_velocity_world + imu_rotation * angular_velocity_imu.cross(robot_in_imu));
  out.angular_velocity = extrinsics.imuRotationInRobot() * angular_velocity_imu;
  return out;
}

}  // namespace point_lio
