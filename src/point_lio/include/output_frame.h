#pragma once

#include "robot_extrinsics.h"

namespace point_lio {

// 输出零点（延迟冻结）。
//
// 滤波器内部世界系 W 从 IMU 初始化起就固定，之后永不重写：状态、协方差、iVox 地图全部留在 W。
// 等 LiDAR IEKF 收敛后再记录 T_WB0，对外统一输出
//     T_OB(t) = (T_WB0)^-1 * T_WB(t)
// 位姿、世界系点云、TF、path 都经过同一个 T_OB，保证 odom 与点云落在同一坐标系。
struct OutputFrame
{
  OutputFrame() = default;

  explicit OutputFrame(const RobotKinematics & frozen_robot)
  : rotation(Eigen::Quaterniond(frozen_robot.rotation).normalized().conjugate().toRotationMatrix()),
    origin(frozen_robot.position) {}

  Eigen::Vector3d position(const Eigen::Vector3d & world_position) const
  {
    return rotation * (world_position - origin);
  }

  // 位置和姿态换到输出系。线速度/角速度是底盘体系量，不随世界系旋转改变。
  void apply(RobotKinematics & robot) const
  {
    robot.position = position(robot.position);
    // 走旋转矩阵复合再归一化：Matrix3d * Quaterniond 是表达式，且矩阵转四元数不保证单位长度。
    robot.rotation = Eigen::Quaterniond(rotation * robot.rotation.toRotationMatrix()).normalized();
  }

  template <typename PointT> void applyPoint(PointT & point) const
  {
    const Eigen::Vector3d world = rotation *
      (point.getVector3fMap().template cast<double>() - origin);
    point.x = static_cast<float>(world.x());
    point.y = static_cast<float>(world.y());
    point.z = static_cast<float>(world.z());
  }

  Eigen::Matrix3d rotation{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d origin{Eigen::Vector3d::Zero()};
};

}  // namespace point_lio
