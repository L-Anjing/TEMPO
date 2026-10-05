#ifndef MINCO_PLANNER__MINCO_UTILS_HPP_
#define MINCO_PLANNER__MINCO_UTILS_HPP_

#include "minco_core/header.hpp"

namespace geometry_utils {
class Trajectory;
}  // namespace geometry_utils

namespace traj_opt {
using Trajectory = geometry_utils::Trajectory;
}  // namespace traj_opt

namespace minco_planner::utils {

// === Generic Utilities ===
template <typename T> inline T clampValue(T v, T lo, T hi)
{
  return std::min(std::max(v, lo), hi);
}

double quaternionToYaw(const geometry_msgs::msg::Quaternion & q);
double calCurvatureDecay(
  double angle, double global_vmax, double deadzone, double saturation, double min_vel, double decay_power);

// === Path Geometry Utilities ===
// --- Velocity Profile Mapping ---
double getDistFromTrapezoid(
  double t, double total_length, double a_ref, double v_peak, double t_acc, double t_flat, double t_dec);

// --- Arc-Length Interpolation ---
Eigen::Vector3d interpolateByArcLength(
  const std::vector<Eigen::Vector3d> & path, const std::vector<double> & accumulated_dist, double s);

// 逐段 LOS 判定：按 los_sampling_ratio × resolution 采样，每个采样点查 footprintFree，
// 每次跨越新格调用 edgeAllowed。UNKNOWN 按可通行处理。查询与 A*/validator 共用同一个 oracle。
bool lineOfSight(const Eigen::Vector3d & from,
  const Eigen::Vector3d & to,
  const std::shared_ptr<rog_map::MapQueryInterface> & query,
  double los_sampling_ratio);

// 贪心 LOS 简化：从当前锚点出发，在 raw 折线上找最远的可视点作为下一个锚点。
// A* 只决定拓扑；这一步删除没有几何必要性的栅格拐点。
std::vector<Eigen::Vector3d> simplifyPathByLOS(const std::vector<Eigen::Vector3d> & raw_path,
  const std::shared_ptr<rog_map::MapQueryInterface> & query,
  double los_sampling_ratio);

// 沿锚点直线段均匀插点，得到 MINCO 的几何 seed。点落在 LOS 直线段上，不回落到 A* 原始格点。
std::vector<Eigen::Vector3d> resampleUniform(
  const std::vector<Eigen::Vector3d> & anchors, double spacing);

// seed 切线：内部点用中心差分 p[i+1]-p[i-1]，首尾用前/后差分。
std::vector<Eigen::Vector3d> computeSeedTangents(const std::vector<Eigen::Vector3d> & seed);

std::vector<Eigen::Vector3d> getSparseWaypoints(const std::vector<Eigen::Vector3d> & path,
  double max_vel,
  double max_acc,
  bool goal_reached,
  const std::function<bool(const Eigen::Vector3d &, const Eigen::Vector3d &)> & is_line_free);

double LimitLocalVel(const std::vector<Eigen::Vector3d> & sparse_path,
  int seg_idx,
  double global_vmax,
  double deadzone,
  double saturation,
  double min_turn_vel,
  double decay_power);

double ComputeNextSpeed(
  double v_curr, double seg_len, double remain_after, double amax, double local_vmax, double terminal_speed);

double ComputeSegmentTime(
  double seg_len, double v_curr, double v_next, double local_vmax, double amax, double min_seg_time);

void VelPropogation(const std::vector<double> & seg_len, double amax, std::vector<double> & local_vmaxs);

// === Costmap Utilities ===
// --- Collision / Visibility Checks ---

bool isLineFree(
  nav2_costmap_2d::Costmap2D * costmap, const Eigen::Vector3d & p1, const Eigen::Vector3d & p2);

// --- Coordinate Conversion ---
bool worldToMap(nav2_costmap_2d::Costmap2D * costmap,
  const rclcpp::Logger & logger,
  double wx,
  double wy,
  unsigned int & mx,
  unsigned int & my);

void mapToWorld(nav2_costmap_2d::Costmap2D * costmap, double mx, double my, double & wx, double & wy);

// --- Costmap Cell Editing ---
void clearRobotCell(nav2_costmap_2d::Costmap2D * costmap, unsigned int mx, unsigned int my);

void compensateLeverArm(double v_lidar_x,
  double v_lidar_y,
  double omega_z,
  double yaw,
  double lidar_offset_x,
  double lidar_offset_y,
  double & vx_global,
  double & vy_global,
  double & omega_global);

}  // namespace minco_planner::utils
#endif  // MINCO_PLANNER__MINCO_UTILS_HPP_
