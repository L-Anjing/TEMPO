#ifndef MINCO_PLANNER__LOCAL_PATH_PROCESSOR_HPP_
#define MINCO_PLANNER__LOCAL_PATH_PROCESSOR_HPP_

#include "minco_core/header.hpp"

namespace minco_planner {

class PlannerModeContext;

struct LocalPathSeed
{
  bool valid{false};
  bool local_end_is_goal{false};
  size_t global_end_index{0U};
  std::vector<Eigen::Vector3d> dense_path;      // A* 原始路径（本段 lookahead 切片）
  std::vector<Eigen::Vector3d> los_anchors;     // LOS 贪心简化后的锚点
  std::vector<Eigen::Vector3d> sparse_waypoints; // 沿锚点直线均匀重采样得到的 MINCO 几何 seed
  std::vector<Eigen::Vector3d> seed_tangents;   // seed 切线（中心差分；MINCO_S3NU 只吃位置，本级用于诊断）
  std::vector<double> local_magnitudes;
};

class LocalPathProcessor
{
public:
  void configure(double lookahead_dist,
    double max_vel,
    double max_acc,
    double traj_goal_tolerance,
    double los_sampling_ratio,
    double seed_spacing,
    rclcpp::Clock::SharedPtr clock,
    rclcpp::Logger logger);

  void updateLimits(double max_vel, double max_acc, double traj_goal_tolerance);

  void resetProgress();

  LocalPathSeed buildSeed(const std::vector<geometry_msgs::msg::PoseStamped> & global_path,
    const geometry_msgs::msg::PoseStamped & current_pose,
    const PlannerModeContext & mode_context) const;

private:
  std::vector<Eigen::Vector3d> extractLocalPath(
    const std::vector<geometry_msgs::msg::PoseStamped> & global_path,
    const Eigen::Vector3d & cur_pos,
    size_t & global_start_index) const;

  bool clipLocalPathByRogBoundary(
    std::vector<Eigen::Vector3d> & path, const PlannerModeContext & mode_context) const;

  double lookahead_dist_{5.0};
  double max_vel_{2.0};
  double max_acc_{4.0};
  double traj_goal_tolerance_{0.5};
  double los_sampling_ratio_{0.5};
  double seed_spacing_{0.45};
  mutable std::mutex progress_mutex_;
  mutable bool has_progress_{false};
  mutable size_t last_start_index_{0U};
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("LocalPathProcessor")};
};

}  // namespace minco_planner

#endif  // MINCO_PLANNER__LOCAL_PATH_PROCESSOR_HPP_
