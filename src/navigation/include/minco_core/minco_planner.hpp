#ifndef MINCO_PLANNER__MINCO_PLANNER_HPP_
#define MINCO_PLANNER__MINCO_PLANNER_HPP_

#include "minco_core/header.hpp"
#include "minco_core/components/trajectory_dynamics.hpp"
#include <rog_map_ros/callback_gate.hpp>

#include <atomic>
#include <limits>
#include <navigation/msg/plan_meta.hpp>
#include <std_msgs/msg/bool.hpp>

namespace rog_map {
class ROGMapROS;
}  // namespace rog_map

namespace minco_planner {

class Visualizer;
class MincoFsm;

class MincoPlanner : public nav2_core::GlobalPlanner
{
public:
  using Ptr = std::shared_ptr<MincoPlanner>;

  // === Constructor & Lifecycle ===
  MincoPlanner();
  ~MincoPlanner();

  void configure(const nav2_util::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void activate() override;
  void deactivate() override;
  void cleanup() override;

  // 释放所有 ROS 实体（订阅/发布/定时器/查询对象/ROGMap）。
  // 只能由析构函数调用：cleanup() 由 executor 回调触发，在那里析构会让正在 rmw_wait()
  // 的 executor 线程读到已释放的 waitset。
  void releaseRosEntities();

  // 校验器（QueryAdapter::footprintFree）实际执行的硬净空半径，供优化器对齐下限。
  double requiredObstacleClearance() const;
  // 优化器使用的障碍距离，取 minco_optimizer.safe_dist 与硬净空+余量的较大者。
  double optimizerObstacleDistance() const;

  // === Core Planning Interfaces ===
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal) override;

  bool PlanGlobalPath(const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal);

  void setMap(const std::shared_ptr<rog_map::MapQueryInterface> & map);

  bool ReplanLocal(const geometry_msgs::msg::PoseStamped & current_pose);
  bool makePlan(const geometry_msgs::msg::Pose & start,
    const geometry_msgs::msg::Pose & goal,
    double tolerance,
    std::function<bool()> cancel_checker,
    nav_msgs::msg::Path & plan);

  // === Callbacks ===
  void safetyTimerCallback();

  // === Utility & Helper Functions ===
  bool checkCollision();
  bool checkCollision(const geometry_utils::Trajectory & traj);

  // Accessors for FSM
  bool isTrajSafe() const { return is_traj_safe_.load(); }
  double getLookaheadDist() const { return lookahead_dist_; }
  bool getRobotPose(geometry_msgs::msg::PoseStamped & pose) const;
  bool checkGoalReached(const geometry_msgs::msg::PoseStamped & current_pose,
    const geometry_msgs::msg::PoseStamped & goal) const;
  bool consumePendingGoal(geometry_msgs::msg::PoseStamped & goal_out);
  void cancelGoal();
  bool externalMpcMode() const { return true; }
  void publishPlanningStatus(const std::string & reason);
  void inputAvailable();
  bool localHorizonExhausted() const { return local_horizon_exhausted_.load(); }
  bool continueLocalPlan(const geometry_msgs::msg::PoseStamped & current_pose,
    const geometry_msgs::msg::PoseStamped & goal);
  // 搜索失败时的状态串：优先用搜索器给的具体原因（GOAL_CONFIRMED_BLOCKED /
  // NO_REACHABLE_BOUNDARY），否则用调用方给的默认值。
  std::string searchFailureStatus(const std::string & fallback) const;
  bool reusePlan(const geometry_msgs::msg::PoseStamped & current_pose, bool publish = true);

  // === Replan policy ===
  // 当前是否已经有一条可执行的缓存路径。没有就必须重新规划。
  bool hasExecutablePlan() const;
  // 路径仍有效时周期性寻找更优路径。只有新路径相对当前剩余路径的改善超过
  // replan.improvement_ratio 才接受并替换 latest_global_path_；返回 true 表示应当重规划。
  bool betterPathAvailable(const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal);
  // 周期性寻找更优路径的间隔（replan.check_period_sec）。
  double replanCheckPeriod() const { return replan_check_period_sec_; }
  // 路径复用策略（详见 params/online_planner.yaml 的 path_reuse / goal_behavior）。
  bool betterPathEnabled() const { return better_path_enable_; }
  double minSwitchInterval() const { return min_switch_interval_; }
  double terminalRadius() const { return terminal_radius_; }
  // 由 FSM 在“当前路径被确认切断”时置位：这一次必须 COLD_START，不能拿撞上障碍的旧轨迹当 seed。
  void setForceColdStart(bool value) { force_cold_start_.store(value); }
  Eigen::Vector3d getCurrentSpeed() const;
  std::vector<Eigen::Vector3d> extractLocalPath(const Eigen::Vector3d & cur_pos);

private:
  friend class PlanReuseTest;
  // === Internal Types ===
  enum class PlanningState
  {
    COLD_START,     // Full replanning with zero initial velocity/acceleration.
    HOT_START       // Replanning with inherited velocity/acceleration.
  };

  // 缓存路径校验结果，用于区分输入缺失、偏离路径和障碍阻断。
  struct PathBlockInfo
  {
    // 几何类：CELL_OCCUPIED / BLOCKED_EDGE / FOOTPRINT_COLLISION / OUT_OF_MAP
    // 流程类：ROBOT_OFF_PATH / MAP_UNAVAILABLE / GOAL_CHANGED
    std::string reason;
    bool geometry_block{false};
  };
  PathBlockInfo last_path_block_info_;
  // === Utility & Helper Functions ===
  PlanningState determinePlanningState(
    const geometry_msgs::msg::Pose & start_pose, const std::vector<Eigen::Vector3d> & new_path);

  void prepareColdStart(const geometry_msgs::msg::Pose & start_pose,
    Eigen::Matrix3d & start_state,
    const std::vector<Eigen::Vector3d> & sparse_path);

  void prepareHotStart(
    const geometry_msgs::msg::Pose & start_pose, double t_dur, Eigen::Matrix3d & start_state);

  void PTAllocation(const std::vector<Eigen::Vector3d> & sparse_path,
    const Eigen::Matrix3d & start_state,
    double terminal_speed,
    PlanningState state,
    bool has_shifted_seed,
    const vec_Vec3f & shifted_waypoints,
    const VecDf & shifted_durations,
    vec_Vec3f & init_ps,
    VecDf & init_ts,
    VecDf & local_vmaxs) const;

  bool validateTrajectory(traj_opt::Trajectory & traj, const Eigen::Vector3d & expected_end_pos);
  DynamicsLimits dynamics_;
  nav_msgs::msg::Path createPlanInternal(const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal);
  std::shared_ptr<rog_map::CallbackGate> callback_gate_{std::make_shared<rog_map::CallbackGate>()};

  rcl_interfaces::msg::SetParametersResult onSetParameters(
    const std::vector<rclcpp::Parameter> & parameters);

  bool configureRogMap(const nav2_util::LifecycleNode::SharedPtr & node, const std::string & plugin_prefix);

  bool ensureMapAvailable();
  void rebuildModeDependentQueries();
  void initPlannerMode(
    const std::string & planner_mode_param, const std::string & map_frame, const std::string & rog_frame);
  bool normalizePoseToFrame(const geometry_msgs::msg::PoseStamped & in,
    const std::string & fallback_frame,
    const std::string & target_frame,
    const std::string & context,
    geometry_msgs::msg::PoseStamped & out) const;

  // === ROS 2 Interfaces (Publishers, Subscribers, Timers) ===
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_pub_;
  rclcpp::Publisher<navigation::msg::PlanMeta>::SharedPtr plan_meta_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cancel_sub_;
  void publishOptimizedPlan(const traj_opt::Trajectory & trajectory, const std_msgs::msg::Header & header);
  void publishPlan(const nav_msgs::msg::Path & path, bool replanned, float path_start_s,
    const std::string & reason);
  std::string robot_frame_{"base_link"};
  std::string last_planning_status_;
  double external_odom_timeout_{0.5};
  double external_map_timeout_{1.0};
  std::chrono::steady_clock::time_point odom_received_;
  nav_msgs::msg::Path cached_plan_;
  std::vector<double> cached_plan_s_;
  double cached_progress_s_{0.0};
  bool cached_plan_reaches_goal_{false};
  std::atomic<bool> local_horizon_exhausted_{false};
  double input_unavailable_grace_sec_{1.0};
  std::chrono::steady_clock::time_point input_unavailable_since_{};
  uint64_t cached_plan_goal_generation_{0};
  uint64_t planning_path_id_{0};
  uint64_t planning_publish_seq_{0};
  uint64_t last_logged_path_id_{std::numeric_limits<uint64_t>::max()};
  std::string last_logged_path_reason_;
  bool last_logged_plan_empty_{true};
  std::mutex planning_output_mutex_;
  std::atomic<uint64_t> pending_goal_generation_{0};
  uint64_t global_path_goal_generation_{0};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr fsm_timer_;
  rclcpp::TimerBase::SharedPtr safety_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;
  rclcpp::Clock::SharedPtr clock_;

  // === TF & Costmap & Frames ===
  std::shared_ptr<tf2_ros::Buffer> tf_;
  nav2_util::LifecycleNode::WeakPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::shared_ptr<rog_map::MapQueryInterface> map_;
  std::shared_ptr<rog_map::MapQueryInterface> rog_query_raw_;
  std::shared_ptr<rog_map::ROGMapROS> rog_map_ros_;
  std::string global_frame_, planning_frame_, output_frame_, map_frame_, rog_frame_, name_;
  PlannerModeParams mode_params_;

  // === Configurations & Parameters ===
  double tolerance_;
  bool allow_unknown_;
  bool priormap_use_nav2_global_search_{true};
  bool priormap_clip_seed_by_rog_boundary_{true};
  bool exploration_prefer_goal_direction_{true};
  bool exploration_use_observed_frontiers_{false};
  double priormap_rog_boundary_margin_{0.8};
  double priormap_rog_boundary_sample_step_{0.1};
  double exploration_boundary_margin_{0.8};
  double exploration_boundary_sample_step_{0.1};
  double opt_freq_;
  double path_los_sampling_ratio_{0.5};
  double path_seed_spacing_{0.45};
  double replan_improvement_ratio_{0.05};
  double replan_check_period_sec_{0.5};
  // 默认关闭 better-path：路径安全就沿当前路径走，只有 PATH_BLOCKED 才换路。
  bool better_path_enable_{false};
  double min_switch_interval_{1.0};
  double terminal_radius_{0.8};
  // 最终 /plan 的弧长均匀采样间距（旧 fastlio_nav2 的 output_spacing）。
  double output_spacing_{0.05};
  // PATH_BLOCKED 校验范围：只校验 [max(0, s_robot - margin), end]，绝不整条重查已走过的几何。
  double path_validation_lookback_margin_{0.2};
  // 默认 false：地图外按 UNKNOWN 处理（冻结的乐观规划语义）。true 时把 OUT_OF_MAP 当成阻塞原因上报。
  bool path_validation_block_out_of_map_{false};
  // MINCO 起止边界构造模式：ODOM（当前行为）| OLD_COMPAT（严格复刻旧 fastlio_nav2）。
  std::string boundary_mode_{"ODOM"};
  // OLD terrain_minco_planner.cpp:819-821 的最短段时间；用于复刻旧版的边界速度幅值。
  double old_compat_min_segment_time_{0.05};
  // 条件式 warm start：同一 goal + 旧轨迹仍可用时，用上一条 trajectory 的剩余段当 MINCO 初值。
  bool warm_start_enable_{true};
  std::atomic<bool> force_cold_start_{false};
  double lookahead_dist_;
  double traj_goal_tolerance_{0.5};
  MincoOptimizer::Config minco_config;

  // === Core Modules (Pointers to FSM, Optimizers, etc.) ===
  std::unique_ptr<Astar> astar_planner_;
  std::unique_ptr<MincoOptimizer> minco_optimizer_;
  std::unique_ptr<PlannerModeContext> mode_context_;
  std::unique_ptr<GlobalPathSearcher> global_path_searcher_;
  std::unique_ptr<LocalPathProcessor> local_path_processor_;
  std::unique_ptr<TrajectorySafetyChecker> safety_checker_;
  std::unique_ptr<MincoFsm> fsm_;
  SimpleCorridorGenerator::Ptr corridor_gen_;
  Ptr planner_handle_;

  // === State Variables & Caches ===
  geometry_utils::Trajectory last_traj_;
  std::vector<geometry_msgs::msg::PoseStamped> latest_global_path_;
  nav_msgs::msg::Odometry latest_odom_;
  geometry_msgs::msg::PoseStamped pending_goal_;

  bool has_last_traj_ = false;
  bool has_pending_goal_{false};
  bool has_latest_odom_{false};
  std::atomic_bool is_traj_safe_{true};
  std::atomic_bool active_{false};

  std::mutex path_mutex_;
  std::mutex goal_mutex_;
  mutable std::mutex odom_mutex_;
  mutable std::mutex mutex_;

  std::unique_ptr<Visualizer> visualizer_;
  rclcpp::Logger logger_{rclcpp::get_logger("MincoPlanner")};
  rclcpp::Logger path_logger_{rclcpp::get_logger("navigation.path")};
  std::string last_validation_failure_reason_{"KINEMATIC_VIOLATION"};
  std::string last_search_failure_reason_;
};

}  // namespace minco_planner

#endif  // MINCO_PLANNER__MINCO_PLANNER_HPP_
