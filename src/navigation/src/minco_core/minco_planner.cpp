// Corresponding header
#include "minco_core/minco_planner.hpp"
#include "nav2_util/node_utils.hpp"

// Project
#include "rog_map/map_registry.hpp"
#include "rog_map/query_adapter.hpp"
#include "rog_map_ros/rog_map_ros2.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <iostream>

namespace minco_planner {

using namespace color_text;

MincoPlanner::MincoPlanner() : tf_(nullptr)
{
}

namespace {
// 优化器障碍距离相对硬净空额外保留的余量。
constexpr double kObstacleClearanceMargin = 0.05;
}  // namespace

// 优化器实际使用的障碍距离：不少于校验器的硬净空 + 余量，避免“优化器贴边、校验器拒绝”。
double MincoPlanner::optimizerObstacleDistance() const
{
  const double hard_clearance = requiredObstacleClearance();
  if (hard_clearance <= 0.0) {
    return minco_config.safe_dist;
  }
  const double required = hard_clearance + kObstacleClearanceMargin;
  if (minco_config.safe_dist + 1e-9 < required) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000,
      "[MincoPlanner] minco_optimizer.safe_dist=%.3f < terrain footprint clearance %.3f + margin "
      "%.3f; using %.3f as the optimizer distance bound.",
      minco_config.safe_dist, hard_clearance, kObstacleClearanceMargin, required);
    return required;
  }
  return minco_config.safe_dist;
}

// 校验器实际使用的硬净空半径：与 QueryAdapter::footprintFree 的算法保持一致
// （外接圆 + clearance_margin + 半个体素对角）。
double MincoPlanner::requiredObstacleClearance() const
{
  // configure 早期 mode_context_ 可能还没完成装配，按 dynamic → raw → map 依次回退。
  auto query = mode_context_ ? mode_context_->dynamicQuery() : nullptr;
  if (!query) {
    query = rog_query_raw_;
  }
  if (!query) {
    query = map_;
  }
  if (!query) {
    return 0.0;
  }
  const auto policy = query->terrainPolicy();
  if (!policy.enable) {
    return 0.0;
  }
  const double radius = policy.yaw_independent_footprint ?
    std::max(policy.flat_clearance_radius,
      std::hypot(policy.terrain_half_length, policy.terrain_half_width)) :
    policy.flat_clearance_radius;
  return radius + policy.clearance_margin + 0.7071067812 * query->resolution();
}

MincoPlanner::~MincoPlanner()
{
  // 析构发生在容器停止 spin 之后，此时释放 ROS 实体是安全的。
  releaseRosEntities();
}

bool MincoPlanner::configureRogMap(
  const nav2_util::LifecycleNode::SharedPtr & node, const std::string & plugin_prefix)
{
  if (!node) {
    RCLCPP_ERROR(logger_, "[MincoPlanner] Cannot configure ROGMap without planner_server LifecycleNode.");
    return false;
  }

  rog_map::Config rog_cfg;
  try {
    rog_cfg.loadFromRosNode(node, plugin_prefix + "rog_map");
    if (rog_cfg.load_pcd_en || rog_cfg.prior_map_enable) {
      throw std::invalid_argument("Online planning does not accept a PCD or prior-map input.");
    }
    rog_map_ros_ = std::make_shared<rog_map::ROGMapROS>(node, rog_cfg, tf_);
    // planner 统一查询：优先用世界固定的全局二维地形图，局部滚动 ROGMap 只提供观测。
    rog_query_raw_ = rog_map_ros_->globalQueryInterface();
    if (!rog_query_raw_) {
      rog_query_raw_ = rog_map_ros_->queryInterface();
      RCLCPP_WARN(logger_,
        "[MincoPlanner] global_map.enable=false: planner falls back to the rolling local ROGMap query.");
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "[MincoPlanner] Failed to configure ROGMap: %s", e.what());
    rog_map_ros_.reset();
    rog_query_raw_.reset();
    map_.reset();
    if (rog_cfg.prior_map_enable) {
      throw;
    }
    return false;
  }

  if (!rog_query_raw_) {
    RCLCPP_ERROR(logger_, "[MincoPlanner] ROGMap queryInterface is null.");
    rog_map_ros_.reset();
    return false;
  }

  rog_map::MapRegistry::set(rog_query_raw_);
  return true;
}

bool MincoPlanner::ensureMapAvailable()
{
  if (rog_query_raw_ || map_) {
    return true;
  }

  auto map = rog_map::MapRegistry::get();
  if (map) {
    rog_query_raw_ = map;
    if (!map_) {
      map_ = map;
    }
    return true;
  }

  auto node = node_.lock();
  if (node) {
    RCLCPP_ERROR_THROTTLE(logger_,
      *node->get_clock(),
      1000,
      "[MincoPlanner] MapQueryInterface unavailable: ROGMap was not created and MapRegistry is empty.");
  } else {
    RCLCPP_ERROR(logger_, "[MincoPlanner] MapQueryInterface unavailable.");
  }
  return false;
}

void MincoPlanner::rebuildModeDependentQueries()
{
  if (!mode_context_) {
    return;
  }

  mode_context_->rebuildQueries(rog_query_raw_, costmap_ros_.get(), tf_, clock_, logger_);
  map_ = mode_context_->dynamicQuery();

  if (global_path_searcher_) {
    global_path_searcher_->setQuery(mode_context_->globalQuery());
  }
  if (astar_planner_) {
    astar_planner_->setMap(mode_context_->globalQuery());
    astar_planner_->setESDFQuery(mode_context_->dynamicQuery());
    // 地图快照这时才存在，用真实硬净空刷新 A* 的碰撞距离（见 configure 里的说明）。
    const double clearance = std::max(minco_config.safe_dist,
      requiredObstacleClearance() + kObstacleClearanceMargin);
    astar_planner_->setCollisionDistance(clearance);
  }
  if (minco_optimizer_) {
    minco_optimizer_->setMap(mode_context_->dynamicQuery());
  }
  if (corridor_gen_) {
    corridor_gen_->setMap(mode_context_->dynamicQuery());
  }
  if (safety_checker_) {
    safety_checker_->setQuery(mode_context_->dynamicQuery());
  }

}

void MincoPlanner::initPlannerMode(
  const std::string & planner_mode_param, const std::string & map_frame, const std::string & rog_frame)
{
  mode_params_.planner_mode = planner_mode_param;
  mode_params_.map_frame = map_frame.empty() ? "map" : map_frame;
  mode_params_.rog_frame = rog_frame.empty() ? "camera_init" : rog_frame;
  mode_params_.priormap_use_nav2_global_search = priormap_use_nav2_global_search_;
  mode_params_.priormap_clip_seed_by_rog_boundary = priormap_clip_seed_by_rog_boundary_;
  mode_params_.priormap_rog_boundary_margin = priormap_rog_boundary_margin_;
  mode_params_.priormap_rog_boundary_sample_step = priormap_rog_boundary_sample_step_;
  mode_params_.exploration_boundary_margin = exploration_boundary_margin_;
  mode_params_.exploration_boundary_sample_step = exploration_boundary_sample_step_;
  mode_params_.exploration_prefer_goal_direction = exploration_prefer_goal_direction_;
  mode_params_.exploration_use_observed_frontiers = exploration_use_observed_frontiers_;

  mode_context_ = std::make_unique<PlannerModeContext>();
  mode_context_->configure(mode_params_, rog_query_raw_, costmap_ros_.get(), tf_, clock_, logger_);

  planning_frame_ = mode_context_->planningFrame();
  output_frame_ = mode_context_->outputFrame();
  map_frame_ = mode_context_->mapFrame();
  rog_frame_ = mode_context_->rogFrame();
  global_frame_ = output_frame_;
  map_ = mode_context_->dynamicQuery();

}

// -----------------------------------------------------------------------------
// 2) Lifecycle management
// -----------------------------------------------------------------------------

void MincoPlanner::configure(const nav2_util::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  callback_gate_ = std::make_shared<rog_map::CallbackGate>();
  node_ = parent;
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;

  auto node = parent.lock();
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  const std::string prefix = name_ + ".";
  // --- General config --------------------------------------------------------

  std::string planner_mode_param = "PRIORMAP";
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "planner_mode", rclcpp::ParameterValue(planner_mode_param));
  node->get_parameter(prefix + "planner_mode", planner_mode_param);

  std::string configured_map_frame = "map";
  std::string configured_rog_frame = "camera_init";
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "frames.map_frame", rclcpp::ParameterValue(configured_map_frame));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "frames.rog_frame", rclcpp::ParameterValue(configured_rog_frame));
  node->get_parameter(prefix + "frames.map_frame", configured_map_frame);
  node->get_parameter(prefix + "frames.rog_frame", configured_rog_frame);
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "frames.robot_frame", rclcpp::ParameterValue(robot_frame_));
  node->get_parameter(prefix + "frames.robot_frame", robot_frame_);
  if (robot_frame_.empty()) { throw std::invalid_argument("frames.robot_frame must not be empty"); }

  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "priormap.use_nav2_global_search", rclcpp::ParameterValue(true));
  node->get_parameter(prefix + "priormap.use_nav2_global_search", priormap_use_nav2_global_search_);

  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "priormap.clip_seed_by_rog_boundary", rclcpp::ParameterValue(true));
  node->get_parameter(prefix + "priormap.clip_seed_by_rog_boundary", priormap_clip_seed_by_rog_boundary_);

  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "priormap.rog_boundary_margin", rclcpp::ParameterValue(0.8));
  node->get_parameter(prefix + "priormap.rog_boundary_margin", priormap_rog_boundary_margin_);

  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "priormap.rog_boundary_sample_step", rclcpp::ParameterValue(0.1));
  node->get_parameter(prefix + "priormap.rog_boundary_sample_step", priormap_rog_boundary_sample_step_);

  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "exploration.boundary_margin", rclcpp::ParameterValue(0.8));
  node->get_parameter(prefix + "exploration.boundary_margin", exploration_boundary_margin_);

  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "exploration.boundary_sample_step", rclcpp::ParameterValue(0.1));
  node->get_parameter(prefix + "exploration.boundary_sample_step", exploration_boundary_sample_step_);

  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "replan.improvement_ratio", rclcpp::ParameterValue(0.05));
  node->get_parameter(prefix + "replan.improvement_ratio", replan_improvement_ratio_);
  if (!std::isfinite(replan_improvement_ratio_) || replan_improvement_ratio_ < 0.0 ||
      replan_improvement_ratio_ >= 1.0) {
    RCLCPP_WARN(node->get_logger(),
      "[MincoPlanner] replan.improvement_ratio must be in [0, 1); using 0.05");
    replan_improvement_ratio_ = 0.05;
  }
  // 旧的 replan.check_period_sec 先读，作为 planner.nominal_replan_hz 的默认值。
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "replan.check_period_sec", rclcpp::ParameterValue(0.5));
  node->get_parameter(prefix + "replan.check_period_sec", replan_check_period_sec_);
  // 规划 tick 频率（检查/候选规划频率，不是“每 tick 必换路径”）。
  // 地图可以 20 Hz 更新，但 planner 只在 nominal_replan_hz 上检查一次。
  double nominal_replan_hz = 1.0 / std::max(1e-3, replan_check_period_sec_);
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "planner.nominal_replan_hz", rclcpp::ParameterValue(nominal_replan_hz));
  node->get_parameter(prefix + "planner.nominal_replan_hz", nominal_replan_hz);
  if (std::isfinite(nominal_replan_hz) && nominal_replan_hz > 0.0) {
    replan_check_period_sec_ = 1.0 / nominal_replan_hz;
  } else {
    RCLCPP_WARN(logger_, "[MincoPlanner] planner.nominal_replan_hz invalid; keeping %.3f s period",
      replan_check_period_sec_);
  }

  // 路径复用：默认关闭 better-path，只保留 PATH_BLOCKED 立即重规划。
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "path_reuse.better_path_enable", rclcpp::ParameterValue(false));
  node->get_parameter(prefix + "path_reuse.better_path_enable", better_path_enable_);
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "path_reuse.min_switch_interval", rclcpp::ParameterValue(1.0));
  node->get_parameter(prefix + "path_reuse.min_switch_interval", min_switch_interval_);
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "path_reuse.input_unavailable_grace_sec", rclcpp::ParameterValue(1.0));
  node->get_parameter(prefix + "path_reuse.input_unavailable_grace_sec", input_unavailable_grace_sec_);
  if (!std::isfinite(input_unavailable_grace_sec_) || input_unavailable_grace_sec_ < 0.0) {
    input_unavailable_grace_sec_ = 1.0;
  }
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "goal_behavior.terminal_radius", rclcpp::ParameterValue(0.8));
  node->get_parameter(prefix + "goal_behavior.terminal_radius", terminal_radius_);
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "path_geometry.output_spacing", rclcpp::ParameterValue(0.05));
  node->get_parameter(prefix + "path_geometry.output_spacing", output_spacing_);
  if (!std::isfinite(output_spacing_) || output_spacing_ <= 0.0) {
    output_spacing_ = 0.05;
  }
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "planner.warm_start_enable", rclcpp::ParameterValue(true));
  node->get_parameter(prefix + "planner.warm_start_enable", warm_start_enable_);
  if (!std::isfinite(min_switch_interval_) || min_switch_interval_ < 0.0) {
    min_switch_interval_ = 1.0;
  }
  if (!std::isfinite(terminal_radius_) || terminal_radius_ < 0.0) {
    terminal_radius_ = 0.8;
  }
  if (!std::isfinite(replan_check_period_sec_) || replan_check_period_sec_ <= 0.0) {
    RCLCPP_WARN(node->get_logger(),
      "[MincoPlanner] replan.check_period_sec must be positive; using 0.5");
    replan_check_period_sec_ = 0.5;
  }

  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "exploration.prefer_goal_direction", rclcpp::ParameterValue(true));
  node->get_parameter(prefix + "exploration.prefer_goal_direction", exploration_prefer_goal_direction_);

  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "exploration.use_observed_frontiers", rclcpp::ParameterValue(false));
  node->get_parameter(prefix + "exploration.use_observed_frontiers", exploration_use_observed_frontiers_);
  nav2_util::declare_parameter_if_not_declared(node, prefix + "external_odom_timeout", rclcpp::ParameterValue(0.5));
  node->get_parameter(prefix + "external_odom_timeout", external_odom_timeout_);
  nav2_util::declare_parameter_if_not_declared(node, prefix + "external_map_timeout", rclcpp::ParameterValue(1.0));
  node->get_parameter(prefix + "external_map_timeout", external_map_timeout_);
  if (!std::isfinite(external_map_timeout_) || external_map_timeout_ <= 0.0) {
    throw std::invalid_argument("external_map_timeout must be finite and positive.");
  }
  if (!std::isfinite(external_odom_timeout_) || external_odom_timeout_ <= 0.0) {
    throw std::invalid_argument("external_odom_timeout must be finite and positive.");
  }

  std::string configured_global_frame = "map";
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "global_frame", rclcpp::ParameterValue(configured_global_frame));
  node->get_parameter(prefix + "global_frame", configured_global_frame);
  global_frame_ = costmap_ros_ ? costmap_ros_->getGlobalFrameID() : configured_global_frame;
  if (!configureRogMap(node, prefix)) {
    throw std::runtime_error("online planning requires a configured ROGMap.");
  }
  initPlannerMode(planner_mode_param, configured_map_frame, configured_rog_frame);
  if (mode_context_->mode() != PlannerMode::EXPLORATION) {
    throw std::invalid_argument("online planning requires planner_mode=EXPLORATION.");
  }

  nav2_util::declare_parameter_if_not_declared(node, prefix + "tolerance", rclcpp::ParameterValue(0.5));
  node->get_parameter(prefix + "tolerance", tolerance_);

  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "allow_unknown", rclcpp::ParameterValue(true));
  node->get_parameter(prefix + "allow_unknown", allow_unknown_);

  // Odometry topic
  std::string odom_topic = "/odom";
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "odom_topic", rclcpp::ParameterValue(odom_topic));
  node->get_parameter(prefix + "odom_topic", odom_topic);

  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "minco_optimizer.opt_freq", rclcpp::ParameterValue(20.0));
  node->get_parameter(prefix + "minco_optimizer.opt_freq", opt_freq_);

  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "path_geometry.los_sampling_ratio", rclcpp::ParameterValue(0.5));
  node->get_parameter(prefix + "path_geometry.los_sampling_ratio", path_los_sampling_ratio_);
  if (!std::isfinite(path_los_sampling_ratio_) || path_los_sampling_ratio_ <= 0.0) {
    path_los_sampling_ratio_ = 0.5;
  }
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "path_geometry.seed_spacing", rclcpp::ParameterValue(0.45));
  node->get_parameter(prefix + "path_geometry.seed_spacing", path_seed_spacing_);
  if (!std::isfinite(path_seed_spacing_) || path_seed_spacing_ <= 0.0) {
    path_seed_spacing_ = 0.45;
  }

  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "minco_optimizer.lookahead_dist", rclcpp::ParameterValue(5.0));
  node->get_parameter(prefix + "minco_optimizer.lookahead_dist", lookahead_dist_);

  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "minco_optimizer.traj_goal_tolerance", rclcpp::ParameterValue(0.3));
  node->get_parameter(prefix + "minco_optimizer.traj_goal_tolerance", traj_goal_tolerance_);

  // --- Optimizer config ------------------------------------------------------

  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "minco_optimizer.safe_dist", rclcpp::ParameterValue(0.3));
  node->get_parameter(prefix + "minco_optimizer.safe_dist", minco_config.safe_dist);

  double collision_dist = minco_config.safe_dist;
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "minco_optimizer.collision_dist", rclcpp::ParameterValue(collision_dist));
  node->get_parameter(prefix + "minco_optimizer.collision_dist", collision_dist);

  auto load_dynamics = [&](const std::string & key, double & value) {
    nav2_util::declare_parameter_if_not_declared(node, prefix + "dynamics." + key,
      rclcpp::ParameterValue(value));
    node->get_parameter(prefix + "dynamics." + key, value);
  };
  load_dynamics("max_vel", dynamics_.max_vel);
  load_dynamics("max_acc", dynamics_.max_acc);
  load_dynamics("max_jerk", dynamics_.max_jerk);
  load_dynamics("planning_vel", dynamics_.planning_vel);
  load_dynamics("optimizer_margin", dynamics_.optimizer_margin);
  load_dynamics("validation_tolerance", dynamics_.validation_tolerance);
  dynamics_.validate();
  // max_vel 是硬件/校验硬上限；MINCO 实际规划用的巡航速度是 planning_vel。
  // 用 10 m/s 级别的软件上限直接规划会得到 ~0.8 s 的短时轨迹：位置罚项按 ∫dt 计权，
  // 时间越短障碍罚项越弱，优化器会切角贴边，validator 整条拒绝（Pos 违规 0.29 m）。
  minco_config.max_vel = std::min(dynamics_.max_vel, dynamics_.planning_vel) * dynamics_.optimizer_margin;
  minco_config.max_acc = dynamics_.max_acc * dynamics_.optimizer_margin;
  minco_config.max_jerk = dynamics_.max_jerk * dynamics_.optimizer_margin;

  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "minco_optimizer.turn_angle_deadzone", rclcpp::ParameterValue(0.174));
  node->get_parameter(prefix + "minco_optimizer.turn_angle_deadzone", minco_config.turn_angle_deadzone);

  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "minco_optimizer.turn_angle_saturation", rclcpp::ParameterValue(1.57));
  node->get_parameter(prefix + "minco_optimizer.turn_angle_saturation", minco_config.turn_angle_saturation);

  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "minco_optimizer.min_turn_vel", rclcpp::ParameterValue(1.0));
  node->get_parameter(prefix + "minco_optimizer.min_turn_vel", minco_config.min_turn_vel);

  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "minco_optimizer.decay_power", rclcpp::ParameterValue(2.0));
  node->get_parameter(prefix + "minco_optimizer.decay_power", minco_config.decay_power);

  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "minco_optimizer.time_allocation_iters", rclcpp::ParameterValue(15));
  node->get_parameter(prefix + "minco_optimizer.time_allocation_iters", minco_config.time_allocation_iters);

  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "minco_optimizer.penalty_weight_time", rclcpp::ParameterValue(0.01));
  node->get_parameter(prefix + "minco_optimizer.penalty_weight_time", minco_config.rho);

  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "minco_optimizer.smooth_eps", rclcpp::ParameterValue(0.01));
  node->get_parameter(prefix + "minco_optimizer.smooth_eps", minco_config.smooth_eps);

  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "minco_optimizer.integral_res", rclcpp::ParameterValue(16));
  node->get_parameter(prefix + "minco_optimizer.integral_res", minco_config.integral_res);

  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "minco_optimizer.opt_accuracy", rclcpp::ParameterValue(1.0e-4));
  node->get_parameter(prefix + "minco_optimizer.opt_accuracy", minco_config.opt_accuracy);

  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "minco_optimizer.print_optimizer_log", rclcpp::ParameterValue(true));
  node->get_parameter(prefix + "minco_optimizer.print_optimizer_log", minco_config.print_optimizer_log);

  double penalty_weight_pos = 0.0;
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "minco_optimizer.penalty_weight_pos", rclcpp::ParameterValue(1000.0));
  node->get_parameter(prefix + "minco_optimizer.penalty_weight_pos", penalty_weight_pos);

  double penalty_weight_vel = 0.0;
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "minco_optimizer.penalty_weight_vel", rclcpp::ParameterValue(1000.0));
  node->get_parameter(prefix + "minco_optimizer.penalty_weight_vel", penalty_weight_vel);

  double penalty_weight_acc = 0.0;
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "minco_optimizer.penalty_weight_acc", rclcpp::ParameterValue(10000.0));
  node->get_parameter(prefix + "minco_optimizer.penalty_weight_acc", penalty_weight_acc);

  double penalty_weight_att = 0.0;
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "minco_optimizer.penalty_weight_att", rclcpp::ParameterValue(1000.0));
  node->get_parameter(prefix + "minco_optimizer.penalty_weight_att", penalty_weight_att);

  double penalty_weight_jerk = 1500.0;
  nav2_util::declare_parameter_if_not_declared(node, prefix + "minco_optimizer.penalty_weight_jerk",
    rclcpp::ParameterValue(penalty_weight_jerk));
  node->get_parameter(prefix + "minco_optimizer.penalty_weight_jerk", penalty_weight_jerk);
  minco_config.penaltyWeights.resize(5);
  minco_config.penaltyWeights(0) = penalty_weight_pos;
  minco_config.penaltyWeights(1) = penalty_weight_vel;
  minco_config.penaltyWeights(2) = penalty_weight_acc;
  minco_config.penaltyWeights(3) = penalty_weight_att;
  minco_config.penaltyWeights(4) = penalty_weight_jerk;

  minco_config.magnitudeBounds.resize(4);
  minco_config.magnitudeBounds(0) = optimizerObstacleDistance();
  minco_config.magnitudeBounds(1) = minco_config.max_vel;
  minco_config.magnitudeBounds(2) = minco_config.max_acc;
  minco_config.magnitudeBounds(3) = minco_config.max_jerk;

  // --- Seed 方向软正则（OLD fastlio_nav2 的 directed_regularity，默认 OFF） -----
  // OLD 权重/阈值：directed_regularity=1000.0、directed_cosine_min=0.1
  // （fastlio_nav2 .../minco_optimizer.hpp:38,72 与 terrain_minco_planner.cpp:829,837）。
  {
    auto & dir_cfg = minco_config.direction_regularization;
    nav2_util::declare_parameter_if_not_declared(node,
      prefix + "minco_optimizer.direction_regularization.enable",
      rclcpp::ParameterValue(dir_cfg.enable));
    node->get_parameter(prefix + "minco_optimizer.direction_regularization.enable", dir_cfg.enable);

    nav2_util::declare_parameter_if_not_declared(node,
      prefix + "minco_optimizer.direction_regularization.weight",
      rclcpp::ParameterValue(dir_cfg.weight));
    node->get_parameter(prefix + "minco_optimizer.direction_regularization.weight", dir_cfg.weight);

    nav2_util::declare_parameter_if_not_declared(node,
      prefix + "minco_optimizer.direction_regularization.cos_min",
      rclcpp::ParameterValue(dir_cfg.cos_min));
    node->get_parameter(
      prefix + "minco_optimizer.direction_regularization.cos_min", dir_cfg.cos_min);

    nav2_util::declare_parameter_if_not_declared(node,
      prefix + "minco_optimizer.direction_regularization.tangent_regularization",
      rclcpp::ParameterValue(dir_cfg.tangent_regularization));
    node->get_parameter(prefix + "minco_optimizer.direction_regularization.tangent_regularization",
      dir_cfg.tangent_regularization);
  }

  // --- MINCO 边界模式（ODOM | OLD_COMPAT，默认 ODOM 保持现有行为） --------------
  {
    nav2_util::declare_parameter_if_not_declared(
      node, prefix + "minco_optimizer.boundary_mode", rclcpp::ParameterValue(boundary_mode_));
    node->get_parameter(prefix + "minco_optimizer.boundary_mode", boundary_mode_);
    for (auto & ch : boundary_mode_) {
      ch = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
    }
    if (boundary_mode_ != "ODOM" && boundary_mode_ != "OLD_COMPAT") {
      RCLCPP_WARN(logger_,
        "[MincoPlanner] minco_optimizer.boundary_mode='%s' is unknown; falling back to ODOM",
        boundary_mode_.c_str());
      boundary_mode_ = "ODOM";
    }
  }

  // --- PATH_BLOCKED 校验范围：只校验剩余路径（带很小的回看余量） ----------------
  nav2_util::declare_parameter_if_not_declared(node,
    prefix + "minco_optimizer.path_validation.lookback_margin",
    rclcpp::ParameterValue(path_validation_lookback_margin_));
  node->get_parameter(
    prefix + "minco_optimizer.path_validation.lookback_margin", path_validation_lookback_margin_);
  if (!std::isfinite(path_validation_lookback_margin_) || path_validation_lookback_margin_ < 0.0) {
    path_validation_lookback_margin_ = 0.2;
  }
  nav2_util::declare_parameter_if_not_declared(node,
    prefix + "minco_optimizer.path_validation.block_out_of_map",
    rclcpp::ParameterValue(path_validation_block_out_of_map_));
  node->get_parameter(
    prefix + "minco_optimizer.path_validation.block_out_of_map", path_validation_block_out_of_map_);

  // --- Corridor config -------------------------------------------------------

  double corridor_robot_radius = 0.4;
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "corridor.robot_radius", rclcpp::ParameterValue(corridor_robot_radius));
  node->get_parameter(prefix + "corridor.robot_radius", corridor_robot_radius);

  double corridor_extra_margin = 0.15;
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "corridor.extra_margin", rclcpp::ParameterValue(corridor_extra_margin));
  node->get_parameter(prefix + "corridor.extra_margin", corridor_extra_margin);

  // --- Components / publishers / timers -------------------------------------

  const auto global_query = mode_context_ ? mode_context_->globalQuery() : nullptr;
  const auto dynamic_query = mode_context_ ? mode_context_->dynamicQuery() : nullptr;
  const unsigned int init_size_x = global_query ? global_query->sizeX() : 1U;
  const unsigned int init_size_y = global_query ? global_query->sizeY() : 1U;
  // A* 必须和轨迹校验器使用同一个净空口径：校验器拒绝“足印与障碍相交”的轨迹，
  // 因此 A* 也不能贴着障碍走。仅用 collision_dist(0.05 m) 生成的最优折线会贴边，
  // 后续 MINCO 无法把它推回车身净空，validateTrajectory 会整条拒绝。
  // safe_dist 作为下限：地图快照可能在 configure 阶段还没建立，此时硬净空读到 0。
  const double astar_collision_dist = std::max(minco_config.safe_dist,
    requiredObstacleClearance() + kObstacleClearanceMargin);
  astar_planner_ = std::make_unique<Astar>(init_size_x, init_size_y);
  astar_planner_->setMap(global_query);
  astar_planner_->setESDFQuery(dynamic_query);
  astar_planner_->setCollisionDistance(astar_collision_dist);

  global_path_searcher_ = std::make_unique<GlobalPathSearcher>();
  global_path_searcher_->configure(
    tf_, astar_planner_.get(), allow_unknown_, tolerance_, clock_, logger_);
  global_path_searcher_->setQuery(global_query);

  local_path_processor_ = std::make_unique<LocalPathProcessor>();
  local_path_processor_->configure(lookahead_dist_,
    minco_config.max_vel,
    minco_config.max_acc,
    traj_goal_tolerance_,
    path_los_sampling_ratio_,
    path_seed_spacing_,
    clock_,
    logger_);

  safety_checker_ = std::make_unique<TrajectorySafetyChecker>();
  safety_checker_->configure(collision_dist, 0.05, clock_, logger_);
  safety_checker_->setQuery(dynamic_query);

  planning_path_id_ = planning_publish_seq_ = 0;
  last_planning_status_.clear();
  cached_plan_ = nav_msgs::msg::Path{};
  cached_plan_s_.clear();
  cached_progress_s_ = 0.0;
    // KEEP_PATH can publish only once. Late RViz subscribers must receive the current
    // validated reference (or its latest explicit retraction), without replanning.
    const auto result_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    plan_pub_ = rclcpp::create_publisher<nav_msgs::msg::Path>(node, "/plan", result_qos);
    plan_meta_pub_ = rclcpp::create_publisher<navigation::msg::PlanMeta>(
      node, "/terrain_minco/plan_meta", result_qos);
    goal_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", rclcpp::QoS(1), [this, gate = callback_gate_](const geometry_msgs::msg::PoseStamped::SharedPtr goal) { gate->run([&] {
        if (!active_.load() || !goal) { return; }
        geometry_msgs::msg::PoseStamped start;
        if (!getRobotPose(start)) {
          // Store the mission even before the first odometry message.
          start.header.frame_id = planning_frame_;
          start.pose.orientation.w = 1.0;
        }
        createPlan(start, *goal);
      }); });
  cancel_sub_ = node->create_subscription<std_msgs::msg::Bool>(
      "/planning/cancel", rclcpp::QoS(1), [this, gate = callback_gate_](const std_msgs::msg::Bool::SharedPtr cancel) { gate->run([&] {
        if (active_.load() && cancel && cancel->data) {
          cancelGoal();
          publishPlanningStatus("cancelled");
        }
      }); });

  auto odom_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
  odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, odom_qos, [this, gate = callback_gate_](const nav_msgs::msg::Odometry::SharedPtr msg) { gate->run([&] {
      if (!msg) {
        return;
      }

      {
        std::lock_guard<std::mutex> lk(odom_mutex_);
        latest_odom_ = *msg;
        odom_received_ = std::chrono::steady_clock::now();
        has_latest_odom_ = true;
      }

    }); });

  visualizer_ = std::make_unique<Visualizer>();
  visualizer_->configure(parent, output_frame_);

  minco_optimizer_ = std::make_unique<MincoOptimizer>(minco_config);
  minco_optimizer_->setMap(mode_context_ ? mode_context_->dynamicQuery() : nullptr);

  corridor_gen_ = std::make_shared<SimpleCorridorGenerator>();
  corridor_gen_->setMap(mode_context_ ? mode_context_->dynamicQuery() : nullptr);
  corridor_gen_->setSafetyMargins(corridor_robot_radius, corridor_extra_margin);

  // Planner handle for FSM (non-owning; lifetime managed by pluginlib).
  planner_handle_ = MincoPlanner::Ptr(this, [](MincoPlanner *) {
  });

  // FSM 与安全校验都只做“读标志 / 校验剩余路径”，代价很低；把 tick 从 20 Hz 提到 50 Hz，
  // 让 PATH_BLOCKED 的检测与触发重规划的等待从最坏 50 ms 降到最坏 20 ms。
  // 完整 A*→LOS→MINCO 仍然只在真正 PATH_BLOCKED 时执行一次，不是每 tick 空跑。
  constexpr double kTriggerHz = 50.0;
  // High-level FSM @ 50Hz.
  fsm_ = std::make_unique<MincoFsm>(planner_handle_);
  fsm_timer_ = node->create_wall_timer(std::chrono::duration<double>(1.0 / kTriggerHz), [this, gate = callback_gate_]() { gate->run([&] {
    if (active_.load() && fsm_) {
      fsm_->callMainFsmOnce();
    }
  }); });

  // Asynchronous safety monitor @ 50Hz.
  safety_timer_ = node->create_wall_timer(
    std::chrono::duration<double>(1.0 / kTriggerHz), [this, gate = callback_gate_] { gate->run([&] { safetyTimerCallback(); }); });
  fsm_timer_->cancel();
  safety_timer_->cancel();

  on_set_parameters_callback_handle_ = node->add_on_set_parameters_callback(
    [this, gate = callback_gate_](const std::vector<rclcpp::Parameter> & parameters) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = false;
      result.reason = "Planner is shutting down";
      gate->run([&] { result = onSetParameters(parameters); });
      return result;
    });
}

void MincoPlanner::setMap(const std::shared_ptr<rog_map::MapQueryInterface> & map)
{
  rog_query_raw_ = map;
  rebuildModeDependentQueries();
}

void MincoPlanner::activate()
{
  active_.store(true);
  if (fsm_timer_) {
    fsm_timer_->reset();
  }
  if (safety_timer_) {
    safety_timer_->reset();
  }
  if (visualizer_) {
    visualizer_->activate();
  }
}

void MincoPlanner::deactivate()
{
  active_.store(false);
  ++pending_goal_generation_;
  publishPlanningStatus("DEACTIVATED");
  if (fsm_timer_) {
    fsm_timer_->cancel();
  }
  if (safety_timer_) {
    safety_timer_->cancel();
  }
  if (visualizer_) {
    visualizer_->deactivate();
  }
}

void MincoPlanner::cleanup()
{
  // 只停回调、取消定时器，绝不在这里析构 ROS 实体。
  // 本函数由 lifecycle manager 在 executor 回调里调用，而 MultiThreadedExecutor 线程可能
  // 正在 rmw_wait() 里持有这些订阅/定时器的 waitset；此时析构它们会造成 use-after-free
  // （实测容器退出 exit -11，gdb 栈停在 rmw_wait）。
  // 实体释放统一放在 releaseRosEntities()，只由析构函数调用——那时 executor 已停止。
  active_.store(false);
  ++pending_goal_generation_;
  callback_gate_->stop();
  if (fsm_timer_) { fsm_timer_->cancel(); }
  if (safety_timer_) { safety_timer_->cancel(); }
  if (rog_map_ros_) { rog_map_ros_->shutdown(); }
  if (visualizer_) { visualizer_->deactivate(); }
}

void MincoPlanner::releaseRosEntities()
{
  callback_gate_->stop();

  on_set_parameters_callback_handle_.reset();

  fsm_timer_.reset();
  safety_timer_.reset();

  fsm_.reset();
  planner_handle_.reset();

  if (visualizer_) {
    visualizer_->cleanup();
    visualizer_.reset();
  }

  astar_planner_.reset();
  global_path_searcher_.reset();
  local_path_processor_.reset();
  safety_checker_.reset();
  mode_context_.reset();
  minco_optimizer_.reset();
  corridor_gen_.reset();
  goal_sub_.reset();
  cancel_sub_.reset();
  plan_pub_.reset();
  plan_meta_pub_.reset();
  odom_sub_.reset();
  costmap_ros_.reset();
  rog_map::MapRegistry::set(nullptr);
  map_.reset();
  rog_query_raw_.reset();
  rog_map_ros_.reset();
}

rcl_interfaces::msg::SetParametersResult MincoPlanner::onSetParameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  const std::string planner_mode_param = name_ + ".planner_mode";
  const auto is_configure_time_mode_param = [this, &planner_mode_param](const std::string & param_name) {
    return param_name.rfind(name_ + ".dynamics.", 0) == 0 ||
           param_name == name_ + ".minco_optimizer.max_velocity" ||
           param_name == name_ + ".minco_optimizer.max_acceleration" ||
           param_name == planner_mode_param || param_name == name_ + ".frames.map_frame" ||
           param_name == name_ + ".frames.rog_frame" ||
           param_name == name_ + ".frames.robot_frame" ||
           param_name == name_ + ".priormap.use_nav2_global_search" ||
           param_name == name_ + ".priormap.clip_seed_by_rog_boundary" ||
           param_name == name_ + ".priormap.rog_boundary_margin" ||
           param_name == name_ + ".priormap.rog_boundary_sample_step" ||
           param_name == name_ + ".exploration.boundary_margin" ||
           param_name == name_ + ".exploration.boundary_sample_step" ||
           param_name == name_ + ".exploration.prefer_goal_direction" ||
           param_name == name_ + ".exploration.use_observed_frontiers" ||
           param_name == name_ + ".external_mpc_mode" ||
           param_name == name_ + ".external_odom_timeout" ||
           param_name == name_ + ".external_map_timeout";
  };
  const std::string penalty_pos_param = name_ + ".minco_optimizer.penalty_weight_pos";
  const std::string penalty_vel_param = name_ + ".minco_optimizer.penalty_weight_vel";
  const std::string penalty_acc_param = name_ + ".minco_optimizer.penalty_weight_acc";
  const std::string penalty_att_param = name_ + ".minco_optimizer.penalty_weight_att";
  double next_penalty_pos =
    (minco_config.penaltyWeights.size() > 0) ? minco_config.penaltyWeights(0) : 1000.0;
  double next_penalty_vel =
    (minco_config.penaltyWeights.size() > 1) ? minco_config.penaltyWeights(1) : 1000.0;
  double next_penalty_acc =
    (minco_config.penaltyWeights.size() > 2) ? minco_config.penaltyWeights(2) : 10000.0;
  double next_penalty_att =
    (minco_config.penaltyWeights.size() > 3) ? minco_config.penaltyWeights(3) : 1000.0;
  bool optimizer_config_changed = false;

  for (const auto & param : parameters) {
    const auto & param_name = param.get_name();

    auto parse_numeric = [&](double & out) -> bool {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        out = param.as_double();
        return true;
      }
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        out = static_cast<double>(param.as_int());
        return true;
      }
      return false;
    };

    if (is_configure_time_mode_param(param_name)) {
      result.successful = false;
      result.reason =
        "Planner mode/frame parameters are configure-time only; restart planner_server to apply.";
      RCLCPP_ERROR(logger_, "[MincoPlanner] %s", result.reason.c_str());
      return result;
    }

    if (param_name == penalty_pos_param || param_name == penalty_vel_param ||
        param_name == penalty_acc_param || param_name == penalty_att_param) {
      double candidate = 0;
      if (!parse_numeric(candidate) || !std::isfinite(candidate) || candidate < 0) {
        result.successful = false;
        result.reason = "Penalty weight must be finite and nonnegative: " + param_name;
        return result;
      }
      if (param_name == penalty_pos_param) {
        next_penalty_pos = candidate;
      } else if (param_name == penalty_vel_param) {
        next_penalty_vel = candidate;
      } else if (param_name == penalty_acc_param) {
        next_penalty_acc = candidate;
      } else {
        next_penalty_att = candidate;
      }

      optimizer_config_changed = true;
      continue;
    }

    if (param_name == name_ + ".minco_optimizer.boundary_mode") {
      if (param.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
        result.successful = false;
        result.reason = "minco_optimizer.boundary_mode must be a string (ODOM | OLD_COMPAT)";
        return result;
      }
      std::string mode = param.as_string();
      for (auto & ch : mode) {
        ch = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
      }
      if (mode != "ODOM" && mode != "OLD_COMPAT") {
        result.successful = false;
        result.reason = "minco_optimizer.boundary_mode must be ODOM or OLD_COMPAT";
        return result;
      }
      boundary_mode_ = mode;
      continue;
    }
  }

  if (optimizer_config_changed) {

    minco_config.penaltyWeights.conservativeResize(5);
    minco_config.penaltyWeights(0) = next_penalty_pos;
    minco_config.penaltyWeights(1) = next_penalty_vel;
    minco_config.penaltyWeights(2) = next_penalty_acc;
    minco_config.penaltyWeights(3) = next_penalty_att;

    minco_config.magnitudeBounds.resize(4);
    minco_config.magnitudeBounds(0) = optimizerObstacleDistance();
    minco_config.magnitudeBounds(1) = minco_config.max_vel;
    minco_config.magnitudeBounds(2) = minco_config.max_acc;
    minco_config.magnitudeBounds(3) = minco_config.max_jerk;

    if (minco_optimizer_) {
      minco_optimizer_->setConfig(minco_config);
    }
    if (local_path_processor_) {
      local_path_processor_->updateLimits(minco_config.max_vel, minco_config.max_acc, traj_goal_tolerance_);
    }

  }

  return result;
}

// -----------------------------------------------------------------------------
// 3) Core business interface
// -----------------------------------------------------------------------------

bool MincoPlanner::normalizePoseToFrame(const geometry_msgs::msg::PoseStamped & in,
  const std::string & fallback_frame,
  const std::string & target_frame,
  const std::string & context,
  geometry_msgs::msg::PoseStamped & out) const
{
  out = in;
  if (out.header.frame_id.empty()) {
    out.header.frame_id = fallback_frame;
    RCLCPP_WARN(logger_,
      "[MincoPlanner] %s pose frame is empty, treating it as %s.",
      context.c_str(),
      fallback_frame.c_str());
  }

  if (out.header.frame_id == target_frame) {
    out.header.frame_id = target_frame;
    return true;
  }

  if (!tf_) {
    RCLCPP_ERROR(logger_,
      "[MincoPlanner] Cannot transform %s pose from %s to %s: TF buffer is null.",
      context.c_str(),
      out.header.frame_id.c_str(),
      target_frame.c_str());
    return false;
  }

  try {
    out = tf_->transform(out, target_frame);
    out.header.frame_id = target_frame;
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_,
      "[MincoPlanner] Failed to transform %s pose from %s to %s: %s",
      context.c_str(),
      in.header.frame_id.c_str(),
      target_frame.c_str(),
      ex.what());
    return false;
  }
}

nav_msgs::msg::Path MincoPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path result;
  callback_gate_->run([&] { result = createPlanInternal(start, goal); });
  return result;
}

nav_msgs::msg::Path MincoPlanner::createPlanInternal(
  const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal)
{
  // Nav2 interface: createPlan() only sets the goal flag for MincoFSM.
  // It must NOT run A* or optimization here.
  nav_msgs::msg::Path path;
  path.header.stamp = clock_->now();
  path.header.frame_id = output_frame_;

  geometry_msgs::msg::PoseStamped normalized_start;
  geometry_msgs::msg::PoseStamped normalized_goal;
  const bool start_ok =
    normalizePoseToFrame(start, planning_frame_, planning_frame_, "createPlan start", normalized_start);
  const bool goal_ok =
    normalizePoseToFrame(goal, planning_frame_, planning_frame_, "createPlan goal", normalized_goal);
  if (!start_ok || !goal_ok) {
    RCLCPP_ERROR(logger_,
      "[MincoPlanner] Failed to normalize createPlan pose(s) to planning frame %s; reject pending goal.",
      planning_frame_.c_str());
    cancelGoal();
    publishPlanningStatus("GOAL_TF_FAILED");
    return path;
  }

  if (!std::isfinite(normalized_goal.pose.position.x) || !std::isfinite(normalized_goal.pose.position.y)) {
    cancelGoal();
    publishPlanningStatus("NONFINITE_GOAL");
    return path;
  }

  // Keep a minimal path for Nav2 callers (e.g., visualization/debug).
  normalized_start.header = path.header;
  normalized_goal.header = path.header;
  path.poses.push_back(normalized_start);
  path.poses.push_back(normalized_goal);

  {
    std::lock_guard<std::mutex> lk(goal_mutex_);
    ++pending_goal_generation_;
    pending_goal_ = normalized_goal;
    has_pending_goal_ = true;
  }
  RCLCPP_INFO(path_logger_, "[Goal] accepted goal=(%.3f, %.3f) m frame=%s",
    normalized_goal.pose.position.x, normalized_goal.pose.position.y,
    normalized_goal.header.frame_id.c_str());
  publishPlanningStatus("GOAL_UPDATED");
  std::lock_guard<std::mutex> lock(mutex_);
  has_last_traj_ = false;
  // The synchronous Nav2 action must never publish the start/goal stub as /plan.
  // The planning-only entry uses /goal_pose; the validated result arrives asynchronously.
  path.poses.clear();

  return path;
}

bool MincoPlanner::PlanGlobalPath(
  const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal)
{
  const uint64_t goal_generation = pending_goal_generation_.load();
  if (!global_path_searcher_ || !mode_context_) {
    return false;
  }
  std::vector<geometry_msgs::msg::PoseStamped> planned_path;
  if (!global_path_searcher_->plan(
        start, goal, *mode_context_, [this]() { return !active_.load(); }, planned_path)) {
    last_search_failure_reason_ = global_path_searcher_->lastFailureReason();
    return false;
  }
  last_search_failure_reason_.clear();

  bool path_valid = false;
  {
    std::lock_guard<std::mutex> path_lock(path_mutex_);
    if (goal_generation != pending_goal_generation_.load()) { return false; }
    latest_global_path_ = std::move(planned_path);
    global_path_goal_generation_ = goal_generation;
    path_valid = latest_global_path_.size() >= 2U;
  }
  if (local_path_processor_) {
    local_path_processor_->resetProgress();
  }
  return path_valid;
}

bool MincoPlanner::ReplanLocal(const geometry_msgs::msg::PoseStamped & current_pose)
{
  if (!active_.load()) {
    return false;
  }
  auto finish = [](bool success, const std::string &) {
    return success;
  };

  if (!minco_optimizer_ || !mode_context_ || !local_path_processor_) {
    return finish(false, "OPTIMIZER_FAILED");
  }

  // Snapshot the global goal for end-state logic.
  uint64_t goal_generation = 0;
  Eigen::Vector3d global_goal(0.0, 0.0, 0.0);
  std::vector<geometry_msgs::msg::PoseStamped> global_path_snapshot;
  {
    std::lock_guard<std::mutex> lock(path_mutex_);
    if (latest_global_path_.empty()) {
      return finish(false, "OPTIMIZER_FAILED");
    }
    global_path_snapshot = latest_global_path_;
    goal_generation = global_path_goal_generation_;
    global_goal.x() = latest_global_path_.back().pose.position.x;
    global_goal.y() = latest_global_path_.back().pose.position.y;
    global_goal.z() = 0.0;
  }

  const LocalPathSeed seed =
    local_path_processor_->buildSeed(global_path_snapshot, current_pose, *mode_context_);
  if (visualizer_) {
    if (!seed.dense_path.empty()) {
      visualizer_->updateLocalEndPoint(seed.dense_path.back(), seed.local_end_is_goal);
    } else {
      visualizer_->clearLocalEndPoint();
    }
  }
  if (!seed.valid) {
    return finish(false, "COLLISION");
  }
  std::vector<Eigen::Vector3d> sparse_path = seed.sparse_waypoints;
  // LocalPathProcessor 已经算好的 seed 切线（size 与 sparse_waypoints 一致），
  // 下面所有会改写 sparse_path 的地方都必须同步改写这份切线，保证 1:1 对齐。
  std::vector<Eigen::Vector3d> seed_tangents = seed.seed_tangents;

  std_msgs::msg::Header header_msg;
  header_msg.frame_id = output_frame_;
  header_msg.stamp = clock_->now();

  // 4. Determine state (HOT/COLD).
  PlanningState state = PlanningState::COLD_START;
  traj_opt::Trajectory last_traj_snapshot;
  bool has_last_traj_snapshot = false;
  double last_traj_start_WT = 0.0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    state = determinePlanningState(current_pose.pose, sparse_path);
    if (has_last_traj_) {
      last_traj_snapshot = last_traj_;
      has_last_traj_snapshot = true;
      last_traj_start_WT = last_traj_.start_WT;
    }
  }

  // 5. Prepare start state.
  Eigen::Matrix3d start_state;
  vec_Vec3f shifted_waypoints;
  VecDf shifted_durations;
  bool has_shifted_seed = false;
  if (state == PlanningState::HOT_START) {
    const double now = clock_->now().seconds() + 0.005;  // small buffer
    const double t_dur = now - last_traj_start_WT;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      prepareHotStart(current_pose.pose, t_dur, start_state);
    }

    // Extract remaining trajectory segment as shifted warm-start seed.
    if (has_last_traj_snapshot) {
      traj_opt::Trajectory remain;
      const double total = last_traj_snapshot.getTotalDuration();
      if (std::isfinite(t_dur) && t_dur > 0.0 && total > t_dur + 1e-3 &&
          last_traj_snapshot.getPartialTrajectoryByTime(t_dur, total, remain)) {
        shifted_waypoints = remain.getWaypoints();
        shifted_durations = remain.getDurations();
        has_shifted_seed = (!shifted_waypoints.empty() && shifted_durations.size() > 0);
      }
    }
  } else {
    prepareColdStart(current_pose.pose, start_state, sparse_path);
    // Avoid reusing stale warm-start guesses.
    minco_optimizer_->setInitPsAndTs(vec_Vec3f{}, VecDf{});
  }
  // 6. Prepare MINCO optimization.
  traj_opt::Trajectory opt_traj;
  Eigen::Matrix3d end_state;
  end_state.setZero();
  end_state.col(0) = sparse_path.back();

  // Remove near-start redundant points from sparse_path.
  while (sparse_path.size() > 2) {
    if ((sparse_path[1] - start_state.col(0)).norm() < 0.2) {
      sparse_path.erase(sparse_path.begin() + 1);
      if (seed_tangents.size() == sparse_path.size() + 1U) {
        seed_tangents.erase(seed_tangents.begin() + 1);
      } else {
        seed_tangents.clear();  // 对齐失败 → 本帧不启用方向罚（optimize 会再验一次数量）
      }
    } else {
      break;
    }
  }

  // Seed 头部单位切线：OLD_COMPAT 用它构造起始边界方向（OLD makeMincoSeed 的 i==0 分支）。
  Eigen::Vector3d seed_start_tangent = sparse_path[1] - sparse_path[0];
  seed_start_tangent.z() = 0.0;
  if (seed_start_tangent.head<2>().norm() > 1.0e-9) {
    seed_start_tangent.normalize();
  } else {
    seed_start_tangent = Eigen::Vector3d(1.0, 0.0, 0.0);
  }

  Eigen::Vector3d tangent = sparse_path.back() - sparse_path[sparse_path.size() - 2U];
  tangent.z() = 0.0;
  const double tangent_norm = tangent.head<2>().norm();
  if (tangent_norm > 0.1) {
    tangent /= tangent_norm;
  } else {
    tangent = Eigen::Vector3d(1.0, 0.0, 0.0);
  }

  double local_length = 0.0;
  for (size_t i = 1U; i < sparse_path.size(); ++i) {
    local_length += (sparse_path[i] - sparse_path[i - 1U]).head<2>().norm();
  }

  const double global_vmax = std::max(0.0, minco_config.max_vel);
  const double amax = std::max(1e-3, minco_config.max_acc);
  const double v_curr = std::max(0.0, start_state.col(1).head<2>().norm());
  const double v_reachable = std::sqrt(std::max(0.0, v_curr * v_curr + 2.0 * amax * local_length));
  double end_speed = std::min(global_vmax, v_reachable);

  const double dist_to_goal = (end_state.col(0) - global_goal).head<2>().norm();
  const double brake_trigger_dist = global_vmax * global_vmax / (2.0 * amax);
  // The straight-line distance is a lower bound on path length, so only inspect
  // the global path suffix after entering the full-speed braking range.
  if (dist_to_goal < brake_trigger_dist) {
    double remaining_length = 0.0;
    for (size_t i = seed.global_end_index + 1U; i < global_path_snapshot.size(); ++i) {
      const auto & p0 = global_path_snapshot[i - 1U].pose.position;
      const auto & p1 = global_path_snapshot[i].pose.position;
      remaining_length += std::hypot(p1.x - p0.x, p1.y - p0.y);
    }
    const double v_brake = std::sqrt(std::max(0.0, 2.0 * amax * remaining_length));
    end_speed = std::min(end_speed, v_brake);
  }
  end_state.col(1) = tangent * end_speed;

  // 7.5 Initial guess Ps/Ts for optimizer (all cases).
  const int N = static_cast<int>(sparse_path.size()) - 1;
  VecDf local_vmaxs(N);
  if (N > 0) {
    vec_Vec3f init_ps;
    VecDf init_ts(N);
    PTAllocation(sparse_path,
      start_state,
      end_speed,
      state,
      has_shifted_seed,
      shifted_waypoints,
      shifted_durations,
      init_ps,
      init_ts,
      local_vmaxs);

    minco_optimizer_->setInitPsAndTs(init_ps, init_ts);
  }

  // 7.6 Boundary construction per boundary_mode (TASK 5)。上面的 PT 分配在两种模式下完全相同，
  // 这里只改 MINCO 的头/尾 PVA。
  if (boundary_mode_ == "OLD_COMPAT") {
    // OLD fastlio_nav2 源码（terrain_minco_planner.cpp:815-821 的 seed 时长分配 +
    // minco_minjerk.cpp:88-100 的边界装配）：
    //   T_i = max(chord_i, 0.05)
    //   head_velocity = t_hat_head * chord_head / T_0
    //   tail_velocity = t_hat_tail * chord_tail / T_{n-1}
    //   head/tail_acceleration = normal(tangent) * curvature * chord^2 / T^2 = 0
    //     （makeMincoSeed 在 terrain_minco_planner.cpp:533 把 curvature 固定为 0）
    // 因此 seed 处的边界速度幅值 = chord / max(chord, 0.05)，任何 chord>0.05 m 的 seed 都精确等于
    // 1.0 m/s（本工程的 seed_spacing=0.45 m，chord 恒 > 0.05）。这里不写死 1.0，而是照抄 OLD 公式。
    // 已知限制：OLD 的 T_i 在优化中会被联合优化，其边界速度随 T_0 浮动；CURRENT 的 headPVA 是
    // 固定硬约束且时间分配被冻结（本轮禁止改时间分配），无法复刻这种“边界速度-段时间”耦合，
    // 所以取 OLD 在 seed 上的精确取值。其余（地图/start/goal/A*/LOS/seed/PT/目标/动力学/校验）完全一致。
    // OLD head_chord = ||first_internal_waypoint - head.position|| = 第一段 seed 弦长（seed[1]-seed[0]）；
    // tail_chord = ||tail.position - last_internal_waypoint|| = 最后一段 seed 弦长。
    const double chord_head = (sparse_path[1] - sparse_path[0]).head<2>().norm();
    const double chord_tail =
      (sparse_path.back() - sparse_path[sparse_path.size() - 2U]).head<2>().norm();
    const double v_head = chord_head / std::max(chord_head, old_compat_min_segment_time_);
    const double v_tail = chord_tail / std::max(chord_tail, old_compat_min_segment_time_);
    start_state.col(1) = seed_start_tangent * v_head;
    start_state.col(2).setZero();
    end_state.col(1) = tangent * v_tail;
    end_state.col(2).setZero();
  }
  // 8. Optimize.
  auto opt_start_time = clock_->now().seconds();
  // 把同一份 seed 切线交给优化器（不重算；尺寸不匹配时优化器内部整项关闭）。
  minco_optimizer_->setSeedTangents(seed_tangents);
  double final_cost =
    minco_optimizer_->optimize(sparse_path, start_state, end_state, local_vmaxs, opt_traj);

  if (!std::isfinite(final_cost)) {
    if (visualizer_) {
      visualizer_->clearCandidateTrajectory("OPTIMIZER_FAILED");
    }
    return finish(false, "OPTIMIZER_FAILED");
  }

  auto opt_end_time = clock_->now().seconds();
  double opt_duration = opt_end_time - opt_start_time;

  // 8.5 Quality gating (hard validation) before publishing.
  const bool validation_ok = validateTrajectory(opt_traj, end_state.col(0));
  if (visualizer_) {
    visualizer_->updateCandidateTrajectory(opt_traj,
      opt_duration,
      validation_ok,
      validation_ok ? "NONE" : last_validation_failure_reason_);
  }

  if (!validation_ok) {
    is_traj_safe_.store(false);
    return finish(false, last_validation_failure_reason_);
  }

  // 9. Publish and cache.
  std::unique_lock<std::mutex> output_lock(planning_output_mutex_);
  if (!active_.load() || goal_generation != pending_goal_generation_.load()) {
    return finish(false, "GOAL_CHANGED");
  }
  geometry_msgs::msg::PoseStamped fresh_pose;
  if (!getRobotPose(fresh_pose) || std::hypot(
      fresh_pose.pose.position.x - current_pose.pose.position.x,
      fresh_pose.pose.position.y - current_pose.pose.position.y) > 0.15) {
    return finish(false, "STALE_START_STATE");
  }
  header_msg.stamp = clock_->now();
  cached_plan_goal_generation_ = goal_generation;
  publishOptimizedPlan(opt_traj, header_msg);

  if (visualizer_) {
    nav_msgs::msg::Path astar_path_msg;
    {
      std::lock_guard<std::mutex> path_lock(path_mutex_);
      astar_path_msg.header.stamp = clock_->now();
      astar_path_msg.header.frame_id = output_frame_;
      astar_path_msg.poses = latest_global_path_;
    }
    visualizer_->update(sparse_path, opt_traj, opt_duration, astar_path_msg);
  }

  // Publish the position/yaw cache as one coherent snapshot for safety and hot-start readers.
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_traj_ = opt_traj;
    last_traj_.start_WT = rclcpp::Time(header_msg.stamp).seconds();
    has_last_traj_ = true;
  }

  is_traj_safe_.store(true);
  return finish(true, "NONE");
}

void MincoPlanner::PTAllocation(const std::vector<Eigen::Vector3d> & sparse_path,
  const Eigen::Matrix3d & start_state,
  double terminal_speed,
  PlanningState state,
  bool has_shifted_seed,
  const vec_Vec3f & shifted_waypoints,
  const VecDf & shifted_durations,
  vec_Vec3f & init_ps,
  VecDf & init_ts,
  VecDf & local_vmaxs) const
{
  const int N = static_cast<int>(sparse_path.size()) - 1;
  if (N <= 0) {
    init_ps.clear();
    init_ts.resize(0);
    local_vmaxs.resize(0);
    return;
  }

  const double global_vmax = std::max(0.0, minco_config.max_vel);
  const double amax = std::max(1e-3, minco_config.max_acc);
  const double kMinSegTime = 0.1;

  local_vmaxs.resize(N);
  local_vmaxs.setConstant(global_vmax);
  init_ts.resize(N);

  init_ps.clear();
  init_ps.reserve(static_cast<size_t>(std::max(0, N - 1)));
  int copyPs = 0;
  if (state == PlanningState::HOT_START && has_shifted_seed) {
    const int oldWp = static_cast<int>(shifted_waypoints.size());
    const int oldPs = std::max(0, oldWp - 2);
    copyPs = std::min(std::max(0, N - 1), oldPs);
  }
  for (int j = 0; j < copyPs; ++j) {
    init_ps.emplace_back(shifted_waypoints[static_cast<size_t>(j + 1)]);
  }
  for (int j = copyPs; j < (N - 1); ++j) {
    init_ps.emplace_back(sparse_path[static_cast<size_t>(j + 1)]);
  }

  std::vector<double> seg_len(static_cast<size_t>(N), 0.0);
  for (int i = 0; i < N; ++i) {
    const double dis =
      (sparse_path[static_cast<size_t>(i + 1)] - sparse_path[static_cast<size_t>(i)]).head<2>().norm();
    seg_len[static_cast<size_t>(i)] = (std::isfinite(dis) && dis > 0.0) ? dis : 0.0;
  }

  std::vector<double> remain_after(static_cast<size_t>(N), 0.0);
  for (int i = N - 2; i >= 0; --i) {
    remain_after[static_cast<size_t>(i)] =
      remain_after[static_cast<size_t>(i + 1)] + seg_len[static_cast<size_t>(i + 1)];
  }

  std::vector<double> local_vmax_vec(static_cast<size_t>(N), global_vmax);
  for (int i = 0; i < N - 1; ++i) {
    local_vmax_vec[static_cast<size_t>(i)] = utils::LimitLocalVel(sparse_path,
      i,
      global_vmax,
      minco_config.turn_angle_deadzone,
      minco_config.turn_angle_saturation,
      minco_config.min_turn_vel,
      minco_config.decay_power);
  }
  utils::VelPropogation(seg_len, amax, local_vmax_vec);
  for (int i = 0; i < N; ++i) {
    local_vmaxs(i) = local_vmax_vec[static_cast<size_t>(i)];
  }

  double v_curr = start_state.col(1).head<2>().norm();
  if (!std::isfinite(v_curr) || v_curr < 0.0) {
    v_curr = 0.0;
  }

  for (int i = 0; i < N; ++i) {
    const double L = seg_len[static_cast<size_t>(i)];
    const double remain = remain_after[static_cast<size_t>(i)];

    if (L <= 1e-6) {
      init_ts(i) = kMinSegTime;
      continue;
    }

    const double local_vmax = local_vmax_vec[static_cast<size_t>(i)];
    const double v_next =
      utils::ComputeNextSpeed(v_curr, L, remain, amax, local_vmax, terminal_speed);
    init_ts(i) = utils::ComputeSegmentTime(L, v_curr, v_next, local_vmax, amax, kMinSegTime);
    v_curr = v_next;
  }

  if (state == PlanningState::HOT_START && has_shifted_seed) {
    const int oldN = std::min(N, static_cast<int>(shifted_durations.size()));
    for (int i = 0; i < oldN; ++i) {
      const double t_seed = shifted_durations(i);
      if (std::isfinite(t_seed) && t_seed > 0.02) {
        init_ts(i) = std::max(init_ts(i), t_seed);
      }
    }
  }
}

bool MincoPlanner::makePlan(const geometry_msgs::msg::Pose & start,
  const geometry_msgs::msg::Pose & goal,
  double tolerance,
  std::function<bool()> cancel_checker,
  nav_msgs::msg::Path & plan)
{
  if (!global_path_searcher_ || !mode_context_) {
    return false;
  }
  if (!global_path_searcher_->makePlan(start, goal, *mode_context_, tolerance, cancel_checker, plan)) {
    return false;
  }
  {
    std::lock_guard<std::mutex> path_lock(path_mutex_);
    latest_global_path_ = plan.poses;
  }
  if (local_path_processor_) {
    local_path_processor_->resetProgress();
  }
  return true;
}

std::vector<Eigen::Vector3d> MincoPlanner::extractLocalPath(const Eigen::Vector3d & cur_pos)
{
  if (!local_path_processor_ || !mode_context_) {
    return {};
  }
  geometry_msgs::msg::PoseStamped current_pose;
  current_pose.pose.position.x = cur_pos.x();
  current_pose.pose.position.y = cur_pos.y();
  current_pose.pose.position.z = cur_pos.z();
  std::vector<geometry_msgs::msg::PoseStamped> global_path_snapshot;
  {
    std::lock_guard<std::mutex> lock(path_mutex_);
    global_path_snapshot = latest_global_path_;
  }
  return local_path_processor_->buildSeed(global_path_snapshot, current_pose, *mode_context_).dense_path;
}

MincoPlanner::PlanningState MincoPlanner::determinePlanningState(
  const geometry_msgs::msg::Pose & start_pose, const std::vector<Eigen::Vector3d> & new_path)
{
  // external_mpc_mode 与“必须 cold start”没有必然关系：外部 MPC 只决定谁跟踪轨迹，
  // 不决定 MINCO 的初值从哪来。这里改成条件式 warm start：同一 goal、上一条轨迹仍然
  // 可用（时间/跟踪误差/速度误差都在阈值内）才 warm start；否则 cold。
  if (!warm_start_enable_ || force_cold_start_.exchange(false)) {
    return PlanningState::COLD_START;
  }
  if (!has_last_traj_) {
    return PlanningState::COLD_START;
  }

  double now = clock_->now().seconds() + 0.005;
  double t_dur = now - last_traj_.start_WT;
  if (t_dur <= 0.0 || t_dur >= last_traj_.getTotalDuration()) {
    return PlanningState::COLD_START;
  }

  Eigen::Vector3d current_pos(start_pose.position.x, start_pose.position.y, 0.0);
  Eigen::Vector3d pred_pos = last_traj_.getPos(t_dur);
  Eigen::Vector3d pred_vel = last_traj_.getVel(t_dur);
  double tracking_error = (current_pos - pred_pos).norm();
  Eigen::Vector3d current_speed = getCurrentSpeed();
  double dynamic_error_threshold = 1.0 + 0.5 * current_speed.head<2>().norm();
  double vel_error = (current_speed - pred_vel).norm();
  // 跟踪误差/速度误差过大说明“上一条轨迹已经不能代表当前状态”，此时不能 warm start。
  // （原来这两处 return HOT_START、真正的 COLD_START 被注释掉了：日志说 downgrade，
  //  行为却是继续 hot start，两者矛盾，等于把 warm start 的安全门禁关掉了。）
  if (tracking_error > dynamic_error_threshold) {
    return PlanningState::COLD_START;
  }

  if (vel_error > 1.0) {
    return PlanningState::COLD_START;
  }

  if (new_path.size() >= 2) {
    Eigen::Vector3d pred_vel = last_traj_.getVel(t_dur);
    if (pred_vel.norm() > 0.1) {
      Eigen::Vector3d path_dir = (new_path[1] - new_path[0]).normalized();
      Eigen::Vector3d vel_dir = pred_vel.normalized();
      double dot = vel_dir.dot(path_dir);

      if (dot < 0.5) {
        return PlanningState::COLD_START;
      }
    }
  }

  return PlanningState::HOT_START;
}

void MincoPlanner::prepareColdStart(const geometry_msgs::msg::Pose & start_pose,
  Eigen::Matrix3d & start_state,
  const std::vector<Eigen::Vector3d> & sparse_path)
{
  start_state.setZero();
  start_state.col(0) = Eigen::Vector3d(start_pose.position.x, start_pose.position.y, 0.0);
  (void)sparse_path;
  Eigen::Vector3d real_speed = getCurrentSpeed();
  start_state.col(1) = real_speed;
}

void MincoPlanner::prepareHotStart(
  const geometry_msgs::msg::Pose & start_pose, double t_dur, Eigen::Matrix3d & start_state)
{
  start_state.setZero();
  // start_state.col(0) = last_traj_.getPos(t_dur);
  start_state.col(0) = Eigen::Vector3d(start_pose.position.x, start_pose.position.y, 0.0);
  start_state.col(1) = last_traj_.getVel(t_dur);
  // Eigen::Vector3d real_speed = getCurrentSpeed();
  start_state.col(2) = last_traj_.getAcc(t_dur);
}

// -----------------------------------------------------------------------------
// 5) Helpers / callbacks / getters
// -----------------------------------------------------------------------------

bool MincoPlanner::validateTrajectory(
  traj_opt::Trajectory & traj, const Eigen::Vector3d & expected_end_pos)
{
  last_validation_failure_reason_ = "KINEMATIC_VIOLATION";
  double dur = traj.getTotalDuration();
  if (!(std::isfinite(dur) && dur > 1e-6)) {
    last_validation_failure_reason_ = "OPTIMIZER_FAILED";
    return false;
  }

  // Check geometry before changing timing. Scaling preserves every spatial point.
  if (!checkCollision(traj)) {
    last_validation_failure_reason_ = "COLLISION"; return false;
  }
  bool dynamics_ok = false;
  for (int attempt = 0; attempt <= 3; ++attempt) {
    const auto extrema = evaluateDynamics(traj);
    if (!extrema.finite) { return false; }
    const double scale = requiredTimeScale(extrema, dynamics_);
    const double ratio = std::max({extrema.velocity / dynamics_.max_vel,
      extrema.acceleration / dynamics_.max_acc, extrema.jerk / dynamics_.max_jerk});
    if (ratio <= 1.0 + 1e-9) { dynamics_ok = true; break; }
    // Uniform scaling changes inherited nonzero boundary v/a. Reject that case
    // rather than introduce an MPC state discontinuity during a hot replan.
    if (attempt == 3 || scale > 4.0 || traj.getVel(0).head<2>().norm() > 1e-4 ||
        traj.getAcc(0).head<2>().norm() > 1e-4 ||
        traj.getVel(traj.getTotalDuration()).head<2>().norm() > 1e-4 ||
        traj.getAcc(traj.getTotalDuration()).head<2>().norm() > 1e-4) { return false; }
    traj = scaleTrajectoryTime(traj, scale * 1.01);
  }
  if (!dynamics_ok) { return false; }
  dur = traj.getTotalDuration();

  // 2) Goal reachability.
  const Eigen::Vector3d end_pos = traj.getPos(dur);
  if (!end_pos.allFinite()) {
    last_validation_failure_reason_ = "KINEMATIC_VIOLATION";
    return false;
  }

  const double goal_err = (end_pos - expected_end_pos).norm();
  if (!(std::isfinite(goal_err) && goal_err <= traj_goal_tolerance_)) {
    last_validation_failure_reason_ = "KINEMATIC_VIOLATION";
    return false;
  }

  // 3) Collision safety.
  if (!checkCollision(traj)) {
    last_validation_failure_reason_ = "COLLISION";
    return false;
  }

  return true;
}

bool MincoPlanner::checkCollision()
{
  if (!safety_checker_) {
    return false;
  }

  // Snapshot trajectory under mutex to avoid data races with ReplanLocal().
  traj_opt::Trajectory traj_snapshot;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_last_traj_) {
      return true;
    }
    traj_snapshot = last_traj_;
  }

  const double dur = traj_snapshot.getTotalDuration();
  if (!(std::isfinite(dur) && dur > 1e-6)) {
    return true;
  }

  return safety_checker_->checkTrajectory(traj_snapshot);
}

bool MincoPlanner::checkCollision(const traj_opt::Trajectory & traj)
{
  if (!safety_checker_) {
    return false;
  }

  const double dur = traj.getTotalDuration();
  if (!(std::isfinite(dur) && dur > 1e-6)) {
    return true;
  }

  return safety_checker_->checkTrajectory(traj);
}

void MincoPlanner::safetyTimerCallback()
{
  if (!active_.load()) {
    return;
  }
  geometry_msgs::msg::PoseStamped pose;
  if (!getRobotPose(pose)) {
    publishPlanningStatus("INPUT_UNAVAILABLE");
  } else if (!reusePlan(pose, false)) {
    if (last_path_block_info_.reason == "MAP_UNAVAILABLE") {
      publishPlanningStatus("INPUT_UNAVAILABLE");
      return;
    }
    is_traj_safe_.store(false);
    if (last_path_block_info_.geometry_block) {
      publishPlanningStatus("obstacle_entered_corridor");
    } else {
      publishPlanningStatus(last_path_block_info_.reason);
    }
  } else {
    inputAvailable();
    is_traj_safe_.store(true);
  }
}

bool MincoPlanner::consumePendingGoal(geometry_msgs::msg::PoseStamped & goal_out)
{
  std::lock_guard<std::mutex> lk(goal_mutex_);
  if (!has_pending_goal_) {
    return false;
  }
  goal_out = pending_goal_;
  has_pending_goal_ = false;
  return true;
}

void MincoPlanner::cancelGoal()
{
  ++pending_goal_generation_;
  std::lock_guard<std::mutex> lk(goal_mutex_);
  has_pending_goal_ = false;
  if (fsm_) {
    fsm_->cancelGoal();
  }
}

bool MincoPlanner::getRobotPose(geometry_msgs::msg::PoseStamped & pose) const
{
  // 地图新鲜度：不依赖具体实现，任何 MapQueryInterface 都可以报告最近更新时间。
  if (!rog_query_raw_) {
    return false;
  }
  const auto updated = rog_query_raw_->lastUpdateSteady();
  if (updated.time_since_epoch().count() == 0 ||
      std::chrono::duration<double>(std::chrono::steady_clock::now() - updated).count() >
        external_map_timeout_) {
    return false;
  }
  const bool direct_odom_pose = mode_context_ && mode_context_->directOdomPose();
  if (!direct_odom_pose && costmap_ros_) {
    if (costmap_ros_->getRobotPose(pose)) {
      pose.header.frame_id = planning_frame_;
      return true;
    }
  }

  nav_msgs::msg::Odometry odom;
  {
    std::lock_guard<std::mutex> lk(odom_mutex_);
    if (!has_latest_odom_) {
      return false;
    }
    if (std::chrono::duration<double>(std::chrono::steady_clock::now() - odom_received_).count() > external_odom_timeout_) {
      return false;
    }
    odom = latest_odom_;
  }

  geometry_msgs::msg::PoseStamped odom_pose;
  odom_pose.header = odom.header;
  odom_pose.pose = odom.pose.pose;
  if (odom.child_frame_id != robot_frame_) { return false; }
  if (!odom.header.frame_id.empty() && odom.header.frame_id != planning_frame_) { return false; }
  const auto & twist = odom.twist.twist;
  if (!Eigen::Vector3d(twist.linear.x, twist.linear.y, twist.linear.z).allFinite() ||
      !Eigen::Vector3d(twist.angular.x, twist.angular.y, twist.angular.z).allFinite()) { return false; }
  const auto & q = odom.pose.pose.orientation;
  Eigen::Quaterniond rotation(q.w, q.x, q.y, q.z);
  if (!rotation.coeffs().allFinite() || rotation.norm() < 1.0e-6) { return false; }
  const Eigen::Vector3d position(odom_pose.pose.position.x, odom_pose.pose.position.y, odom_pose.pose.position.z);
  if (!position.allFinite()) { return false; }

  if (direct_odom_pose) {
    if (odom_pose.header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(logger_,
        *clock_,
        2000,
        "[MincoPlanner] EXPLORATION odom frame is empty, treating it as %s.",
        planning_frame_.c_str());
    }
    odom_pose.header.frame_id = planning_frame_;
    pose = odom_pose;
    return true;
  }

  if (odom_pose.header.frame_id.empty()) {
    RCLCPP_WARN_THROTTLE(logger_,
      *clock_,
      2000,
      "[MincoPlanner] PRIORMAP odom frame is empty, treating it as %s before transforming to %s.",
      rog_frame_.c_str(),
      planning_frame_.c_str());
    odom_pose.header.frame_id = rog_frame_;
  }

  if (odom_pose.header.frame_id == planning_frame_ || odom_pose.header.frame_id == map_frame_) {
    odom_pose.header.frame_id = planning_frame_;
    pose = odom_pose;
    return true;
  }

  if (!tf_) {
    RCLCPP_WARN_THROTTLE(logger_,
      *clock_,
      2000,
      "[MincoPlanner] Cannot transform PRIORMAP odom pose from %s to %s: TF buffer is null.",
      odom_pose.header.frame_id.c_str(),
      planning_frame_.c_str());
    return false;
  }

  try {
    pose = tf_->transform(odom_pose, planning_frame_);
    pose.header.frame_id = planning_frame_;
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(logger_,
      *clock_,
      2000,
      "[MincoPlanner] Failed to transform odom pose from %s to %s: %s",
      odom_pose.header.frame_id.c_str(),
      planning_frame_.c_str(),
      ex.what());
    return false;
  }
}

bool MincoPlanner::checkGoalReached(const geometry_msgs::msg::PoseStamped & current_pose,
  const geometry_msgs::msg::PoseStamped & goal) const
{
  const double dx = current_pose.pose.position.x - goal.pose.position.x;
  const double dy = current_pose.pose.position.y - goal.pose.position.y;
  const double dist = std::hypot(dx, dy);
  return std::isfinite(dist) && dist <= traj_goal_tolerance_;
}

Eigen::Vector3d MincoPlanner::getCurrentSpeed() const
{
  std::lock_guard<std::mutex> lk(odom_mutex_);
  if (has_latest_odom_) {
    const auto & twist = latest_odom_.twist.twist;
    const auto & q = latest_odom_.pose.pose.orientation;
    Eigen::Quaterniond rotation(q.w, q.x, q.y, q.z);
    const Eigen::Vector3d velocity(twist.linear.x, twist.linear.y, twist.linear.z);
    const Eigen::Vector3d omega(twist.angular.x, twist.angular.y, twist.angular.z);
    if (!rotation.coeffs().allFinite() || rotation.norm() < 1.0e-6 ||
        !velocity.allFinite() || !omega.allFinite()) { return Eigen::Vector3d::Zero(); }
    const Eigen::Vector3d world_velocity = rotation.normalized() * velocity;
    return Eigen::Vector3d(world_velocity.x(), world_velocity.y(), 0.0);
  }
  return Eigen::Vector3d::Zero();
}

void MincoPlanner::publishPlan(const nav_msgs::msg::Path & path, bool replanned,
  float path_start_s, const std::string & reason)
{
  // Caller serializes publications with planning_output_mutex_. Match fastlio_nav2 exactly.
  if (!plan_pub_ || !plan_meta_pub_) { return; }
  navigation::msg::PlanMeta meta;
  meta.header = path.header;
  meta.path_id = planning_path_id_;
  meta.publish_seq = ++planning_publish_seq_;
  meta.replanned = replanned;
  meta.path_start_s = path_start_s;
  meta.reason = reason;
  plan_meta_pub_->publish(meta);
  plan_pub_->publish(path);
  const bool empty = path.poses.empty();
  if (meta.replanned || meta.path_id != last_logged_path_id_ ||
      meta.reason != last_logged_path_reason_ || empty != last_logged_plan_empty_) {
    RCLCPP_INFO(path_logger_,
      "[PlanMeta] path_id=%lu publish_seq=%lu replanned=%s path_start_s=%.3f reason=%s points=%zu",
      static_cast<unsigned long>(meta.path_id), static_cast<unsigned long>(meta.publish_seq),
      meta.replanned ? "true" : "false", static_cast<double>(meta.path_start_s),
      meta.reason.c_str(), path.poses.size());
    last_logged_path_id_ = meta.path_id;
    last_logged_path_reason_ = meta.reason;
    last_logged_plan_empty_ = empty;
  }
}

std::string MincoPlanner::searchFailureStatus(const std::string & fallback) const
{
  return last_search_failure_reason_.empty() ? fallback : last_search_failure_reason_;
}

void MincoPlanner::inputAvailable()
{
  std::lock_guard<std::mutex> output_lock(planning_output_mutex_);
  input_unavailable_since_ = {};
}

void MincoPlanner::publishPlanningStatus(const std::string & reason)
{
  std::lock_guard<std::mutex> output_lock(planning_output_mutex_);
  if (!plan_pub_) { return; }
  if (reason == "INPUT_UNAVAILABLE") {
    const auto now = std::chrono::steady_clock::now();
    if (input_unavailable_since_ == std::chrono::steady_clock::time_point{}) {
      input_unavailable_since_ = now;
    }
    if (cached_plan_.poses.size() >= 2U && is_traj_safe_.load() &&
        cached_plan_goal_generation_ == pending_goal_generation_.load() &&
        std::chrono::duration<double>(now - input_unavailable_since_).count() <
        input_unavailable_grace_sec_) {
      return;  // Do not deduplicate this status before checking grace expiry on later ticks.
    }
    is_traj_safe_.store(false);
  } else {
    input_unavailable_since_ = {};
  }
  if (reason == last_planning_status_) { return; }
  last_planning_status_ = reason;
  // §15：只有“当前路径确实已经不可用”才清空并发布空 /plan。
  // 新候选规划失败（NO_VALID_ROUTE / PLANNING_FAILED / SEARCH_FAILED / KINEMATIC_VIOLATION）
  // 而当前路径仍然安全时，必须保留旧路径，否则 cached_plan_ 被清空后
  // hasExecutablePlan() 恒为 false -> path_blocked 恒真 -> 每个 tick 都在重规划。
  const bool candidate_failure =
    reason == "NO_VALID_ROUTE" || reason == "PLANNING_FAILED" ||
    reason == "SEARCH_FAILED" || reason == "KINEMATIC_VIOLATION";
  if (candidate_failure && cached_plan_.poses.size() >= 2U && is_traj_safe_.load()) {
    return;
  }
  local_horizon_exhausted_.store(false);
  cached_plan_ = nav_msgs::msg::Path{};
  cached_plan_s_.clear();
  cached_progress_s_ = 0.0;
  nav_msgs::msg::Path path;
  path.header.frame_id = output_frame_;
  path.header.stamp = clock_->now();
  // The reference interface has no status enum. An empty path explicitly retracts the reference.
  publishPlan(path, false, 0.0F, reason);
  std::lock_guard<std::mutex> lock(mutex_);
  has_last_traj_ = false;
}

void MincoPlanner::publishOptimizedPlan(const traj_opt::Trajectory & trajectory,
  const std_msgs::msg::Header & header)
{
  nav_msgs::msg::Path path;
  path.header = header;
  // 旧 fastlio_nav2 是**弧长均匀**采样（output_spacing，默认 0.05 m），不是固定时间步。
  // 固定 0.05 s 会让慢速段点特别密、高速段特别稀，controller 的视觉/跟踪手感会飘。
  const double duration = std::max(1e-3, trajectory.getTotalDuration());
  constexpr double kDenseDt = 0.01;
  const int dense_steps = std::max(2, static_cast<int>(std::ceil(duration / kDenseDt)));
  std::vector<Eigen::Vector3d> dense;
  dense.reserve(static_cast<size_t>(dense_steps) + 1U);
  for (int i = 0; i <= dense_steps; ++i) {
    const auto p = trajectory.getPos(std::min(duration, i * kDenseDt));
    dense.emplace_back(p.x(), p.y(), 0.0);
  }
  // 沿弧长重采样。
  std::vector<Eigen::Vector2d> points;
  points.emplace_back(dense.front().head<2>());
  double accumulated = 0.0;
  for (size_t i = 1; i < dense.size(); ++i) {
    const Eigen::Vector2d a = dense[i - 1].head<2>();
    const Eigen::Vector2d b = dense[i].head<2>();
    const double segment = (b - a).norm();
    if (segment < 1e-9) {
      continue;
    }
    double travelled = 0.0;
    while (accumulated + (segment - travelled) >= output_spacing_) {
      const double need = output_spacing_ - accumulated;
      travelled += need;
      points.emplace_back(a + (b - a) * (travelled / segment));
      accumulated = 0.0;
    }
    accumulated += segment - travelled;
  }
  const Eigen::Vector2d tail = dense.back().head<2>();
  if ((tail - points.back()).norm() > 1e-6) {
    points.emplace_back(tail);
  }
  Eigen::Quaterniond goal_orientation = Eigen::Quaterniond::Identity();
  {
    std::lock_guard<std::mutex> goal_lock(goal_mutex_);
    const auto & q = pending_goal_.pose.orientation;
    goal_orientation = Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized();
  }
  for (size_t i = 0; i < points.size(); ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = header;
    pose.pose.position.x = points[i].x();
    pose.pose.position.y = points[i].y();
    pose.pose.position.z = 0.0;
    if (i + 1U < points.size()) {
      // 中间 pose：yaw = 轨迹切线方向。
      const Eigen::Vector2d tangent = points[i + 1U] - points[i];
      const double yaw = std::atan2(tangent.y(), tangent.x());
      pose.pose.orientation.z = std::sin(0.5 * yaw);
      pose.pose.orientation.w = std::cos(0.5 * yaw);
    } else {
      // 最后一个 pose 保留用户请求的 goal orientation（旧版行为）。
      pose.pose.orientation.x = goal_orientation.x();
      pose.pose.orientation.y = goal_orientation.y();
      pose.pose.orientation.z = goal_orientation.z();
      pose.pose.orientation.w = goal_orientation.w();
    }
    path.poses.push_back(pose);
  }
  cached_plan_ = path;
  cached_plan_s_.assign(path.poses.size(), 0.0);
  for (size_t i = 1; i < path.poses.size(); ++i) {
    const auto & a = path.poses[i - 1].pose.position;
    const auto & b = path.poses[i].pose.position;
    cached_plan_s_[i] = cached_plan_s_[i - 1] + std::hypot(b.x - a.x, b.y - a.y);
  }
  cached_progress_s_ = 0.0;
  {
    std::lock_guard<std::mutex> goal_lock(goal_mutex_);
    const auto & end = path.poses.back().pose.position;
    const auto & goal = pending_goal_.pose.position;
    cached_plan_reaches_goal_ = std::hypot(end.x - goal.x, end.y - goal.y) <= traj_goal_tolerance_;
  }
  local_horizon_exhausted_.store(false);
  input_unavailable_since_ = {};
  last_planning_status_.clear();
  ++planning_path_id_;
  publishPlan(path, true, 0.0F, "minco_accepted");
}

namespace {
double polylineLength(const std::vector<geometry_msgs::msg::PoseStamped> & path)
{
  double total = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    total += std::hypot(path[i].pose.position.x - path[i - 1].pose.position.x,
      path[i].pose.position.y - path[i - 1].pose.position.y);
  }
  return total;
}
}  // namespace

bool MincoPlanner::hasExecutablePlan() const
{
  return cached_plan_.poses.size() >= 2U;
}

bool MincoPlanner::betterPathAvailable(const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  if (!global_path_searcher_ || !mode_context_) {
    return false;
  }
  const double j_old = cached_plan_s_.empty() ? 0.0 :
    std::max(0.0, cached_plan_s_.back() - cached_progress_s_);
  const uint64_t goal_generation = pending_goal_generation_.load();

  std::vector<geometry_msgs::msg::PoseStamped> candidate;
  if (!global_path_searcher_->plan(start, goal, *mode_context_,
        [this]() { return !active_.load(); }, candidate) || candidate.size() < 2U) {
    last_search_failure_reason_ = global_path_searcher_->lastFailureReason();
    return false;
  }
  last_search_failure_reason_.clear();
  const double j_new = polylineLength(candidate);
  const double ratio = j_old > 1e-6 ? (j_old - j_new) / j_old : 0.0;
  if (goal_generation != pending_goal_generation_.load()) {
    return false;
  }
  if (ratio < replan_improvement_ratio_) {
    return false;
  }
  latest_global_path_ = std::move(candidate);
  global_path_goal_generation_ = goal_generation;
  local_path_processor_->resetProgress();
  return true;
}

bool MincoPlanner::continueLocalPlan(const geometry_msgs::msg::PoseStamped & current_pose,
  const geometry_msgs::msg::PoseStamped & goal)
{
  bool route_exhausted;
  {
    std::lock_guard<std::mutex> lock(path_mutex_);
    route_exhausted = latest_global_path_.size() < 2U ||
      global_path_goal_generation_ != pending_goal_generation_.load();
    if (!route_exhausted) {
      const auto & end = latest_global_path_.back().pose.position;
      route_exhausted = std::hypot(end.x - current_pose.pose.position.x,
        end.y - current_pose.pose.position.y) < 0.75;
    }
  }
  // Only a genuinely exhausted global/boundary route needs a new search. Otherwise
  // buildSeed reuses its monotonic route progress and crops the next lookahead.
  if (route_exhausted && !PlanGlobalPath(current_pose, goal)) { return false; }
  return ReplanLocal(current_pose);
}

bool MincoPlanner::reusePlan(const geometry_msgs::msg::PoseStamped & current_pose, bool publish)
{
  std::lock_guard<std::mutex> output_lock(planning_output_mutex_);
  local_horizon_exhausted_.store(false);
  if (cached_plan_.poses.size() < 2) { return !publish; }
  last_path_block_info_ = PathBlockInfo{};
  if (!active_.load() || cached_plan_goal_generation_ != pending_goal_generation_.load()) {
    last_path_block_info_.reason = "GOAL_CHANGED";
    return false;
  }
  // 路径校验用的查询对象：局部滚动图要冻结快照（坐标会随滑窗变化），
  // 全局地形图坐标固定，直接使用即可。
  std::shared_ptr<rog_map::MapQueryInterface> query;
  if (const auto adapter = std::dynamic_pointer_cast<rog_map::QueryAdapter>(rog_query_raw_)) {
    const auto snapshot = adapter->snapshot();
    if (!snapshot) { last_path_block_info_.reason = "MAP_UNAVAILABLE"; return false; }
    auto frozen = std::make_shared<rog_map::QueryAdapter>();
    frozen->update(snapshot, nullptr);
    query = frozen;
  } else if (rog_query_raw_) {
    query = rog_query_raw_;
  } else {
    last_path_block_info_.reason = "MAP_UNAVAILABLE";
    return false;
  }
  const Eigen::Vector2d robot(current_pose.pose.position.x, current_pose.pose.position.y);
  double best_distance = std::numeric_limits<double>::infinity();
  double progress = cached_progress_s_;
  size_t next_index = 1;
  for (size_t i = 1; i < cached_plan_.poses.size(); ++i) {
    if (cached_plan_s_[i] < cached_progress_s_) { continue; }
    const auto & a = cached_plan_.poses[i - 1].pose.position;
    const auto & b = cached_plan_.poses[i].pose.position;
    const Eigen::Vector2d start(a.x, a.y), delta(b.x - a.x, b.y - a.y);
    const double length = delta.norm();
    if (length < 1.0e-8) { continue; }
    const double lower = std::clamp((cached_progress_s_ - cached_plan_s_[i - 1]) / length, 0.0, 1.0);
    const double projected_t = (robot - start).dot(delta) / delta.squaredNorm();
    const double t = std::clamp(projected_t, lower, 1.0);
    // A rolling endpoint is not a lateral boundary. Forward overrun along its tangent
    // consumes the horizon; the mission endpoint still uses the normal bounded distance.
    const bool forward_overrun = !cached_plan_reaches_goal_ &&
      i + 1U == cached_plan_.poses.size() && projected_t > 1.0;
    const double distance = (robot - start - (forward_overrun ? projected_t : t) * delta).norm();
    if (distance < best_distance) {
      best_distance = distance;
      progress = cached_plan_s_[i - 1] + t * length;
      next_index = i;
    }
  }
  if (best_distance > traj_goal_tolerance_) {
    last_path_block_info_.reason = "ROBOT_OFF_PATH";
    return false;
  }
  nav_msgs::msg::Path path;
  path.header = cached_plan_.header;
  path.header.stamp = clock_->now();
  auto first = current_pose;
  first.header = path.header;
  first.pose.position.z = 0.0;
  first.pose.orientation = geometry_msgs::msg::Quaternion{};
  first.pose.orientation.w = 1.0;
  path.poses.push_back(first);
  for (size_t i = next_index; i < cached_plan_.poses.size(); ++i) {
    auto pose = cached_plan_.poses[i];
    pose.header = path.header;
    path.poses.push_back(pose);
  }
  // Validate the remaining path with a small lookback on the frozen map snapshot.
  const double spacing = std::max(1.0e-4, query->resolution() * 0.5);
  const double s_begin = std::max(0.0, progress - path_validation_lookback_margin_);
  size_t begin_index = 0U;
  while (begin_index + 1U < cached_plan_.poses.size() &&
         cached_plan_s_[begin_index] < s_begin) {
    ++begin_index;
  }
  Eigen::Vector3d previous(
    cached_plan_.poses[begin_index].pose.position.x, cached_plan_.poses[begin_index].pose.position.y, 0.0);
  std::string blocked_reason;
  for (size_t i = begin_index + 1U; i < cached_plan_.poses.size(); ++i) {
    const auto & p = cached_plan_.poses[i].pose.position;
    const Eigen::Vector3d end(p.x, p.y, 0.0);
    const double segment_length = (end - previous).norm();
    if (!query->edgeAllowed(previous, end)) {
      blocked_reason = "BLOCKED_EDGE";
      break;
    }
    const int samples = std::max(1, static_cast<int>(std::ceil(segment_length / spacing)));
    bool segment_blocked = false;
    for (int j = 0; j <= samples; ++j) {
      const double ratio = static_cast<double>(j) / static_cast<double>(samples);
      const Eigen::Vector3d point = previous + (end - previous) * ratio;
      if (!query->footprintFree(point)) {
        blocked_reason = "FOOTPRINT_COLLISION";
        segment_blocked = true;
        break;
      }
      unsigned int mx = 0;
      unsigned int my = 0;
      if (!query->worldToMap(point.x(), point.y(), mx, my)) {
        // 地图尚未覆盖到这里：按 UNKNOWN 处理。默认视为可通行（冻结的乐观规划语义）；
        // 只有显式打开 block_out_of_map 时才把 OUT_OF_MAP 当成阻塞原因上报。
        if (path_validation_block_out_of_map_) {
          blocked_reason = "OUT_OF_MAP";
          segment_blocked = true;
          break;
        }
        continue;
      }
      if (!query->isFree(mx, my)) {
        blocked_reason = "CELL_OCCUPIED";
        segment_blocked = true;
        break;
      }
    }
    if (segment_blocked) {
      break;
    }
    previous = end;
  }
  // 进度必须每次都落盘（包括校验失败），和“是否重发路径”无关。否则 betterPathAvailable 里的
  // j_old = cached_plan_s_.back() - cached_progress_s_ 会退化成整条历史全长，
  // 也会让下一次校验继续从陈旧的进度出发。
  // （旧 fastlio_nav2 每次复用都会把裁剪后的进度写回。）
  cached_progress_s_ = progress;
  if (!blocked_reason.empty()) {
    last_path_block_info_.reason = blocked_reason;
    last_path_block_info_.geometry_block = true;
    return false;
  }
  // Collision validation takes precedence over rolling the local window forward.
  const bool horizon_exhausted = !cached_plan_reaches_goal_ &&
    cached_plan_s_.back() - progress < 0.75;
  local_horizon_exhausted_.store(horizon_exhausted);
  if (horizon_exhausted) {
    last_path_block_info_.reason = "LOCAL_HORIZON_EXHAUSTED";
  }
  if (publish) {
    // A goal may change in another callback while the route is being validated.
    if (!active_.load() || cached_plan_goal_generation_ != pending_goal_generation_.load()) {
      last_path_block_info_.reason = "GOAL_CHANGED";
      return false;
    }
    last_planning_status_.clear();
    publishPlan(path, false, static_cast<float>(progress), "cache_trim");
  }
  return true;
}

}  // namespace minco_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(minco_planner::MincoPlanner, nav2_core::GlobalPlanner)
