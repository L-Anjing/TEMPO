#include "minco_core/components/global_path_searcher.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <queue>
#include <string>
#include <rog_map/query_adapter.hpp>
#include <rog_map/terrain_types.hpp>

namespace minco_planner {

namespace {

// 规划可通行判定：已知障碍(254)/膨胀格(253)不可通行；UNKNOWN(255) 与 FREE 一样可通行。
bool isTraversableCost(const unsigned char cost)
{
  return cost < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE ||
         cost == nav2_costmap_2d::NO_INFORMATION;
}


const char * cellCostLabel(const unsigned char cost)
{
  if (cost == nav2_costmap_2d::NO_INFORMATION) {
    return "unknown";
  }
  if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
    return "blocked/inflated";
  }
  if (cost > 128u) {
    return "high-cost";
  }
  return "free";
}



void logCellDiagnostics(const rclcpp::Logger & logger,
  const std::shared_ptr<rog_map::MapQueryInterface> & query,
  const std::string & source,
  const std::string & label,
  unsigned int mx,
  unsigned int my,
  bool allow_unknown)
{
  if (!query || !query->isValid(mx, my)) {
    RCLCPP_WARN(logger,
      "[MincoPlanner] %s %s cell=(%u,%u) is invalid for map size=%ux%u",
      source.c_str(),
      label.c_str(),
      mx,
      my,
      query ? query->sizeX() : 0U,
      query ? query->sizeY() : 0U);
    return;
  }

  double wx = 0.0;
  double wy = 0.0;
  query->mapToWorld(mx, my, wx, wy);
  const unsigned char cost = query->value(mx, my);

  int valid = 0;
  int unknown = 0;
  int free = 0;
  int blocked = 0;
  int invalid = 0;
  constexpr int kRadius = 2;
  for (int dy = -kRadius; dy <= kRadius; ++dy) {
    for (int dx = -kRadius; dx <= kRadius; ++dx) {
      const int cx = static_cast<int>(mx) + dx;
      const int cy = static_cast<int>(my) + dy;
      if (cx < 0 || cy < 0 || cx >= static_cast<int>(query->sizeX()) ||
          cy >= static_cast<int>(query->sizeY())) {
        ++invalid;
        continue;
      }
      const unsigned int ux = static_cast<unsigned int>(cx);
      const unsigned int uy = static_cast<unsigned int>(cy);
      const unsigned char nb_cost = query->value(ux, uy);
      ++valid;
      if (nb_cost == nav2_costmap_2d::NO_INFORMATION) {
        ++unknown;
      } else if (nb_cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        ++blocked;
      } else {
        ++free;
      }
    }
  }

  RCLCPP_WARN(logger,
    "[MincoPlanner] %s %s diag: cell=(%u,%u) world=(%.3f,%.3f) cost=%u(%s) query_free=%s "
    "allow_unknown=%s 5x5(valid=%d invalid=%d free=%d unknown=%d blocked=%d)",
    source.c_str(),
    label.c_str(),
    mx,
    my,
    wx,
    wy,
    static_cast<unsigned int>(cost),
    cellCostLabel(cost),
    query->isFree(mx, my) ? "true" : "false",
    allow_unknown ? "true" : "false",
    valid,
    invalid,
    free,
    unknown,
    blocked);
}

}  // namespace

void GlobalPathSearcher::configure(std::shared_ptr<tf2_ros::Buffer> tf,
  Astar * astar,
  bool allow_unknown,
  double tolerance,
  rclcpp::Clock::SharedPtr clock,
  rclcpp::Logger logger)
{
  tf_ = std::move(tf);
  astar_ = astar;
  allow_unknown_ = allow_unknown;
  tolerance_ = tolerance;
  clock_ = std::move(clock);
  logger_ = logger;
}

void GlobalPathSearcher::setQuery(const std::shared_ptr<rog_map::MapQueryInterface> & global_query)
{
  global_query_ = global_query;
  if (astar_) {
    astar_->setMap(global_query_);
  }
}

bool GlobalPathSearcher::normalizePoseToFrame(const geometry_msgs::msg::PoseStamped & in,
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

bool GlobalPathSearcher::plan(const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const PlannerModeContext & mode_context,
  std::function<bool()> cancel_checker,
  std::vector<geometry_msgs::msg::PoseStamped> & latest_global_path)
{
  last_failure_reason_.clear();
  if (mode_context.mode() == PlannerMode::PRIORMAP) {
    return planPriorMap(start, goal, mode_context, cancel_checker, latest_global_path);
  }
  return planExploration(start, goal, mode_context, cancel_checker, latest_global_path);
}

bool GlobalPathSearcher::planPriorMap(const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const PlannerModeContext & mode_context,
  const std::function<bool()> & cancel_checker,
  std::vector<geometry_msgs::msg::PoseStamped> & latest_global_path)
{
  if (!astar_ || !mode_context.globalQuery()) {
    RCLCPP_ERROR(logger_, "[MincoPlanner] Nav2 costmap global search query is unavailable.");
    return false;
  }

  geometry_msgs::msg::PoseStamped start_map;
  geometry_msgs::msg::PoseStamped goal_map;
  if (!normalizePoseToFrame(
        start, mode_context.mapFrame(), mode_context.mapFrame(), "PRIORMAP start", start_map) ||
      !normalizePoseToFrame(
        goal, mode_context.mapFrame(), mode_context.mapFrame(), "PRIORMAP goal", goal_map)) {
    return false;
  }

  nav_msgs::msg::Path dummy;
  dummy.header.stamp = clock_->now();
  dummy.header.frame_id = mode_context.outputFrame();

  return makePlanOnQuery(start_map.pose,
    goal_map.pose,
    mode_context.globalQuery(),
    mode_context.outputFrame(),
    "Nav2 costmap",
    tolerance_,
    cancel_checker,
    dummy,
    latest_global_path);
}

bool GlobalPathSearcher::planExploration(const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const PlannerModeContext & mode_context,
  const std::function<bool()> & cancel_checker,
  std::vector<geometry_msgs::msg::PoseStamped> & latest_global_path)
{
  // 主链只有一个目标：用户给的 goal。这里不做 frontier/边界目标替换，也不自动换目标。
  // 只允许三种结果：
  //   goal known FREE / UNKNOWN -> 对原 goal 规划
  //   goal confirmed OCCUPIED   -> GOAL_CONFIRMED_BLOCKED
  //   goal 在地图外              -> OUTSIDE_GLOBAL_MAP
  //   路径暂时不存在            -> PLANNING_FAILED
  auto query = mode_context.globalQuery();
  if (!astar_ || !query) {
    last_failure_reason_ = "QUERY_UNAVAILABLE";
    RCLCPP_ERROR(logger_, "[MincoPlanner] ROGMap global search query is unavailable.");
    return false;
  }
  // 在线地形坐标在滑窗时会变，冻结一份快照让本次搜索坐标一致（全局图坐标固定，无需冻结）。
  if (const auto adapter = std::dynamic_pointer_cast<rog_map::QueryAdapter>(query)) {
    const auto snapshot = adapter->snapshot();
    if (snapshot && snapshot->terrain_config.enable) {
      auto frozen = std::make_shared<rog_map::QueryAdapter>();
      frozen->update(snapshot, nullptr);
      query = frozen;
    }
  }

  geometry_msgs::msg::PoseStamped start_rog;
  geometry_msgs::msg::PoseStamped goal_rog;
  if (!normalizePoseToFrame(
        start, mode_context.rogFrame(), mode_context.rogFrame(), "EXPLORATION start", start_rog) ||
      !normalizePoseToFrame(
        goal, mode_context.rogFrame(), mode_context.rogFrame(), "EXPLORATION goal", goal_rog)) {
    last_failure_reason_ = "FRAME_ERROR";
    return false;
  }

  unsigned int sx = 0;
  unsigned int sy = 0;
  if (!query->worldToMap(start_rog.pose.position.x, start_rog.pose.position.y, sx, sy)) {
    last_failure_reason_ = "START_OUTSIDE_GLOBAL_MAP";
    RCLCPP_ERROR(logger_,
      "[MincoPlanner] START_OUTSIDE_GLOBAL_MAP: start (%.3f, %.3f) is outside the global map "
      "(map=%ux%u origin=(%.3f,%.3f) res=%.3f).",
      start_rog.pose.position.x,
      start_rog.pose.position.y,
      query->sizeX(),
      query->sizeY(),
      query->originX(),
      query->originY(),
      query->resolution());
    return false;
  }

  unsigned int gx = 0;
  unsigned int gy = 0;
  if (!query->worldToMap(goal_rog.pose.position.x, goal_rog.pose.position.y, gx, gy)) {
    // 方案 A：40x40 全局图应覆盖整个比赛场地；地图外的 goal 直接判失败，
    // 不再“取射线最后一个地图内格”做隐式目标替换。
    last_failure_reason_ = "OUTSIDE_GLOBAL_MAP";
    RCLCPP_ERROR(logger_,
      "[MincoPlanner] OUTSIDE_GLOBAL_MAP: goal (%.3f, %.3f) is outside the global map "
      "(map=%ux%u origin=(%.3f,%.3f) res=%.3f); enlarge rog_map.global_map size to cover the field.",
      goal_rog.pose.position.x,
      goal_rog.pose.position.y,
      query->sizeX(),
      query->sizeY(),
      query->originX(),
      query->originY(),
      query->resolution());
    return false;
  }

  const unsigned char goal_cost = query->value(gx, gy);
  if (!isTraversableCost(goal_cost)) {
    // §15：目标被确认落在已知障碍里 -> 任务失败，绝不移动目标。
    last_failure_reason_ = "GOAL_CONFIRMED_BLOCKED";
    RCLCPP_ERROR(logger_,
      "[MincoPlanner] GOAL_CONFIRMED_BLOCKED: goal (%.3f, %.3f) cell=(%u,%u) cost=%u(%s) is a "
      "known obstacle; task failed, the goal is not moved.",
      goal_rog.pose.position.x,
      goal_rog.pose.position.y,
      gx,
      gy,
      static_cast<unsigned int>(goal_cost),
      cellCostLabel(goal_cost));
    return false;
  }

  nav_msgs::msg::Path plan;
  plan.header.stamp = clock_->now();
  plan.header.frame_id = mode_context.outputFrame();
  if (makePlanOnQuery(start_rog.pose,
        goal_rog.pose,
        query,
        mode_context.outputFrame(),
        "ROGMap",
        tolerance_,
        cancel_checker,
        plan,
        latest_global_path)) {
    return true;
  }
  last_failure_reason_ = "PLANNING_FAILED";
  RCLCPP_WARN(logger_,
    "[MincoPlanner] PLANNING_FAILED: no path to the original goal (%.3f, %.3f); the goal is not moved.",
    goal_rog.pose.position.x,
    goal_rog.pose.position.y);
  return false;
}

bool GlobalPathSearcher::makePlan(const geometry_msgs::msg::Pose & start,
  const geometry_msgs::msg::Pose & goal,
  const PlannerModeContext & mode_context,
  double tolerance,
  std::function<bool()> cancel_checker,
  nav_msgs::msg::Path & plan)
{
  std::vector<geometry_msgs::msg::PoseStamped> ignored_path;
  return makePlanOnQuery(start,
    goal,
    mode_context.globalQuery(),
    mode_context.outputFrame(),
    mode_context.mode() == PlannerMode::PRIORMAP ? "Nav2 costmap" : "ROGMap",
    tolerance,
    cancel_checker,
    plan,
    ignored_path);
}

bool GlobalPathSearcher::makePlanOnQuery(const geometry_msgs::msg::Pose & start,
  const geometry_msgs::msg::Pose & goal,
  const std::shared_ptr<rog_map::MapQueryInterface> & query,
  const std::string & output_frame,
  const std::string & failure_source,
  double tolerance,
  std::function<bool()> cancel_checker,
  nav_msgs::msg::Path & plan,
  std::vector<geometry_msgs::msg::PoseStamped> & latest_global_path)
{
  plan.poses.clear();
  plan.header.stamp = clock_->now();
  plan.header.frame_id = output_frame;
  if (!query) {
    RCLCPP_ERROR(
      logger_, "[MincoPlanner] %s query is not available for global planning.", failure_source.c_str());
    return false;
  }

  double wx = start.position.x;
  double wy = start.position.y;
  unsigned int mx_start = 0;
  unsigned int my_start = 0;
  if (!query->worldToMap(wx, wy, mx_start, my_start)) {
    RCLCPP_ERROR(logger_,
      "%s worldToMap failed for start world coordinates (%.2f, %.2f). "
      "map size=%ux%u origin=(%.3f, %.3f) resolution=%.3f frame=%s",
      failure_source.c_str(),
      wx,
      wy,
      query->sizeX(),
      query->sizeY(),
      query->originX(),
      query->originY(),
      query->resolution(),
      output_frame.c_str());
    return false;
  }

  const unsigned int raw_mx_start = mx_start;
  const unsigned int raw_my_start = my_start;
  // start 允许落在 UNKNOWN 上（车身滤除 + 雷达盲区使它本来就经常不可见），
  // 只有 start 已经明确落在已知障碍里才拒绝。
  const bool start_blocked = !isTraversableCost(query->value(mx_start, my_start));
  if (start_blocked) {
    RCLCPP_ERROR(logger_,
      "%s start cell is a known obstacle at world coordinates (%.2f, %.2f)",
      failure_source.c_str(),
      wx,
      wy);
    logCellDiagnostics(logger_, query, failure_source, "start(raw)", raw_mx_start, raw_my_start, allow_unknown_);
    return false;
  }

  wx = goal.position.x;
  wy = goal.position.y;
  unsigned int mx_goal = 0;
  unsigned int my_goal = 0;
  const bool goal_inside_map = query->worldToMap(wx, wy, mx_goal, my_goal);
  const unsigned int nx = query->sizeX();
  const unsigned int ny = query->sizeY();
  if (!goal_inside_map) {
    // 不做目标替换：地图外的 goal 直接判失败，由调用方决定是 OUTSIDE_GLOBAL_MAP 还是别的状态。
    RCLCPP_ERROR(logger_,
      "%s goal (%.2f, %.2f) is outside the global map; the goal is not moved. "
      "map size=%ux%u origin=(%.3f, %.3f) resolution=%.3f frame=%s",
      failure_source.c_str(),
      wx,
      wy,
      nx,
      ny,
      query->originX(),
      query->originY(),
      query->resolution(),
      output_frame.c_str());
    return false;
  }
  const unsigned char start_cost = query->value(mx_start, my_start);
  const unsigned char goal_cost = query->value(mx_goal, my_goal);
  RCLCPP_INFO(logger_,
    "[MincoPlanner] %s global search input: planner=%s frame=%s map=%ux%u origin=(%.3f,%.3f) "
    "res=%.3f tolerance=%.3f allow_unknown=%s start_world=(%.3f,%.3f) start_cell=(%u,%u)->(%u,%u) "
    "start_cost=%u(%s) goal_world=(%.3f,%.3f) goal_cell=(%u,%u) goal_cost=%u(%s)",
    failure_source.c_str(),
    "Astar",
    output_frame.c_str(),
    nx,
    ny,
    query->originX(),
    query->originY(),
    query->resolution(),
    tolerance,
    allow_unknown_ ? "true" : "false",
    start.position.x,
    start.position.y,
    raw_mx_start,
    raw_my_start,
    mx_start,
    my_start,
    static_cast<unsigned int>(start_cost),
    cellCostLabel(start_cost),
    goal.position.x,
    goal.position.y,
    mx_goal,
    my_goal,
    static_cast<unsigned int>(goal_cost),
    cellCostLabel(goal_cost));

  // A* 只在 start/goal 附近的有界窗口里搜索：全局 40x40 m 图有 640k 格，
  // 直接对整图做势场传播会阻塞 executor（生命周期心跳 4 s 超时会把 planner_server 判死）。
  auto search_query = query;
  unsigned int smx_start = mx_start;
  unsigned int smy_start = my_start;
  unsigned int smx_goal = mx_goal;
  unsigned int smy_goal = my_goal;
  {
    const double cx = 0.5 * (start.position.x + goal.position.x);
    const double cy = 0.5 * (start.position.y + goal.position.y);
    const double span =
      0.5 * std::hypot(goal.position.x - start.position.x, goal.position.y - start.position.y);
    const double half_extent = std::max(6.0, span + 4.0);
    auto roi = std::make_shared<RoiQueryAdapter>(query, cx, cy, half_extent);
    unsigned int rx = 0;
    unsigned int ry = 0;
    if (roi->sizeX() > 0 && roi->sizeY() > 0 &&
        roi->worldToMap(start.position.x, start.position.y, rx, ry)) {
      smx_start = rx;
      smy_start = ry;
      if (roi->worldToMap(goal.position.x, goal.position.y, rx, ry)) {
        smx_goal = rx;
        smy_goal = ry;
        search_query = roi;
      }
    }
  }
  const unsigned int search_nx = search_query->sizeX();
  const unsigned int search_ny = search_query->sizeY();
  const size_t search_size = static_cast<size_t>(search_nx) * static_cast<size_t>(search_ny);

  {
    if (!astar_) {
      return false;
    }
    astar_->setMap(search_query);
    astar_->setSize(search_nx, search_ny);
    astar_->setStart(static_cast<int>(smx_start), static_cast<int>(smy_start));
    astar_->setGoal(static_cast<int>(smx_goal), static_cast<int>(smy_goal));
    astar_->setupNavFn(true);
    std::vector<unsigned char> query_costmap_copy;
    if (!search_query->copyValues(query_costmap_copy) ||
        query_costmap_copy.size() != search_size) {
      RCLCPP_ERROR(logger_,
        "%s Astar failed to copy map values: copied=%zu expected=%zu",
        failure_source.c_str(),
        query_costmap_copy.size(),
        search_size);
      astar_->setMap(query);
      return false;
    }
    astar_->setCostmap(query_costmap_copy.data(), true, allow_unknown_);

    int max_total_cycles = static_cast<int>(search_size) * 9999;
    int cycles_per_step = std::max(static_cast<int>(search_size / 20),
      static_cast<int>(search_nx + search_ny));
    while (max_total_cycles > 0) {
      if (cancel_checker && cancel_checker()) {
        astar_->setMap(query);
        return false;
      }
      if (!astar_->propNavFnAstar(cycles_per_step, cancel_checker)) {
        break;
      }
      max_total_cycles -= cycles_per_step;
    }

    if (!astar_->calcPath(static_cast<int>(search_size / 2)) || astar_->getPathLen() < 2) {
      RCLCPP_ERROR(logger_, "%s Astar failed to find path. See endpoint diagnostics below.", failure_source.c_str());
      if (raw_mx_start != mx_start || raw_my_start != my_start) {
        logCellDiagnostics(
          logger_, query, failure_source, "start(raw-before-projection)", raw_mx_start, raw_my_start, allow_unknown_);
      }
      logCellDiagnostics(logger_, query, failure_source, "start(used)", mx_start, my_start, allow_unknown_);
      logCellDiagnostics(logger_, query, failure_source, "goal", mx_goal, my_goal, allow_unknown_);
      astar_->setMap(query);
      return false;
    }
    astar_->setMap(query);

    float * path_x = astar_->getPathX();
    float * path_y = astar_->getPathY();
    const int len = astar_->getPathLen();

    latest_global_path.clear();
    latest_global_path.reserve(len);
    plan.poses.reserve(len);

    for (int i = 0; i < len; ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = plan.header;

      double path_wx = 0.0;
      double path_wy = 0.0;
      search_query->mapToWorld(
        static_cast<unsigned int>(path_x[i]), static_cast<unsigned int>(path_y[i]), path_wx, path_wy);

      pose.pose.position.x = path_wx;
      pose.pose.position.y = path_wy;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.w = 1.0;
      latest_global_path.push_back(pose);
      plan.poses.push_back(pose);
    }
  }

  if (!plan.poses.empty()) {
    plan.poses.back().pose.position.x = goal.position.x;
    plan.poses.back().pose.position.y = goal.position.y;
    plan.poses.back().pose.position.z = goal.position.z;
    plan.poses.back().pose.orientation = goal.orientation;
    latest_global_path.back().pose.position.x = goal.position.x;
    latest_global_path.back().pose.position.y = goal.position.y;
    latest_global_path.back().pose.position.z = goal.position.z;
    latest_global_path.back().pose.orientation = goal.orientation;
  }

  return !plan.poses.empty();
}

}  // namespace minco_planner
