#include "minco_core/components/trajectory_safety_checker.hpp"

#include "data_structure/base/trajectory.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace minco_planner {

void TrajectorySafetyChecker::configure(
  double safe_dist, double sample_dt, rclcpp::Clock::SharedPtr clock, rclcpp::Logger logger)
{
  safe_dist_ = safe_dist;
  sample_dt_ = sample_dt > 1e-6 ? sample_dt : 0.05;
  clock_ = std::move(clock);
  logger_ = logger;
}

void TrajectorySafetyChecker::setQuery(std::shared_ptr<rog_map::MapQueryInterface> dynamic_query)
{
  dynamic_query_ = std::move(dynamic_query);
}

bool TrajectorySafetyChecker::ensureQueryAvailable() const
{
  if (dynamic_query_) {
    return true;
  }
  RCLCPP_ERROR_THROTTLE(logger_,
    *clock_,
    1000,
    "[MincoPlanner] ROGMap dynamic query is unavailable for trajectory safety check.");
  return false;
}

bool TrajectorySafetyChecker::checkPoint(const Eigen::Vector3d & pos) const
{
  // 安全检查先拒绝二维 layer 中的致命/膨胀代价值，再用 ROGMap distance field 与 safe_dist_ 比较。
  // ROGMap field 已扣除 field.inflation_radius，safe_dist_ 会继续增加规划安全余量。
  if (!ensureQueryAvailable()) {
    return false;
  }

  if (!dynamic_query_->footprintFree(pos)) {
    // 把优化器看到的那一路信息一起打出来：如果 esdf 距离明显大于硬净空，
    // 说明 optimizer 的障碍代价用的判据和 footprint 扫描不是同一个障碍集合。
    const auto esdf = dynamic_query_->query(pos);
    const auto policy = dynamic_query_->terrainPolicy();
    const double hard = policy.enable ? std::max(policy.flat_clearance_radius,
        std::hypot(policy.terrain_half_length, policy.terrain_half_width)) +
        policy.clearance_margin + 0.7071067812 * dynamic_query_->resolution() : 0.0;
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000,
      "[Trajectory collision] footprint blocked at (%.3f %.3f %.3f) esdf_ok=%s esdf_dist=%.4f "
      "soft_clearance=%.4f hard_clearance=%.4f",
      pos.x(), pos.y(), pos.z(), esdf.ok ? "true" : "false", esdf.distance,
      dynamic_query_->softClearance(pos, safe_dist_), hard);
    return false;
  }

  unsigned int mx = 0;
  unsigned int my = 0;
  if (!dynamic_query_->worldToMap(pos.x(), pos.y(), mx, my)) {
    // 当前地图还没覆盖到这里：按 UNKNOWN 处理，允许通行，不做 ESDF 距离判据。
    // 足印检查已在上面执行；地图外的障碍无从判断，因此不再额外拒绝。
    return true;
  }
  const unsigned char cost = dynamic_query_->value(mx, my);
  if (cost == nav2_costmap_2d::LETHAL_OBSTACLE || cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
    return false;
  }
  double esdf_dist = 0.0;
  Eigen::Vector3d esdf_grad = Eigen::Vector3d::Zero();
  const auto query = dynamic_query_->query(pos);
  if (!query.ok) {
    RCLCPP_WARN_THROTTLE(logger_,
      *clock_,
      1000,
      "[MincoPlanner] Trajectory safety query failed: %s",
      rog_map::queryStatusName(query.status));
    return false;
  }
  esdf_dist = query.distance;
  esdf_grad = query.gradient;
  (void)esdf_grad;
  if (!std::isfinite(esdf_dist) || esdf_dist <= safe_dist_) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000,
      "[Trajectory collision] clearance=%.6f required=%.6f at (%.3f %.3f %.3f) cost=%u",
      esdf_dist, safe_dist_, pos.x(), pos.y(), pos.z(), static_cast<unsigned int>(cost));
    return false;
  }
  return true;
}

bool TrajectorySafetyChecker::checkTrajectory(const traj_opt::Trajectory & traj) const
{
  if (!ensureQueryAvailable()) {
    return false;
  }
  if (traj.empty()) {
    return false;
  }

  const double dur = traj.getTotalDuration();
  if (!std::isfinite(dur) || dur <= 1e-9) {
    return false;
  }

  const double resolution = dynamic_query_->resolution();
  const double max_velocity = traj.getMaxVelRate();
  if (!std::isfinite(resolution) || resolution <= 1e-6 ||
      !std::isfinite(max_velocity) || max_velocity < 0.0) {
    RCLCPP_WARN(logger_, "[Trajectory collision] invalid resolution/max_velocity %.6f/%.6f", resolution, max_velocity);
    return false;
  }

  const double spatial_dt = max_velocity > 1e-6 ? resolution / max_velocity : sample_dt_;
  const double effective_dt = std::min(sample_dt_, spatial_dt);
  if (!std::isfinite(effective_dt) || effective_dt <= 0.0) {
    return false;
  }

  const double required_intervals = std::ceil(dur / effective_dt);
  if (!std::isfinite(required_intervals) ||
      required_intervals > static_cast<double>(std::numeric_limits<int>::max())) {
    return false;
  }
  const int interval_count = std::max(1, static_cast<int>(required_intervals));
  Eigen::Vector3d previous = traj.getPos(0.0);
  for (int i = 0; i <= interval_count; ++i) {
    const double t = dur * static_cast<double>(i) / static_cast<double>(interval_count);
    const Eigen::Vector3d position = traj.getPos(t);
    const bool point_ok = checkPoint(position);
    const bool edge_ok = dynamic_query_->edgeAllowed(previous, position);
    if (!point_ok || !edge_ok) {
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000,
        "[Trajectory collision] t=%.6f position=(%.3f %.3f %.3f) point_ok=%d edge_ok=%d",
        t, position.x(), position.y(), position.z(), point_ok, edge_ok);
      return false;
    }
    previous = position;
  }
  return true;
}

double TrajectorySafetyChecker::getDistance(const Eigen::Vector3d & pos) const
{
  if (!dynamic_query_) {
    return 0.0;
  }
  double dist = 0.0;
  Eigen::Vector3d grad = Eigen::Vector3d::Zero();
  const auto query = dynamic_query_->query(pos);
  dist = query.distance;
  grad = query.gradient;
  (void)grad;
  return query.ok ? dist : std::numeric_limits<double>::quiet_NaN();
}

bool TrajectorySafetyChecker::projectOutOfObstacle(Eigen::Vector3d & pos, double margin) const
{
  if (!dynamic_query_) {
    return false;
  }
  double esdf_dist = 0.0;
  Eigen::Vector3d esdf_grad = Eigen::Vector3d::Zero();
  const auto query = dynamic_query_->query(pos);
  if (!query.ok) {
    return false;
  }
  esdf_dist = query.distance;
  esdf_grad = query.gradient;
  if (esdf_dist < 0.0 && esdf_grad.norm() > 1e-6) {
    pos += (margin - esdf_dist) * esdf_grad.normalized();
    return true;
  }
  return false;
}

}  // namespace minco_planner
