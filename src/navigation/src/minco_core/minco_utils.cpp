#include "minco_core/minco_utils.hpp"

#include "data_structure/base/trajectory.h"

namespace minco_planner::utils {

double quaternionToYaw(const geometry_msgs::msg::Quaternion & q)
{
  const tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
  return std::isfinite(yaw) ? yaw : 0.0;
}

double calCurvatureDecay(
  double angle, double global_vmax, double deadzone, double saturation, double min_vel, double decay_power)
{
  if (angle < deadzone) {
    return global_vmax;
  }

  const double clamped_angle = std::min(angle, saturation);
  const double ratio = (clamped_angle - deadzone) / saturation;
  const double local_vmax = min_vel + (global_vmax - min_vel) * std::exp(-ratio * decay_power);

  return std::max(local_vmax, min_vel);
}

double getDistFromTrapezoid(
  double t, double total_length, double a_ref, double v_peak, double t_acc, double t_flat, double t_dec)
{
  const double t_total = t_acc + std::max(0.0, t_flat) + std::max(0.0, t_dec);
  if (t >= t_total) {
    return total_length;
  }

  const double d_acc = 0.5 * v_peak * v_peak / a_ref;
  const double d_flat = v_peak * std::max(0.0, t_flat);

  if (t <= t_acc) {
    const double v = a_ref * t;
    return clampValue(0.5 * v * v / a_ref, 0.0, total_length);
  }

  if (t <= t_acc + t_flat) {
    const double dt = t - t_acc;
    const double s = d_acc + v_peak * dt;
    return clampValue(s, 0.0, total_length);
  }

  const double dt = t - (t_acc + t_flat);
  const double v = std::max(0.0, v_peak - a_ref * dt);
  const double s = d_acc + d_flat + (v_peak + v) * 0.5 * dt;
  return clampValue(s, 0.0, total_length);
}

Eigen::Vector3d interpolateByArcLength(
  const std::vector<Eigen::Vector3d> & path, const std::vector<double> & accumulated_dist, double s)
{
  if (path.empty()) {
    return Eigen::Vector3d::Zero();
  }
  if (path.size() == 1 || accumulated_dist.size() != path.size()) {
    return path.front();
  }

  const double s_clamped = clampValue(s, 0.0, accumulated_dist.back());
  auto it = std::lower_bound(accumulated_dist.begin(), accumulated_dist.end(), s_clamped);
  if (it == accumulated_dist.begin()) {
    return path.front();
  }
  if (it == accumulated_dist.end()) {
    return path.back();
  }

  const size_t idx1 = static_cast<size_t>(std::distance(accumulated_dist.begin(), it));
  const size_t idx0 = idx1 - 1;
  const double s0 = accumulated_dist[idx0];
  const double s1 = accumulated_dist[idx1];
  const double denom = (s1 - s0);
  if (denom <= 1e-9) {
    return path[idx1];
  }
  const double ratio = clampValue((s_clamped - s0) / denom, 0.0, 1.0);
  return path[idx0] + ratio * (path[idx1] - path[idx0]);
}

bool lineOfSight(const Eigen::Vector3d & from,
  const Eigen::Vector3d & to,
  const std::shared_ptr<rog_map::MapQueryInterface> & query,
  double los_sampling_ratio)
{
  if (!query || !from.allFinite() || !to.allFinite()) {
    return false;
  }
  const double resolution = query->resolution();
  if (!(std::isfinite(resolution) && resolution > 0.0)) {
    return false;
  }
  const double spacing = std::max(1e-3, resolution * std::max(0.01, los_sampling_ratio));
  const double length = (to - from).head<2>().norm();
  const int samples = std::max(1, static_cast<int>(std::ceil(length / spacing)));

  unsigned int prev_mx = 0;
  unsigned int prev_my = 0;
  bool has_prev_cell = false;

  for (int i = 0; i <= samples; ++i) {
    const Eigen::Vector3d point = from + (to - from) * (static_cast<double>(i) / samples);
    // 足印硬边界：与 validator / cache 校验同一个 oracle。
    if (!query->footprintFree(point)) {
      return false;
    }
    unsigned int mx = 0;
    unsigned int my = 0;
    if (!query->worldToMap(point.x(), point.y(), mx, my)) {
      // 地图外按 UNKNOWN 处理：可通行，不做格检查。
      has_prev_cell = false;
      continue;
    }
    if (has_prev_cell && (mx != prev_mx || my != prev_my)) {
      // 每次跨越到新格都做一次边检查：已知障碍/膨胀、非法地形边都在这层被拒。
      double prev_wx = 0.0;
      double prev_wy = 0.0;
      double cur_wx = 0.0;
      double cur_wy = 0.0;
      query->mapToWorld(prev_mx, prev_my, prev_wx, prev_wy);
      query->mapToWorld(mx, my, cur_wx, cur_wy);
      if (!query->edgeAllowed(Eigen::Vector3d(prev_wx, prev_wy, 0.0),
            Eigen::Vector3d(cur_wx, cur_wy, 0.0))) {
        return false;
      }
    }
    prev_mx = mx;
    prev_my = my;
    has_prev_cell = true;
  }
  return true;
}

std::vector<Eigen::Vector3d> simplifyPathByLOS(const std::vector<Eigen::Vector3d> & raw_path,
  const std::shared_ptr<rog_map::MapQueryInterface> & query,
  double los_sampling_ratio)
{
  std::vector<Eigen::Vector3d> anchors;
  if (raw_path.size() < 2U) {
    return raw_path;
  }
  anchors.push_back(raw_path.front());
  size_t current = 0U;
  const size_t last = raw_path.size() - 1U;
  while (current < last) {
    size_t next = last;
    while (next > current + 1U &&
      !lineOfSight(raw_path[current], raw_path[next], query, los_sampling_ratio)) {
      --next;
    }
    anchors.push_back(raw_path[next]);
    current = next;
  }
  return anchors;
}

std::vector<Eigen::Vector3d> resampleUniform(
  const std::vector<Eigen::Vector3d> & anchors, double spacing)
{
  if (anchors.size() < 2U) {
    return anchors;
  }
  const double step = (std::isfinite(spacing) && spacing > 1e-3) ? spacing : 0.45;
  std::vector<Eigen::Vector3d> seed;
  seed.push_back(anchors.front());
  for (size_t i = 0; i + 1U < anchors.size(); ++i) {
    const Eigen::Vector3d delta = anchors[i + 1U] - anchors[i];
    const double distance = delta.head<2>().norm();
    const int pieces = std::max(1, static_cast<int>(std::ceil(distance / step)));
    for (int k = 1; k <= pieces; ++k) {
      Eigen::Vector3d point = anchors[i] + (static_cast<double>(k) / pieces) * delta;
      point.z() = 0.0;
      if ((point - seed.back()).head<2>().norm() > 1e-6) {
        seed.push_back(point);
      }
    }
  }
  return seed;
}

std::vector<Eigen::Vector3d> computeSeedTangents(const std::vector<Eigen::Vector3d> & seed)
{
  std::vector<Eigen::Vector3d> tangents;
  if (seed.size() < 2U) {
    return tangents;
  }
  tangents.resize(seed.size(), Eigen::Vector3d::UnitX());
  const auto normalized2d = [](const Eigen::Vector3d & v) -> Eigen::Vector3d {
    Eigen::Vector3d t(v.x(), v.y(), 0.0);
    const double norm = t.head<2>().norm();
    return norm > 1e-9 ? Eigen::Vector3d(t.x() / norm, t.y() / norm, 0.0) : Eigen::Vector3d::UnitX();
  };
  tangents.front() = normalized2d(seed[1] - seed[0]);
  tangents.back() = normalized2d(seed.back() - seed[seed.size() - 2U]);
  for (size_t i = 1; i + 1U < seed.size(); ++i) {
    // 中心差分：用跨过该点的方向，避免切线继承分段折角。
    const Eigen::Vector3d central = seed[i + 1U] - seed[i - 1U];
    tangents[i] = central.head<2>().norm() > 1e-9 ? normalized2d(central) : tangents[i - 1U];
  }
  return tangents;
}

std::vector<Eigen::Vector3d> getSparseWaypoints(const std::vector<Eigen::Vector3d> & path,
  double max_vel,
  double max_acc,
  bool goal_reached,
  const std::function<bool(const Eigen::Vector3d &, const Eigen::Vector3d &)> & is_line_free)
{
  std::vector<Eigen::Vector3d> sparse;
  if (path.empty()) {
    return sparse;
  }

  sparse.push_back(path.front());
  if (path.size() < 2) {
    return sparse;
  }
  if (path.size() == 2) {
    sparse.push_back(path.back());
    return sparse;
  }

  // 1) Build arc-length mapping
  std::vector<double> accumulated_dist;
  accumulated_dist.resize(path.size(), 0.0);
  for (size_t i = 1; i < path.size(); ++i) {
    double seg_len = (path[i] - path[i - 1]).head<2>().norm();

    accumulated_dist[i] = accumulated_dist[i - 1] + seg_len;
  }
  const double total_length = accumulated_dist.back();
  if (!(std::isfinite(total_length) && total_length > 1e-3)) {
    sparse.push_back(path.back());
    return sparse;
  }

  // 2) Heuristic trapezoid / triangle velocity profile
  const double v_ref = max_vel;
  const double a_ref = max_acc;

  if (v_ref <= 1e-6) {
    sparse.push_back(path.back());
    return sparse;
  }

  const double d_acc_ref = v_ref * v_ref / (2.0 * a_ref);
  double v_peak = v_ref;
  double t_acc = v_ref / a_ref;
  double t_flat = 0.0;

  double t_dec = goal_reached ? t_acc : 0.0;  // 到达全局终点时保留减速段，否则开放式巡航
  const double d_dec = goal_reached ? d_acc_ref : 0.0;  // 末段减速所需距离
  if (total_length > d_acc_ref + d_dec) {
    const double d_flat = total_length - (d_acc_ref + d_dec);
    t_flat = d_flat / v_ref;
  } else {
    if (goal_reached) {
      v_peak = std::sqrt(std::max(0.0, total_length * a_ref));  // 三角形 (加速+减速)
      t_acc = v_peak / a_ref;
      t_dec = t_acc;
      t_flat = 0.0;
    } else {
      v_peak = std::sqrt(std::max(0.0, 2.0 * total_length * a_ref));  // 纯加速 (v^2 = 2as)
      if (v_peak > v_ref) {
        v_peak = v_ref;
        double d_acc = 0.5 * v_peak * v_peak / a_ref;
        t_acc = v_peak / a_ref;
        t_dec = 0.0;
        t_flat = (total_length - d_acc) / v_ref;
      } else {
        t_acc = v_peak / a_ref;
        t_dec = 0.0;
        t_flat = 0.0;
      }
    }
  }

  const double t_total = t_acc + t_flat + std::max(0.0, t_dec);
  if (!(std::isfinite(t_total) && t_total > 1e-6)) {
    sparse.push_back(path.back());
    return sparse;
  }

  auto arcLengthToIndex = [&accumulated_dist](double s) -> size_t {
    const double s_clamped = clampValue(s, 0.0, accumulated_dist.back());
    auto it = std::lower_bound(accumulated_dist.begin(), accumulated_dist.end(), s_clamped);
    if (it == accumulated_dist.begin()) {
      return 0u;
    }
    if (it == accumulated_dist.end()) {
      return accumulated_dist.size() - 1u;
    }
    const size_t idx1 = static_cast<size_t>(std::distance(accumulated_dist.begin(), it));
    const size_t idx0 = idx1 - 1u;
    const double s0 = accumulated_dist[idx0];
    const double s1 = accumulated_dist[idx1];
    if (!(std::isfinite(s0) && std::isfinite(s1)) || (s1 - s0) <= 1e-12) {
      return idx1;
    }
    return ((s_clamped - s0) < (s1 - s_clamped)) ? idx0 : idx1;
  };

  // Helper: find a corner index within (start_idx, end_idx) by max perpendicular distance
  // to the straight segment (start -> end) in 2D.
  auto findCornerIndex = [&path](size_t start_idx, size_t end_idx) -> size_t {
    if (end_idx <= start_idx + 1u) {
      return start_idx;
    }

    const Eigen::Vector2d a = path[start_idx].head<2>();
    const Eigen::Vector2d b = path[end_idx].head<2>();
    const Eigen::Vector2d ab = b - a;
    const double ab2 = ab.squaredNorm();
    if (ab2 <= 1e-12) {
      return (start_idx + end_idx) / 2u;
    }

    double best_dist2 = -1.0;
    size_t best_k = (start_idx + end_idx) / 2u;
    for (size_t k = start_idx + 1u; k < end_idx; ++k) {
      const Eigen::Vector2d p = path[k].head<2>();
      const double t = clampValue((p - a).dot(ab) / ab2, 0.0, 1.0);
      const Eigen::Vector2d proj = a + t * ab;
      const double dist2 = (p - proj).squaredNorm();
      if (dist2 > best_dist2) {
        best_dist2 = dist2;
        best_k = k;
      }
    }
    return best_k;
  };

  // 3) Build Ideal Indices by time-uniform sampling in trapezoid time, then s(t)->raw index.
  // const double desired_spatial_res = 0.6;
  // const int n_segments_spatial = static_cast<int>(std::ceil(total_length / desired_spatial_res));
  const int n_segments_time = static_cast<int>(std::ceil(t_total / 0.5));
  const int n_segments = std::max(4, n_segments_time);
  const double dt = t_total / static_cast<double>(n_segments);

  std::vector<size_t> target_indices;
  target_indices.reserve(static_cast<size_t>(n_segments + 1));
  target_indices.push_back(0u);

  size_t last_added = 0u;
  for (int i = 1; i < n_segments; ++i) {
    const double t = static_cast<double>(i) * dt;
    const double s = getDistFromTrapezoid(t, total_length, a_ref, v_peak, t_acc, t_flat, t_dec);
    size_t idx = arcLengthToIndex(s);
    // enforce strictly increasing indices to preserve ordering and avoid duplicates
    idx = std::max(idx, last_added);
    if (idx == last_added) {
      if (idx + 1u < path.size()) {
        idx = idx + 1u;
      }
    }
    if (idx > last_added && idx < path.size()) {
      target_indices.push_back(idx);
      last_added = idx;
    }
  }

  const size_t last_idx = path.size() - 1u;
  if (target_indices.empty() || target_indices.back() != last_idx) {
    target_indices.push_back(last_idx);
  }

  // 4) Safety Verification & Repair
  size_t current_safe_idx = 0u;
  for (size_t ti = 1; ti < target_indices.size(); ++ti) {
    const size_t target_idx = target_indices[ti];
    if (target_idx <= current_safe_idx || target_idx >= path.size()) {
      continue;
    }

    // Try to connect current_safe_idx -> target_idx; if collision, insert corner(s).
    size_t guard = 0u;
    while (guard++ < 32u && target_idx > current_safe_idx) {
      const bool line_free = is_line_free ? is_line_free(path[current_safe_idx], path[target_idx]) : true;
      if (line_free) {
        Eigen::Vector3d p = path[target_idx];
        p.z() = 0.0;
        if ((p - sparse.back()).head<2>().norm() > 1e-6) {
          sparse.push_back(p);
        }
        current_safe_idx = target_idx;
        break;
      }

      // Collision: recover a corner point inside (current_safe_idx, target_idx)
      size_t corner_idx = findCornerIndex(current_safe_idx, target_idx);
      if (corner_idx <= current_safe_idx || corner_idx >= target_idx) {
        // Fallback: force progress by inserting the next raw point.
        corner_idx = current_safe_idx + 1u;
        if (corner_idx >= target_idx) {
          // Worst-case: cannot progress further, just stop trying this target.
          break;
        }
      }

      Eigen::Vector3d corner = path[corner_idx];
      corner.z() = 0.0;
      if ((corner - sparse.back()).head<2>().norm() > 1e-6) {
        sparse.push_back(corner);
      }
      current_safe_idx = corner_idx;
    }
  }

  // Ensure goal is included
  if ((path.back() - sparse.back()).head<2>().norm() > 1e-6) {
    Eigen::Vector3d goal = path.back();
    goal.z() = 0.0;
    sparse.push_back(goal);
  }

  return sparse;
}

double LimitLocalVel(const std::vector<Eigen::Vector3d> & sparse_path,
  int seg_idx,
  double global_vmax,
  double deadzone,
  double saturation,
  double min_turn_vel,
  double decay_power)
{
  if (!(std::isfinite(global_vmax) && global_vmax > 0.0)) {
    return 0.0;
  }

  const int n = static_cast<int>(sparse_path.size());
  if (seg_idx < 0 || seg_idx + 2 >= n) {
    return global_vmax;
  }

  const Eigen::Vector2d d1 =
    (sparse_path[static_cast<size_t>(seg_idx + 1)] - sparse_path[static_cast<size_t>(seg_idx)]).head<2>();
  const Eigen::Vector2d d2 =
    (sparse_path[static_cast<size_t>(seg_idx + 2)] - sparse_path[static_cast<size_t>(seg_idx + 1)])
      .head<2>();
  const double n1 = d1.norm();
  const double n2 = d2.norm();
  if (n1 <= 0.2 || n2 <= 0.2) {
    return global_vmax;
  }

  const double dot = clampValue(d1.dot(d2) / (n1 * n2), -1.0, 1.0);
  const double angle = std::acos(dot);
  return calCurvatureDecay(angle, global_vmax, deadzone, saturation, min_turn_vel, decay_power);
}

double ComputeNextSpeed(double v_curr,
  double seg_len,
  double remain_after,
  double amax,
  double local_vmax,
  double terminal_speed)
{
  if (!(std::isfinite(v_curr) && v_curr >= 0.0)) {
    v_curr = 0.0;
  }
  if (!(std::isfinite(seg_len) && seg_len > 0.0)) {
    return 0.0;
  }

  const double a_safe = std::max(1e-3, std::abs(amax));
  double v_next = std::sqrt(std::max(0.0, v_curr * v_curr + 2.0 * a_safe * seg_len));
  if (std::isfinite(local_vmax) && local_vmax > 0.0) {
    v_next = std::min(v_next, local_vmax);
  }

  const double v_terminal =
    (std::isfinite(terminal_speed) && terminal_speed > 0.0) ? terminal_speed : 0.0;
  const double v_cap_terminal = std::sqrt(
    std::max(0.0, v_terminal * v_terminal + 2.0 * a_safe * std::max(0.0, remain_after)));
  v_next = std::min(v_next, v_cap_terminal);

  if (!std::isfinite(v_next) || v_next < 0.0) {
    return 0.0;
  }
  return v_next;
}

double ComputeSegmentTime(
  double seg_len, double v_curr, double v_next, double local_vmax, double amax, double min_seg_time)
{
  const double t_min = std::max(1e-3, min_seg_time);
  double t = 0.0;
  const double v_sum = v_curr + v_next;
  if (v_sum > 1e-3) {
    t = 2.0 * seg_len / v_sum;
  } else {
    t = seg_len / 0.1;
  }

  const double a_safe = std::max(1e-3, std::abs(amax));
  const double required_dist = std::abs(v_curr * v_curr - v_next * v_next) / (2.0 * a_safe);
  if (seg_len >= required_dist) {
    const double t_kinematic = std::abs(v_curr - v_next) / a_safe;
    t = std::max(t, t_kinematic);
  }
  if (local_vmax > 1e-6 && std::abs(v_next - local_vmax) < 1e-6 && v_curr < local_vmax - 1e-6) {
    const double d_acc = (local_vmax * local_vmax - v_curr * v_curr) / (2.0 * a_safe);
    if (std::isfinite(d_acc) && d_acc > 0.0 && seg_len > d_acc) {
      const double t_acc = (local_vmax - v_curr) / a_safe;
      const double t_cruise = (seg_len - d_acc) / local_vmax;
      const double t_alt = t_acc + t_cruise;
      if (std::isfinite(t_alt) && t_alt > 0.0) {
        t = std::max(t, t_alt);
      }
    }
  }

  if (!std::isfinite(t) || t < t_min) {
    return t_min;
  }
  return t;
}

void VelPropogation(const std::vector<double> & seg_len, double amax, std::vector<double> & local_vmaxs)
{
  if (seg_len.empty() || local_vmaxs.empty() || seg_len.size() != local_vmaxs.size()) {
    return;
  }

  const int n = static_cast<int>(seg_len.size());
  const double a_safe = std::max(1e-3, std::abs(amax));

  for (int i = n - 2; i >= 0; --i) {
    const double L = std::max(0.0, seg_len[static_cast<size_t>(i)]);
    const double v_next = std::max(0.0, local_vmaxs[static_cast<size_t>(i + 1)]);
    const double v_cap = std::sqrt(std::max(0.0, v_next * v_next + 2.0 * a_safe * L));
    local_vmaxs[static_cast<size_t>(i)] =
      std::min(std::max(0.0, local_vmaxs[static_cast<size_t>(i)]), v_cap);
  }

  for (int i = 1; i < n; ++i) {
    const double L = std::max(0.0, seg_len[static_cast<size_t>(i - 1)]);
    const double v_prev = std::max(0.0, local_vmaxs[static_cast<size_t>(i - 1)]);
    const double v_cap = std::sqrt(std::max(0.0, v_prev * v_prev + 2.0 * a_safe * L));
    local_vmaxs[static_cast<size_t>(i)] =
      std::min(std::max(0.0, local_vmaxs[static_cast<size_t>(i)]), v_cap);
  }
}

bool isLineFree(
  nav2_costmap_2d::Costmap2D * costmap, const Eigen::Vector3d & p1, const Eigen::Vector3d & p2)
{
  if (!costmap) {
    return true;
  }

  unsigned int mx, my;
  double dist = (p2 - p1).norm();
  int steps = static_cast<int>(std::ceil(dist / costmap->getResolution()));
  if (steps <= 0) {
    return true;
  }

  for (int i = 0; i <= steps; ++i) {
    double t = static_cast<double>(i) / static_cast<double>(steps);
    Eigen::Vector3d p = p1 + (p2 - p1) * t;
    if (costmap->worldToMap(p.x(), p.y(), mx, my)) {
      if (costmap->getCost(mx, my) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        return false;
      }
    }
  }
  return true;
}

bool worldToMap(nav2_costmap_2d::Costmap2D * costmap,
  const rclcpp::Logger & logger,
  double wx,
  double wy,
  unsigned int & mx,
  unsigned int & my)
{
  if (!costmap) {
    RCLCPP_DEBUG(logger, "worldToMap: costmap is null");
    return false;
  }

  if (wx < costmap->getOriginX() || wy < costmap->getOriginY()) {
    RCLCPP_DEBUG(logger,
      "worldToMap: Position (%.2f, %.2f) is before origin (%.2f, %.2f)",
      wx,
      wy,
      costmap->getOriginX(),
      costmap->getOriginY());
    return false;
  }

  double dx = (wx - costmap->getOriginX()) / costmap->getResolution();
  double dy = (wy - costmap->getOriginY()) / costmap->getResolution();

  if (dx < 0.0 || dy < 0.0) {
    RCLCPP_DEBUG(logger, "worldToMap: Computed cell coordinates (%.2f, %.2f) are negative", dx, dy);
    return false;
  }

  int mx_int = static_cast<int>(std::round(dx));
  int my_int = static_cast<int>(std::round(dy));

  if (mx_int < 0 || my_int < 0 || mx_int >= static_cast<int>(costmap->getSizeInCellsX()) ||
      my_int >= static_cast<int>(costmap->getSizeInCellsY())) {
    RCLCPP_DEBUG(logger,
      "worldToMap: Cell coordinates (%d, %d) are out of bounds [0, %u) x [0, %u)",
      mx_int,
      my_int,
      costmap->getSizeInCellsX(),
      costmap->getSizeInCellsY());
    return false;
  }

  mx = static_cast<unsigned int>(mx_int);
  my = static_cast<unsigned int>(my_int);
  return true;
}

void mapToWorld(nav2_costmap_2d::Costmap2D * costmap, double mx, double my, double & wx, double & wy)
{
  if (!costmap) {
    wx = 0.0;
    wy = 0.0;
    return;
  }
  wx = costmap->getOriginX() + mx * costmap->getResolution();
  wy = costmap->getOriginY() + my * costmap->getResolution();
}

void clearRobotCell(nav2_costmap_2d::Costmap2D * costmap, unsigned int mx, unsigned int my)
{
  if (!costmap) {
    return;
  }
  costmap->setCost(mx, my, nav2_costmap_2d::FREE_SPACE);
}
void compensateLeverArm(double v_lidar_x,
  double v_lidar_y,
  double omega_z,
  double yaw,
  double lidar_offset_x,
  double lidar_offset_y,
  double & vx_global,
  double & vy_global,
  double & omega_global)
{
  const double v_body_x = v_lidar_x + omega_z * lidar_offset_y;
  const double v_body_y = v_lidar_y - omega_z * lidar_offset_x;

  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);

  vx_global = v_body_x * cos_yaw - v_body_y * sin_yaw;
  vy_global = v_body_x * sin_yaw + v_body_y * cos_yaw;
  omega_global = omega_z;
}
}  // namespace minco_planner::utils
