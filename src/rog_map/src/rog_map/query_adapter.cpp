#include <rog_map/query_adapter.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace rog_map {

namespace {

// 规划可通行判定：只有已经观测到的障碍(254)与膨胀格(253)不可通行。
// UNKNOWN(255 = NO_INFORMATION) 在规划层与 FREE 等价：goal 只是终点点位，不要求它先被观测到，
// 因此未观测区域允许穿越。ROGMap 自身的占据状态不变，UNKNOWN 仍然是 UNKNOWN。
bool isTraversableValue(const unsigned char value)
{
  return value < 253U || value == 255U;  // 255 = NO_INFORMATION = UNKNOWN
}

// 地图覆盖范围之外按 UNKNOWN 处理，同样可通行。
bool isTraversableCell(const MapSnapshot & snap, int x, int y)
{
  if (x < 0 || y < 0 || x >= snap.width || y >= snap.height) {
    return true;
  }
  return isTraversableValue(
    snap.values[static_cast<size_t>(y) * static_cast<size_t>(snap.width) + static_cast<size_t>(x)]);
}

bool sampleBilinear(
  const MapSnapshot & snap, int ix, int iy, double fx, double fy, double & dist, Eigen::Vector3d & grad)
{
  const size_t width = static_cast<size_t>(snap.width);
  const size_t idx00 = static_cast<size_t>(iy) * width + static_cast<size_t>(ix);
  const size_t idx10 = static_cast<size_t>(iy) * width + static_cast<size_t>(ix + 1);
  const size_t idx01 = static_cast<size_t>(iy + 1) * width + static_cast<size_t>(ix);
  const size_t idx11 = static_cast<size_t>(iy + 1) * width + static_cast<size_t>(ix + 1);

  const double d00 = snap.distances[idx00];
  const double d10 = snap.distances[idx10];
  const double d01 = snap.distances[idx01];
  const double d11 = snap.distances[idx11];
  if (!std::isfinite(d00) || !std::isfinite(d10) || !std::isfinite(d01) || !std::isfinite(d11)) {
    return false;
  }

  const double lerp_y0 = (1.0 - fx) * d00 + fx * d10;
  const double lerp_y1 = (1.0 - fx) * d01 + fx * d11;
  dist = (1.0 - fy) * lerp_y0 + fy * lerp_y1;
  if (!std::isfinite(dist)) {
    return false;
  }

  grad.x() = ((1.0 - fy) * (d10 - d00) + fy * (d11 - d01)) / snap.resolution;
  grad.y() = ((1.0 - fx) * (d01 - d00) + fx * (d11 - d10)) / snap.resolution;
  grad.z() = 0.0;
  if (!grad.allFinite()) {
    grad.setZero();
  }
  return true;
}

bool sampleQuadratic(
  const MapSnapshot & snap, int ix, int iy, double fx, double fy, double & dist, Eigen::Vector3d & grad)
{
  if (ix < 1 || iy < 1 || ix + 1 >= snap.width || iy + 1 >= snap.height) {
    return false;
  }
  const size_t width = static_cast<size_t>(snap.width);
  const auto sample = [&snap, width](int x, int y) {
    return snap.distances[static_cast<size_t>(y) * width + static_cast<size_t>(x)];
  };
  const double wx[3] = {0.5 * fx * (fx - 1.0), 1.0 - fx * fx, 0.5 * fx * (fx + 1.0)};
  const double wy[3] = {0.5 * fy * (fy - 1.0), 1.0 - fy * fy, 0.5 * fy * (fy + 1.0)};
  const double dwx[3] = {fx - 0.5, -2.0 * fx, fx + 0.5};
  const double dwy[3] = {fy - 0.5, -2.0 * fy, fy + 0.5};

  dist = 0.0;
  double dd_dx_pix = 0.0;
  double dd_dy_pix = 0.0;
  for (int dy = 0; dy < 3; ++dy) {
    for (int dx = 0; dx < 3; ++dx) {
      const double d = sample(ix + dx - 1, iy + dy - 1);
      if (!std::isfinite(d)) {
        return false;
      }
      dist += wx[dx] * wy[dy] * d;
      dd_dx_pix += dwx[dx] * wy[dy] * d;
      dd_dy_pix += wx[dx] * dwy[dy] * d;
    }
  }
  if (!std::isfinite(dist)) {
    return false;
  }
  grad.x() = dd_dx_pix / snap.resolution;
  grad.y() = dd_dy_pix / snap.resolution;
  grad.z() = 0.0;
  if (!grad.allFinite()) {
    grad.setZero();
  }
  return true;
}

void clampResult(const MapSnapshot & snap, double & dist, Eigen::Vector3d & grad)
{
  if (!std::isfinite(dist)) {
    dist = snap.field_max_distance;
    grad.setZero();
    return;
  }
  if (snap.field_clamp_distance) {
    const double unclamped = dist;
    dist = std::clamp(dist, snap.field_min_distance, snap.field_max_distance);
    if (unclamped != dist) {
      grad.setZero();
    }
  }
  if (!grad.allFinite()) {
    grad.setZero();
  }
}

QueryResult makeResult(const MapSnapshot & snap, QueryStatus status)
{
  QueryResult result;
  result.status = status;
  result.projection_sequence = snap.projection_sequence;
  result.mask_sequence = snap.mask_sequence;
  result.field_sequence = snap.field_sequence;
  result.snapshot_sequence = snap.snapshot_sequence != 0U ? snap.snapshot_sequence : snap.sequence;
  result.field_age_ms =
    snap.field_stamp > 0.0 && snap.stamp >= snap.field_stamp ? (snap.stamp - snap.field_stamp) * 1000.0
                                                             : std::numeric_limits<double>::quiet_NaN();
  return result;
}

}  // namespace

void QueryAdapter::update(
  const std::shared_ptr<const MapSnapshot> & snapshot, const std::shared_ptr<DynamicLayer> & field)
{
  std::lock_guard<std::mutex> lock(mutex_);
  snapshot_ = snapshot;
  field_ = field;
}

std::shared_ptr<const MapSnapshot> QueryAdapter::snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return snapshot_;
}

QueryCounters QueryAdapter::queryCounters() const
{
  std::lock_guard<std::mutex> lock(counters_mutex_);
  return counters_;
}

void recordQueryStatus(QueryCounters & counters, QueryStatus status)
{
  if (status == QueryStatus::OK) {
    ++counters.ok;
    return;
  }
  ++counters.failed;
  switch (status) {
  case QueryStatus::OK:
    break;
  case QueryStatus::OUT_OF_MAP:
    ++counters.out_of_map;
    break;
  case QueryStatus::SNAPSHOT_INVALID:
    ++counters.snapshot_invalid;
    break;
  case QueryStatus::FIELD_UNINITIALIZED:
    ++counters.field_uninitialized;
    break;
  case QueryStatus::INTERPOLATION_FAILED:
    ++counters.interpolation_failed;
    break;
  case QueryStatus::TF_FAILED:
    ++counters.tf_failed;
    break;
  case QueryStatus::NONFINITE_INPUT:
    ++counters.nonfinite_input;
    break;
  case QueryStatus::NONFINITE_OUTPUT:
    ++counters.nonfinite_output;
    break;
  }
}

QueryResult QueryAdapter::recorded(QueryResult result) const
{
  std::lock_guard<std::mutex> lock(counters_mutex_);
  recordQueryStatus(counters_, result.status);
  return result;
}

bool QueryAdapter::worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const
{
  const auto snap = snapshot();
  if (!snap || snap->width <= 0 || snap->height <= 0 || snap->resolution <= 0.0) {
    return false;
  }
  if (wx < snap->origin_x || wy < snap->origin_y) {
    return false;
  }
  const int ix = static_cast<int>(std::floor((wx - snap->origin_x) / snap->resolution));
  const int iy = static_cast<int>(std::floor((wy - snap->origin_y) / snap->resolution));
  if (ix < 0 || iy < 0 || ix >= snap->width || iy >= snap->height) {
    return false;
  }
  mx = static_cast<unsigned int>(ix);
  my = static_cast<unsigned int>(iy);
  return true;
}

void QueryAdapter::mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const
{
  const auto snap = snapshot();
  if (!snap || snap->resolution <= 0.0) {
    wx = 0.0;
    wy = 0.0;
    return;
  }
  wx = snap->origin_x + (static_cast<double>(mx) + 0.5) * snap->resolution;
  wy = snap->origin_y + (static_cast<double>(my) + 0.5) * snap->resolution;
}

unsigned int QueryAdapter::sizeX() const
{
  const auto snap = snapshot();
  return snap ? static_cast<unsigned int>(snap->width) : 0U;
}

unsigned int QueryAdapter::sizeY() const
{
  const auto snap = snapshot();
  return snap ? static_cast<unsigned int>(snap->height) : 0U;
}

double QueryAdapter::resolution() const
{
  const auto snap = snapshot();
  return snap ? snap->resolution : 0.0;
}

double QueryAdapter::originX() const
{
  const auto snap = snapshot();
  return snap ? snap->origin_x : 0.0;
}

double QueryAdapter::originY() const
{
  const auto snap = snapshot();
  return snap ? snap->origin_y : 0.0;
}

uint8_t QueryAdapter::value(unsigned int mx, unsigned int my) const
{
  const auto snap = snapshot();
  if (!snap || mx >= static_cast<unsigned int>(snap->width) ||
      my >= static_cast<unsigned int>(snap->height)) {
    // 地图外按 UNKNOWN 上报，而不是障碍。
    return 255U;  // NO_INFORMATION
  }
  const size_t idx = static_cast<size_t>(my) * static_cast<size_t>(snap->width) + static_cast<size_t>(mx);
  return snap->values[idx];
}

const unsigned char * QueryAdapter::values() const
{
  const auto snap = snapshot();
  if (!snap || snap->values.empty()) {
    return nullptr;
  }
  return snap->values.data();
}

bool QueryAdapter::copyValues(std::vector<unsigned char> & out) const
{
  const auto snap = snapshot();
  if (!snap || snap->values.empty()) {
    out.clear();
    return false;
  }
  out.assign(snap->values.begin(), snap->values.end());
  return true;
}

bool QueryAdapter::isValid(unsigned int mx, unsigned int my) const
{
  const auto snap = snapshot();
  return snap && mx < static_cast<unsigned int>(snap->width) &&
         my < static_cast<unsigned int>(snap->height);
}

bool QueryAdapter::isFree(unsigned int mx, unsigned int my) const
{
  const auto snap = snapshot();
  if (!snap || mx >= static_cast<unsigned int>(snap->width) ||
      my >= static_cast<unsigned int>(snap->height)) {
    return false;
  }
  const size_t idx = static_cast<size_t>(my) * static_cast<size_t>(snap->width) + static_cast<size_t>(mx);
  // 规划语义：只有已经观测到的障碍与膨胀格不可通行，UNKNOWN(255) 与 FREE(0) 一样可通行。
  // ROGMap 自身的占据状态不变，UNKNOWN 仍然是 UNKNOWN。
  return isTraversableValue(snap->values[idx]);
}

bool QueryAdapter::terrain(unsigned int mx, unsigned int my, TerrainCell & cell) const
{
  const auto snap = snapshot();
  if (!snap || snap->terrain_cells.empty() || mx >= static_cast<unsigned int>(snap->width) ||
      my >= static_cast<unsigned int>(snap->height)) { return false; }
  const size_t idx = static_cast<size_t>(my) * snap->width + mx;
  if (idx >= snap->terrain_cells.size()) { return false; }
  cell = snap->terrain_cells[idx];
  return true;
}

bool QueryAdapter::footprintFree(const Eigen::Vector3d & pos) const
{
  unsigned int blocker_x = 0;
  unsigned int blocker_y = 0;
  unsigned char blocker_value = 0;
  return footprintFree(pos, blocker_x, blocker_y, blocker_value);
}

bool QueryAdapter::footprintFree(const Eigen::Vector3d & pos,
  unsigned int & blocker_x,
  unsigned int & blocker_y,
  unsigned char & blocker_value) const
{
  blocker_x = 0;
  blocker_y = 0;
  blocker_value = 0;
  const auto snap = snapshot();
  if (!snap || !snap->terrain_config.enable) { return true; }
  if (!pos.allFinite() || snap->resolution <= 0.0 || !std::isfinite(snap->resolution) ||
      snap->width <= 0 || snap->height <= 0 || snap->terrain_cells.size() != snap->values.size() ||
      snap->values.size() != static_cast<size_t>(snap->width) * snap->height) { return false; }
  const int mx = static_cast<int>(std::floor((pos.x() - snap->origin_x) / snap->resolution));
  const int my = static_cast<int>(std::floor((pos.y() - snap->origin_y) / snap->resolution));
  // 中心在地图外时不代表有障碍，只是还没被覆盖：按 UNKNOWN 处理，允许通行。
  if (mx < 0 || my < 0 || mx >= snap->width || my >= snap->height) { return true; }
  const auto & center = snap->terrain_cells[static_cast<size_t>(my) * snap->width + mx];
  const auto & cfg = snap->terrain_config;
  const bool directional = directionalTerrain(center) && !cfg.yaw_independent_footprint;
  const Eigen::Vector2d along = directional && center.uphill.norm() > 0.1 ?
    center.uphill.normalized().eval() : Eigen::Vector2d(1.0, 0.0);
  const Eigen::Vector2d across(-along.y(), along.x());
  // Expand by a voxel's half diagonal so checking cell centers covers whole obstacle cells.
  const double margin = cfg.clearance_margin + 0.7071067812 * snap->resolution;
  const double radius = (cfg.yaw_independent_footprint ?
    std::max(cfg.flat_clearance_radius, std::hypot(cfg.terrain_half_length, cfg.terrain_half_width)) :
    cfg.flat_clearance_radius) + margin;
  const double length = cfg.terrain_half_length + margin;
  const double width = cfg.terrain_half_width + margin;
  const double extent = directional ? std::hypot(length, width) : radius;
  const int cells = static_cast<int>(std::ceil(extent / snap->resolution)) + 1;
  for (int y = my - cells; y <= my + cells; ++y) {
    for (int x = mx - cells; x <= mx + cells; ++x) {
      const Eigen::Vector2d delta(snap->origin_x + (x + 0.5) * snap->resolution - pos.x(),
        snap->origin_y + (y + 0.5) * snap->resolution - pos.y());
      const bool inside = directional ?
        std::abs(delta.dot(along)) <= length && std::abs(delta.dot(across)) <= width :
        delta.squaredNorm() <= radius * radius;
      // 只有已观测到的障碍/膨胀格阻挡足印；UNKNOWN 与地图外都按可通行处理。
      if (inside && !isTraversableCell(*snap, x, y)) {
        blocker_x = static_cast<unsigned int>(std::max(0, std::min(x, snap->width - 1)));
        blocker_y = static_cast<unsigned int>(std::max(0, std::min(y, snap->height - 1)));
        blocker_value = snap->values[static_cast<size_t>(blocker_y) * snap->width + blocker_x];
        return false;
      }
    }
  }
  return true;
}

bool QueryAdapter::edgeAllowed(const Eigen::Vector3d & from, const Eigen::Vector3d & to) const
{
  const auto snap = snapshot();
  if (!snap || !snap->terrain_config.enable) { return true; }
  if (!from.allFinite() || !to.allFinite() || snap->resolution <= 0.0 ||
      !std::isfinite(snap->resolution) || snap->terrain_cells.size() != snap->values.size()) { return false; }
  const Eigen::Vector2d travel = (to - from).head<2>();
  const int steps = std::max(1, static_cast<int>(std::ceil(travel.norm() / (0.5 * snap->resolution))));
  const TerrainCell * previous = nullptr;
  for (int i = 0; i <= steps; ++i) {
    const Eigen::Vector3d p = from + (to - from) * (static_cast<double>(i) / steps);
    const int x = static_cast<int>(std::floor((p.x() - snap->origin_x) / snap->resolution));
    const int y = static_cast<int>(std::floor((p.y() - snap->origin_y) / snap->resolution));
    // 未覆盖区域按 UNKNOWN 处理：不参与地形/高差判断，也不需要方向对齐。
    if (x < 0 || y < 0 || x >= snap->width || y >= snap->height) { previous = nullptr; continue; }
    const size_t idx = static_cast<size_t>(y) * snap->width + x;
    if (idx >= snap->terrain_cells.size() || !isTraversableValue(snap->values[idx])) { return false; }
    const auto & cell = snap->terrain_cells[idx];
    if (!terrainDirectionAllowed(cell, travel, snap->terrain_config.alignment_cosine)) { return false; }
    // 高差只在两个都已观测出支撑面高度的格子之间比较。
    // 未观测格（support_observed=false 或 surface_z=NaN）按 UNKNOWN 可通行处理：
    // 缺观测不是“高差障碍”的证据，不能拿 NaN 直接拒绝。
    if (previous && previous != &cell &&
        std::isfinite(previous->surface_z) && std::isfinite(cell.surface_z)) {
      const bool step = cell.type == TerrainType::STEP || previous->type == TerrainType::STEP;
      const double max_dz = step ? snap->terrain_config.stair_rise +
        snap->terrain_config.stair_rise_tolerance + snap->resolution :
        std::tan(snap->terrain_config.slope_max_deg * 0.01745329252) *
        1.41421356237 * snap->resolution + 0.5 * snap->resolution;
      if (std::abs(cell.surface_z - previous->surface_z) > max_dz) { return false; }
    }
    previous = &cell;
  }
  return true;
}

double QueryAdapter::softClearance(const Eigen::Vector3d & pos, double fallback) const
{
  const auto snap = snapshot();
  if (!snap || !snap->terrain_config.enable || !pos.allFinite() || snap->resolution <= 0.0) { return fallback; }
  const int x = static_cast<int>(std::floor((pos.x() - snap->origin_x) / snap->resolution));
  const int y = static_cast<int>(std::floor((pos.y() - snap->origin_y) / snap->resolution));
  if (x < 0 || y < 0 || x >= snap->width || y >= snap->height) { return fallback; }
  const size_t idx = static_cast<size_t>(y) * snap->width + x;
  if (idx >= snap->terrain_cells.size()) { return fallback; }
  const auto & cfg = snap->terrain_config;
  // 半径必须与 footprintFree() 完全一致（含 0.7071·resolution 半体素项），否则优化器会
  // 把轨迹优化到 footprintFree 仍然拒绝的距离上。调用方给的 fallback（planner 的
  // minco_optimizer.safe_dist）只作为下限加余量，不能被忽略。
  const double half_voxel = 0.7071067812 * snap->resolution;
  const double hard_clearance = cfg.yaw_independent_footprint ?
    std::max(cfg.flat_clearance_radius, std::hypot(cfg.terrain_half_length, cfg.terrain_half_width)) +
      cfg.clearance_margin + half_voxel :
    (directionalTerrain(snap->terrain_cells[idx]) ?
      std::min(cfg.terrain_half_length, cfg.terrain_half_width) : cfg.flat_clearance_radius) +
      cfg.clearance_margin + half_voxel;
  return std::max(hard_clearance, fallback);
}

TerrainConfig QueryAdapter::terrainPolicy() const
{
  const auto snap = snapshot();
  return snap ? snap->terrain_config : TerrainConfig{};
}

std::chrono::steady_clock::time_point QueryAdapter::lastUpdateSteady() const
{
  const auto snap = snapshot();
  return snap ? snap->update_steady : std::chrono::steady_clock::time_point{};
}

QueryResult QueryAdapter::query(const Eigen::Vector3d & pos) const
{
  // 向规划器暴露二维 ESDF 查询；距离来自 field mask，不直接来自 layer_value。
  // quadratic 在边界或邻域不足时回退到 bilinear，失败用 QueryStatus 显式表达。
  QueryResult invalid;
  if (!pos.allFinite()) {
    invalid.status = QueryStatus::NONFINITE_INPUT;
    return recorded(invalid);
  }

  const auto snap = snapshot();
  if (!snap || snap->width <= 1 || snap->height <= 1 || snap->resolution <= 0.0 ||
      snap->values.size() != static_cast<size_t>(snap->width) * static_cast<size_t>(snap->height)) {
    return recorded(invalid);
  }
  if (snap->distances.size() != static_cast<size_t>(snap->width) * static_cast<size_t>(snap->height)) {
    return recorded(makeResult(*snap, QueryStatus::FIELD_UNINITIALIZED));
  }

  const double px = (pos.x() - snap->origin_x) / snap->resolution;
  const double py = (pos.y() - snap->origin_y) / snap->resolution;
  const int ix = static_cast<int>(std::floor(px));
  const int iy = static_cast<int>(std::floor(py));
  if (ix < 0 || iy < 0 || ix >= snap->width - 1 || iy >= snap->height - 1) {
    return recorded(makeResult(*snap, QueryStatus::OUT_OF_MAP));
  }

  const double fx = px - static_cast<double>(ix);
  const double fy = py - static_cast<double>(iy);

  double dist = std::numeric_limits<double>::quiet_NaN();
  Eigen::Vector3d grad = Eigen::Vector3d::Zero();
  bool ok = false;
  if (snap->interpolation == InterpolationMode::QUADRATIC) {
    ok = sampleQuadratic(*snap, ix, iy, fx, fy, dist, grad);
  }
  if (!ok) {
    ok = sampleBilinear(*snap, ix, iy, fx, fy, dist, grad);
  }
  if (!ok) {
    return recorded(makeResult(*snap, QueryStatus::INTERPOLATION_FAILED));
  }

  clampResult(*snap, dist, grad);
  if (!std::isfinite(dist) || !grad.allFinite()) {
    return recorded(makeResult(*snap, QueryStatus::NONFINITE_OUTPUT));
  }

  QueryResult result = makeResult(*snap, QueryStatus::OK);
  result.ok = true;
  result.distance = dist;
  result.gradient = grad;
  return recorded(result);
}

bool QueryAdapter::evaluate(const Eigen::Vector3d & pos, double & dist, Eigen::Vector3d & grad) const
{
  const QueryResult result = query(pos);
  dist = result.distance;
  grad = result.gradient;
  return result.ok;
}

}  // namespace rog_map
