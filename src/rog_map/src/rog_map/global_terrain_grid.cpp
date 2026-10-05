#include <rog_map/global_terrain_grid.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>

namespace rog_map {

namespace {
constexpr double kDegToRad = 0.017453292519943295;
constexpr char kElevationLayer[] = "elevation";
constexpr char kObservedLayer[] = "observed";
constexpr char kSupportConfidenceLayer[] = "support_confidence";
constexpr char kObstacleConfidenceLayer[] = "obstacle_confidence";
constexpr char kVoidConfidenceLayer[] = "void_confidence";
constexpr char kTraversabilityLayer[] = "traversability";
constexpr char kBlockedLayer[] = "blocked";

double nowSeconds()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
}  // namespace

ColumnFreeEvidence evaluateColumnFreeEvidence(const CellData & column,
  double z_ref,
  double resolution,
  const GlobalTerrainConfig & config)
{
  ColumnFreeEvidence evidence;
  const int bits = static_cast<int>(column.z_mask_bits);
  if (bits <= 0 || !std::isfinite(z_ref) || resolution <= 0.0) {
    return evidence;
  }
  const int max_bit = std::min(bits, 64) - 1;
  // 绝对高度 -> 位图下标（位 0 = column.z_mask_origin）。
  const auto bit_of = [&column, resolution](double z) {
    return static_cast<int>(std::llround((z - static_cast<double>(column.z_mask_origin)) /
      resolution));
  };
  const auto band_counts = [&](double z_lo, double z_hi, int & free_count, int & occupied_count,
                               int & unknown_count) {
    int lo = bit_of(z_lo);
    int hi = bit_of(z_hi);
    if (lo > hi) {
      std::swap(lo, hi);
    }
    lo = std::max(0, lo);
    hi = std::min(max_bit, hi);
    free_count = 0;
    occupied_count = 0;
    unknown_count = 0;
    for (int bit = lo; bit <= hi; ++bit) {
      const uint64_t mask = 1ULL << bit;
      if ((column.free_z_mask & mask) != 0ULL) {
        ++free_count;
      } else if ((column.occupied_z_mask & mask) != 0ULL) {
        ++occupied_count;
      } else {
        ++unknown_count;
      }
    }
    return hi >= lo ? (hi - lo + 1) : 0;
  };

  const int support_bits = band_counts(z_ref - config.support_band_down, z_ref + config.support_band_up,
    evidence.free_count, evidence.occupied_count, evidence.unknown_count);
  int below_free = 0;
  int below_occupied = 0;
  int below_unknown = 0;
  const int below_bits = band_counts(z_ref - config.drop_probe_depth, z_ref - config.support_band_down,
    below_free, below_occupied, below_unknown);

  const int min_free = std::max(1, config.min_free_voxels);
  evidence.support_band_observed = support_bits > 0 && evidence.free_count >= min_free;
  evidence.below_band_observed = below_bits > 0 && below_free >= min_free;
  evidence.support_hit_near_z_ref = evidence.occupied_count > 0;
  evidence.free_ratio = support_bits > 0 ?
    static_cast<float>(evidence.free_count) / static_cast<float>(support_bits) : 0.0F;
  evidence.strong_no_support = evidence.support_band_observed && evidence.below_band_observed &&
    !evidence.support_hit_near_z_ref;
  return evidence;
}

void GlobalTerrainGrid::configure(const GlobalTerrainConfig & config)
{
  std::lock_guard<std::mutex> lock(mutex_);
  cfg_ = config;
  if (!cfg_.enable) {
    width_ = height_ = 0;
    cells_.clear();
    values_.clear();
    mask_.clear();
    blocked_.clear();
    field_.reset();
    grid_map_ = grid_map::GridMap{};
    return;
  }
  if (cfg_.resolution <= 0.0 || cfg_.size_x <= 0.0 || cfg_.size_y <= 0.0) {
    cfg_.enable = false;
    width_ = height_ = 0;
    cells_.clear();
    values_.clear();
    mask_.clear();
    blocked_.clear();
    field_.reset();
    grid_map_ = grid_map::GridMap{};
    return;
  }
  width_ = std::max(1, static_cast<int>(std::llround(cfg_.size_x / cfg_.resolution)));
  height_ = std::max(1, static_cast<int>(std::llround(cfg_.size_y / cfg_.resolution)));
  const size_t count = static_cast<size_t>(width_) * static_cast<size_t>(height_);
  cells_.assign(count, GlobalTerrainCell{});
  grid_map_ = grid_map::GridMap{};
  grid_map_.setGeometry(
    grid_map::Length(width_ * cfg_.resolution, height_ * cfg_.resolution),
    cfg_.resolution,
    grid_map::Position(
      cfg_.origin_x + 0.5 * width_ * cfg_.resolution,
      cfg_.origin_y + 0.5 * height_ * cfg_.resolution));
  grid_map_.add(kElevationLayer, std::numeric_limits<float>::quiet_NaN());
  grid_map_.add(kObservedLayer, 0.0);
  grid_map_.add(kSupportConfidenceLayer, 0.0);
  grid_map_.add(kObstacleConfidenceLayer, 0.0);
  grid_map_.add(kVoidConfidenceLayer, 0.0);
  grid_map_.add(kTraversabilityLayer, 255.0);
  grid_map_.add(kBlockedLayer, 0.0);
  values_.assign(count, 255U);
  mask_.assign(count, 1U);
  blocked_.assign(count, 0U);
  field_ = std::make_shared<DynamicLayer>();
  field_dirty_ = true;
  last_field_time_ = -std::numeric_limits<double>::infinity();
}

bool GlobalTerrainGrid::enabled() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return cfg_.enable && width_ > 0 && height_ > 0;
}

const GlobalTerrainConfig & GlobalTerrainGrid::config() const
{
  return cfg_;
}

bool GlobalTerrainGrid::inBounds(int ix, int iy) const
{
  return ix >= 0 && iy >= 0 && ix < width_ && iy < height_;
}

size_t GlobalTerrainGrid::idxOf(int ix, int iy) const
{
  return static_cast<size_t>(iy) * static_cast<size_t>(width_) + static_cast<size_t>(ix);
}

int GlobalTerrainGrid::indexOf(unsigned int mx, unsigned int my) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!inBounds(static_cast<int>(mx), static_cast<int>(my))) {
    return -1;
  }
  return static_cast<int>(idxOf(static_cast<int>(mx), static_cast<int>(my)));
}

bool GlobalTerrainGrid::cellIndex(double wx, double wy, int & ix, int & iy) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!cfg_.enable || cfg_.resolution <= 0.0) {
    return false;
  }
  ix = static_cast<int>(std::floor((wx - cfg_.origin_x) / cfg_.resolution));
  iy = static_cast<int>(std::floor((wy - cfg_.origin_y) / cfg_.resolution));
  return inBounds(ix, iy);
}

void GlobalTerrainGrid::updateFromLocal(const ProjectionLayer & projection,
  const TerrainLayer & terrain,
  double now,
  const Eigen::Vector3d & robot_position)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    applyLocalPatchLocked(projection, terrain, now, robot_position);
  }
  // EDT 较贵，放在锁外按周期重建。
  rebuildFieldIfNeeded();
}

void GlobalTerrainGrid::applyLocalPatchLocked(const ProjectionLayer & projection,
  const TerrainLayer & terrain,
  double now,
  const Eigen::Vector3d & robot_position)
{
  (void)robot_position;
  const auto start = std::chrono::steady_clock::now();
  if (!cfg_.enable || projection.empty() || width_ <= 0 || height_ <= 0) {
    return;
  }
  const int lw = projection.width();
  const int lh = projection.height();
  const double lres = projection.resolution();
  if (lw <= 0 || lh <= 0 || std::abs(lres - cfg_.resolution) > 1e-9) {
    // 第一版要求局部与全局同分辨率；不同分辨率需要面积累积，本轮不做。
    return;
  }
  const auto & pcells = projection.cells();
  const auto & tcells = terrain.cells();
  const auto & tvalues = terrain.values();
  const size_t lcount = static_cast<size_t>(lw) * static_cast<size_t>(lh);
  if (pcells.size() < lcount || tcells.size() < lcount || tvalues.size() < lcount) {
    return;
  }
  const Eigen::Vector2d origin = projection.origin();
  bool changed = false;
  int patch_min_x = width_;
  int patch_max_x = -1;
  int patch_min_y = height_;
  int patch_max_y = -1;
  for (int y = 0; y < lh; ++y) {
    for (int x = 0; x < lw; ++x) {
      const size_t l = static_cast<size_t>(y) * static_cast<size_t>(lw) + static_cast<size_t>(x);
      const CellData & local = pcells[l];
      const TerrainCell & tcell = tcells[l];
      const uint8_t tvalue = tvalues[l];
      // 只有拿到真实测量的列才产生证据：投影层 confidence>=1 表示该列至少有一个
      // KNOWN_FREE/OCCUPIED 体素（射线经过或命中）。未观测列不做任何修改，
      // 历史地图因此得以保留。
      const bool measured = local.confidence >= 1.0f;
      const bool step_now = tcell.type == TerrainType::STEP;
      const bool obstacle_now = (tvalue == 254U) || step_now;
      if (!measured && !obstacle_now) {
        continue;
      }
      const double wx = origin.x() + (static_cast<double>(x) + 0.5) * lres;
      const double wy = origin.y() + (static_cast<double>(y) + 0.5) * lres;
      const int gx = static_cast<int>(std::floor((wx - cfg_.origin_x) / cfg_.resolution));
      const int gy = static_cast<int>(std::floor((wy - cfg_.origin_y) / cfg_.resolution));
      if (!inBounds(gx, gy)) {
        continue;
      }
      GlobalTerrainCell & cell = cells_[idxOf(gx, gy)];
      cell.last_update = now;
      changed = true;
      patch_min_x = std::min(patch_min_x, gx);
      patch_max_x = std::max(patch_max_x, gx);
      patch_min_y = std::min(patch_min_y, gy);
      patch_max_y = std::max(patch_max_y, gy);

      if (obstacle_now) {
        cell.obstacle_confidence = std::min(1.0f, cell.obstacle_confidence + cfg_.obstacle_hit_gain);
      } else if (measured) {
        // 射线经过/重新观测为空：快速降低障碍置信度（动态障碍清除的唯一来源）。
        cell.obstacle_confidence = std::max(0.0f, cell.obstacle_confidence - cfg_.obstacle_miss_gain);
      }

      const bool support_now = !obstacle_now && std::isfinite(tcell.surface_z);
      if (support_now) {
        cell.surface_z = tcell.surface_z;
        const float gain = tcell.support_observed ? cfg_.support_hit_gain : cfg_.support_seed_gain;
        cell.support_confidence = std::min(1.0f, cell.support_confidence + gain);
        // 只有真实测量（而不是足印补种）才算“观测到这一格”。
        cell.observed = true;
        cell.void_confidence = 0.0f;
      } else if (measured) {
        cell.support_confidence = std::max(0.0f, cell.support_confidence - cfg_.support_miss_gain);
        // 负障碍（drop-off）证据：只有“A 有可靠 support + B 在 z_ref 附近的 support band
        // 被真实 ray miss 确认 FREE + z_ref 下方 also FREE + band 内没有支撑回波”才算
        // 一帧强 no-support。support band 还是 UNKNOWN 时不下任何结论（不是 BLOCKED）。
        const bool free_projection = local.type == CellType::FREE || local.type == CellType::PASSABLE;
        if (free_projection) {
          // z_ref = 预计地面高度：第一版直接取邻居支撑面的高度（平地/缓坡/平台都适用），
          // 不用固定的 global z，也不用 robot_z。
          double z_ref = std::numeric_limits<double>::quiet_NaN();
          const int offsets4[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
          for (const auto & offset : offsets4) {
            const int lx = x + offset[0];
            const int ly = y + offset[1];
            if (lx < 0 || ly < 0 || lx >= lw || ly >= lh) {
              continue;
            }
            const TerrainCell & neighbor =
              tcells[static_cast<size_t>(ly) * static_cast<size_t>(lw) + static_cast<size_t>(lx)];
            if (std::isfinite(neighbor.surface_z)) {
              z_ref = neighbor.surface_z;
              break;
            }
          }
          if (std::isfinite(z_ref)) {
            const ColumnFreeEvidence evidence = evaluateColumnFreeEvidence(local, z_ref, lres, cfg_);
            // 这一列相对于已知支撑面被观测过，才让它在全局图里算 observed。
            cell.observed = true;
            if (evidence.strong_no_support) {
              cell.void_confidence = std::min(1.0f, cell.void_confidence + cfg_.void_gain);
            }
            // 没有强证据就不加：UNKNOWN 不会因为“没点”变成 BLOCKED。
          }
        }
      }
      if (obstacle_now) {
        cell.observed = true;
        cell.void_confidence = 0.0f;
      }

      // 迟滞：避免边界格反复翻转。
      if (!cell.obstacle && cell.obstacle_confidence >= cfg_.obstacle_enter) {
        cell.obstacle = true;
      } else if (cell.obstacle && cell.obstacle_confidence <= cfg_.obstacle_exit) {
        cell.obstacle = false;
      }
      if (!cell.support_valid && cell.support_confidence >= cfg_.support_enter) {
        cell.support_valid = true;
      } else if (cell.support_valid && cell.support_confidence <= cfg_.support_exit) {
        cell.support_valid = false;
        cell.surface_z = std::numeric_limits<float>::quiet_NaN();
      }
      syncCellToGridMapLocked(gx, gy);
    }
  }

  if (!changed) {
    stats_.last_update_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    return;
  }

  // 只重算被这一帧 patch 影响到的世界格（外扩一圈，因为边状态会波及邻格）。
  // 这就是“global map + local compute”：不做全图刷新。
  recomputeRegion(patch_min_x - 1, patch_max_x + 1, patch_min_y - 1, patch_max_y + 1);
  field_dirty_ = true;
  last_update_steady_ = std::chrono::steady_clock::now();
  ++stats_.updates;
  if (stats_.updates % 40 == 0) {
    recountStats();
  }
  stats_.last_update_ms =
    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
}

void GlobalTerrainGrid::recomputeRegion(int x0, int x1, int y0, int y1)
{
  if (width_ <= 0 || height_ <= 0) {
    return;
  }
  x0 = std::max(0, x0);
  y0 = std::max(0, y0);
  x1 = std::min(width_ - 1, x1);
  y1 = std::min(height_ - 1, y1);
  const int offsets[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
  for (int y = y0; y <= y1; ++y) {
    for (int x = x0; x <= x1; ++x) {
      const size_t i = idxOf(x, y);
      values_[i] = cells_[i].obstacle ? 254U : (cells_[i].observed ? 0U : 255U);
      bool blocked = cells_[i].obstacle;
      for (const auto & offset : offsets) {
        if (!blocked && edgeStateLocked(x, y, x + offset[0], y + offset[1]) == EdgeState::BLOCKED) {
          blocked = true;
        }
      }
      blocked_[i] = blocked ? 1U : 0U;
      mask_[i] = blocked_[i] ? 0U : 1U;
      grid_map_.at(kTraversabilityLayer, grid_map::Index(x, y)) =
        static_cast<float>(values_[i]);
      grid_map_.at(kBlockedLayer, grid_map::Index(x, y)) =
        static_cast<float>(blocked_[i]);
    }
  }

}

void GlobalTerrainGrid::syncCellToGridMapLocked(int ix, int iy)
{
  if (!inBounds(ix, iy) || grid_map_.getSize()(0) != width_ ||
      grid_map_.getSize()(1) != height_) {
    return;
  }
  const GlobalTerrainCell & cell = cells_[idxOf(ix, iy)];
  const grid_map::Index index(ix, iy);
  grid_map_.at(kElevationLayer, index) = cell.surface_z;
  grid_map_.at(kObservedLayer, index) = cell.observed ? 1.0F : 0.0F;
  grid_map_.at(kSupportConfidenceLayer, index) = cell.support_confidence;
  grid_map_.at(kObstacleConfidenceLayer, index) = cell.obstacle_confidence;
  grid_map_.at(kVoidConfidenceLayer, index) = cell.void_confidence;
}

void GlobalTerrainGrid::recountStats()
{
  uint64_t obstacles = 0, supports = 0, observed = 0, blocked = 0;
  for (size_t i = 0; i < cells_.size(); ++i) {
    if (cells_[i].obstacle) {
      ++obstacles;
    }
    if (cells_[i].support_valid) {
      ++supports;
    }
    if (cells_[i].observed) {
      ++observed;
    }
    if (blocked_[i]) {
      ++blocked;
    }
  }
  stats_.obstacle_cells = obstacles;
  stats_.support_cells = supports;
  stats_.observed_cells = observed;
  stats_.blocked_edge_cells = blocked > obstacles ? blocked - obstacles : 0;
}

std::chrono::steady_clock::time_point GlobalTerrainGrid::lastUpdateSteady() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return last_update_steady_;
}

void GlobalTerrainGrid::invalidateField()
{
  std::lock_guard<std::mutex> lock(mutex_);
  field_dirty_ = true;
  last_field_time_ = -std::numeric_limits<double>::infinity();
}

// 必须在 updateFromLocal 末尾（不持锁）调用：全局 40x40 的 EDT 较贵，按周期重建。
void GlobalTerrainGrid::rebuildFieldIfNeeded()
{
  std::shared_ptr<DynamicLayer> field;
  std::vector<uint8_t> mask;
  int width = 0, height = 0;
  double resolution = 0.0;
  Eigen::Vector2d origin = Eigen::Vector2d::Zero();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!cfg_.enable || !field_ || !cfg_.field_enable || !field_dirty_) {
      return;
    }
    const double now = nowSeconds();
    if (last_field_time_ > 0.0 && now - last_field_time_ < cfg_.field_update_period_sec) {
      return;
    }
    field = field_;
    mask = mask_;
    width = width_;
    height = height_;
    resolution = cfg_.resolution;
    origin = Eigen::Vector2d(cfg_.origin_x, cfg_.origin_y);
    stats_.field_rebuilds += 1;
    field_dirty_ = false;
    last_field_time_ = now;
  }
  if (width <= 0 || height <= 0 || !field) {
    return;
  }
  const auto start = std::chrono::steady_clock::now();
  field->update(width,
    height,
    resolution,
    origin,
    mask,
    0.0,
    cfg_.field_max_distance,
    cfg_.field_min_distance,
    cfg_.field_clamp_distance,
    InterpolationMode::BILINEAR);
  const double ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - start).count();
  std::lock_guard<std::mutex> lock(mutex_);
  stats_.last_field_ms = ms;
}

void GlobalTerrainGrid::mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  wx = cfg_.origin_x + (static_cast<double>(mx) + 0.5) * cfg_.resolution;
  wy = cfg_.origin_y + (static_cast<double>(my) + 0.5) * cfg_.resolution;
}

bool GlobalTerrainGrid::worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!cfg_.enable || cfg_.resolution <= 0.0) {
    return false;
  }
  const int ix = static_cast<int>(std::floor((wx - cfg_.origin_x) / cfg_.resolution));
  const int iy = static_cast<int>(std::floor((wy - cfg_.origin_y) / cfg_.resolution));
  if (!inBounds(ix, iy)) {
    return false;
  }
  mx = static_cast<unsigned int>(ix);
  my = static_cast<unsigned int>(iy);
  return true;
}

unsigned int GlobalTerrainGrid::sizeX() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return static_cast<unsigned int>(std::max(0, width_));
}

unsigned int GlobalTerrainGrid::sizeY() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return static_cast<unsigned int>(std::max(0, height_));
}

double GlobalTerrainGrid::resolution() const { return cfg_.resolution; }
double GlobalTerrainGrid::originX() const { return cfg_.origin_x; }
double GlobalTerrainGrid::originY() const { return cfg_.origin_y; }

uint8_t GlobalTerrainGrid::value(unsigned int mx, unsigned int my) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!inBounds(static_cast<int>(mx), static_cast<int>(my))) {
    return 255U;
  }
  return values_[idxOf(static_cast<int>(mx), static_cast<int>(my))];
}

const unsigned char * GlobalTerrainGrid::values() const { return values_.data(); }

bool GlobalTerrainGrid::copyValues(std::vector<unsigned char> & out) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  out.assign(values_.begin(), values_.end());
  return !out.empty();
}

bool GlobalTerrainGrid::isValid(unsigned int mx, unsigned int my) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return inBounds(static_cast<int>(mx), static_cast<int>(my));
}

bool GlobalTerrainGrid::cellTraversable(unsigned int mx, unsigned int my) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!inBounds(static_cast<int>(mx), static_cast<int>(my))) {
    return true;  // 地图外按 UNKNOWN 处理，乐观允许。
  }
  // UNKNOWN -> true, FREE -> true, OCCUPIED -> false
  return !cells_[idxOf(static_cast<int>(mx), static_cast<int>(my))].obstacle;
}

bool GlobalTerrainGrid::isFree(unsigned int mx, unsigned int my) const
{
  return cellTraversable(mx, my);
}

EdgeState GlobalTerrainGrid::edgeStateLocked(int ax, int ay, int bx, int by) const
{
  if (!inBounds(ax, ay) || !inBounds(bx, by)) {
    return EdgeState::UNKNOWN;
  }
  const GlobalTerrainCell & a = cells_[idxOf(ax, ay)];
  const GlobalTerrainCell & b = cells_[idxOf(bx, by)];
  // 情况 C：任一侧未观测 -> UNKNOWN（乐观允许）。
  if (!a.observed || !b.observed) {
    return EdgeState::UNKNOWN;
  }
  if (a.obstacle || b.obstacle) {
    return EdgeState::BLOCKED;
  }
  if (a.support_valid && b.support_valid) {
    const double distance = std::hypot(static_cast<double>(bx - ax), static_cast<double>(by - ay)) *
                            cfg_.resolution;
    if (distance <= 1e-9 || !std::isfinite(a.surface_z) || !std::isfinite(b.surface_z)) {
      return EdgeState::BLOCKED;
    }
    // 情况 A：高度梯度超过最大坡道梯度 -> BLOCKED。
    // 支撑面高度是体素量化的（surface_z 取的是占据体素中心 z），相邻平地格天然可能差
    // 一个体素（0.05 m / 0.05 m = 梯度 1.0 > tan35°），所以必须留一个体素的余量，
    // 否则整片平地会被打成棋盘状的 BLOCKED 边。
    const double dz =
      std::abs(static_cast<double>(a.surface_z) - static_cast<double>(b.surface_z));
    const double max_dz = std::tan(cfg_.slope_max_deg * kDegToRad) * distance + cfg_.resolution;
    return dz <= max_dz ? EdgeState::PASSABLE : EdgeState::BLOCKED;
  }
  // 情况 B：一侧有支撑面、另一侧被多帧强 no-support 证据确认（悬空/坑/坡道侧边）-> BLOCKED。
  if (a.support_valid && !b.support_valid && b.void_confidence >= cfg_.void_confirm) {
    return EdgeState::BLOCKED;
  }
  if (b.support_valid && !a.support_valid && a.void_confidence >= cfg_.void_confirm) {
    return EdgeState::BLOCKED;
  }
  // 一侧有支撑、另一侧尚未确认悬空：证据不足 -> UNKNOWN（乐观允许）。
  if (a.support_valid != b.support_valid) {
    return EdgeState::UNKNOWN;
  }
  // 两侧都观测到但没有支撑面：没有足够信息判定高差 -> UNKNOWN。
  return EdgeState::UNKNOWN;
}

EdgeState GlobalTerrainGrid::edgeState(unsigned int ax, unsigned int ay, unsigned int bx,
  unsigned int by) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return edgeStateLocked(static_cast<int>(ax), static_cast<int>(ay), static_cast<int>(bx),
    static_cast<int>(by));
}

bool GlobalTerrainGrid::edgeAllowedCells(unsigned int ax, unsigned int ay, unsigned int bx,
  unsigned int by) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return edgeStateLocked(static_cast<int>(ax), static_cast<int>(ay), static_cast<int>(bx),
    static_cast<int>(by)) != EdgeState::BLOCKED;
}

bool GlobalTerrainGrid::blockedCellLocked(size_t idx) const
{
  return idx < blocked_.size() && blocked_[idx] != 0U;
}

bool GlobalTerrainGrid::edgeAllowed(const Eigen::Vector3d & from, const Eigen::Vector3d & to) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!cfg_.enable) {
    return true;
  }
  if (!from.allFinite() || !to.allFinite()) {
    return false;
  }
  const Eigen::Vector2d travel = (to - from).head<2>();
  const int steps = std::max(1, static_cast<int>(std::ceil(travel.norm() / (0.5 * cfg_.resolution))));
  int prev_x = 0, prev_y = 0;
  bool has_prev = false;
  for (int i = 0; i <= steps; ++i) {
    const Eigen::Vector3d p = from + (to - from) * (static_cast<double>(i) / steps);
    const int x = static_cast<int>(std::floor((p.x() - cfg_.origin_x) / cfg_.resolution));
    const int y = static_cast<int>(std::floor((p.y() - cfg_.origin_y) / cfg_.resolution));
    if (!inBounds(x, y)) {
      has_prev = false;
      continue;  // 地图外 UNKNOWN：允许
    }
    // 注意：这里已经在锁内，不能调用 cellTraversable()（它会再次加同一把非递归锁）。
    if (cells_[idxOf(x, y)].obstacle) {
      return false;
    }
    if (has_prev && (x != prev_x || y != prev_y)) {
      if (edgeStateLocked(prev_x, prev_y, x, y) == EdgeState::BLOCKED) {
        return false;
      }
    }
    prev_x = x;
    prev_y = y;
    has_prev = true;
  }
  return true;
}

bool GlobalTerrainGrid::terrain(unsigned int mx, unsigned int my, TerrainCell & cell) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  cell = TerrainCell{};
  if (!cfg_.enable || !inBounds(static_cast<int>(mx), static_cast<int>(my))) {
    return false;
  }
  const size_t idx = idxOf(static_cast<int>(mx), static_cast<int>(my));
  const GlobalTerrainCell & c = cells_[idx];
  if (!c.observed) {
    return true;  // type 保持 UNKNOWN
  }
  cell.support_observed = c.support_valid;
  cell.surface_z = c.surface_z;
  if (c.obstacle) {
    cell.type = TerrainType::OBSTACLE;
    return true;
  }
  if (!c.support_valid || !std::isfinite(c.surface_z)) {
    cell.type = TerrainType::UNKNOWN;
    return true;
  }
  // 由相邻支撑面中心差分得到坡度方向，供 MINCO 的走向对齐项使用。
  double gx = 0.0, gy = 0.0;
  int samples = 0;
  const int offsets[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
  for (const auto & offset : offsets) {
    const int nx = static_cast<int>(mx) + offset[0];
    const int ny = static_cast<int>(my) + offset[1];
    if (!inBounds(nx, ny)) {
      continue;
    }
    const GlobalTerrainCell & n = cells_[idxOf(nx, ny)];
    if (!n.support_valid || !std::isfinite(n.surface_z)) {
      continue;
    }
    gx += (static_cast<double>(n.surface_z) - static_cast<double>(c.surface_z)) /
          (static_cast<double>(offset[0]) * cfg_.resolution);
    gy += (static_cast<double>(n.surface_z) - static_cast<double>(c.surface_z)) /
          (static_cast<double>(offset[1]) * cfg_.resolution);
    ++samples;
  }
  if (samples == 0) {
    cell.type = TerrainType::FLAT;
    return true;
  }
  const double gradient = std::hypot(gx, gy);
  if (gradient > std::tan(cfg_.slope_min_deg * kDegToRad)) {
    cell.type = TerrainType::SLOPE;
    if (gradient > 1e-9) {
      cell.uphill = Eigen::Vector2d(gx / gradient, gy / gradient);
    }
  } else {
    cell.type = TerrainType::FLAT;
  }
  return true;
}

TerrainConfig GlobalTerrainGrid::terrainPolicy() const
{
  TerrainConfig policy;
  policy.enable = cfg_.enable;
  policy.slope_min_deg = cfg_.slope_min_deg;
  policy.slope_max_deg = cfg_.slope_max_deg;
  policy.alignment_cosine = cfg_.alignment_cosine;
  policy.alignment_weight = cfg_.alignment_weight;
  policy.flat_clearance_radius = cfg_.flat_clearance_radius;
  policy.terrain_half_length = cfg_.terrain_half_length;
  policy.terrain_half_width = cfg_.terrain_half_width;
  policy.yaw_independent_footprint = cfg_.yaw_independent_footprint;
  policy.clearance_margin = cfg_.clearance_margin;
  return policy;
}

double GlobalTerrainGrid::softClearance(const Eigen::Vector3d & pos, double fallback) const
{
  (void)pos;
  if (!cfg_.enable) {
    return fallback;
  }
  const double half_voxel = 0.7071067812 * cfg_.resolution;
  const double hard_clearance = cfg_.yaw_independent_footprint ?
    std::max(cfg_.flat_clearance_radius,
      std::hypot(cfg_.terrain_half_length, cfg_.terrain_half_width)) +
      cfg_.clearance_margin + half_voxel :
    cfg_.flat_clearance_radius + cfg_.clearance_margin + half_voxel;
  return std::max(hard_clearance, fallback);
}

bool GlobalTerrainGrid::footprintFree(const Eigen::Vector3d & pos) const
{
  // 足迹检查会被 A*/LOS 高频调用（每 0.025 m 一次），因此不在锁内做整个圆盘扫描：
  // 锁内只把窗口内的 blocked_ 拷出来，扫描在锁外完成。
  struct Window
  {
    int x0{0}, y0{0}, w{0}, h{0};
    double origin_x{0.0}, origin_y{0.0}, resolution{0.05}, radius{0.0};
    std::vector<uint8_t> blocked;
  } win;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!cfg_.enable) {
      return true;
    }
    if (!pos.allFinite() || cfg_.resolution <= 0.0) {
      return false;
    }
    const int mx = static_cast<int>(std::floor((pos.x() - cfg_.origin_x) / cfg_.resolution));
    const int my = static_cast<int>(std::floor((pos.y() - cfg_.origin_y) / cfg_.resolution));
    if (!inBounds(mx, my)) {
      return true;  // 地图外按 UNKNOWN 处理
    }
    const double margin = cfg_.clearance_margin + 0.7071067812 * cfg_.resolution;
    const double radius = (cfg_.yaw_independent_footprint ?
      std::max(cfg_.flat_clearance_radius,
        std::hypot(cfg_.terrain_half_length, cfg_.terrain_half_width)) :
      cfg_.flat_clearance_radius) + margin;
    const int cells = static_cast<int>(std::ceil(radius / cfg_.resolution)) + 1;
    win.x0 = std::max(0, mx - cells);
    win.y0 = std::max(0, my - cells);
    const int x1 = std::min(width_ - 1, mx + cells);
    const int y1 = std::min(height_ - 1, my + cells);
    if (x1 < win.x0 || y1 < win.y0) {
      return true;
    }
    win.w = x1 - win.x0 + 1;
    win.h = y1 - win.y0 + 1;
    win.origin_x = cfg_.origin_x;
    win.origin_y = cfg_.origin_y;
    win.resolution = cfg_.resolution;
    win.radius = radius;
    win.blocked.resize(static_cast<size_t>(win.w) * static_cast<size_t>(win.h));
    for (int y = 0; y < win.h; ++y) {
      const size_t src = idxOf(win.x0, win.y0 + y);
      std::memcpy(win.blocked.data() + static_cast<size_t>(y) * static_cast<size_t>(win.w),
        blocked_.data() + src, static_cast<size_t>(win.w));
    }
  }
  const double r2 = win.radius * win.radius;
  for (int y = 0; y < win.h; ++y) {
    for (int x = 0; x < win.w; ++x) {
      if (win.blocked[static_cast<size_t>(y) * static_cast<size_t>(win.w) +
            static_cast<size_t>(x)] == 0U) {
        continue;
      }
      const double wx = win.origin_x + (static_cast<double>(win.x0 + x) + 0.5) * win.resolution;
      const double wy = win.origin_y + (static_cast<double>(win.y0 + y) + 0.5) * win.resolution;
      const double dx = wx - pos.x();
      const double dy = wy - pos.y();
      if (dx * dx + dy * dy <= r2) {
        return false;
      }
    }
  }
  return true;
}

QueryResult GlobalTerrainGrid::query(const Eigen::Vector3d & pos) const
{
  QueryResult result;
  if (!cfg_.enable) {
    result.status = QueryStatus::SNAPSHOT_INVALID;
    return result;
  }
  if (!pos.allFinite()) {
    result.status = QueryStatus::NONFINITE_INPUT;
    return result;
  }
  std::shared_ptr<DynamicLayer> field;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    field = field_;
    result.snapshot_sequence = stats_.updates;
  }
  if (!field || !field->isValid()) {
    result.status = QueryStatus::FIELD_UNINITIALIZED;
    return result;
  }
  double dist = std::numeric_limits<double>::quiet_NaN();
  Eigen::Vector3d grad = Eigen::Vector3d::Zero();
  field->evaluate(pos, dist, grad);
  if (!std::isfinite(dist) || !grad.allFinite()) {
    result.status = QueryStatus::INTERPOLATION_FAILED;
    return result;
  }
  result.ok = true;
  result.status = QueryStatus::OK;
  result.distance = dist;
  result.gradient = grad;
  return result;
}

bool GlobalTerrainGrid::evaluate(const Eigen::Vector3d & pos, double & dist,
  Eigen::Vector3d & grad) const
{
  const QueryResult result = query(pos);
  dist = result.distance;
  grad = result.gradient;
  return result.ok;
}

GlobalTerrainGrid::Stats GlobalTerrainGrid::stats() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return stats_;
}

const std::vector<GlobalTerrainCell> & GlobalTerrainGrid::cells() const { return cells_; }

grid_map::GridMap GlobalTerrainGrid::gridMapSnapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return grid_map_;
}

std::vector<uint8_t> GlobalTerrainGrid::exportEdgeStates() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  // 每格 2 个方向：0=RIGHT, 1=UP；越界记 UNKNOWN(0)。
  std::vector<uint8_t> out(static_cast<size_t>(std::max(0, width_)) *
      static_cast<size_t>(std::max(0, height_)) * 2U, 0U);
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      const size_t base = (static_cast<size_t>(y) * static_cast<size_t>(width_) +
        static_cast<size_t>(x)) * 2U;
      if (x + 1 < width_) {
        out[base] = static_cast<uint8_t>(edgeStateLocked(x, y, x + 1, y));
      }
      if (y + 1 < height_) {
        out[base + 1] = static_cast<uint8_t>(edgeStateLocked(x, y, x, y + 1));
      }
    }
  }
  return out;
}

std::vector<uint8_t> GlobalTerrainGrid::exportVizClasses() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<uint8_t> out(cells_.size(), static_cast<uint8_t>(VizClass::UNKNOWN));
  for (size_t i = 0; i < cells_.size(); ++i) {
    const GlobalTerrainCell & c = cells_[i];
    if (c.obstacle) {
      out[i] = static_cast<uint8_t>(VizClass::OBSTACLE);
    } else if (i < blocked_.size() && blocked_[i] != 0U) {
      out[i] = static_cast<uint8_t>(VizClass::BLOCKED_EDGE);
    } else if (c.observed) {
      out[i] = static_cast<uint8_t>(VizClass::FREE);
    }
  }
  return out;
}

}  // namespace rog_map
