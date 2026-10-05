// Slope/stair detector adapted from the user-owned fastlio_nav2
// src/fast_lio/src/pcd_to_occupancy_map.cpp (online ROGMap input, no prior PCD).
#include <rog_map/terrain_layer.hpp>
#include <algorithm>
#include <array>
#include <deque>
#include <utility>

namespace rog_map {
namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr int8_t kTerrainFlat = 0;
constexpr int8_t kTerrainObstacle = 1;
constexpr int8_t kTerrainSlope = 2;
constexpr int8_t kTerrainStep = 3;
}

void TerrainLayer::update(const ProjectionLayer & projection, const TerrainConfig & config,
  const Eigen::Vector3d & robot_position, const Eigen::Quaterniond & rotation)
{
  config_ = config;
  resolution_ = projection.resolution();
  const int width = projection.width();
  const int height = projection.height();
  const auto & observed = projection.cells();
  const size_t count = observed.size();
  cells_.assign(count, TerrainCell{});
  values_.assign(count, 255U);
  // mask 是二维 ESDF 的障碍源（0=障碍，1=可通行）。未观测区域在规划层按可通行处理，
  // 所以这里默认给 1，只有判定为障碍的格子才置 0；occupancy（values_）仍然保持 255=UNKNOWN。
  mask_.assign(count, 1U);
  if (!count || width <= 0 || height <= 0 || !robot_position.allFinite() ||
      !rotation.coeffs().allFinite() || rotation.norm() < 1.0e-6) {
    return;
  }
  const Eigen::Quaterniond attitude = rotation.normalized();
  const double expected_ground = robot_position.z();
  const Eigen::Vector2d origin = projection.origin();

  std::vector<float> surface(count, std::numeric_limits<float>::quiet_NaN());
  for (size_t i = 0; i < count; ++i) {
    if (observed[i].confidence >= 1.0F && std::isfinite(observed[i].occupied_z_min_abs)) {
      surface[i] = observed[i].occupied_z_min_abs;
      cells_[i].support_observed = true;
    }
  }
  if (config.seed_current_footprint) {
    int ground_evidence = 0;
    for (size_t i = 0; i < count; ++i) {
      const Eigen::Vector2d xy = origin + resolution_ * Eigen::Vector2d(
        static_cast<double>(i % static_cast<size_t>(width)) + 0.5,
        static_cast<double>(i / static_cast<size_t>(width)) + 0.5);
      if (std::isfinite(surface[i]) && (xy - robot_position.head<2>()).norm() <=
          config.flat_clearance_radius + config.slope_fit_radius &&
          std::abs(surface[i] - expected_ground) <= config.ground_seed_tolerance) { ++ground_evidence; }
    }
    // The physically occupied body footprint supplies a start seed only after nearby ground is seen.
    // Missing ground ahead remains UNKNOWN. Measured surfaces and obstacles are never overwritten.
    if (ground_evidence >= 8) {
      // The seed must cover the same region QueryAdapter::footprintFree later enforces, otherwise a
      // single unobserved cell inside the footprint (for example inside the lidar blind spot around
      // an offset sensor) rejects every route at the start.
      const double footprint_margin = config.clearance_margin + 0.7071067812 * resolution_;
      const double seed_radius = std::max(config.flat_clearance_radius,
        std::hypot(config.terrain_half_length, config.terrain_half_width)) + footprint_margin;
      const double seed_length = config.terrain_half_length + footprint_margin;
      const double seed_width = config.terrain_half_width + footprint_margin;
      for (size_t i = 0; i < count; ++i) {
        if (std::isfinite(surface[i])) { continue; }
        Eigen::Vector3d delta(origin.x() + (i % static_cast<size_t>(width) + 0.5) * resolution_ - robot_position.x(),
          origin.y() + (i / static_cast<size_t>(width) + 0.5) * resolution_ - robot_position.y(), 0.0);
        const Eigen::Vector3d normal = attitude * Eigen::Vector3d::UnitZ();
        if (normal.z() < 0.5) { continue; }
        delta.z() = -(normal.x() * delta.x() + normal.y() * delta.y()) / normal.z();
        const Eigen::Vector3d body_delta = attitude.conjugate() * delta;
        const bool inside_body =
          std::abs(body_delta.x()) <= config.terrain_half_length &&
          std::abs(body_delta.y()) <= config.terrain_half_width;
        const bool inside_clearance = config.yaw_independent_footprint ?
          delta.head<2>().squaredNorm() <= seed_radius * seed_radius :
          std::abs(body_delta.x()) <= seed_length && std::abs(body_delta.y()) <= seed_width;
        if (inside_body || inside_clearance) {
          surface[i] = static_cast<float>(expected_ground + delta.z());
        }
      }
    }
  }
  // Keep missing support missing; median only suppresses isolated returns in observed columns.
  auto filtered = surface;
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const size_t idx = cell_index(x, y, width);
      if (!std::isfinite(surface[idx])) { continue; }
      std::array<float, 9> local{};
      int n = 0;
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          if (x + dx < 0 || x + dx >= width || y + dy < 0 || y + dy >= height) { continue; }
          const float z = surface[cell_index(x + dx, y + dy, width)];
          if (std::isfinite(z)) { local[static_cast<size_t>(n++)] = z; }
        }
      }
      if (n >= 3) {
        std::sort(local.begin(), local.begin() + n);
        filtered[idx] = local[static_cast<size_t>(n / 2)];
      }
    }
  }
  std::vector<int8_t> labels(count, kTerrainFlat);
  std::vector<double> dx(count, 0.0), dy(count, 0.0);
  detect_slopes(filtered, width, height, labels, dx, dy);
  detect_stairs(filtered, width, height, labels, dx, dy);
  remove_small_terrain_components(width, height, kTerrainSlope, labels, dx, dy);
  remove_small_terrain_components(width, height, kTerrainStep, labels, dx, dy);

  std::vector<uint8_t> supported(count, 0U);
  for (size_t i = 0; i < count; ++i) {
    cells_[i].surface_z = filtered[i];
    if (!std::isfinite(surface[i])) { continue; }
    const double span = observed[i].occupied_z_max_abs - surface[i];
    const bool riser = labels[i] == kTerrainStep &&
      span <= config.stair_rise + config.stair_rise_tolerance + resolution_;
    // Directional terrain must never clear an object above the walking surface.
    const bool obstacle = labels[i] == kTerrainObstacle ||
      (!riser && observed[i].occupied_z_max_abs - filtered[i] >
        config.obstacle_min_height + 0.5 * resolution_);
    if (obstacle) {
      cells_[i].type = TerrainType::OBSTACLE;
      values_[i] = 254U;
      mask_[i] = 0U;
      continue;
    }
    supported[i] = 1U;
    cells_[i].type = static_cast<TerrainType>(labels[i]);
    cells_[i].uphill = Eigen::Vector2d(dx[i], dy[i]);
  }

  // Seed only observed ground near the current robot. This rejects floating box tops,
  // disconnected upper surfaces and unrecognized cliffs instead of treating ray-free air as ground.
  std::deque<size_t> pending;
  std::vector<uint8_t> reachable(count, 0U);
  for (size_t i = 0; i < count; ++i) {
    if (!supported[i]) { continue; }
    const Eigen::Vector2d xy = origin + resolution_ * Eigen::Vector2d(
      static_cast<double>(i % static_cast<size_t>(width)) + 0.5,
      static_cast<double>(i / static_cast<size_t>(width)) + 0.5);
    if ((xy - robot_position.head<2>()).norm() <= config.slope_fit_radius &&
        std::abs(filtered[i] - expected_ground) <= config.ground_seed_tolerance) {
      reachable[i] = 1U;
      pending.push_back(i);
    }
  }
  constexpr int offsets[8][2] = {{-1,0},{1,0},{0,-1},{0,1},{-1,-1},{-1,1},{1,-1},{1,1}};
  const double max_gradient = std::tan(config.slope_max_deg * kPi / 180.0);
  while (!pending.empty()) {
    const size_t i = pending.front(); pending.pop_front();
    const int x = static_cast<int>(i % static_cast<size_t>(width));
    const int y = static_cast<int>(i / static_cast<size_t>(width));
    for (const auto & offset : offsets) {
      const int nx = x + offset[0], ny = y + offset[1];
      if (nx < 0 || nx >= width || ny < 0 || ny >= height) { continue; }
      const size_t j = cell_index(nx, ny, width);
      if (!supported[j] || reachable[j]) { continue; }
      if (offset[0] && offset[1] &&
          (!supported[cell_index(nx, y, width)] || !supported[cell_index(x, ny, width)])) { continue; }
      const Eigen::Vector2d travel(offset[0], offset[1]);
      // This flood fill identifies connected support. Travel direction is checked by
      // the search and trajectory validator, independently of grid orientation.
      const double dz = std::abs(filtered[j] - filtered[i]);
      const bool step = cells_[i].type == TerrainType::STEP || cells_[j].type == TerrainType::STEP;
      const double allowed_dz = step ? config.stair_rise + config.stair_rise_tolerance + resolution_ :
        max_gradient * travel.norm() * resolution_ + 0.5 * resolution_;
      if (dz > allowed_dz) { continue; }
      reachable[j] = 1U;
      pending.push_back(j);
    }
  }
  for (size_t i = 0; i < count; ++i) {
    if (reachable[i]) {
      values_[i] = 0U;
      mask_[i] = 1U;
    } else if (supported[i]) {
      cells_[i].type = TerrainType::UNKNOWN;
      cells_[i].uphill.setZero();
    }
  }
}

void TerrainLayer::detect_slopes(
    const std::vector<float> & surface, const int width, const int height,
    std::vector<int8_t> & terrain, std::vector<double> & direction_x,
    std::vector<double> & direction_y) const
  {
    const int radius = std::max(1, static_cast<int>(std::ceil(
      config_.slope_fit_radius / resolution_)));
    const double min_slope = std::tan(config_.slope_min_deg * kPi / 180.0);
    const double max_slope = std::tan(config_.slope_max_deg * kPi / 180.0);

    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        const std::size_t center_index = cell_index(x, y, width);
        if (!std::isfinite(surface[center_index])) {
          continue;
        }
        double sx = 0.0, sy = 0.0, sz = 0.0;
        double sxx = 0.0, sxy = 0.0, syy = 0.0;
        double sxz = 0.0, syz = 0.0;
        double min_height = std::numeric_limits<double>::infinity();
        double max_height = -std::numeric_limits<double>::infinity();
        int samples = 0;
        for (int dy = -radius; dy <= radius; ++dy) {
          for (int dx = -radius; dx <= radius; ++dx) {
            const int nx = x + dx;
            const int ny = y + dy;
            if (nx < 0 || nx >= width || ny < 0 || ny >= height ||
              dx * dx + dy * dy > radius * radius)
            {
              continue;
            }
            const float height_value = surface[cell_index(nx, ny, width)];
            if (!std::isfinite(height_value)) {
              continue;
            }
            const double px = static_cast<double>(dx) * resolution_;
            const double py = static_cast<double>(dy) * resolution_;
            const double pz = height_value;
            sx += px;
            sy += py;
            sz += pz;
            sxx += px * px;
            sxy += px * py;
            syy += py * py;
            sxz += px * pz;
            syz += py * pz;
            min_height = std::min(min_height, pz);
            max_height = std::max(max_height, pz);
            ++samples;
          }
        }
        if (samples < 8 || max_height - min_height < 0.04) {
          continue;
        }

        const double inv_n = 1.0 / static_cast<double>(samples);
        const double cxx = sxx - sx * sx * inv_n;
        const double cxy = sxy - sx * sy * inv_n;
        const double cyy = syy - sy * sy * inv_n;
        const double cxz = sxz - sx * sz * inv_n;
        const double cyz = syz - sy * sz * inv_n;
        const double determinant = cxx * cyy - cxy * cxy;
        if (std::abs(determinant) < 1.0e-9) {
          continue;
        }
        const double gx = (cxz * cyy - cyz * cxy) / determinant;
        const double gy = (cyz * cxx - cxz * cxy) / determinant;
        const double gradient = std::hypot(gx, gy);
        if (gradient < min_slope) {
          continue;
        }
        const double mean_x = sx * inv_n;
        const double mean_y = sy * inv_n;
        const double mean_z = sz * inv_n;
        double squared_error = 0.0;
        int residual_samples = 0;
        for (int dy = -radius; dy <= radius; ++dy) {
          for (int dx = -radius; dx <= radius; ++dx) {
            const int nx = x + dx;
            const int ny = y + dy;
            if (nx < 0 || nx >= width || ny < 0 || ny >= height ||
              dx * dx + dy * dy > radius * radius)
            {
              continue;
            }
            const float height_value = surface[cell_index(nx, ny, width)];
            if (!std::isfinite(height_value)) {
              continue;
            }
            const double px = static_cast<double>(dx) * resolution_;
            const double py = static_cast<double>(dy) * resolution_;
            const double predicted = mean_z + gx * (px - mean_x) + gy * (py - mean_y);
            const double error = static_cast<double>(height_value) - predicted;
            squared_error += error * error;
            ++residual_samples;
          }
        }
        const double residual = std::sqrt(
          squared_error / std::max(1, residual_samples));
        if (residual > config_.slope_max_residual) {
          continue;
        }
        if (gradient > max_slope) {
          terrain[center_index] = kTerrainObstacle;
          continue;
        }
        terrain[center_index] = kTerrainSlope;
        direction_x[center_index] = gx / gradient;
        direction_y[center_index] = gy / gradient;
      }
    }
  }

void TerrainLayer::detect_stairs(
    const std::vector<float> & surface, const int width, const int height,
    std::vector<int8_t> & terrain, std::vector<double> & direction_x,
    std::vector<double> & direction_y) const
  {
    constexpr std::array<std::pair<int, int>, 4> edge_directions {{
      {1, 0}, {0, 1}, {1, 1}, {1, -1}}};
    std::vector<StepSeed> seeds;

    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        const float first = surface[cell_index(x, y, width)];
        if (!std::isfinite(first)) {
          continue;
        }
        for (const auto & [ox, oy] : edge_directions) {
          const int nx = x + ox;
          const int ny = y + oy;
          if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
            continue;
          }
          const float second = surface[cell_index(nx, ny, width)];
          if (!std::isfinite(second)) {
            continue;
          }
          const double height_delta = static_cast<double>(second - first);
          if (std::abs(std::abs(height_delta) - config_.stair_rise) > config_.stair_rise_tolerance) {
            continue;
          }
          const double grid_norm = std::hypot(static_cast<double>(ox), static_cast<double>(oy));
          const double sign = height_delta > 0.0 ? 1.0 : -1.0;
          const int step_x = static_cast<int>(sign) * ox;
          const int step_y = static_cast<int>(sign) * oy;
          const double up_x = sign * static_cast<double>(ox) / grid_norm;
          const double up_y = sign * static_cast<double>(oy) / grid_norm;
          const int low_x = height_delta > 0.0 ? x : nx;
          const int low_y = height_delta > 0.0 ? y : ny;
          const double low_height = std::min(first, second);
          const int nominal_steps = std::max(2, static_cast<int>(std::lround(
            config_.stair_tread / (resolution_ * grid_norm))));
          bool repeated_rise = false;
          for (int delta_steps = -2; delta_steps <= 2 && !repeated_rise; ++delta_steps) {
            const int travel = std::max(2, nominal_steps + delta_steps);
            const int bx = low_x + step_x * travel;
            const int by = low_y + step_y * travel;
            const int ax = bx + step_x;
            const int ay = by + step_y;
            if (bx < 0 || bx >= width || by < 0 || by >= height ||
              ax < 0 || ax >= width || ay < 0 || ay >= height)
            {
              continue;
            }
            const float before = surface[cell_index(bx, by, width)];
            const float after = surface[cell_index(ax, ay, width)];
            if (!std::isfinite(before) || !std::isfinite(after)) {
              continue;
            }
            const double next_delta = static_cast<double>(after - before);
            repeated_rise = std::abs(next_delta - config_.stair_rise) <= config_.stair_rise_tolerance &&
              static_cast<double>(before) >= low_height + 0.5 * config_.stair_rise;
          }
          if (repeated_rise) {
            seeds.push_back({low_x, low_y, up_x, up_y});
          }
        }
      }
    }

    std::vector<StepSeed> supported_seeds;
    supported_seeds.reserve(seeds.size());
    for (const auto & seed : seeds) {
      double across_min = 0.0;
      double across_max = 0.0;
      int same_riser_support = 0;
      for (const auto & candidate : seeds) {
        if (seed.dx * candidate.dx + seed.dy * candidate.dy < 0.95) {
          continue;
        }
        const double mx = static_cast<double>(candidate.x - seed.x) * resolution_;
        const double my = static_cast<double>(candidate.y - seed.y) * resolution_;
        const double along = mx * seed.dx + my * seed.dy;
        const double across = -mx * seed.dy + my * seed.dx;
        if (std::abs(along) <= 0.10 && std::abs(across) <= 0.75) {
          across_min = std::min(across_min, across);
          across_max = std::max(across_max, across);
          ++same_riser_support;
        }
      }
      if (same_riser_support >= 4 && across_max - across_min >= config_.stair_min_width) {
        supported_seeds.push_back(seed);
      }
    }

    const double along_before = 0.15;
    const double along_after = config_.stair_tread + 0.20;
    const int radius = static_cast<int>(std::ceil(
      std::max(config_.stair_half_width, along_after) / resolution_));
    for (const auto & seed : supported_seeds) {
      for (int dy = -radius; dy <= radius; ++dy) {
        for (int dx = -radius; dx <= radius; ++dx) {
          const int x = seed.x + dx;
          const int y = seed.y + dy;
          if (x < 0 || x >= width || y < 0 || y >= height) {
            continue;
          }
          const double mx = static_cast<double>(dx) * resolution_;
          const double my = static_cast<double>(dy) * resolution_;
          const double along = mx * seed.dx + my * seed.dy;
          const double across = -mx * seed.dy + my * seed.dx;
          if (along < -along_before || along > along_after ||
            std::abs(across) > config_.stair_half_width)
          {
            continue;
          }
          const std::size_t index = cell_index(x, y, width);
          terrain[index] = kTerrainStep;
          direction_x[index] += seed.dx;
          direction_y[index] += seed.dy;
        }
      }
    }
    for (std::size_t i = 0; i < terrain.size(); ++i) {
      if (terrain[i] != kTerrainStep) {
        continue;
      }
      const double norm = std::hypot(direction_x[i], direction_y[i]);
      if (norm > 1.0e-9) {
        direction_x[i] /= norm;
        direction_y[i] /= norm;
      }
    }
  }

void TerrainLayer::remove_small_terrain_components(
    const int width, const int height, const int8_t label,
    std::vector<int8_t> & terrain, std::vector<double> & direction_x,
    std::vector<double> & direction_y) const
  {
    const int minimum_cells = std::max(1, static_cast<int>(std::ceil(
      config_.terrain_min_component_area / (resolution_ * resolution_))));
    std::vector<uint8_t> visited(terrain.size(), 0);
    constexpr std::array<std::pair<int, int>, 8> neighbors {{
      {-1, -1}, {0, -1}, {1, -1}, {-1, 0},
      {1, 0}, {-1, 1}, {0, 1}, {1, 1}}};
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        const std::size_t start = cell_index(x, y, width);
        if (visited[start] || terrain[start] != label) {
          continue;
        }
        std::deque<std::pair<int, int>> queue;
        std::vector<std::size_t> component;
        queue.push_back({x, y});
        visited[start] = 1;
        while (!queue.empty()) {
          const auto [cx, cy] = queue.front();
          queue.pop_front();
          component.push_back(cell_index(cx, cy, width));
          for (const auto & [dx, dy] : neighbors) {
            const int nx = cx + dx;
            const int ny = cy + dy;
            if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
              continue;
            }
            const std::size_t next = cell_index(nx, ny, width);
            if (!visited[next] && terrain[next] == label) {
              visited[next] = 1;
              queue.push_back({nx, ny});
            }
          }
        }
        if (static_cast<int>(component.size()) < minimum_cells) {
          for (const std::size_t index : component) {
            terrain[index] = kTerrainObstacle;
            direction_x[index] = 0.0;
            direction_y[index] = 0.0;
          }
        }
      }
    }
  }


}  // namespace rog_map
