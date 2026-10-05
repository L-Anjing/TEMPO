// GlobalTerrainGrid 语义验收：坡道可通行、平台/楼梯边缘 BLOCKED、动态障碍可清除、地图持久。
#include <gtest/gtest.h>

#include <rog_map/global_terrain_grid.hpp>
#include <rog_map/projection_layer.hpp>
#include <rog_map/terrain_layer.hpp>

#include <algorithm>
#include <cmath>
#include <functional>

namespace {

constexpr int kW = 81;
constexpr double kRes = 0.05;

// 垂直位图的参考原点（绝对高度）。bit i 对应 kMaskOrigin + i*kRes。
constexpr double kMaskOrigin = -1.0;
constexpr int kMaskBits = 64;

int bitFor(double z) { return static_cast<int>(std::llround((z - kMaskOrigin) / kRes)); }

void setBit(uint64_t & mask, double z)
{
  const int bit = bitFor(z);
  if (bit >= 0 && bit < kMaskBits) {
    mask |= (1ULL << bit);
  }
}

// 真实 ray miss 扫过的自由空间：把 [z_lo, z_hi] 每一层都置成 FREE。
void setFreeBand(rog_map::ColumnStats & stats, double z_lo, double z_hi)
{
  if (z_lo > z_hi) {
    std::swap(z_lo, z_hi);
  }
  for (double z = z_lo; z <= z_hi + 1e-9; z += kRes) {
    setBit(stats.free_z_mask, z);
  }
}

rog_map::ColumnStats finish(rog_map::ColumnStats stats)
{
  stats.z_mask_origin = static_cast<float>(kMaskOrigin);
  stats.z_mask_bits = static_cast<uint8_t>(kMaskBits);
  return stats;
}

// 有支撑面：z 处一个占据体素，band 内没有 FREE。
rog_map::ColumnStats surface(double z)
{
  rog_map::ColumnStats stats;
  stats.observed_count = 4;
  stats.occupied_count = 1;
  stats.occupied_z_min_abs = stats.occupied_z_max_abs = z;
  stats.occupied_z_index_min = stats.occupied_z_index_max =
    static_cast<int>(std::floor(z / kRes));
  setBit(stats.occupied_z_mask, z);
  return finish(stats);
}

// 未观测列：没有任何 voxel 状态（UNKNOWN）。
rog_map::ColumnStats unknown_column()
{
  return finish(rog_map::ColumnStats{});
}

// 只有高处 ray 穿过：XY 列被观测过，但预计地面高度附近仍是 UNKNOWN。
rog_map::ColumnStats ray_high_only(double z_high)
{
  rog_map::ColumnStats stats;
  stats.observed_count = 4;
  setFreeBand(stats, z_high - kRes, z_high + kRes);
  return finish(stats);
}

// 射线穿过预计地面高度并继续进入更低空间：强 no-support 证据。
rog_map::ColumnStats free_through_surface(double z_ref)
{
  rog_map::ColumnStats stats;
  stats.observed_count = 8;
  setFreeBand(stats, z_ref - 0.20, z_ref + 0.05);
  return finish(stats);
}

// 竖直实心障碍：z 从 z0 到 z1 全是占据体素。
rog_map::ColumnStats solid(double z0, double z1)
{
  rog_map::ColumnStats stats;
  stats.observed_count = 8;
  stats.occupied_count = 8;
  stats.occupied_z_min_abs = z0;
  stats.occupied_z_max_abs = z1;
  stats.occupied_z_index_min = static_cast<int>(std::floor(z0 / kRes));
  stats.occupied_z_index_max = static_cast<int>(std::floor(z1 / kRes));
  for (double z = z0; z <= z1 + 1e-9; z += kRes) {
    setBit(stats.occupied_z_mask, z);
  }
  return finish(stats);
}

rog_map::ProjectionLayer makeProjection(
  const std::function<rog_map::ColumnStats(int, int)> & scanner, double ox = 0.0, double oy = 0.0)
{
  rog_map::ProjectionLayer layer;
  rog_map::ProjectionLayerConfig config;
  config.hysteresis_en = false;
  config.mask_filter_en = false;
  config.unknown_as_occupied = false;
  config.min_observed_voxels = 2;
  layer.update(kW, kW, kRes, Eigen::Vector2d(ox, oy), 1.0, config, scanner);
  return layer;
}

rog_map::GlobalTerrainConfig gridConfig(double size_x = kW * kRes, double size_y = kW * kRes)
{
  rog_map::GlobalTerrainConfig config;
  config.enable = true;
  config.resolution = kRes;
  config.size_x = size_x;
  config.size_y = size_y;
  config.origin_x = 0.0;
  config.origin_y = 0.0;
  config.field_enable = false;  // 边语义测试不需要 ESDF
  return config;
}

void feed(rog_map::GlobalTerrainGrid & grid, const rog_map::ProjectionLayer & projection,
  double now, const Eigen::Vector3d & robot = Eigen::Vector3d(0.5, 0.5, 0.0))
{
  rog_map::TerrainLayer terrain;
  rog_map::TerrainConfig tcfg;
  tcfg.enable = true;
  terrain.update(projection, tcfg, robot);
  grid.updateFromLocal(projection, terrain, now, robot);
}

}  // namespace

// 平地：格子可通行、邻接边 PASSABLE、允许规划。
TEST(GlobalTerrainGrid, FlatGroundIsPassable)
{
  rog_map::GlobalTerrainGrid grid;
  grid.configure(gridConfig());
  const auto projection = makeProjection([](int, int) { return surface(0.0); });
  feed(grid, projection, 1.0);

  const int mid = 40;
  EXPECT_TRUE(grid.cellTraversable(mid, mid));
  EXPECT_EQ(grid.edgeState(mid, mid, mid + 1, mid), rog_map::EdgeState::PASSABLE);
  EXPECT_TRUE(grid.edgeAllowedCells(mid, mid, mid + 1, mid));
  EXPECT_TRUE(grid.edgeAllowed(Eigen::Vector3d(1.0, 2.0, 0.0), Eigen::Vector3d(2.0, 2.0, 0.0)));
  EXPECT_TRUE(grid.footprintFree(Eigen::Vector3d(2.0, 2.0, 0.0)));
  const auto backend = grid.gridMapSnapshot();
  EXPECT_TRUE(backend.exists("elevation"));
  EXPECT_TRUE(backend.exists("traversability"));
  EXPECT_FLOAT_EQ(backend.at("traversability", grid_map::Index(mid, mid)), 0.0F);
}

// 坡道：高度梯度 <= tan(slope_max) -> PASSABLE，不新增 RAMP 类型。
TEST(GlobalTerrainGrid, RampIsPassable)
{
  rog_map::GlobalTerrainGrid grid;
  grid.configure(gridConfig());
  // 每格抬升 0.02 m -> 梯度 0.4 (< tan35° = 0.70)。
  const auto projection = makeProjection([](int x, int) {
    return surface(static_cast<double>(x) * kRes * 0.4);
  });
  feed(grid, projection, 1.0);

  for (int x = 20; x < 60; ++x) {
    EXPECT_TRUE(grid.cellTraversable(static_cast<unsigned int>(x), 40)) << "x=" << x;
    EXPECT_EQ(grid.edgeState(static_cast<unsigned int>(x), 40,
                static_cast<unsigned int>(x + 1), 40), rog_map::EdgeState::PASSABLE) << "x=" << x;
  }
}

// 平台边缘：相邻支撑面高度跳变 -> BLOCKED_EDGE，且足印跨边被拒绝。
TEST(GlobalTerrainGrid, PlatformEdgeIsBlocked)
{
  rog_map::GlobalTerrainGrid grid;
  grid.configure(gridConfig());
  const auto projection = makeProjection([](int x, int) {
    return surface(x < 40 ? 0.0 : 0.4);
  });
  feed(grid, projection, 1.0);

  const int y = 40;
  EXPECT_FALSE(grid.edgeAllowedCells(39, y, 40, y));
  EXPECT_EQ(grid.edgeState(39, y, 40, y), rog_map::EdgeState::BLOCKED);
  // 平台内部远离边缘的格子本身仍然可通行。
  EXPECT_TRUE(grid.cellTraversable(60, y));
  // 机器人足印横跨边缘时必须被拒绝。
  const double edge_world_x = 40 * kRes;
  EXPECT_FALSE(grid.footprintFree(Eigen::Vector3d(edge_world_x, (y + 0.5) * kRes, 0.0)));
}

// 楼梯：单级 0.15 m 的跳变远大于最大坡道梯度 -> BLOCKED（普通 planner 不再放行）。
TEST(GlobalTerrainGrid, StairRiserIsBlocked)
{
  rog_map::GlobalTerrainGrid grid;
  grid.configure(gridConfig());
  const auto projection = makeProjection([](int x, int) {
    return surface(x < 40 ? 0.0 : 0.15);
  });
  feed(grid, projection, 1.0);
  EXPECT_FALSE(grid.edgeAllowedCells(39, 40, 40, 40));
}

// 动态障碍：出现 -> OCCUPIED/BLOCKED；被重新观测为空 -> 清除。
TEST(GlobalTerrainGrid, DynamicObstacleAppearsAndClears)
{
  rog_map::GlobalTerrainGrid grid;
  grid.configure(gridConfig());
  const int x = 40;
  const int y = 40;

  const auto clear_proj = makeProjection([](int, int) { return surface(0.0); });
  feed(grid, clear_proj, 1.0);
  ASSERT_TRUE(grid.cellTraversable(x, y));

  const auto blocked_proj = makeProjection([](int cx, int cy) {
    return (cx == 40 && cy == 40) ? solid(0.0, 0.6) : surface(0.0);
  });
  feed(grid, blocked_proj, 2.0);
  EXPECT_FALSE(grid.cellTraversable(x, y));
  EXPECT_FALSE(grid.edgeAllowedCells(x - 1, y, x, y));

  // 重新观测为空：障碍证据快速下降 -> FREE（不靠“没点”清除，而是靠新的空观测）。
  for (int i = 0; i < 3; ++i) {
    feed(grid, clear_proj, 3.0 + i);
  }
  EXPECT_TRUE(grid.cellTraversable(x, y));
  EXPECT_TRUE(grid.edgeAllowedCells(x - 1, y, x, y));
}

// 地图持久：机器人移动到新的 patch 后，旧 patch 的观测不消失。
TEST(GlobalTerrainGrid, MapPersistsAfterRobotMoves)
{
  rog_map::GlobalTerrainGrid grid;
  grid.configure(gridConfig(8.0, 4.0));
  const auto first = makeProjection([](int, int) { return surface(0.0); }, 0.0, 0.0);
  feed(grid, first, 1.0, Eigen::Vector3d(1.0, 1.0, 0.0));
  ASSERT_TRUE(grid.cellTraversable(20, 40));
  const size_t idx = static_cast<size_t>(40) * 81U + 20U;
  ASSERT_TRUE(grid.cells().at(idx).observed);
  const double stamp_before = grid.cells().at(idx).last_update;

  // 第二个 patch 完全不覆盖 x<4 m 的区域。
  const auto second = makeProjection([](int, int) { return surface(0.0); }, 4.0, 0.0);
  feed(grid, second, 2.0, Eigen::Vector3d(5.0, 1.0, 0.0));

  // 旧区域的观测与时间戳都不变：全局图不会因为机器人移动而丢失历史。
  EXPECT_TRUE(grid.cells().at(idx).observed);
  EXPECT_DOUBLE_EQ(grid.cells().at(idx).last_update, stamp_before);
  EXPECT_TRUE(grid.cellTraversable(20, 40));
  EXPECT_TRUE(grid.edgeAllowedCells(19, 40, 20, 40));
  // 新 patch 也被写入。
  EXPECT_TRUE(grid.cells().at(static_cast<size_t>(40) * 81U + 100U).observed);
}

// Case A：真实平台内部 —— 有 support -> PASSABLE。
TEST(GlobalTerrainGrid, CaseA_PlatformInteriorIsPassable)
{
  rog_map::GlobalTerrainGrid grid;
  grid.configure(gridConfig());
  const auto projection = makeProjection([](int, int) { return surface(0.0); });
  for (int i = 0; i < 6; ++i) {
    feed(grid, projection, 1.0 + i);
  }
  for (int x = 30; x < 60; ++x) {
    EXPECT_TRUE(grid.cellTraversable(static_cast<unsigned int>(x), 40));
    EXPECT_EQ(grid.edgeState(static_cast<unsigned int>(x), 40,
                static_cast<unsigned int>(x + 1), 40), rog_map::EdgeState::PASSABLE) << "x=" << x;
  }
}

// Case B：未观测区域 —— 没有 support 且 voxel 仍是 UNKNOWN -> UNKNOWN_EDGE，绝对不能 BLOCKED。
TEST(GlobalTerrainGrid, CaseB_UnknownRegionStaysUnknown)
{
  auto config = gridConfig();
  config.void_gain = 0.34f;  // 即使打开 case B 判定，也不许把 UNKNOWN 封掉
  rog_map::GlobalTerrainGrid grid;
  grid.configure(config);
  const auto projection = makeProjection([](int x, int) {
    return x < 40 ? surface(0.0) : unknown_column();
  });
  for (int i = 0; i < 10; ++i) {
    feed(grid, projection, 1.0 + i);
  }
  EXPECT_NE(grid.edgeState(39, 40, 40, 40), rog_map::EdgeState::BLOCKED);
  EXPECT_NE(grid.edgeState(60, 40, 61, 40), rog_map::EdgeState::BLOCKED);
}

// Case C：确认 drop-off —— z_ref 附近与下方都是 FREE，多帧后 -> BLOCKED_EDGE。
TEST(GlobalTerrainGrid, CaseC_ConfirmedDropOffBecomesBlocked)
{
  auto config = gridConfig();
  config.void_gain = 0.34f;
  config.void_confirm = 0.75f;
  rog_map::GlobalTerrainGrid grid;
  grid.configure(config);
  const auto projection = makeProjection([](int x, int) {
    return x < 40 ? surface(0.0) : free_through_surface(0.0);
  });
  feed(grid, projection, 1.0);
  // 单帧不给 BLOCKED（多帧累计）。
  EXPECT_NE(grid.edgeState(39, 40, 40, 40), rog_map::EdgeState::BLOCKED);
  for (int i = 0; i < 3; ++i) {
    feed(grid, projection, 2.0 + i);
  }
  EXPECT_EQ(grid.edgeState(39, 40, 40, 40), rog_map::EdgeState::BLOCKED);
  EXPECT_FALSE(grid.edgeAllowedCells(39, 40, 40, 40));
}

// Case D：只有高处 ray 穿过 —— XY 列被观测，但 support band 仍是 UNKNOWN -> UNKNOWN_EDGE。
TEST(GlobalTerrainGrid, CaseD_HighRayOnlyStaysUnknown)
{
  auto config = gridConfig();
  config.void_gain = 0.34f;
  rog_map::GlobalTerrainGrid grid;
  grid.configure(config);
  const auto projection = makeProjection([](int x, int) {
    return x < 40 ? surface(0.0) : ray_high_only(0.5);
  });
  for (int i = 0; i < 10; ++i) {
    feed(grid, projection, 1.0 + i);
  }
  EXPECT_NE(grid.edgeState(39, 40, 40, 40), rog_map::EdgeState::BLOCKED);
  EXPECT_TRUE(grid.edgeAllowedCells(39, 40, 40, 40));
}

// 统一查询语义：UNKNOWN 与 FREE 都可通行，只有 OCCUPIED 不可通行。
TEST(GlobalTerrainGrid, UnknownIsOptimisticAndOnlyOccupiedBlocks)
{
  rog_map::GlobalTerrainGrid grid;
  grid.configure(gridConfig());
  const auto projection = makeProjection([](int x, int) {
    if (x < 10) {
      return surface(0.0);
    }
    return ray_high_only(0.5);  // 只被高处的射线穿过：预计地面高度附近仍未知
  });
  feed(grid, projection, 1.0);

  // 未观测/无支撑区域仍然可通行（乐观策略）。
  EXPECT_TRUE(grid.cellTraversable(70, 40));
  EXPECT_TRUE(grid.edgeAllowedCells(70, 40, 71, 40));
  // 已观测到支撑面的区域也是 FREE。
  EXPECT_EQ(grid.value(5, 40), 0U);
}
