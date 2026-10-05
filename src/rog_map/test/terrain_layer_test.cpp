#include <gtest/gtest.h>
#include <rog_map/terrain_layer.hpp>
#include <rog_map/query_adapter.hpp>

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>

namespace {
constexpr int kWidth = 81;
constexpr double kResolution = 0.05;

rog_map::ProjectionLayer makeProjection(const std::function<rog_map::ColumnStats(int,int)> & scanner)
{
  rog_map::ProjectionLayer layer;
  rog_map::ProjectionLayerConfig config;
  config.hysteresis_en = false;
  config.mask_filter_en = false;
  layer.update(kWidth, kWidth, kResolution, Eigen::Vector2d::Zero(), 1.0, config, scanner);
  return layer;
}

rog_map::ColumnStats surface(double z)
{
  rog_map::ColumnStats stats;
  stats.observed_count = 2;
  stats.occupied_count = 1;
  stats.occupied_z_min_abs = stats.occupied_z_max_abs = z;
  stats.occupied_z_index_min = stats.occupied_z_index_max = static_cast<int>(std::floor(z / kResolution));
  return stats;
}
}

TEST(OnlineTerrain, RayFreeAirHasNoSupportingGround)
{
  const auto projection = makeProjection([](int,int) {
    rog_map::ColumnStats stats; stats.observed_count = 20; return stats;
  });
  rog_map::TerrainLayer terrain;
  rog_map::TerrainConfig config; config.enable = true;
  terrain.update(projection, config, Eigen::Vector3d(2.0,2.0,0.0));
  // mask 只服务二维 ESDF，未观测区域在规划层按可通行处理；这里改为断言地形类型与 occupancy：
  // 没有支撑面就不应构造出可通行地面，type 保持 UNKNOWN、occupancy 保持 255。
  EXPECT_EQ(std::count_if(terrain.cells().begin(), terrain.cells().end(),
              [](const rog_map::TerrainCell & c) { return c.type == rog_map::TerrainType::FLAT; }), 0);
  EXPECT_EQ(std::count(terrain.values().begin(), terrain.values().end(), 0U), 0);
}

TEST(OnlineTerrain, RampDirectionDoesNotAllowSidewaysMotion)
{
  const auto projection = makeProjection([](int x,int) { return surface(0.2 * ((x + 0.5) * kResolution - 2.0)); });
  rog_map::TerrainLayer terrain;
  rog_map::TerrainConfig config; config.enable = true;
  terrain.update(projection, config, Eigen::Vector3d(2.0,2.0,0.0));
  const auto & cell = terrain.cells()[40 * kWidth + 40];
  ASSERT_EQ(cell.type, rog_map::TerrainType::SLOPE);
  EXPECT_TRUE(rog_map::terrainDirectionAllowed(cell, Eigen::Vector2d(1,0), config.alignment_cosine));
  EXPECT_TRUE(rog_map::terrainDirectionAllowed(cell, Eigen::Vector2d(-1,0), config.alignment_cosine));
  EXPECT_FALSE(rog_map::terrainDirectionAllowed(cell, Eigen::Vector2d(0,1), config.alignment_cosine));
}

TEST(OnlineTerrain, FloatingUpperSurfaceCannotBecomeConnectedGround)
{
  const auto projection = makeProjection([](int x,int y) {
    return surface(x >= 50 && x <= 60 && y >= 50 && y <= 60 ? 0.5 : 0.0);
  });
  rog_map::TerrainLayer terrain;
  rog_map::TerrainConfig config; config.enable = true;
  terrain.update(projection, config, Eigen::Vector3d(1.0,1.0,0.0));
  // 漂浮的上表面不能变成可通行地面：occupancy 仍是 UNKNOWN，类型不是 FLAT；
  // 机器人脚下的真实连通地面才是 FREE。
  EXPECT_EQ(terrain.values()[55 * kWidth + 55], 255U);
  EXPECT_NE(terrain.cells()[55 * kWidth + 55].type, rog_map::TerrainType::FLAT);
  EXPECT_EQ(terrain.values()[20 * kWidth + 20], 0U);
}

TEST(OnlineTerrain, RotatedRampSupportIsIndependentOfGridDirections)
{
  constexpr double angle = 20.0 * 0.01745329252;
  const auto projection = makeProjection([](int x,int y) {
    return surface(0.2 * (std::cos(angle) * ((x + 0.5) * kResolution - 2.0) +
      std::sin(angle) * ((y + 0.5) * kResolution - 2.0)));
  });
  rog_map::TerrainLayer terrain;
  rog_map::TerrainConfig config; config.enable = true;
  terrain.update(projection, config, Eigen::Vector3d(2.0,2.0,0.0));
  EXPECT_EQ(terrain.cells()[50 * kWidth + 50].type, rog_map::TerrainType::SLOPE);
  EXPECT_EQ(terrain.mask()[50 * kWidth + 50], 1U);
}

TEST(OnlineTerrain, YawFreeClearanceUsesTheBodyCircumscribedRadius)
{
  auto snapshot = std::make_shared<rog_map::MapSnapshot>();
  snapshot->width = snapshot->height = kWidth;
  snapshot->resolution = kResolution;
  snapshot->terrain_config.enable = true;
  snapshot->values.assign(kWidth * kWidth, 0U);
  snapshot->terrain_cells.resize(kWidth * kWidth);
  for (auto & cell : snapshot->terrain_cells) {
    cell.type = rog_map::TerrainType::SLOPE; cell.surface_z = 0.0F;
    cell.uphill = Eigen::Vector2d(1.0, 0.0);
  }
  snapshot->values[41 * kWidth + 50] = 254U;
  rog_map::QueryAdapter query; query.update(snapshot, nullptr);
  EXPECT_FALSE(query.footprintFree(Eigen::Vector3d(2.0,2.0,0.0)));
}

TEST(OnlineTerrain, WallOnStairsStaysBlocked)
{
  const auto projection = makeProjection([](int x,int y) {
    const double px = (x + 0.5) * kResolution;
    const double z = px < 1.0 ? 0.0 : 0.15 * (1.0 + std::floor((px - 1.0) / 0.30));
    auto stats = surface(z);
    if (x == 40 && y == 40) {
      stats.occupied_count = 10;
      stats.observed_count = 30;
      stats.occupied_z_max_abs = z + 1.0;
      stats.occupied_z_index_max = stats.occupied_z_index_min + 20;
    }
    return stats;
  });
  rog_map::TerrainLayer terrain;
  rog_map::TerrainConfig config; config.enable = true;
  terrain.update(projection, config, Eigen::Vector3d(0.5,2.0,0.0));
  EXPECT_TRUE(std::any_of(terrain.cells().begin(), terrain.cells().end(), [](const auto & cell) {
    return cell.type == rog_map::TerrainType::STEP;
  }));
  EXPECT_EQ(terrain.cells()[40 * kWidth + 40].type, rog_map::TerrainType::OBSTACLE);
  EXPECT_EQ(terrain.mask()[40 * kWidth + 40], 0U);
}

TEST(OnlineTerrain, FootprintAcceptsUnknownAndDirectionRejectsCliff)
{
  auto snapshot = std::make_shared<rog_map::MapSnapshot>();
  snapshot->width = snapshot->height = kWidth;
  snapshot->resolution = kResolution;
  snapshot->terrain_config.enable = true;
  snapshot->values.assign(kWidth * kWidth, 0U);
  snapshot->terrain_cells.resize(kWidth * kWidth);
  for (auto & cell : snapshot->terrain_cells) {
    cell.type = rog_map::TerrainType::FLAT; cell.surface_z = 0.0F;
  }
  rog_map::QueryAdapter query; query.update(snapshot, nullptr);
  EXPECT_TRUE(query.footprintFree(Eigen::Vector3d(2.0,2.0,0.0)));
  snapshot = std::make_shared<rog_map::MapSnapshot>(*snapshot);
  // UNKNOWN 在规划层与 FREE 等价：足印范围内是未观测区域也放行。
  snapshot->values[40 * kWidth + 41] = 255U;
  query.update(snapshot, nullptr);
  EXPECT_TRUE(query.footprintFree(Eigen::Vector3d(2.0,2.0,0.0)));
  // 已观测到的障碍仍然拒绝。
  snapshot = std::make_shared<rog_map::MapSnapshot>(*snapshot);
  snapshot->values[40 * kWidth + 41] = 254U;
  query.update(snapshot, nullptr);
  EXPECT_FALSE(query.footprintFree(Eigen::Vector3d(2.0,2.0,0.0)));
  snapshot = std::make_shared<rog_map::MapSnapshot>(*snapshot);
  snapshot->values[40 * kWidth + 41] = 0U;
  snapshot->terrain_cells[40 * kWidth + 41].surface_z = 0.50F;
  query.update(snapshot, nullptr);
  EXPECT_FALSE(query.edgeAllowed(Eigen::Vector3d(2.025,2.025,0), Eigen::Vector3d(2.075,2.025,0)));
}

TEST(OnlineTerrain, BlindSpotSeedNeverFillsMissingGroundAhead)
{
  const auto projection = makeProjection([](int x,int y) {
    if ((x >= 34 && x <= 46 && y >= 34 && y <= 46) || (x >= 60 && y >= 60)) {
      rog_map::ColumnStats stats; stats.observed_count = 20; return stats;
    }
    return surface(0.0);
  });
  rog_map::TerrainLayer terrain;
  rog_map::TerrainConfig config; config.enable = true; config.seed_current_footprint = true;
  terrain.update(projection, config, Eigen::Vector3d(2.0,2.0,0.0));
  EXPECT_EQ(terrain.values()[40 * kWidth + 40], 0U);
  EXPECT_FALSE(terrain.cells()[40 * kWidth + 40].support_observed);
  // 前方盲区没有被补种成地面：occupancy 仍是 UNKNOWN。
  EXPECT_EQ(terrain.values()[65 * kWidth + 65], 255U);
}

TEST(OnlineTerrain, BodySeedCoversEnforcedFootprintClearance)
{
  // Blind hole wider than the physical body but narrower than the enforced footprint clearance:
  // the start seed must still make the whole footprint traversable, or every route is rejected.
  const auto projection = makeProjection([](int x, int y) {
    const double wx = (x + 0.5) * kResolution;
    const double wy = (y + 0.5) * kResolution;
    if (std::hypot(wx - 2.0, wy - 2.0) <= 0.55) {
      rog_map::ColumnStats stats; stats.observed_count = 20; return stats;
    }
    return surface(0.0);
  });
  rog_map::TerrainLayer terrain;
  rog_map::TerrainConfig config; config.enable = true; config.seed_current_footprint = true;
  terrain.update(projection, config, Eigen::Vector3d(2.0, 2.0, 0.0));

  auto snapshot = std::make_shared<rog_map::MapSnapshot>();
  snapshot->width = snapshot->height = kWidth;
  snapshot->resolution = kResolution;
  snapshot->terrain_config = config;
  snapshot->values = terrain.values();
  snapshot->terrain_cells = terrain.cells();
  rog_map::QueryAdapter query;
  query.update(snapshot, nullptr);
  EXPECT_TRUE(query.footprintFree(Eigen::Vector3d(2.0, 2.0, 0.0)));
}
