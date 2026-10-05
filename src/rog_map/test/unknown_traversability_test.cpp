#include <gtest/gtest.h>
#include <rog_map/query_adapter.hpp>

#include <memory>
#include <vector>

namespace {

constexpr int kWidth = 40;
constexpr int kHeight = 40;
constexpr double kResolution = 0.05;
constexpr unsigned char kFree = 0U;
constexpr unsigned char kInflated = 253U;
constexpr unsigned char kObstacle = 254U;
constexpr unsigned char kUnknown = 255U;

rog_map::MapSnapshot makeSnapshot()
{
  auto snapshot = rog_map::MapSnapshot();
  snapshot.width = kWidth;
  snapshot.height = kHeight;
  snapshot.resolution = kResolution;
  snapshot.origin_x = 0.0;
  snapshot.origin_y = 0.0;
  snapshot.values.assign(static_cast<size_t>(kWidth) * kHeight, kUnknown);
  snapshot.terrain_cells.assign(static_cast<size_t>(kWidth) * kHeight, rog_map::TerrainCell{});
  for (auto & cell : snapshot.terrain_cells) {
    cell.type = rog_map::TerrainType::FLAT;
    cell.surface_z = 0.0F;
    cell.support_observed = true;
  }
  snapshot.terrain_config.enable = true;
  return snapshot;
}

// 网格坐标 -> 世界坐标（取格子中心）。
double worldCoord(int cell)
{
  return (static_cast<double>(cell) + 0.5) * kResolution;
}

unsigned char & cellAt(rog_map::MapSnapshot & snapshot, int x, int y)
{
  return snapshot.values[static_cast<size_t>(y) * kWidth + static_cast<size_t>(x)];
}

}  // namespace

TEST(UnknownTraversability, EdgeDoesNotInventHeightBarrierFromUnobservedNaN) {
  auto snapshot = makeSnapshot();
  for (auto & cell : snapshot.terrain_cells) { cell = rog_map::TerrainCell{}; }
  auto adapter = std::make_shared<rog_map::QueryAdapter>();
  adapter->update(std::make_shared<const rog_map::MapSnapshot>(snapshot), nullptr);
  const Eigen::Vector3d from(worldCoord(10), worldCoord(10), 0);
  const Eigen::Vector3d to(worldCoord(12), worldCoord(12), 0);
  EXPECT_TRUE(adapter->edgeAllowed(from, to));
  cellAt(snapshot, 11, 11) = kObstacle;
  adapter->update(std::make_shared<const rog_map::MapSnapshot>(snapshot), nullptr);
  EXPECT_FALSE(adapter->edgeAllowed(from, to));
}

// 规划语义：UNKNOWN 与 FREE 一样可通行，只有已观测到的障碍/膨胀格不可通行。
TEST(UnknownTraversability, IsFreeTreatsUnknownAsTraversable)
{
  auto snapshot = makeSnapshot();
  auto adapter = std::make_shared<rog_map::QueryAdapter>();
  cellAt(snapshot, 10, 10) = kFree;
  cellAt(snapshot, 11, 10) = kUnknown;
  cellAt(snapshot, 12, 10) = kObstacle;
  cellAt(snapshot, 13, 10) = kInflated;
  adapter->update(std::make_shared<const rog_map::MapSnapshot>(snapshot), nullptr);

  EXPECT_TRUE(adapter->isFree(10, 10));
  EXPECT_TRUE(adapter->isFree(11, 10));
  EXPECT_FALSE(adapter->isFree(12, 10));
  EXPECT_FALSE(adapter->isFree(13, 10));
}

// 地图还没有覆盖到的地方按 UNKNOWN 处理：value 上报 255，而不是障碍。
TEST(UnknownTraversability, OutOfMapIsReportedAsUnknown)
{
  auto snapshot = makeSnapshot();
  auto adapter = std::make_shared<rog_map::QueryAdapter>();
  adapter->update(std::make_shared<const rog_map::MapSnapshot>(snapshot), nullptr);

  EXPECT_EQ(adapter->value(kWidth + 5U, kHeight + 5U), 255U);
  EXPECT_FALSE(adapter->isFree(kWidth + 5U, 5U));
}

// 足印范围内全是 UNKNOWN 时必须放行（车身滤除 + 雷达盲区让起点本来就不可见）。
TEST(UnknownTraversability, FootprintFreeAcceptsUnknownRegion)
{
  auto snapshot = makeSnapshot();
  auto adapter = std::make_shared<rog_map::QueryAdapter>();
  adapter->update(std::make_shared<const rog_map::MapSnapshot>(snapshot), nullptr);

  const Eigen::Vector3d pos(worldCoord(20), worldCoord(20), 0.0);
  EXPECT_TRUE(adapter->footprintFree(pos));
}

// 已知障碍进入足印才拒绝，并报告阻挡格用于诊断。
TEST(UnknownTraversability, FootprintFreeRejectsKnownObstacle)
{
  auto snapshot = makeSnapshot();
  cellAt(snapshot, 21, 20) = kObstacle;
  auto adapter = std::make_shared<rog_map::QueryAdapter>();
  adapter->update(std::make_shared<const rog_map::MapSnapshot>(snapshot), nullptr);

  unsigned int bx = 0;
  unsigned int by = 0;
  unsigned char bv = 0;
  EXPECT_FALSE(adapter->footprintFree(Eigen::Vector3d(worldCoord(20), worldCoord(20), 0.0), bx, by, bv));
  EXPECT_EQ(bx, 21U);
  EXPECT_EQ(by, 20U);
  EXPECT_EQ(bv, kObstacle);
}

// 足印伸出地图边界不算障碍：地图外按 UNKNOWN 处理。
TEST(UnknownTraversability, FootprintFreeAcceptsMapBoundary)
{
  auto snapshot = makeSnapshot();
  auto adapter = std::make_shared<rog_map::QueryAdapter>();
  adapter->update(std::make_shared<const rog_map::MapSnapshot>(snapshot), nullptr);

  const Eigen::Vector3d pos(worldCoord(1), worldCoord(1), 0.0);
  EXPECT_TRUE(adapter->footprintFree(pos));
}

// 边检查与足印同语义：跨 UNKNOWN 允许，跨已知障碍拒绝。
TEST(UnknownTraversability, EdgeAllowedFollowsTheSamePolicy)
{
  auto snapshot = makeSnapshot();
  auto adapter = std::make_shared<rog_map::QueryAdapter>();
  adapter->update(std::make_shared<const rog_map::MapSnapshot>(snapshot), nullptr);

  const Eigen::Vector3d from(worldCoord(20), worldCoord(20), 0.0);
  const Eigen::Vector3d to_unknown(worldCoord(24), worldCoord(20), 0.0);
  EXPECT_TRUE(adapter->edgeAllowed(from, to_unknown));

  auto blocked = makeSnapshot();
  cellAt(blocked, 22, 20) = kObstacle;
  auto blocked_adapter = std::make_shared<rog_map::QueryAdapter>();
  blocked_adapter->update(std::make_shared<const rog_map::MapSnapshot>(blocked), nullptr);
  EXPECT_FALSE(blocked_adapter->edgeAllowed(from, to_unknown));
}
