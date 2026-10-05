#pragma once

#include <Eigen/Core>
#include <grid_map_core/GridMap.hpp>

#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <vector>

#include <rog_map/field_layer.hpp>
#include <rog_map/map_query_interface.hpp>
#include <rog_map/projection_layer.hpp>
#include <rog_map/terrain_layer.hpp>
#include <rog_map/terrain_types.hpp>

namespace rog_map {

// 数据结构（EdgeState / GlobalTerrainCell / GlobalTerrainConfig）定义在 terrain_types.hpp，
// 以便 config.hpp 只依赖轻量头文件。
// 单个 XY 列在“预计地面高度 z_ref”附近的 3D 自由空间证据。
// free/unknown/occupied 都来自 ROGMap 真实 raycasting 维护的 voxel 状态，
// 而不是“这一列被某条 ray 碰过”。
struct ColumnFreeEvidence
{
  int free_count{0};
  int unknown_count{0};
  int occupied_count{0};
  // support band（[z_ref - down, z_ref + up]）是否被真实 ray miss 确认为自由空间。
  bool support_band_observed{false};
  // below band（[z_ref - drop_probe_depth, z_ref - down]）是否也是自由空间。
  bool below_band_observed{false};
  // support band 内是否存在占据/支撑回波（有的话就不是 no-support）。
  bool support_hit_near_z_ref{false};
  float free_ratio{0.0F};
  // 强 no-support：support band 与 below band 都被确认自由，且 band 内没有支撑回波。
  bool strong_no_support{false};
};

// 用局部投影列（含 3D FREE/OCCUPIED 垂直位图）评估 ColumnFreeEvidence。
ColumnFreeEvidence evaluateColumnFreeEvidence(const CellData & column,
  double z_ref,
  double resolution,
  const GlobalTerrainConfig & config);

// 全局地形图：固定世界系二维格 + 邻接边图，实现 MapQueryInterface 供 planner 统一查询。
//
// 更新方式：local observation -> global patch。每帧只把机器人附近 ROGMap 局部
// projection/terrain 的结果增量写回对应世界格，历史区域保持不变。
class GlobalTerrainGrid : public MapQueryInterface
{
public:
  GlobalTerrainGrid() = default;

  void configure(const GlobalTerrainConfig & config);

  bool enabled() const;
  const GlobalTerrainConfig & config() const;

  // 用一帧局部观测更新世界坐标下的对应 patch。
  void updateFromLocal(const ProjectionLayer & projection,
    const TerrainLayer & terrain,
    double now,
    const Eigen::Vector3d & robot_position);

  // 强制下一次查询前重建 ESDF。
  void invalidateField();

  // --- MapQueryInterface ---
  bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const override;
  void mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const override;
  unsigned int sizeX() const override;
  unsigned int sizeY() const override;
  double resolution() const override;
  double originX() const override;
  double originY() const override;
  uint8_t value(unsigned int mx, unsigned int my) const override;
  const unsigned char * values() const override;
  bool copyValues(std::vector<unsigned char> & out) const override;
  bool isValid(unsigned int mx, unsigned int my) const override;
  bool isFree(unsigned int mx, unsigned int my) const override;
  bool terrain(unsigned int mx, unsigned int my, TerrainCell & cell) const override;
  bool footprintFree(const Eigen::Vector3d & pos) const override;
  bool edgeAllowed(const Eigen::Vector3d & from, const Eigen::Vector3d & to) const override;
  double softClearance(const Eigen::Vector3d & pos, double fallback) const override;
  TerrainConfig terrainPolicy() const override;
  QueryResult query(const Eigen::Vector3d & pos) const override;
  bool evaluate(const Eigen::Vector3d & pos, double & dist, Eigen::Vector3d & grad) const override;
  std::chrono::steady_clock::time_point lastUpdateSteady() const override;

  // --- 统一规划查询（A* / LOS / 校验共用同一套语义）---
  // UNKNOWN -> true, FREE -> true, OCCUPIED -> false
  bool cellTraversable(unsigned int mx, unsigned int my) const;
  // UNKNOWN_EDGE -> true, PASSABLE_EDGE -> true, BLOCKED_EDGE -> false
  bool edgeAllowedCells(unsigned int ax, unsigned int ay, unsigned int bx, unsigned int by) const;
  EdgeState edgeState(unsigned int ax, unsigned int ay, unsigned int bx, unsigned int by) const;

  // --- 诊断 / 测试 ---
  struct Stats
  {
    uint64_t updates{0};
    uint64_t observed_cells{0};
    uint64_t support_cells{0};
    uint64_t obstacle_cells{0};
    uint64_t blocked_edge_cells{0};
    uint64_t field_rebuilds{0};
    double last_update_ms{0.0};
    double last_field_ms{0.0};
  };
  Stats stats() const;
  const std::vector<GlobalTerrainCell> & cells() const;
  int indexOf(unsigned int mx, unsigned int my) const;
  bool cellIndex(double wx, double wy, int & ix, int & iy) const;
  // 每个格到右/上邻格的边状态（诊断与验收用）。越界为 UNKNOWN。
  std::vector<uint8_t> exportEdgeStates() const;

  // 可视化分类（行主序，长度 = sizeX*sizeY）：
  //   0 = UNKNOWN（不绘制）、1 = 已观测可通行、2 = BLOCKED 边、3 = 障碍。
  // 只把“有信息的格子”画出来，未知区域留空，避免每帧发布 64 万个点。
  enum class VizClass : uint8_t
  {
    UNKNOWN = 0,
    FREE = 1,
    BLOCKED_EDGE = 2,
    OBSTACLE = 3
  };
  std::vector<uint8_t> exportVizClasses() const;

  // Snapshot of the world-fixed terrain layers used by the planning backend.
  grid_map::GridMap gridMapSnapshot() const;

private:
  bool inBounds(int ix, int iy) const;
  size_t idxOf(int ix, int iy) const;
  // 调用前必须已持有 mutex_。
  EdgeState edgeStateLocked(int ax, int ay, int bx, int by) const;
  void applyLocalPatchLocked(const ProjectionLayer & projection,
    const TerrainLayer & terrain,
    double now,
    const Eigen::Vector3d & robot_position);
  // 只重算 [x0,x1]x[y0,y1] 窗口内的 values_/blocked_（含边界一圈），不做全图刷新。
  void recomputeRegion(int x0, int x1, int y0, int y1);
  void recountStats();
  void syncCellToGridMapLocked(int ix, int iy);
  // 该格是否为“被封住的格”：自身障碍，或存在一条 BLOCKED 边。
  bool blockedCellLocked(size_t idx) const;
  void rebuildFieldIfNeeded();

  mutable std::mutex mutex_;
  GlobalTerrainConfig cfg_{};
  int width_{0};
  int height_{0};
  std::vector<GlobalTerrainCell> cells_;
  grid_map::GridMap grid_map_;
  std::vector<uint8_t> values_;      // 0 free / 254 occupied / 255 unknown
  std::vector<uint8_t> mask_;        // ESDF 源：0 blocked，1 free
  std::vector<uint8_t> blocked_;     // 1 = 障碍或邻接 BLOCKED 边
  std::shared_ptr<DynamicLayer> field_;
  mutable bool field_dirty_{true};
  double last_field_time_{-std::numeric_limits<double>::infinity()};
  std::chrono::steady_clock::time_point last_update_steady_{};
  mutable Stats stats_{};
};

}  // namespace rog_map
