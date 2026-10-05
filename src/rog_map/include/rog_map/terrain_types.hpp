#pragma once

#include <Eigen/Core>
#include <cmath>
#include <cstdint>
#include <limits>

namespace rog_map {

struct TerrainConfig
{
  bool enable{false};
  bool seed_current_footprint{false};
  double ground_seed_tolerance{0.10};
  double slope_min_deg{5.0};
  double slope_max_deg{35.0};
  double slope_fit_radius{0.20};
  double slope_max_residual{0.020};
  double stair_rise{0.15};
  double stair_tread{0.30};
  double stair_rise_tolerance{0.025};
  double stair_half_width{0.55};
  double stair_min_width{0.65};
  double terrain_min_component_area{0.50};
  double obstacle_min_height{0.15};
  // --- no-support（负障碍 / drop-off）证据参数 ---------------------------------
  // support band：围绕“预计地面高度 z_ref”的高度带；band 内必须被真实 ray miss
  // 确认为 FREE，才可能产生 no-support 证据。
  double support_band_up{0.05};
  double support_band_down{0.05};
  // below band：z_ref 以下这一段。射线已经穿过预期地面高度并继续进入更低空间，
  // 这才是强 no-support 证据（把“地面点暂时没打到”和“这里确实是空的”分开）。
  double drop_probe_depth{0.20};
  // band 内至少要有这么多 FREE 体素才算“确认自由空间”。
  int min_free_voxels{2};
  double alignment_cosine{0.9659258263};
  double alignment_weight{2000.0};
  double flat_clearance_radius{0.5656854249};
  double terrain_half_length{0.40};
  double terrain_half_width{0.40};
  bool yaw_independent_footprint{true};
  double clearance_margin{0.025};
};

// Same labels as fastlio_nav2. UNKNOWN has no observed support surface.
enum class TerrainType : uint8_t { FLAT = 0, OBSTACLE = 1, SLOPE = 2, STEP = 3, UNKNOWN = 255 };

struct TerrainCell
{
  TerrainType type{TerrainType::UNKNOWN};
  float surface_z{std::numeric_limits<float>::quiet_NaN()};
  Eigen::Vector2d uphill{Eigen::Vector2d::Zero()};
  bool support_observed{false};
};

inline bool directionalTerrain(const TerrainCell & cell)
{
  return cell.type == TerrainType::SLOPE || cell.type == TerrainType::STEP;
}

inline bool terrainDirectionAllowed(const TerrainCell & cell, const Eigen::Vector2d & travel,
  double alignment_cosine)
{
  if (!directionalTerrain(cell) || travel.norm() < 1.0e-6) {
    return true;
  }
  return cell.uphill.norm() > 0.1 &&
    std::abs(travel.normalized().dot(cell.uphill.normalized())) >= alignment_cosine;
}

// ---------------------------------------------------------------------------
// 全局二维地形图的数据类型。
//
// 这些是纯数据结构，放在这里（而不是 global_terrain_grid.hpp）是为了避免
// config.hpp -> global_terrain_grid.hpp -> projection_layer.hpp -> sliding_map.h
// -> config.hpp 的循环包含。
// ---------------------------------------------------------------------------

// 邻接边状态。UNKNOWN 按乐观策略允许通行；BLOCKED 一律禁止。
enum class EdgeState : uint8_t
{
  UNKNOWN = 0,
  PASSABLE = 1,
  BLOCKED = 2
};

// 世界系固定的二维地形格。ROGMap 的滚动 3D 窗口只负责“机器人附近的观测”，
// 全局图才是 planner 使用的世界固定地图：不随机器人移动丢失，但可被新观测动态修正。
struct GlobalTerrainCell
{
  bool observed{false};         // 至少被观测过一次（有真实支撑面或障碍）
  bool support_valid{false};    // 有可用的支撑面高度
  float surface_z{std::numeric_limits<float>::quiet_NaN()};
  bool obstacle{false};         // 已知障碍（含 STEP）
  float support_confidence{0.0f};
  float obstacle_confidence{0.0f};
  // 射线穿过、但在地面高度没有任何回波：连续多帧确认后视为“悬空/坑”，
  // 用来实现“一侧有支撑面、另一侧已观测但没有支撑面 -> BLOCKED_EDGE”。
  // 单帧的射线穿过不足以确认（局部图仍在填充时会造成大片误封）。
  float void_confidence{0.0f};
  double last_update{0.0};
};

struct GlobalTerrainConfig
{
  bool enable{false};
  double resolution{0.05};
  double size_x{40.0};
  double size_y{40.0};
  // 世界系下网格原点（左下角）。默认把世界原点放在地图中心。
  double origin_x{-20.0};
  double origin_y{-20.0};
  // 坡道判定：相邻支撑格高度梯度 <= tan(slope_max_deg) 视为 PASSABLE。
  double slope_max_deg{35.0};
  double slope_min_deg{5.0};
  // 障碍清除/建立的证据增益（0~1）。
  float obstacle_hit_gain{0.60f};
  float obstacle_miss_gain{0.55f};
  float support_hit_gain{0.45f};
  float support_seed_gain{0.20f};
  float support_miss_gain{0.30f};
  // 射线穿过但无地面回波的累积增益，以及“确认悬空”的阈值。
  // no-support（case B）证据增益：每来一帧“强 no-support”就加这么多。
  float void_gain{0.34f};
  float void_confirm{0.75f};
  // support band / below band 的几何（与 terrain.* 同源，见 Config 加载）。
  double support_band_up{0.05};
  double support_band_down{0.05};
  double drop_probe_depth{0.20};
  int min_free_voxels{2};

  // 迟滞阈值：避免边界抖动。
  float obstacle_enter{0.50f};
  float obstacle_exit{0.20f};
  float support_enter{0.40f};
  float support_exit{0.15f};
  // ESDF（MINCO / A* 动态碰撞用）。全局图较大，按周期重建而不是每帧。
  bool field_enable{true};
  double field_max_distance{6.0};
  double field_min_distance{-3.0};
  bool field_clamp_distance{true};
  double field_update_period_sec{0.2};
  // 车身净空（与 terrain.* 保持一致，决定 footprintFree/softClearance）。
  bool yaw_independent_footprint{true};
  double flat_clearance_radius{0.5656854249};
  double terrain_half_length{0.40};
  double terrain_half_width{0.40};
  double clearance_margin{0.025};
  double alignment_cosine{0.9659258263};
  double alignment_weight{2000.0};
};

}  // namespace rog_map
