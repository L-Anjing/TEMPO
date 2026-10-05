#pragma once

#include <rog_map/projection_layer.hpp>
#include <rog_map/terrain_types.hpp>
#include <Eigen/Geometry>

namespace rog_map {

// One walking surface per XY column. Optional current-body blind-spot seed only;
// all future support must be observed. No prior PCD.
class TerrainLayer
{
public:
  void update(const ProjectionLayer & projection, const TerrainConfig & config,
    const Eigen::Vector3d & robot_position,
    const Eigen::Quaterniond & rotation = Eigen::Quaterniond::Identity());
  const std::vector<TerrainCell> & cells() const { return cells_; }
  const std::vector<uint8_t> & values() const { return values_; }
  const std::vector<uint8_t> & mask() const { return mask_; }

private:
  void detect_slopes(const std::vector<float> & surface, int width, int height,
    std::vector<int8_t> & terrain, std::vector<double> & direction_x,
    std::vector<double> & direction_y) const;
  void detect_stairs(const std::vector<float> & surface, int width, int height,
    std::vector<int8_t> & terrain, std::vector<double> & direction_x,
    std::vector<double> & direction_y) const;
  void remove_small_terrain_components(int width, int height, int8_t label,
    std::vector<int8_t> & terrain, std::vector<double> & direction_x,
    std::vector<double> & direction_y) const;
  struct StepSeed { int x; int y; double dx; double dy; };
  static size_t cell_index(int x, int y, int width)
  { return static_cast<size_t>(y) * static_cast<size_t>(width) + static_cast<size_t>(x); }
  TerrainConfig config_;
  double resolution_{0.05};
  std::vector<TerrainCell> cells_;
  std::vector<uint8_t> values_;
  std::vector<uint8_t> mask_;
};

}  // namespace rog_map
