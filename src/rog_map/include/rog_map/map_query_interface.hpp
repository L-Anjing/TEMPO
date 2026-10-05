#pragma once

#include <Eigen/Core>

#include <chrono>
#include <cstdint>
#include <limits>
#include <vector>
#include <rog_map/terrain_types.hpp>

namespace rog_map {

enum class QueryStatus : uint8_t
{
  OK = 0,
  OUT_OF_MAP,
  SNAPSHOT_INVALID,
  FIELD_UNINITIALIZED,
  INTERPOLATION_FAILED,
  TF_FAILED,
  NONFINITE_INPUT,
  NONFINITE_OUTPUT
};

struct QueryResult
{
  bool ok{false};
  QueryStatus status{QueryStatus::SNAPSHOT_INVALID};
  double distance{std::numeric_limits<double>::quiet_NaN()};
  Eigen::Vector3d gradient{Eigen::Vector3d::Zero()};
  uint64_t projection_sequence{0};
  uint64_t mask_sequence{0};
  uint64_t field_sequence{0};
  uint64_t snapshot_sequence{0};
  double field_age_ms{std::numeric_limits<double>::quiet_NaN()};
};

const char * queryStatusName(QueryStatus status);

class MapQueryInterface
{
public:
  virtual ~MapQueryInterface() = default;

  virtual bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const = 0;
  virtual void mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const = 0;

  virtual unsigned int sizeX() const = 0;
  virtual unsigned int sizeY() const = 0;
  virtual double resolution() const = 0;
  virtual double originX() const = 0;
  virtual double originY() const = 0;

  virtual uint8_t value(unsigned int mx, unsigned int my) const = 0;
  virtual const unsigned char * values() const = 0;
  virtual bool copyValues(std::vector<unsigned char> & out) const
  {
    const auto * data = values();
    const size_t count = static_cast<size_t>(sizeX()) * static_cast<size_t>(sizeY());
    if (!data || count == 0U) {
      out.clear();
      return false;
    }
    out.assign(data, data + count);
    return true;
  }

  virtual bool isValid(unsigned int mx, unsigned int my) const = 0;
  virtual bool isFree(unsigned int mx, unsigned int my) const = 0;
  // Returns false for legacy maps without online terrain metadata.
  virtual bool terrain(unsigned int mx, unsigned int my, TerrainCell & cell) const
  { (void)mx; (void)my; (void)cell; return false; }
  virtual bool footprintFree(const Eigen::Vector3d & pos) const { (void)pos; return true; }
  virtual bool edgeAllowed(const Eigen::Vector3d & from, const Eigen::Vector3d & to) const
  { (void)from; (void)to; return true; }
  virtual double softClearance(const Eigen::Vector3d & pos, double fallback) const
  { (void)pos; return fallback; }
  virtual TerrainConfig terrainPolicy() const { return {}; }
  // 该查询对象最近一次地图更新的 steady_clock 时间，供上层判断地图新鲜度。
  // 默认返回 epoch（表示“不支持/尚不可用”）。
  virtual std::chrono::steady_clock::time_point lastUpdateSteady() const { return {}; }

  virtual QueryResult query(const Eigen::Vector3d & pos) const;
  virtual bool evaluate(const Eigen::Vector3d & pos, double & dist, Eigen::Vector3d & grad) const = 0;
};

}  // namespace rog_map
