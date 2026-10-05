#ifndef MINCO_PLANNER__MAP_QUERY_ADAPTERS_HPP_
#define MINCO_PLANNER__MAP_QUERY_ADAPTERS_HPP_

#include "minco_core/header.hpp"

namespace minco_planner {

class Nav2CostmapQuery : public rog_map::MapQueryInterface
{
public:
  explicit Nav2CostmapQuery(nav2_costmap_2d::Costmap2D * costmap);

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
  rog_map::QueryResult query(const Eigen::Vector3d & pos) const override;
  bool evaluate(const Eigen::Vector3d & pos, double & dist, Eigen::Vector3d & grad) const override;

private:
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
};

class FrameAwareRogQuery : public rog_map::MapQueryInterface
{
public:
  FrameAwareRogQuery(std::shared_ptr<rog_map::MapQueryInterface> raw,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::string planning_frame,
    std::string rog_frame,
    rclcpp::Clock::SharedPtr clock,
    rclcpp::Logger logger);

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
  rog_map::QueryResult query(const Eigen::Vector3d & pos) const override;
  bool evaluate(const Eigen::Vector3d & pos, double & dist, Eigen::Vector3d & grad) const override;

private:
  bool transformPlanningToRog(const Eigen::Vector3d & in, Eigen::Vector3d & out) const;
  bool transformRogToPlanning(const Eigen::Vector3d & in, Eigen::Vector3d & out) const;
  bool rotateRogToPlanning(const Eigen::Vector3d & in, Eigen::Vector3d & out) const;

  std::shared_ptr<rog_map::MapQueryInterface> raw_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string planning_frame_;
  std::string rog_frame_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_;
};

// 给全局地形图套一个搜索窗口：A* 只在 start/goal 外扩 margin 的子网格上搜索。
// 40x40 m @0.05 的全局图有 640k 格，直接做势场传播会把 executor 阻塞数秒
// （生命周期心跳 4 s 超时会把 planner_server 判死）。世界坐标查询全部转发给 base。
class RoiQueryAdapter : public rog_map::MapQueryInterface
{
public:
  RoiQueryAdapter(std::shared_ptr<rog_map::MapQueryInterface> base,
    double min_x, double min_y, double margin);

  bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const override;
  void mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const override;
  unsigned int sizeX() const override { return static_cast<unsigned int>(w_); }
  unsigned int sizeY() const override { return static_cast<unsigned int>(h_); }
  double resolution() const override { return res_; }
  double originX() const override { return origin_x_; }
  double originY() const override { return origin_y_; }
  uint8_t value(unsigned int mx, unsigned int my) const override;
  const unsigned char * values() const override { return values_.data(); }
  bool copyValues(std::vector<unsigned char> & out) const override;
  bool isValid(unsigned int mx, unsigned int my) const override;
  bool isFree(unsigned int mx, unsigned int my) const override;
  bool terrain(unsigned int mx, unsigned int my, rog_map::TerrainCell & cell) const override;
  bool footprintFree(const Eigen::Vector3d & pos) const override { return base_->footprintFree(pos); }
  bool edgeAllowed(const Eigen::Vector3d & from, const Eigen::Vector3d & to) const override
  { return base_->edgeAllowed(from, to); }
  double softClearance(const Eigen::Vector3d & pos, double fallback) const override
  { return base_->softClearance(pos, fallback); }
  rog_map::TerrainConfig terrainPolicy() const override { return base_->terrainPolicy(); }
  rog_map::QueryResult query(const Eigen::Vector3d & pos) const override { return base_->query(pos); }
  bool evaluate(const Eigen::Vector3d & pos, double & dist, Eigen::Vector3d & grad) const override
  { return base_->evaluate(pos, dist, grad); }
  std::chrono::steady_clock::time_point lastUpdateSteady() const override
  { return base_->lastUpdateSteady(); }

private:
  bool inRoi(int lx, int ly) const { return lx >= 0 && ly >= 0 && lx < w_ && ly < h_; }
  std::shared_ptr<rog_map::MapQueryInterface> base_;
  int ox_{0}, oy_{0}, w_{0}, h_{0};
  int base_w_{0}, base_h_{0};
  double res_{0.0}, origin_x_{0.0}, origin_y_{0.0};
  std::vector<unsigned char> values_;
};

}  // namespace minco_planner

#endif  // MINCO_PLANNER__MAP_QUERY_ADAPTERS_HPP_
