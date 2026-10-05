#ifndef MINCO_PLANNER__GLOBAL_PATH_SEARCHER_HPP_
#define MINCO_PLANNER__GLOBAL_PATH_SEARCHER_HPP_

#include "minco_core/header.hpp"

namespace minco_planner {

class PlannerModeContext;

class GlobalPathSearcher
{
public:
  void configure(std::shared_ptr<tf2_ros::Buffer> tf,
    Astar * astar,
    bool allow_unknown,
    double tolerance,
    rclcpp::Clock::SharedPtr clock,
    rclcpp::Logger logger);

  void setQuery(const std::shared_ptr<rog_map::MapQueryInterface> & global_query);

  // 最近一次 plan() 失败的机器可读原因（空 = 成功）。
  const std::string & lastFailureReason() const { return last_failure_reason_; }

  bool plan(const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const PlannerModeContext & mode_context,
    std::function<bool()> cancel_checker,
    std::vector<geometry_msgs::msg::PoseStamped> & latest_global_path);

  bool makePlan(const geometry_msgs::msg::Pose & start,
    const geometry_msgs::msg::Pose & goal,
    const PlannerModeContext & mode_context,
    double tolerance,
    std::function<bool()> cancel_checker,
    nav_msgs::msg::Path & plan);

private:
  bool normalizePoseToFrame(const geometry_msgs::msg::PoseStamped & in,
    const std::string & fallback_frame,
    const std::string & target_frame,
    const std::string & context,
    geometry_msgs::msg::PoseStamped & out) const;

  bool planPriorMap(const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const PlannerModeContext & mode_context,
    const std::function<bool()> & cancel_checker,
    std::vector<geometry_msgs::msg::PoseStamped> & latest_global_path);

  bool planExploration(const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const PlannerModeContext & mode_context,
    const std::function<bool()> & cancel_checker,
    std::vector<geometry_msgs::msg::PoseStamped> & latest_global_path);

  bool makePlanOnQuery(const geometry_msgs::msg::Pose & start,
    const geometry_msgs::msg::Pose & goal,
    const std::shared_ptr<rog_map::MapQueryInterface> & query,
    const std::string & output_frame,
    const std::string & failure_source,
    double tolerance,
    std::function<bool()> cancel_checker,
    nav_msgs::msg::Path & plan,
    std::vector<geometry_msgs::msg::PoseStamped> & latest_global_path);

  std::shared_ptr<tf2_ros::Buffer> tf_;
  Astar * astar_{nullptr};
  std::shared_ptr<rog_map::MapQueryInterface> global_query_;
  bool allow_unknown_{true};
  double tolerance_{0.5};
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("GlobalPathSearcher")};
  std::string last_failure_reason_;
};

}  // namespace minco_planner

#endif  // MINCO_PLANNER__GLOBAL_PATH_SEARCHER_HPP_
