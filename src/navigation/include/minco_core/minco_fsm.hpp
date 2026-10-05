#ifndef MINCO_PLANNER__MINCO_FSM_HPP_
#define MINCO_PLANNER__MINCO_FSM_HPP_

#include "minco_core/header.hpp"

namespace minco_planner {

class MincoPlanner;

class MincoFsm
{
public:
  // === Internal Types ===
  enum class State
  {
    INIT,
    WAIT_GOAL,
    GENERATE_TRAJ,
    FOLLOW_TRAJ,
  };

  using PlannerPtr = std::shared_ptr<MincoPlanner>;

  // === Constructor & Lifecycle ===
  explicit MincoFsm(const PlannerPtr & planner);

  // === Core Planning Interfaces ===
  void callMainFsmOnce();
  void cancelGoal();

  // === Utility & Helper Functions ===
  State getState() const { return state_; }

private:
  // === Utility & Helper Functions ===
  // --- State Transition ---
  void changeState(const char * caller, State new_state);

  // === Core Modules (Pointers to FSM, Optimizers, etc.) ===
  PlannerPtr planner_;

  // === State Variables & Caches ===
  // --- FSM State ---
  State state_{State::INIT};
  State last_state_{State::INIT};

  // --- Goal Lifecycle ---
  bool has_goal_{false};
  geometry_msgs::msg::PoseStamped goal_;
  std::chrono::steady_clock::time_point last_external_attempt_{};
  std::chrono::steady_clock::time_point last_path_switch_{};
};

}  // namespace minco_planner

#endif  // MINCO_PLANNER__MINCO_FSM_HPP_
