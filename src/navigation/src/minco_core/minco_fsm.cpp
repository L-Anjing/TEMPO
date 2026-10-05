#include "minco_core/minco_fsm.hpp"

#include <cmath>

#include "minco_core/minco_planner.hpp"

namespace minco_planner {

MincoFsm::MincoFsm(const PlannerPtr & planner) : planner_(planner) {}

void MincoFsm::cancelGoal()
{
  has_goal_ = false;
  changeState("CANCEL_GOAL", State::WAIT_GOAL);
}

void MincoFsm::callMainFsmOnce()
{
  if (!planner_) {
    return;
  }

  geometry_msgs::msg::PoseStamped new_goal;
  if (planner_->consumePendingGoal(new_goal)) {
    goal_ = new_goal;
    has_goal_ = true;
    last_external_attempt_ = {};
    last_path_switch_ = {};
    changeState("NEW_GOAL", State::GENERATE_TRAJ);
  }

  geometry_msgs::msg::PoseStamped current_pose;
  const bool has_odom = planner_->getRobotPose(current_pose);
  if (!has_odom) {
    planner_->publishPlanningStatus("INPUT_UNAVAILABLE");
  } else {
    planner_->inputAvailable();
  }

  switch (state_) {
    case State::INIT:
      if (has_odom) {
        changeState("INIT", State::WAIT_GOAL);
      }
      return;

    case State::WAIT_GOAL:
      if (has_goal_) {
        changeState("WAIT_GOAL", State::GENERATE_TRAJ);
      }
      return;

    case State::GENERATE_TRAJ: {
      if (!has_goal_) {
        changeState("NO_GOAL", State::WAIT_GOAL);
        return;
      }
      if (!has_odom) {
        return;
      }
      if (planner_->checkGoalReached(current_pose, goal_)) {
        if (planner_->getCurrentSpeed().head<2>().norm() < 0.3) {
          has_goal_ = false;
          planner_->publishPlanningStatus("goal_reached");
          changeState("GOAL_REACHED", State::WAIT_GOAL);
        }
        return;
      }

      const auto attempt = std::chrono::steady_clock::now();
      if (attempt - last_external_attempt_ < std::chrono::milliseconds(200)) {
        return;
      }
      last_external_attempt_ = attempt;
      if (!planner_->PlanGlobalPath(current_pose, goal_) || !planner_->ReplanLocal(current_pose)) {
        planner_->publishPlanningStatus(planner_->searchFailureStatus("NO_VALID_ROUTE"));
        return;
      }
      changeState("PLAN_READY", State::FOLLOW_TRAJ);
      return;
    }

    case State::FOLLOW_TRAJ: {
      if (!has_goal_) {
        changeState("NO_GOAL", State::WAIT_GOAL);
        return;
      }
      if (!has_odom) {
        return;
      }
      if (planner_->checkGoalReached(current_pose, goal_)) {
        if (planner_->getCurrentSpeed().head<2>().norm() < 0.3) {
          has_goal_ = false;
          planner_->publishPlanningStatus("goal_reached");
          changeState("GOAL_REACHED", State::WAIT_GOAL);
        }
        return;
      }

      const bool path_blocked = !planner_->hasExecutablePlan() || !planner_->isTrajSafe();
      planner_->setForceColdStart(path_blocked);

      if (!path_blocked && planner_->localHorizonExhausted()) {
        const auto attempt = std::chrono::steady_clock::now();
        if (attempt - last_external_attempt_ < std::chrono::milliseconds(200)) {
          return;
        }
        last_external_attempt_ = attempt;
        if (!planner_->continueLocalPlan(current_pose, goal_)) {
          planner_->publishPlanningStatus(planner_->searchFailureStatus("NO_VALID_ROUTE"));
        }
        return;
      }

      bool replan_now = path_blocked;
      if (!path_blocked) {
        if (!planner_->betterPathEnabled()) {
          return;
        }
        const double dx = goal_.pose.position.x - current_pose.pose.position.x;
        const double dy = goal_.pose.position.y - current_pose.pose.position.y;
        if (std::hypot(dx, dy) < planner_->terminalRadius()) {
          return;
        }
        const auto attempt = std::chrono::steady_clock::now();
        if (attempt - last_external_attempt_ <
            std::chrono::duration<double>(planner_->replanCheckPeriod()) ||
            attempt - last_path_switch_ <
            std::chrono::duration<double>(planner_->minSwitchInterval())) {
          return;
        }
        last_external_attempt_ = attempt;
        replan_now = planner_->betterPathAvailable(current_pose, goal_);
        if (replan_now) {
          last_path_switch_ = attempt;
        }
      }

      if (replan_now &&
          (!planner_->PlanGlobalPath(current_pose, goal_) || !planner_->ReplanLocal(current_pose))) {
        planner_->publishPlanningStatus(planner_->searchFailureStatus("NO_VALID_ROUTE"));
        changeState("REPLAN_FAILED", State::GENERATE_TRAJ);
      }
      return;
    }
  }
}

void MincoFsm::changeState(const char * caller, State new_state)
{
  if (state_ == new_state) {
    return;
  }
  (void)caller;
  last_state_ = state_;
  state_ = new_state;
}

}  // namespace minco_planner
