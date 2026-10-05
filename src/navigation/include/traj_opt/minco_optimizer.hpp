#ifndef MINCO_PLANNER__MINCO_OPTIMIZER_HPP_
#define MINCO_PLANNER__MINCO_OPTIMIZER_HPP_

#include <Eigen/Core>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>
#include <vector>

#include "data_structure/base/trajectory.h"
#include "traj_opt/minco.h"
#include "utils/header/color_text.hpp"
#include "utils/header/eigen_alias.hpp"
#include "utils/optimization/lbfgs.h"
#include "utils/optimization/optimization_utils.h"

#include "rog_map/map_query_interface.hpp"

namespace minco_planner {

using namespace super_utils;
using namespace traj_opt;
using namespace optimization_utils;
using namespace math_utils;
using namespace color_text;

class MincoOptimizer
{
public:
  // === Internal Types ===
  // --- Seed direction soft-penalty metrics (diagnostics only) ---
  // 在最近一次 optimize 的**最终解**上按同一逐点公式复核。
  struct DirectionMetrics
  {
    bool enabled{false};
    double mean_cos{std::numeric_limits<double>::quiet_NaN()};
    double min_cos{std::numeric_limits<double>::quiet_NaN()};
    double penalty_cost{0.0};
    // 首次 cost 评估时方向项对 xi 段梯度的贡献范数（证明该项真的进了优化器）。
    double first_grad_norm{-1.0};
  };

  // --- Optimizer Configuration ---
  struct Config
  {
    double safe_dist{0.3};
    double max_vel{5.0};
    double max_acc{5.0};
    double max_jerk{10.0};
    double turn_angle_deadzone{0.174};
    double turn_angle_saturation{1.57};
    double min_turn_vel{1.0};
    double decay_power{2.0};

    double rho{0.01};
    double smooth_eps{0.01};

    // OLD fastlio_nav2 的「有向几何正则性」软罚（逐采样点、按弧长积分）：
    //   dir     = normalize(T·v; eps)                   // 几何切向，不含伪物理速度
    //   cos     = dir · t̂_seed                          // t̂_seed 由相邻段边界切线线性混合
    //   under   = max(0, cos_min − cos)
    //   penalty = weight · 0.5 · under²
    // 默认 OFF。开启后它仍然只是软代价：不固定任何内部切线、不引入内部速度变量、
    // 不限制 waypoint 位置、不加任何等式约束，净空/避障罚仍可把它压过去。
    struct DirectionRegularization
    {
      bool enable{false};
      double weight{1000.0};        // OLD Weights::directed_regularity
      double cos_min{0.1};          // OLD Params::directed_cosine_min
      // OLD GeometryLimits::tangent_regularization：‖T·v‖→0 时给方向归一化做正则。
      double tangent_regularization{1e-6};
    } direction_regularization;

    VecDf magnitudeBounds, penaltyWeights;
    int time_allocation_iters{5};
    int integral_res{16};
    double opt_accuracy{1e-4};

    bool print_optimizer_log{true};
  } cfg_;

  // === Constructor & Lifecycle ===
  MincoOptimizer(const Config & cfg) : cfg_(cfg)
  {
    opt_vars_.minco_solver_ = std::make_shared<traj_opt::MINCO_S3NU>();
    opt_vars_.magnitudeBounds.resize(cfg_.magnitudeBounds.size());
    opt_vars_.penaltyWeights.resize(cfg_.penaltyWeights.size());
    opt_vars_.magnitudeBounds = cfg_.magnitudeBounds;
    opt_vars_.penaltyWeights = cfg_.penaltyWeights;
    opt_vars_.rho = cfg_.rho;
    opt_vars_.smooth_eps = cfg_.smooth_eps;
    opt_vars_.integral_res = cfg_.integral_res;
  }

  // === Core Planning Interfaces ===
  // --- Configuration and Initialization ---
  void setConfig(const Config & cfg)
  {
    cfg_ = cfg;
    opt_vars_.magnitudeBounds.resize(cfg_.magnitudeBounds.size());
    opt_vars_.penaltyWeights.resize(cfg_.penaltyWeights.size());
    opt_vars_.magnitudeBounds = cfg_.magnitudeBounds;
    opt_vars_.penaltyWeights = cfg_.penaltyWeights;
    opt_vars_.rho = cfg_.rho;
    opt_vars_.smooth_eps = cfg_.smooth_eps;
    opt_vars_.integral_res = cfg_.integral_res;
  }

  void setInitPsAndTs(const vec_Vec3f & init_ps, const VecDf & init_ts);

  void setMap(const std::shared_ptr<rog_map::MapQueryInterface> & map) { opt_vars_.map = map; }

  // 把 LocalPathProcessor 已经算好的 seed 切线喂给优化器（与 waypoints 一一对应，
  // size 必须等于 N+1）。优化器只做单位化与零长度保护，**不重算切线**。
  void setSeedTangents(const std::vector<Eigen::Vector3d> & seed_tangents)
  {
    auto & dir = opt_vars_.direction;
    dir.seed_tangents.clear();
    dir.seed_tangents.reserve(seed_tangents.size());
    for (const auto & t : seed_tangents) {
      const Eigen::Vector2d planar(t.x(), t.y());
      const double norm = planar.norm();
      // 退化（零长度）切线记为 (0,0)：它是「无效」标记，对应采样点不产生任何罚。
      dir.seed_tangents.push_back(
        norm > 1e-9 ? Eigen::Vector2d(planar.x() / norm, planar.y() / norm)
                    : Eigen::Vector2d::Zero());
    }
  }

  const DirectionMetrics & lastDirectionMetrics() const { return last_direction_metrics_; }

  // 优化后的内部 waypoint（3 x (N-1)），用于诊断几何自由度是否真的被使用。
  Eigen::Matrix<double, 3, Eigen::Dynamic> optimizedWaypoints() const { return opt_vars_.points; }

  int lastIterationCount() const { return last_iteration_count_; }
  int lastReturnCode() const { return last_return_code_; }
  double lastObjectiveTotal() const { return last_objective_total_; }
  uint64_t lastQueryFailureCount() const { return last_query_failure_count_; }

  // --- Trajectory Optimization ---
  double optimize(const std::vector<Eigen::Vector3d> & waypoints,
    const Eigen::Matrix3d & start_state,
    const Eigen::Matrix3d & end_state,
    const VecDf & local_magnitudes,
    geometry_utils::Trajectory & out_traj);

private:
  // === Internal Types ===
  // --- Seed direction soft penalty runtime context ---
  // enable 与 seed_tangents.size()==piece_num+1 同时成立才生效；否则整项为 0。
  struct DirectionTerm
  {
    bool enable{false};
    double weight{1000.0};
    double cos_min{0.1};
    double tangent_regularization{1e-6};
    std::vector<Eigen::Vector2d> seed_tangents;  // size = piece_num + 1（零向量 = 退化，不罚）

    // 首次 cost 评估的方向项梯度探针（rhs 只为方向项，供单独 propogateGrad）。
    bool probe_pending{false};
    MatD3f probe_grad_coeffs;
    VecDf probe_grad_times;
    double first_grad_norm{-1.0};

    [[nodiscard]] bool active(const int piece_num) const
    {
      return enable &&
             seed_tangents.size() == static_cast<size_t>(piece_num) + 1U;
    }
  };

  // --- Optimization Runtime Variables ---
  struct OptVars
  {
    int piece_num;
    double first_grad_xi_norm{-1.0};  // 诊断：首次 costFunctional 调用时 xi 段梯度范数
    int dim_t;
    int dim_p;
    int iter_num{0};
    double rho;
    double smooth_eps;
    double integral_res;
    bool default_init{true};

    // Environment map pointer.
    std::shared_ptr<rog_map::MapQueryInterface> map;

    VecDf magnitudeBounds;
    VecDf penaltyWeights;
    VecDf local_magnitudes;

    Eigen::Matrix3d headPVA;
    Eigen::Matrix3d tailPVA;
    Mat3Df waypoint_attractor;

    // Optimization cache for warm-start initialization from previous solution.
    VecDf init_ts;
    vec_Vec3f init_ps;

    // Optimization variables.
    VecDf times;
    Mat3Df points;
    VecDf x;

    // Gradient cache.
    Mat3Df gradByPoints;
    VecDf gradByTimes;
    MatD3f partialGradByCoeffs;
    VecDf partialGradByTimes;

    // Result cache.
    VecDf penalty_log;
    uint64_t query_failure_count{0};

    // MINCO solver instance.
    std::shared_ptr<traj_opt::MINCO_S3NU> minco_solver_;

    // Seed 方向软罚上下文（配置 + 切线 + 首次评估梯度探针）。
    DirectionTerm direction;
  };

  // === Utility & Helper Functions ===
  // --- Problem Setup ---

  bool setupProblemAndCheck(const std::vector<Eigen::Vector3d> & waypoints,
    const Eigen::Matrix3d & start_state,
    const Eigen::Matrix3d & end_state);

  void DefaultInit();

  // 在最终解上复核方向软罚指标（与 constraintsFunctional 内共用逐点公式）。
  static void accumulateDirectionMetrics(const DirectionTerm & direction,
    const MatD3f & coeffs,
    const VecDf & times,
    const int & integral_res,
    DirectionMetrics & metrics);

  static double costFunctional(void * ptr, const VecDf & x, VecDf & g);

  // --- Constraint and Barrier Functions ---
  static void constraintsFunctional(const VecDf & T,
    const MatD3f & coeffs,
    const Mat3Df & waypoint_attractor,
    const std::shared_ptr<rog_map::MapQueryInterface> & map,
    const double & smooth_eps,
    const int & integral_res,
    const VecDf & magnitudeBounds,
    const VecDf & local_magnitudes,
    const VecDf & penaltyWeights,
    DirectionTerm & direction,
    double & cost,
    VecDf & partialGradByTimes,
    MatD3f & partialGradByCoeffs,
    VecDf & penalty_log,
    uint64_t & query_failure_count);

  // --- Time Reparameterization ---
  static void forwardT(const VecDf & tau, VecDf & T) { T = tau.array().exp(); }
  static void backwardT(const VecDf & T, VecDf & tau) { tau = T.array().log(); }

  // === State Variables & Caches ===
  OptVars opt_vars_;
  DirectionMetrics last_direction_metrics_;
  int last_iteration_count_{0};
  int last_return_code_{0};
  double last_objective_total_{std::numeric_limits<double>::quiet_NaN()};
  uint64_t last_query_failure_count_{0};
};

}  // namespace minco_planner

#endif
