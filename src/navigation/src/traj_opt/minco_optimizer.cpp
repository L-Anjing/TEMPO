#include "traj_opt/minco_optimizer.hpp"

#include <cstdlib>

#define POS_IDX 0
#define VEL_IDX 1
#define ACC_IDX 2
#define ATT_IDX 3
namespace minco_planner {
using Mat63f = Eigen::Matrix<double, 6, 3>;

namespace {

// ── seed 方向软罚的逐采样点公式：逐字复刻 OLD fastlio_nav2 ──
// OLD: src/navigation/src/terrain_core/path_planner/trajectory/minco_optimizer.cpp
//   * 切线混合         : 301-303
//   * 方向与雅可比     : 307-317
//   * 罚项             : 773-782（directed_cosine_min / directed_regularity）
// 数学式（在 OLD 中 sample.direction 是「几何切向」，不含任何伪物理速度）：
//   dir     = T·v / sqrt(‖T·v‖² + ε²)                     ε = tangent_regularization
//   cos     = dir · t̂_seed
//   under   = max(0, cos_min − cos)
//   penalty = weight · 0.5 · under²
//   ∂dir/∂v = T · ( I/‖·‖ − (T·v)(T·v)ᵀ/‖·‖³ )
//   ∂penalty/∂v = −weight · under · (∂dir/∂v)ᵀ · t̂_seed
// 积分测度是**弧长** dt·‖v‖，所以速度→0 时整项（代价与梯度）都恰好为 0。
struct DirectionSample
{
  double cos{1.0};
  double under{0.0};
  double penalty{0.0};
  Eigen::Matrix2d direction_jacobian{Eigen::Matrix2d::Zero()};
};

DirectionSample evaluateDirectionSample(const Eigen::Vector2d & velocity,
  const double duration,
  const Eigen::Vector2d & seed_tangent,
  const double weight,
  const double cos_min,
  const double tangent_regularization)
{
  DirectionSample out;
  const Eigen::Vector2d scaled = duration * velocity;
  const double norm_sq = scaled.squaredNorm()
    + tangent_regularization * tangent_regularization;
  const double norm = std::sqrt(norm_sq);
  const Eigen::Vector2d direction = scaled / norm;
  out.direction_jacobian = duration * (Eigen::Matrix2d::Identity() / norm
    - scaled * scaled.transpose() / (norm_sq * norm));
  out.cos = direction.dot(seed_tangent);
  out.under = std::max(cos_min - out.cos, 0.0);
  out.penalty = weight * 0.5 * out.under * out.under;
  return out;
}

// 段边界切线按段内分数线性混合后归一化（OLD 的 seed_blend）。
// 任一端是退化（零向量）或混合后对消到零 → 返回 false，该采样点不产生任何罚。
bool blendedSeedTangent(const Eigen::Vector2d & begin,
  const Eigen::Vector2d & end,
  const double fraction,
  Eigen::Vector2d & out)
{
  if (begin.squaredNorm() <= 0.0 || end.squaredNorm() <= 0.0) {
    return false;
  }
  const Eigen::Vector2d blend = begin * (1.0 - fraction) + end * fraction;
  if (blend.norm() <= 1e-9) {
    return false;
  }
  out = blend.normalized();
  return true;
}

}  // namespace

double MincoOptimizer::optimize(const std::vector<Eigen::Vector3d> & waypoints,
  const Eigen::Matrix3d & start_state,
  const Eigen::Matrix3d & end_state,
  const VecDf & local_magnitudes,
  geometry_utils::Trajectory & out_traj)
{
  // 1. Setup the optimization problem
  last_iteration_count_ = 0;
  last_return_code_ = 0;
  last_objective_total_ = std::numeric_limits<double>::quiet_NaN();
  last_query_failure_count_ = 0;
  opt_vars_.query_failure_count = 0;

  // Seed 方向软罚：刷新配置。切线数组由 MincoPlanner 通过 setSeedTangents() 喂入，
  // 优化器绝不重算切线；尺寸不匹配就在这里整项关掉。
  opt_vars_.direction.enable = cfg_.direction_regularization.enable;
  opt_vars_.direction.weight = cfg_.direction_regularization.weight;
  opt_vars_.direction.cos_min = cfg_.direction_regularization.cos_min;
  opt_vars_.direction.tangent_regularization =
    cfg_.direction_regularization.tangent_regularization;
  opt_vars_.direction.probe_pending = false;
  opt_vars_.direction.first_grad_norm = -1.0;
  last_direction_metrics_ = DirectionMetrics{};
  last_direction_metrics_.enabled = opt_vars_.direction.enable;

  if (!setupProblemAndCheck(waypoints, start_state, end_state)) {
    cout << YELLOW << " -- [TrajOpt] Error in setup problem, force return." << RESET << endl;
    last_return_code_ = lbfgs::LBFGSERR_INVALIDPARAMETERS;
    last_objective_total_ = INFINITY;
    last_direction_metrics_.enabled = false;
    return INFINITY;
  }

  {
    const bool size_ok = opt_vars_.direction.active(opt_vars_.piece_num);
    bool finite_ok = size_ok;
    if (size_ok) {
      for (const auto & tangent : opt_vars_.direction.seed_tangents) {
        if (!tangent.allFinite()) { finite_ok = false; break; }
      }
    }
    opt_vars_.direction.enable = size_ok && finite_ok;
    last_direction_metrics_.enabled = opt_vars_.direction.enable;
  }

  // 2. Pack variables x = [tau, xi]
  // tau: unconstrained time variables
  // xi: stacked intermediate control points
  // [Theory] We optimize an unconstrained vector and map tau -> positive segment times.
  VecDf x(opt_vars_.dim_t + opt_vars_.dim_p);
  Eigen::Map<VecDf> tau(x.data(), opt_vars_.dim_t);
  Eigen::Map<VecDf> xi(x.data() + opt_vars_.dim_t, opt_vars_.dim_p);
  opt_vars_.local_magnitudes = local_magnitudes;
  opt_vars_.penalty_log.resize(5);  // Energy, pos, vel, acc, attract
  opt_vars_.penalty_log.setZero();

  if (opt_vars_.times.minCoeff() < 1e-3) {
    cout << YELLOW << " -- [TrajOpt] Error, the init times have zero, force return." << RESET << endl;
    cout << " -- Head PVA: " << endl;
    cout << opt_vars_.headPVA << endl;
    cout << " -- Tail PVA: " << endl;
    cout << opt_vars_.tailPVA << endl;
    cout << " -- Times: " << endl;
    cout << opt_vars_.times.transpose() << endl;
    last_return_code_ = lbfgs::LBFGSERR_INVALIDPARAMETERS;
    last_objective_total_ = INFINITY;
    return INFINITY;
  }

  gcopter::backwardMapTToTau(opt_vars_.times, tau);
  xi = Eigen::Map<const VecDf>(opt_vars_.points.data(), opt_vars_.points.size());
  const VecDf xi_initial = xi;  // 判定用：优化前后 xi 的实际差
  opt_vars_.first_grad_xi_norm = -1.0;

  // 诊断：解析梯度 vs 数值梯度（仅当设置 MINCO_DEBUG_GRADIENT 时）。
  if (std::getenv("MINCO_DEBUG_GRADIENT")) {
    VecDf g0(x.size());
    const double f0 = costFunctional(&this->opt_vars_, x, g0);
    const double eps = 1e-6;
    std::vector<int> probe;
    for (int k = 0; k < std::min(opt_vars_.dim_t, 2); ++k) { probe.push_back(k); }
    for (int k = 0; k < std::min(opt_vars_.dim_p, 4); ++k) { probe.push_back(opt_vars_.dim_t + k); }
    cout << " -- [MincoOpt][grad-check] f0=" << f0 << " |g_tau|=" << g0.head(opt_vars_.dim_t).norm()
         << " |g_xi|=" << g0.tail(opt_vars_.dim_p).norm() << endl;
    for (const int k : probe) {
      VecDf xp = x, xm = x, gp(x.size()), gm(x.size());
      xp[k] += eps;
      xm[k] -= eps;
      const double fp = costFunctional(&this->opt_vars_, xp, gp);
      const double fm = costFunctional(&this->opt_vars_, xm, gm);
      const double numeric = (fp - fm) / (2.0 * eps);
      cout << " -- [MincoOpt][grad-check] idx=" << k << " analytic=" << g0[k]
           << " numeric=" << numeric
           << " ratio=" << (std::abs(numeric) > 1e-30 ? g0[k] / numeric : 0.0) << endl;
    }
  }

  opt_vars_.iter_num = 0;
  double minCostFunctional = 0.0;

  // 3. Configure L-BFGS
  lbfgs::lbfgs_parameter_t lbfgs_params;
  lbfgs_params.mem_size = 256;
  lbfgs_params.past = 3;
  lbfgs_params.min_step = 1.0e-32;
  lbfgs_params.g_epsilon = 0.0;
  lbfgs_params.delta = cfg_.opt_accuracy;  // Gradient tolerance
  lbfgs_params.max_iterations = 256;

  // 4. Run the optimizer
  // [Theory] L-BFGS is a limited-memory quasi-Newton method.
  int ret = lbfgs::lbfgs_optimize(x,
    minCostFunctional,
    &MincoOptimizer::costFunctional,
    nullptr,
    nullptr,
    &this->opt_vars_,
    lbfgs_params);
  {
    const double xi_move = (xi - xi_initial).norm();
    cout << " -- [MincoOpt][xi-probe] dim_t=" << opt_vars_.dim_t << " dim_p=" << opt_vars_.dim_p
         << " xi_move_norm=" << xi_move
         << " first_grad_xi_norm=" << opt_vars_.first_grad_xi_norm
         << " soft_clearance_used=" << (opt_vars_.magnitudeBounds.size() > 0 ?
              opt_vars_.magnitudeBounds[0] : -1.0)
         << " penalty_pos_w=" << (opt_vars_.penaltyWeights.size() > 0 ?
              opt_vars_.penaltyWeights[0] : -1.0)
         << " query_fail=" << opt_vars_.query_failure_count << endl;
  }
  last_iteration_count_ = opt_vars_.iter_num;
  last_return_code_ = ret;
  last_objective_total_ = minCostFunctional;
  last_query_failure_count_ = opt_vars_.query_failure_count;

  if (cfg_.print_optimizer_log) {
    cout << " -- [MincoOpt] Opt finish, with iter num: " << opt_vars_.iter_num << "\n";
    cout << "\tEnergy: " << opt_vars_.penalty_log(0) << endl;
    cout << "\tPos: " << opt_vars_.penalty_log(1) << endl;
    cout << "\tVel: " << opt_vars_.penalty_log(2) << endl;
    cout << "\tAcc: " << opt_vars_.penalty_log(3) << endl;
    cout << "\tAttract: " << opt_vars_.penalty_log(4) << endl;
    cout << "\tOptimized Time: " << opt_vars_.times.norm() << endl;
  }

  if (ret >= 0 || ret == lbfgs::LBFGSERR_MAXIMUMITERATION) {
    // 5. Decode the solution and build the output trajectory
    gcopter::forwardMapTauToT(tau, opt_vars_.times);
    opt_vars_.points = Eigen::Map<const Mat3Df>(xi.data(), 3, opt_vars_.piece_num - 1);

    // opt_vars_.minco_solver_->setConditions(opt_vars_.headPVA, opt_vars_.tailPVA, opt_vars_.piece_num);
    opt_vars_.minco_solver_->setParameters(opt_vars_.points, opt_vars_.times);
    opt_vars_.minco_solver_->getTrajectory(out_traj);

    // 方向软罚指标：在**最终解**上按同一逐点公式复核，不读被拒绝的试探点。
    if (opt_vars_.direction.enable) {
      accumulateDirectionMetrics(opt_vars_.direction,
        opt_vars_.minco_solver_->getCoeffs(),
        opt_vars_.times,
        cfg_.integral_res,
        last_direction_metrics_);
    }

    opt_vars_.init_ts = opt_vars_.times;
    opt_vars_.init_ps.clear();
    opt_vars_.init_ps.reserve(static_cast<size_t>(opt_vars_.points.cols()));
    for (int i = 0; i < opt_vars_.points.cols(); ++i) {
      opt_vars_.init_ps.emplace_back(opt_vars_.points.col(i));
    }
    opt_vars_.default_init = false;
  } else {
    // 5'. Optimization failed
    minCostFunctional = INFINITY;
  }
  // 方向项首次评估梯度探针：无论成败都如实汇报，用于证明该项真的进入优化器。
  last_direction_metrics_.first_grad_norm = opt_vars_.direction.first_grad_norm;
  // Keep the objective and the L-BFGS status code as separate result channels.
  return minCostFunctional;
}

void MincoOptimizer::accumulateDirectionMetrics(const DirectionTerm & direction,
  const MatD3f & coeffs,
  const VecDf & times,
  const int & integral_res,
  DirectionMetrics & metrics)
{
  metrics.mean_cos = std::numeric_limits<double>::quiet_NaN();
  metrics.min_cos = std::numeric_limits<double>::quiet_NaN();
  metrics.penalty_cost = 0.0;
  const int n = static_cast<int>(times.size());
  if (n <= 0 || integral_res <= 0 || !direction.active(n)) {
    return;
  }

  const double integral_frac = 1.0 / static_cast<double>(integral_res);
  double cos_sum = 0.0;
  double min_cos = std::numeric_limits<double>::infinity();
  double penalty_cost = 0.0;
  long long sample_count = 0;
  for (int i = 0; i < n; ++i) {
    const Mat63f & c = coeffs.block<6, 3>(i * 6, 0);
    const double step = times(i) * integral_frac;
    for (int j = 0; j <= integral_res; ++j) {
      const double s1 = j * step;
      const double s2 = s1 * s1;
      const double s3 = s2 * s1;
      const double s4 = s2 * s2;
      Vec6f beta1;
      beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
      const Eigen::Vector2d velocity = (c.transpose() * beta1).head<2>().cast<double>();

      Eigen::Vector2d seed_tangent;
      if (!blendedSeedTangent(direction.seed_tangents[static_cast<size_t>(i)],
            direction.seed_tangents[static_cast<size_t>(i) + 1U],
            j * integral_frac, seed_tangent)) {
        continue;
      }
      const DirectionSample sample = evaluateDirectionSample(velocity,
        times(i), seed_tangent, direction.weight, direction.cos_min,
        direction.tangent_regularization);
      cos_sum += sample.cos;
      min_cos = std::min(min_cos, sample.cos);
      ++sample_count;
      const double node = (j == 0 || j == integral_res) ? 0.5 : 1.0;
      penalty_cost += node * step * velocity.norm() * sample.penalty;
    }
  }
  if (sample_count == 0) {
    return;
  }
  metrics.mean_cos = cos_sum / static_cast<double>(sample_count);
  metrics.min_cos = min_cos;
  metrics.penalty_cost = penalty_cost;
}

double MincoOptimizer::costFunctional(void * ptr, const VecDf & x, VecDf & g)
{
  OptVars & opt_vars_ = *(static_cast<OptVars *>(ptr));
  const auto & dim_t = opt_vars_.dim_t;
  const auto & dim_p = opt_vars_.dim_p;
  const auto & waypoint_attractor = opt_vars_.waypoint_attractor;
  const auto & integral_res = opt_vars_.integral_res;
  const auto & smooth_eps = opt_vars_.smooth_eps;
  const auto & rho = opt_vars_.rho;
  const auto & penaltyWeights = opt_vars_.penaltyWeights;
  const auto & magnitudeBounds = opt_vars_.magnitudeBounds;
  const auto & local_magnitudes = opt_vars_.local_magnitudes;
  const auto & minco_solver_ = opt_vars_.minco_solver_;
  const auto & map = opt_vars_.map;

  opt_vars_.iter_num++;

  // 1. Unpack variables and gradient buffers
  const Eigen::Map<const VecDf> tau(x.data(), dim_t);
  const Eigen::Map<const VecDf> xi(x.data() + dim_t, dim_p);
  Eigen::Map<VecDf> grad_tau(g.data(), dim_t);
  Eigen::Map<VecDf> grad_points(g.data() + dim_t, dim_p);

  // 2. Map tau -> segment times
  VecDf times;
  gcopter::forwardMapTauToT(tau, times);

  Mat3Df points;
  if (dim_p > 0) {
    points = Eigen::Map<const Eigen::Matrix<double, 3, Eigen::Dynamic>>(xi.data(), 3, dim_p / 3);
  } else {
    points.resize(3, 0);
  }

  // 3. Update MINCO polynomial parameters
  minco_solver_->setParameters(points, times);

  // 4. Energy term (MINCO internal objective)
  double cost = 0.0;
  MatD3f partialGradByCoeffs(6 * times.size(), 3);
  VecDf partialGradByTimes(times.size());
  partialGradByCoeffs.setZero();
  partialGradByTimes.setZero();
  minco_solver_->getEnergy(cost);
  minco_solver_->getEnergyPartialGradByCoeffs(partialGradByCoeffs);
  minco_solver_->getEnergyPartialGradByTimes(partialGradByTimes);
  opt_vars_.penalty_log(0) = cost;

  // 5. Penalty terms (ESDF / vel / acc / attract / seed direction)
  const uint64_t query_failure_count_before = opt_vars_.query_failure_count;
  // 方向项梯度探针只在**首次** cost 评估时请求（与 first_grad_xi_norm 同一判定）。
  opt_vars_.direction.probe_pending =
    opt_vars_.direction.enable && (opt_vars_.first_grad_xi_norm < 0.0);
  constraintsFunctional(times,
    minco_solver_->getCoeffs(),
    waypoint_attractor,
    map,
    smooth_eps,
    integral_res,
    magnitudeBounds,
    local_magnitudes,
    penaltyWeights,
    opt_vars_.direction,
    cost,
    partialGradByTimes,
    partialGradByCoeffs,
    opt_vars_.penalty_log,
    opt_vars_.query_failure_count);
  if (opt_vars_.query_failure_count != query_failure_count_before) {
    g.setZero();
    return INFINITY;
  }

  // 6. Propagate gradients to points and times
  Mat3Df gradByPoints;
  VecDf gradByTimes;
  minco_solver_->propogateGrad(partialGradByCoeffs, partialGradByTimes, gradByPoints, gradByTimes);

  // 6b. 方向项单独做一次伴随回传，量出它对 xi 段梯度的真实贡献（只做一次）。
  if (opt_vars_.direction.probe_pending) {
    Mat3Df probe_grad_by_points;
    VecDf probe_grad_by_times;
    minco_solver_->propogateGrad(opt_vars_.direction.probe_grad_coeffs,
      opt_vars_.direction.probe_grad_times, probe_grad_by_points, probe_grad_by_times);
    opt_vars_.direction.first_grad_norm = probe_grad_by_points.norm();
    opt_vars_.direction.probe_pending = false;
  }

  // 7. Time regularization
  cost += rho * times.sum();
  gradByTimes.array() += rho;

  // 8. Backprop time gradient (T -> tau)
  gcopter::propagateGradientTToTau(tau, gradByTimes, grad_tau);

  if (dim_p > 0) {
    grad_points = Eigen::Map<VecDf>(gradByPoints.data(), gradByPoints.size());
    if (opt_vars_.first_grad_xi_norm < 0.0) {
      opt_vars_.first_grad_xi_norm = grad_points.norm();
    }
  }

  return cost;
}

void MincoOptimizer::constraintsFunctional(const VecDf & T,
  const MatD3f & coeffs,
  const Mat3Df & waypoint_attractor,
  const std::shared_ptr<rog_map::MapQueryInterface> & map,
  const double & smooth_eps,
  const int & integral_res,
  const VecDf & magnitudeBounds,
  const VecDf & local_magnitudes,
  const VecDf & penaltyWeights,
  DirectionTerm & direction,
  // outputs
  double & cost,
  VecDf & partialGradByTimes,
  MatD3f & partialGradByCoeffs,
  VecDf & penalty_log,
  uint64_t & query_failure_count)
{
  // 1. Unpack bounds and weights
  const auto & safe_dist = magnitudeBounds[0];
  // const auto & vmax = magnitudeBounds[1];
  const auto & amax = magnitudeBounds[2];

  // const auto & vmaxSqr = vmax * vmax;
  const auto & amaxSqr = amax * amax;
  const double jmax = magnitudeBounds.size() > 3 ? magnitudeBounds[3] : std::numeric_limits<double>::infinity();
  const double weightJer = penaltyWeights.size() > 4 ? penaltyWeights[4] : 0.0;

  const auto & weightPos = penaltyWeights[0];
  const auto & weightVel = penaltyWeights[1];
  const auto & weightAcc = penaltyWeights[2];
  const auto & weightAtt = penaltyWeights[3];
  const auto terrain_policy = map ? map->terrainPolicy() : rog_map::TerrainConfig{};

  const auto & piece_num = T.size();

  // 2. Setup piece-wise sampling
  const double integralFrac = 1.0 / integral_res;
  VecDf max_pena(4);
  max_pena.setZero();

  // Seed 方向软罚是否生效：开关 + 切线数量必须与段数严格一致，否则整项不参与。
  const bool direction_on = direction.enable &&
    direction.seed_tangents.size() == static_cast<size_t>(piece_num) + 1U;
  if (direction_on && direction.probe_pending) {
    // 首次评估：只为方向项准备一份独立的 rhs，供 costFunctional 单独伴随回传。
    direction.probe_grad_coeffs.setZero(6 * piece_num, 3);
    direction.probe_grad_times.setZero(piece_num);
  }

  // 3. Loop over pieces and samples
  for (int i = 0; i < piece_num; i++) {
    const Mat63f & c = coeffs.block<6, 3>(i * 6, 0);
    const auto & step = T(i) * integralFrac;
    const auto & local_vmax = local_magnitudes(i);
    const auto & local_vmaxSqr = local_vmax * local_vmax;
    for (int j = 0; j <= integral_res; j++) {
      double s1 = j * step;
      double s2 = s1 * s1;
      double s3 = s2 * s1;
      double s4 = s2 * s2;
      double s5 = s4 * s1;
      Vec6f beta0, beta1, beta2, beta3, beta4;
      beta0 << 1.0, s1, s2, s3, s4, s5;
      beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
      beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;
      beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2;
      beta4 << 0.0, 0.0, 0.0, 0.0, 24.0, 120.0 * s1;

      const Vec3f pos = c.transpose() * beta0;
      const Vec3f vel = c.transpose() * beta1;
      const Vec3f acc = c.transpose() * beta2;
      const Vec3f jer = c.transpose() * beta3;
      const Vec3f snap = c.transpose() * beta4;

      double tmp_cost{0.0};
      Vec3f gradPos{0.0, 0.0, 0.0}, gradVel{0.0, 0.0, 0.0}, gradAcc{0.0, 0.0, 0.0};
      Vec3f gradJer = Vec3f::Zero();
      const auto node = (j == 0 || j == integral_res) ? 0.5 : 1.0;
      const double alpha = j * integralFrac;

      // For position cost
      if (weightPos > 0.0 && map) {
        double esdf_dist = -safe_dist;
        Eigen::Vector3d esdf_grad = Eigen::Vector3d::Zero();
        const auto query = map->query(pos.cast<double>());
        if (query.ok) {
          esdf_dist = query.distance;
          esdf_grad = query.gradient;
        } else {
          ++query_failure_count;
        }

        const double violaPos = map->softClearance(pos.cast<double>(), safe_dist) - esdf_dist;
        double violaPosPena, violaPosPenaD;
        if (gcopter::smoothedL1(violaPos, smooth_eps, violaPosPena, violaPosPenaD)) {
          // d(violaPos)/dpos = -grad(dist)
          gradPos += (-weightPos * violaPosPenaD) * esdf_grad.cast<double>();
          tmp_cost += weightPos * violaPosPena;
          if (violaPos > max_pena(POS_IDX))
            max_pena(POS_IDX) = violaPos;
        }
      }

      if (map && terrain_policy.enable) {
        unsigned int mx = 0, my = 0;
        rog_map::TerrainCell terrain;
        if (map->worldToMap(pos.x(), pos.y(), mx, my) && map->terrain(mx, my, terrain) &&
            rog_map::directionalTerrain(terrain) && terrain.uphill.norm() > 0.1) {
          const Eigen::Vector2d along = terrain.uphill.normalized();
          const Eigen::Vector2d across(-along.y(), along.x());
          const Eigen::Vector2d planar_velocity = vel.head<2>();
          const double lateral = planar_velocity.dot(across);
          const double sin_sq = 1.0 - terrain_policy.alignment_cosine * terrain_policy.alignment_cosine;
          const double violation = lateral * lateral - sin_sq * planar_velocity.squaredNorm();
          double penalty = 0.0, derivative = 0.0;
          if (gcopter::smoothedL1(violation, smooth_eps, penalty, derivative)) {
            gradVel.head<2>() += 2.0 * terrain_policy.alignment_weight * derivative *
              (lateral * across - sin_sq * planar_velocity);
            tmp_cost += terrain_policy.alignment_weight * penalty;
          }
        }
      }

      // For velocity cost
      const auto & violaVel = vel.head<2>().squaredNorm() - local_vmaxSqr;
      double violaVelPena, violaVelPenaD;
      if (weightVel > 0 && gcopter::smoothedL1(violaVel, smooth_eps, violaVelPena, violaVelPenaD)) {
        gradVel.head<2>() += weightVel * violaVelPenaD * 2.0 * vel.head<2>();
        tmp_cost += weightVel * violaVelPena;
        if (violaVel > max_pena(VEL_IDX))
          max_pena(VEL_IDX) = violaVel;
      }

      // For acceleration cost
      const auto & violaAcc = acc.head<2>().squaredNorm() - amaxSqr;
      double violaAccPena, violaAccPenaD;
      if (weightAcc > 0 && gcopter::smoothedL1(violaAcc, smooth_eps, violaAccPena, violaAccPenaD)) {
        gradAcc.head<2>() += weightAcc * violaAccPenaD * 2.0 * acc.head<2>();
        tmp_cost += weightAcc * violaAccPena;
        if (violaAcc > max_pena(ACC_IDX))
          max_pena(ACC_IDX) = violaAcc;
      }

      const double violaJer = jer.head<2>().squaredNorm() - jmax * jmax;
      double jerkPenalty = 0.0, jerkDerivative = 0.0;
      if (weightJer > 0 && gcopter::smoothedL1(violaJer, smooth_eps, jerkPenalty, jerkDerivative)) {
        gradJer.head<2>() += weightJer * jerkDerivative * 2.0 * jer.head<2>();
        tmp_cost += weightJer * jerkPenalty;
      }

      // Seed 方向软正则（OLD: minco_optimizer.cpp:773-782）：只把几何切向拉向
      // seed 切线，零空间很大（cos_min=0.1 → 允许 ~84° 偏差），净空/避障可自由压过它。
      // 按 OLD 的弧长测度 dt·‖v‖ 计权，因此 ‖v‖=0 的采样点代价与梯度都恰好为 0
      // （同时天然避开 T·v→0 时 ∂dir/∂v 的正则奇异性）。
      if (direction_on) {
        Eigen::Vector2d seed_tangent;
        if (blendedSeedTangent(direction.seed_tangents[static_cast<size_t>(i)],
              direction.seed_tangents[static_cast<size_t>(i) + 1U], alpha, seed_tangent)) {
          const Eigen::Vector2d planar_velocity = vel.head<2>().cast<double>();
          const double speed = planar_velocity.norm();
          if (speed > 0.0) {
            const DirectionSample dir_sample = evaluateDirectionSample(planar_velocity,
              T(i), seed_tangent, direction.weight, direction.cos_min,
              direction.tangent_regularization);
            const double direction_cost = speed * dir_sample.penalty;
            // OLD: gradient.velocity -= w·under·(∂dir/∂v)ᵀ·t̂_seed
            const Eigen::Vector2d direction_velocity_gradient =
              -direction.weight * dir_sample.under
              * dir_sample.direction_jacobian.transpose() * seed_tangent;
            const Eigen::Vector2d speed_gradient = planar_velocity / speed;
            // 弧长测度 ∂(speed)/∂v = v/‖v‖；再乘 integration_measure = dt·speed。
            const Eigen::Vector2d effective_velocity_gradient =
              speed * direction_velocity_gradient + dir_sample.penalty * speed_gradient;
            tmp_cost += direction_cost;
            gradVel.head<2>() += effective_velocity_gradient.cast<double>();
            if (direction.probe_pending) {
              // 只属于方向项的 rhs（与主累加同一 node·step 测度），供单独伴随回传。
              // z 分量恒为 0：方向罚是平面量（与 OLD 一致）。
              const Eigen::Vector3d effective_velocity_gradient_3d(
                effective_velocity_gradient.x(), effective_velocity_gradient.y(), 0.0);
              direction.probe_grad_coeffs.block<6, 3>(i * 6, 0) +=
                beta1 * effective_velocity_gradient_3d.transpose() * node * step;
              direction.probe_grad_times(i) +=
                effective_velocity_gradient.dot(acc.head<2>().cast<double>())
                  * alpha * node * step
                + node * integralFrac * direction_cost;
            }
          }
        }
      }

      // For attract point cost
      if (weightAtt > 0.0) {
        const auto is_end = ((j == integral_res) && (i != piece_num - 1));
        const auto idx = is_end ? i + 1 : i;
        if (is_end) {
          Vec3f p_a = pos - waypoint_attractor.col(idx);
          const auto & violaAtt = p_a.squaredNorm() - 0.1 * 0.1;  // Dead zone: 0.1m
          double violaAttPena, violaAttPenaD;
          if (violaAtt > max_pena(ATT_IDX))
            max_pena(ATT_IDX) = violaAtt;
          if (gcopter::smoothedL1(violaAtt, smooth_eps, violaAttPena, violaAttPenaD)) {
            Vec3f gradAtt = weightAtt * violaAttPenaD * 2.0 * p_a;
            partialGradByCoeffs.block<6, 3>(i * 6, 0) += beta0 * gradAtt.transpose();
            cost += weightAtt * violaAttPena;
            if (is_end) {
              partialGradByTimes(i) += gradAtt.dot(vel);
            }
          }
        }
      }

      partialGradByCoeffs.block<6, 3>(i * 6, 0) +=
        (beta0 * gradPos.transpose() + beta1 * gradVel.transpose() + beta2 * gradAcc.transpose() + beta3 * gradJer.transpose()) * node *
        step;
      partialGradByTimes(i) +=
        (gradPos.dot(vel) + gradVel.dot(acc) + gradAcc.dot(jer) + gradJer.dot(snap)) * alpha * node * step +
        node * integralFrac * tmp_cost;
      cost += node * step * tmp_cost;
    }
  }

  // 4. Save penalty logs
  penalty_log(1) = max_pena(POS_IDX);
  penalty_log(2) = max_pena(VEL_IDX);
  penalty_log(3) = max_pena(ACC_IDX);
  penalty_log(4) = max_pena(ATT_IDX);
}

bool MincoOptimizer::setupProblemAndCheck(const std::vector<Eigen::Vector3d> & waypoints,
  const Eigen::Matrix3d & start_state,
  const Eigen::Matrix3d & end_state)
{
  // 1. Decide the number of polynomial pieces
  int N = waypoints.size() - 1;
  if (N <= 0) {
    return false;
  }

  // 2. Allocate optimization context
  opt_vars_.piece_num = N;
  opt_vars_.headPVA = start_state;
  opt_vars_.tailPVA = end_state;
  opt_vars_.times.resize(N);
  opt_vars_.points.resize(3, N - 1);
  opt_vars_.waypoint_attractor.resize(3, N + 1);
  for (int i = 0; i < N - 1; ++i) {
    opt_vars_.waypoint_attractor.col(i + 1) = waypoints[i + 1];
  }
  opt_vars_.waypoint_attractor.col(0) = opt_vars_.headPVA.col(0);
  opt_vars_.waypoint_attractor.rightCols(1) = opt_vars_.tailPVA.col(0);

  // 3. Set variable dimensions
  opt_vars_.dim_t = N;
  opt_vars_.dim_p = 3 * (N - 1);

  // 4. Allocate gradient buffers
  opt_vars_.gradByPoints.resize(3, N - 1);
  opt_vars_.gradByTimes.resize(N);
  opt_vars_.partialGradByCoeffs.resize(6 * N, 3);
  opt_vars_.partialGradByTimes.resize(N);

  // 5. Initialize time and point guesses
  DefaultInit();

  // Apply warm-start (shifted hot start) if provided and sizes match.
  if (!opt_vars_.default_init) {
    const bool ts_ok = (opt_vars_.init_ts.size() == opt_vars_.times.size());
    const bool ps_ok = (opt_vars_.init_ps.size() == static_cast<size_t>(opt_vars_.points.cols()));
    if (ts_ok && ps_ok) {
      opt_vars_.times = opt_vars_.init_ts;
      for (int i = 0; i < opt_vars_.points.cols(); ++i) {
        opt_vars_.points.col(i) = opt_vars_.init_ps[static_cast<size_t>(i)];
      }
    } else {
      // Size mismatch: fallback to cold init.
      opt_vars_.default_init = true;
    }
  }

  // 6. Set boundary conditions for MINCO
  opt_vars_.minco_solver_->setConditions(opt_vars_.headPVA, opt_vars_.tailPVA, N);
  return true;
}

void MincoOptimizer::DefaultInit()
{
  // 1. Extract segment distance computation logic.
  const auto computeDis = [this]() -> VecDf {
    return (opt_vars_.waypoint_attractor.leftCols(opt_vars_.piece_num) -
            opt_vars_.waypoint_attractor.rightCols(opt_vars_.piece_num))
      .colwise()
      .norm()
      .transpose();
  };
  const VecDf dis = computeDis();

  // 2. Allocate initial times with kinematic-aware startup / stopping compensation.
  const double max_vel = cfg_.max_vel;
  const double max_acc = cfg_.max_acc;
  // 时间初值必须来自“真实导航巡航速度”（minco_config.max_vel 已由 dynamics.planning_vel 限定），
  // 不能用硬件速度上限：那会让所有中间段落到 0.1 s 下限，初值轨迹加速度极大、
  // 初值代价被加速度罚项主导（实测 7.8e7），优化器从病态初值收敛到贴边轨迹。
  const double cruise_speed = std::max(1e-3, max_vel);
  const double v_tail = opt_vars_.tailPVA.col(1).norm();
  const bool is_stopping = (v_tail < 0.1);

  opt_vars_.times.resize(opt_vars_.piece_num);
  for (int i = 0; i < opt_vars_.piece_num; ++i) {
    const double d = std::max(0.0, dis(i));
    double t = 0.1;
    if (i == 0) {
      // First segment: conservative startup time to avoid speed spike.
      const double t_acc = std::sqrt(std::max(0.0, 2.0 * d / max_acc));
      const double t_cons = d / std::max(1e-3, cruise_speed * 0.5);
      t = std::max(t_acc, t_cons);
    } else if (i == opt_vars_.piece_num - 1 && is_stopping) {
      // Last segment: add braking compensation for stop behavior.
      const double t_cruise = d / cruise_speed;
      const double t_brake = std::sqrt(std::max(0.0, 2.0 * d / max_acc));
      t = std::max(t_cruise, t_brake);
    } else {
      // Middle segments: cruise-speed allocation, never below half the acceleration-limited time.
      const double t_cruise = d / cruise_speed;
      const double t_acc = std::sqrt(std::max(0.0, 2.0 * d / max_acc));
      t = std::max(t_cruise, 0.5 * t_acc);
    }

    opt_vars_.times(i) = t;
  }
  opt_vars_.times = opt_vars_.times.cwiseMax(0.1);

  // 3. Initialize intermediate points from guide waypoints (unchanged).
  opt_vars_.points = opt_vars_.waypoint_attractor.block(0, 1, 3, opt_vars_.piece_num - 1);
}

void MincoOptimizer::setInitPsAndTs(const vec_Vec3f & init_ps, const VecDf & init_ts)
{
  // Store warm-start guesses. They will be applied after problem dimensions
  // are known (inside setupProblemAndCheck()).
  if (init_ps.empty() || init_ts.size() == 0) {
    opt_vars_.default_init = true;
    opt_vars_.init_ps.clear();
    opt_vars_.init_ts.resize(0);
    return;
  }

  opt_vars_.default_init = false;
  opt_vars_.init_ps = init_ps;
  opt_vars_.init_ts = init_ts;
}
}  // namespace minco_planner
