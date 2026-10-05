#pragma once

#include <Eigen/Geometry>
#include <deque>
#include <cmath>

namespace point_lio {

// A scan-corrected state and raw IMU samples use the same sensor clock and IMU frame.
// The caller serializes access. Prediction never modifies the mapping filter.
class ImuOdometryPredictor
{
public:
  struct Sample {
    double stamp;
    Eigen::Vector3d acc, gyro;
  };
  struct State {
    double stamp{0.0};
    Eigen::Vector3d pos{Eigen::Vector3d::Zero()}, vel{Eigen::Vector3d::Zero()};
    Eigen::Matrix3d rot{Eigen::Matrix3d::Identity()};
    Eigen::Vector3d gravity{0.0, 0.0, -9.81};
    Eigen::Vector3d ba{Eigen::Vector3d::Zero()}, bg{Eigen::Vector3d::Zero()};
    Eigen::Vector3d omega{Eigen::Vector3d::Zero()};
    double acc_scale{9.81};
  };

  void reset() { samples_.clear(); seeded_ = false; last_output_ = -1.0; }

  void push(const Sample & sample)
  {
    if (!std::isfinite(sample.stamp) || !sample.acc.allFinite() || !sample.gyro.allFinite()) {
      return;
    }
    if (!samples_.empty() && sample.stamp < samples_.back().stamp) { reset(); }
    if (!samples_.empty() && sample.stamp == samples_.back().stamp) { return; }
    samples_.push_back(sample);
    while (samples_.size() > 2 && samples_.back().stamp - samples_[1].stamp > 2.0) {
      samples_.pop_front();
    }
  }

  double latestStamp() const { return samples_.empty() ? -1.0 : samples_.back().stamp; }

  void seed(const State & state)
  {
    if (!std::isfinite(state.stamp) || !state.pos.allFinite() || !state.vel.allFinite() ||
        !state.rot.allFinite() || !state.gravity.allFinite() || !state.ba.allFinite() ||
        !state.bg.allFinite() || !std::isfinite(state.acc_scale) || state.acc_scale <= 0.0) {
      return;
    }
    if (seeded_ && state.stamp < correction_stamp_) { return; }
    state_ = state;
    correction_stamp_ = state.stamp;
    seeded_ = true;
  }

  // Only output a new measured IMU timestamp. Stop on stale corrections or IMU gaps;
  // filling repeated/future timestamps would hide a stalled estimator from the controller.
  bool predict(State & result, double max_correction_age, double max_imu_gap = 0.03)
  {
    if (!seeded_ || samples_.size() < 2) { return false; }
    const double target = samples_.back().stamp;
    if (target <= last_output_ || target < state_.stamp ||
        target - correction_stamp_ > max_correction_age || samples_.front().stamp > state_.stamp) {
      return false;
    }
    State next = state_;
    for (size_t i = 1; i < samples_.size(); ++i) {
      const Sample & left = samples_[i - 1];
      const Sample & right = samples_[i];
      if (right.stamp <= next.stamp) { continue; }
      const double interval = right.stamp - left.stamp;
      if (interval > max_imu_gap || left.stamp > next.stamp + 1.e-6) { return false; }
      const double dt = right.stamp - next.stamp;
      const double alpha = (next.stamp - left.stamp) / interval;
      const Eigen::Vector3d acc_start = left.acc + alpha * (right.acc - left.acc);
      const Eigen::Vector3d gyro_start = left.gyro + alpha * (right.gyro - left.gyro);
      const Eigen::Vector3d omega = 0.5 * (gyro_start + right.gyro) - next.bg;
      const Eigen::Vector3d angle = omega * dt;
      const double magnitude = angle.norm();
      const Eigen::Matrix3d rotation_step = magnitude > 1.e-12 ?
        Eigen::AngleAxisd(magnitude, angle / magnitude).toRotationMatrix() :
        Eigen::Matrix3d::Identity();
      const Eigen::Matrix3d end_rotation = next.rot * rotation_step;
      const Eigen::Vector3d acc_world = 0.5 * (
        next.rot * (acc_start * next.acc_scale - next.ba) +
        end_rotation * (right.acc * next.acc_scale - next.ba)) + next.gravity;
      next.pos += next.vel * dt + 0.5 * acc_world * dt * dt;
      next.vel += acc_world * dt;
      next.rot = Eigen::Quaterniond(end_rotation).normalized().toRotationMatrix();
      next.omega = right.gyro - next.bg;
      next.stamp = right.stamp;
    }
    state_ = next;
    result = next;
    last_output_ = next.stamp;
    return true;
  }

private:
  std::deque<Sample> samples_;
  State state_;
  bool seeded_{false};
  double correction_stamp_{0.0}, last_output_{-1.0};
};

}  // namespace point_lio
