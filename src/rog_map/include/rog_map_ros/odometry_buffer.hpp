#pragma once
#include <rog_map/rog_map_core/common_lib.hpp>
#include <algorithm>
#include <deque>

namespace rog_map {
using super_utils::Pose;
class OdometryBuffer {
  struct Sample { double stamp; Pose pose; };
  std::deque<Sample> samples_;
  double duration_;
public:
  explicit OdometryBuffer(double duration = 2.0) : duration_(duration) {}
  void clear() { samples_.clear(); }
  bool insert(double stamp, const Pose & pose) {
    if (!std::isfinite(stamp) || stamp <= 0 || !pose.first.allFinite() ||
        !pose.second.coeffs().allFinite() || pose.second.norm() < 1e-6) { return false; }
    const auto it = std::lower_bound(samples_.begin(), samples_.end(), stamp,
      [](const Sample & sample, double time) { return sample.stamp < time; });
    Sample sample{stamp, {pose.first, pose.second.normalized()}};
    if (it != samples_.end() && it->stamp == stamp) { *it = sample; }
    else { samples_.insert(it, sample); }
    while (samples_.size() > 2 && samples_.back().stamp - samples_.front().stamp > duration_) {
      samples_.pop_front();
    }
    while (samples_.size() > 4096) { samples_.pop_front(); }
    return true;
  }
  bool interpolate(double stamp, Pose & pose, double max_gap) const {
    if (samples_.empty() || stamp < samples_.front().stamp || stamp > samples_.back().stamp) { return false; }
    const auto upper = std::lower_bound(samples_.begin(), samples_.end(), stamp,
      [](const Sample & sample, double time) { return sample.stamp < time; });
    if (upper->stamp == stamp) { pose = upper->pose; return true; }
    if (upper == samples_.begin()) { return false; }
    const auto & lower = *std::prev(upper);
    const double gap = upper->stamp - lower.stamp;
    if (gap <= 0 || gap > max_gap) { return false; }
    const double alpha = (stamp - lower.stamp) / gap;
    pose.first = (1 - alpha) * lower.pose.first + alpha * upper->pose.first;
    pose.second = lower.pose.second.slerp(alpha, upper->pose.second).normalized();
    return true;
  }
};
}  // namespace rog_map
