#pragma once

#include <cmath>
#include <algorithm>
#include <cstdint>
#include <limits>

namespace point_lio
{

class SensorTimeRateLimiter
{
public:
  bool should_publish(double sensor_time, double frequency_hz)
  {
    if (!std::isfinite(sensor_time) || !std::isfinite(frequency_hz) || frequency_hz <= 0.0) {
      return false;
    }

    if (!std::isfinite(last_publish_sensor_time_) ||
        sensor_time < last_publish_sensor_time_ || frequency_hz != frequency_hz_) {
      last_publish_sensor_time_ = sensor_time;
      epoch_ = sensor_time;
      next_publish_slot_ = 1;
      frequency_hz_ = frequency_hz;
      return true;
    }

    const double time_epsilon = std::max(1.0e-9,
      4.0 * std::numeric_limits<double>::epsilon() * std::max(std::abs(sensor_time), std::abs(epoch_)));
    const double elapsed = sensor_time - epoch_;
    if (elapsed + time_epsilon < static_cast<double>(next_publish_slot_) / frequency_hz) {
      return false;
    }

    // Keep the schedule anchored to sensor time. Restarting a 5 ms period at each
    // 2 ms state update would reduce a requested 200 Hz stream to about 167 Hz.
    // After a gap publish once, then skip elapsed slots instead of sending a burst.
    next_publish_slot_ = static_cast<uint64_t>(std::floor((elapsed + time_epsilon) * frequency_hz)) + 1;
    last_publish_sensor_time_ = sensor_time;
    return true;
  }

private:
  double last_publish_sensor_time_ = std::numeric_limits<double>::quiet_NaN();
  double epoch_{0.0};
  double frequency_hz_{0.0};
  uint64_t next_publish_slot_{1};
};

}  // namespace point_lio
