#pragma once
#include <condition_variable>
#include <mutex>

namespace rog_map {
// A queued callback owns this gate and checks it before touching its raw owner.
class CallbackGate {
  std::mutex mutex_;
  std::condition_variable idle_;
  bool accepting_{true};
  size_t running_{0};
  void finish() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (--running_ == 0) { idle_.notify_all(); }
  }
public:
  template <typename Function> bool run(Function function) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!accepting_) { return false; }
      ++running_;
    }
    try { function(); } catch (...) { finish(); throw; }
    finish(); return true;
  }
  void stop() {
    std::unique_lock<std::mutex> lock(mutex_);
    accepting_ = false;
    idle_.wait(lock, [&] { return running_ == 0; });
  }
};
}  // namespace rog_map
