#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/component_manager.hpp>
#include <pthread.h>
#include <signal.h>
#include <exception>
#include <thread>

int main(int argc, char ** argv)
{
  // Block before creating ROS/DDS threads. The main thread owns signal handling,
  // so lifecycle preshutdown cannot free entities while executor wait sets run.
  sigset_t signals;
  sigemptyset(&signals); sigaddset(&signals, SIGINT); sigaddset(&signals, SIGTERM);
  if (pthread_sigmask(SIG_BLOCK, &signals, nullptr) != 0) { return 1; }
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions(), 4);
  auto manager = std::make_shared<rclcpp_components::ComponentManager>(executor, "online_navigation_container");
  executor->add_node(manager);
  const pthread_t main_thread = pthread_self();
  std::exception_ptr worker_error;
  std::thread worker([&] {
    try { executor->spin(); }
    catch (...) { worker_error = std::current_exception(); pthread_kill(main_thread, SIGTERM); }
  });
  int signal = 0;
  sigwait(&signals, &signal);
  RCLCPP_INFO(manager->get_logger(), "Stopping executor before lifecycle cleanup (signal=%d)", signal);
  executor->cancel();
  worker.join();
  // All container executor threads have joined before ROS invokes Nav2 cleanup.
  rclcpp::shutdown();
  // Nav2 cleanup may already detach callback groups. Expire the manager's weak
  // executor first so its destructor does not try to detach those groups again.
  executor.reset();
  manager.reset();
  return worker_error ? 1 : 0;
}
