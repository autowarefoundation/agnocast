#pragma once

#include <array>
#include <atomic>
#include <csignal>
#include <mutex>
#include <thread>

namespace agnocast
{

class SignalHandler
{
public:
  static void install();
  static void uninstall();
  static void register_shutdown_event(int eventfd);
  static void unregister_shutdown_event(int eventfd);

private:
  enum class State {
    NotInstalled,
    Installed,
    Uninstalling,
  };

  static constexpr size_t MAX_EXECUTORS_NUM = 128;

  static State state_;
  static std::mutex mutex_;
  static std::array<int, MAX_EXECUTORS_NUM> eventfds_;
  static size_t eventfd_count_;
  static std::atomic<bool> stop_requested_;
  static std::atomic<bool> signal_received_;
  static std::atomic<int> signal_eventfd_;
  static std::thread * signal_thread_;
  static std::atomic<int> handler_inflight_count_;
  static struct sigaction old_sigint_action_;
  static struct sigaction old_sigterm_action_;

  static void notify_signal_eventfd();
  static void wait_for_signal_eventfd();
  static void signal_processing_loop();
  static void signal_handler(int signum);
  static void notify_all_executors();
};

}  // namespace agnocast
