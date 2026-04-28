#pragma once

#include <atomic>
#include <csignal>
#include <mutex>
#include <set>
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

  static State state_;
  static std::mutex mutex_;
  static std::set<int> eventfds_;
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
