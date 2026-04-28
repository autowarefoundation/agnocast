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
  // Installs signal handlers.
  // Safe to call multiple times; if already installed, it does nothing.
  //
  // Effects:
  // - Installs handlers for SIGINT and SIGTERM.
  // - Starts a dedicated thread to process signals.
  // - Initializes eventfds_.
  static void install();

  // Uninstalls signal handlers.
  // Safe to call multiple times. If it is already uninstalled or currently
  // in the process of uninstalling, this function does nothing. This ensures
  // that the teardown process is executed exactly once by a single caller.
  //
  // Effects:
  // - Stops the dedicated thread.
  // - Restores the original signal handlers.
  // - Clears eventfds_.
  // - Cleans up internal data structures.
  static void uninstall();

  // Registers an eventfd to be notified on SIGINT or SIGTERM.
  // Must be called while the signal handler is active (i.e., between
  // calls to install() and uninstall()).
  //
  // Returns true on success, false on failure.
  [[nodiscard]] static bool register_shutdown_event(int eventfd);

  // Unregisters a previously registered eventfd.
  // After this call, no further notifications will be sent to the specified eventfd.
  // This operation always succeeds.
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
