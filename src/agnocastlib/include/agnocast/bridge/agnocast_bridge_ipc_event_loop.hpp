#pragma once

#include <rclcpp/logger.hpp>

#include <mqueue.h>
#include <sys/types.h>

#include <functional>
#include <string>

namespace agnocast
{
class BridgeIpcEventLoop
{
public:
  using EventCallback = std::function<void(int fd)>;
  using SignalCallback = std::function<void()>;

  BridgeIpcEventLoop(pid_t target_pid, const rclcpp::Logger & logger);
  ~BridgeIpcEventLoop();

  BridgeIpcEventLoop(const BridgeIpcEventLoop &) = delete;
  BridgeIpcEventLoop & operator=(const BridgeIpcEventLoop &) = delete;

  bool spin_once(int timeout_ms);

  void set_parent_mq_handler(EventCallback cb);
  void set_peer_mq_handler(EventCallback cb);
  void set_signal_handler(SignalCallback cb);

  void close_parent_mq();

private:
  rclcpp::Logger logger_;

  int epoll_fd_ = -1;
  int signal_fd_ = -1;

  mqd_t mq_parent_fd_ = (mqd_t)-1;
  mqd_t mq_peer_fd_ = (mqd_t)-1;

  std::string mq_parent_name_;
  std::string mq_self_name_;

  EventCallback parent_cb_;
  EventCallback peer_cb_;
  SignalCallback signal_cb_;

  static void ignore_signals(std::initializer_list<int> signals);
  static sigset_t block_signals(std::initializer_list<int> signals);

  void setup_mq(pid_t target_pid);
  void setup_signals();
  void setup_epoll();

  static mqd_t create_and_open_mq(const std::string & name, const std::string & label);
  void add_fd_to_epoll(int fd, const std::string & label) const;

  void cleanup_resources();
};

}  // namespace agnocast
