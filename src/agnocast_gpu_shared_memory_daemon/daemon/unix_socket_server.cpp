#include "unix_socket_server.hpp"

#include "agnocast_gpu_shared_memory_daemon/socket_io.hpp"
#include "request_handler.hpp"

#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <utility>

namespace agnocast::gpu_shared_memory_daemon
{

namespace
{

constexpr int kPollTimeoutMs = 200;
constexpr int kListenBacklog = 16;

// Creates the parent directory of `path` (single level, e.g. /run/agnocast).
// Ignores an already-existing directory.
void ensure_parent_dir(const std::string & path)
{
  const auto slash = path.find_last_of('/');
  if (slash == std::string::npos || slash == 0) {
    return;
  }
  const std::string dir = path.substr(0, slash);
  if (::mkdir(dir.c_str(), 0755) != 0 && errno != EEXIST) {
    std::fprintf(
      stderr, "[agnocast_gpu_shared_memory_daemon] warning: mkdir(%s) failed: %s\n", dir.c_str(),
      std::strerror(errno));
  }
}

}  // namespace

UnixSocketServer::UnixSocketServer(GpuSharedMemoryPool & pool, std::string socket_path)
: pool_(pool), socket_path_(std::move(socket_path))
{
}

UnixSocketServer::~UnixSocketServer()
{
  request_stop();
  for (auto & thread : connection_threads_) {
    if (thread.joinable()) {
      thread.join();
    }
  }
  if (listen_fd_ >= 0) {
    ::close(listen_fd_);
    listen_fd_ = -1;
  }
  ::unlink(socket_path_.c_str());
}

bool UnixSocketServer::start()
{
  struct sockaddr_un addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.sun_family = AF_UNIX;
  if (socket_path_.size() >= sizeof(addr.sun_path)) {
    std::fprintf(
      stderr, "[agnocast_gpu_shared_memory_daemon] socket path too long: %s\n",
      socket_path_.c_str());
    return false;
  }

  ensure_parent_dir(socket_path_);

  listen_fd_ = ::socket(AF_UNIX, SOCK_STREAM, 0);
  if (listen_fd_ < 0) {
    std::fprintf(
      stderr, "[agnocast_gpu_shared_memory_daemon] socket() failed: %s\n", std::strerror(errno));
    return false;
  }

  // Remove any stale socket file left by a previous (crashed) instance.
  ::unlink(socket_path_.c_str());
  std::strncpy(addr.sun_path, socket_path_.c_str(), sizeof(addr.sun_path) - 1);

  if (::bind(listen_fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) != 0) {
    std::fprintf(
      stderr, "[agnocast_gpu_shared_memory_daemon] bind(%s) failed: %s\n", socket_path_.c_str(),
      std::strerror(errno));
    ::close(listen_fd_);
    listen_fd_ = -1;
    return false;
  }

  // Allow client processes to connect. TODO: tighten to a dedicated group once the
  // deployment story (which users run publishers/subscribers) is settled.
  ::chmod(socket_path_.c_str(), 0666);

  if (::listen(listen_fd_, kListenBacklog) != 0) {
    std::fprintf(
      stderr, "[agnocast_gpu_shared_memory_daemon] listen() failed: %s\n", std::strerror(errno));
    ::close(listen_fd_);
    listen_fd_ = -1;
    ::unlink(socket_path_.c_str());
    return false;
  }

  return true;
}

void UnixSocketServer::run()
{
  while (!stop_.load()) {
    const int ready = wait_readable(listen_fd_, kPollTimeoutMs);
    if (ready < 0) {
      break;
    }
    if (ready == 0) {
      continue;  // timeout: re-check stop_
    }

    const int client_fd = ::accept(listen_fd_, nullptr, nullptr);
    if (client_fd < 0) {
      if (errno == EINTR) {
        continue;
      }
      continue;  // transient accept error: keep serving
    }
    connection_threads_.emplace_back([this, client_fd] { handle_connection(client_fd); });
  }

  for (auto & thread : connection_threads_) {
    if (thread.joinable()) {
      thread.join();
    }
  }
  connection_threads_.clear();
}

void UnixSocketServer::handle_connection(int fd)
{
  RequestHandler handler(pool_);
  while (!stop_.load()) {
    const int ready = wait_readable(fd, kPollTimeoutMs);
    if (ready < 0) {
      break;
    }
    if (ready == 0) {
      continue;  // timeout: re-check stop_
    }

    MessageHeader header;
    std::vector<std::uint8_t> payload;
    if (!read_message(fd, header, payload)) {
      break;  // peer closed or protocol error
    }

    MessageType response_type;
    std::vector<std::uint8_t> response_payload;
    if (!handler.handle(header, payload, response_type, response_payload)) {
      break;  // malformed/unexpected request: close connection
    }
    if (!write_message(fd, response_type, response_payload)) {
      break;
    }
  }
  ::close(fd);
}

}  // namespace agnocast::gpu_shared_memory_daemon
