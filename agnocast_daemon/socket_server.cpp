// SPDX-License-Identifier: Apache-2.0
#include "socket_server.hpp"

#include <sys/epoll.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <thread>
#include <vector>

static constexpr int kMaxEpollEvents = 64;
static constexpr int kEpollTimeoutMs = 1000;

// Maximum size of a single request message (RequestHeader + largest payload).
// AddSubscriberRequest (524 bytes) is the largest payload; 600 adds margin.
static constexpr size_t kMaxRequestSize = sizeof(RequestHeader) + 600;

// ============================================================
// Construction / destruction
// ============================================================

SocketServer::SocketServer(MetadataStore & store, MemoryAllocator & allocator)
: store_(store), allocator_(allocator), handlers_(store, allocator)
{
  server_fd_ = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0);
  if (server_fd_ < 0) {
    throw std::runtime_error(std::string("socket() failed: ") + strerror(errno));
  }

  // Remove stale socket file so bind() does not fail with EADDRINUSE.
  // A legitimate previous run cleans up on exit, but do it here defensively.
  unlink(AGNOCAST_DAEMON_SOCKET_PATH);

  sockaddr_un addr{};
  addr.sun_family = AF_UNIX;
  strncpy(addr.sun_path, AGNOCAST_DAEMON_SOCKET_PATH, sizeof(addr.sun_path) - 1);

  if (bind(server_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
    close(server_fd_);
    throw std::runtime_error(std::string("bind() failed: ") + strerror(errno));
  }

  // Allow owner + group access only (0660).
  chmod(AGNOCAST_DAEMON_SOCKET_PATH, 0660);

  if (listen(server_fd_, SOMAXCONN) < 0) {
    close(server_fd_);
    unlink(AGNOCAST_DAEMON_SOCKET_PATH);
    throw std::runtime_error(std::string("listen() failed: ") + strerror(errno));
  }
}

SocketServer::~SocketServer()
{
  if (server_fd_ >= 0) close(server_fd_);
  if (epoll_fd_ >= 0) close(epoll_fd_);
  // Remove socket file so a subsequent daemon start does not see a stale path.
  unlink(AGNOCAST_DAEMON_SOCKET_PATH);
}

// ============================================================
// Main event loop
// ============================================================

void SocketServer::run()
{
  epoll_fd_ = epoll_create1(EPOLL_CLOEXEC);
  if (epoll_fd_ < 0) {
    throw std::runtime_error(std::string("epoll_create1() failed: ") + strerror(errno));
  }

  epoll_event ev{};
  ev.events = EPOLLIN;
  ev.data.fd = server_fd_;
  if (epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, server_fd_, &ev) < 0) {
    throw std::runtime_error(std::string("epoll_ctl() failed: ") + strerror(errno));
  }

  epoll_event events[kMaxEpollEvents];
  while (!shutdown_requested_) {
    const int nfds = epoll_wait(epoll_fd_, events, kMaxEpollEvents, kEpollTimeoutMs);
    if (nfds < 0) {
      if (errno == EINTR) continue;
      fprintf(stderr, "agnocast_daemon: epoll_wait() failed: %s\n", strerror(errno));
      break;
    }
    for (int i = 0; i < nfds; ++i) {
      if (events[i].data.fd == server_fd_) {
        accept_client();
      }
    }
  }
}

void SocketServer::request_shutdown() noexcept
{
  shutdown_requested_.store(true, std::memory_order_relaxed);
}

// ============================================================
// Client connection
// ============================================================

void SocketServer::accept_client()
{
  const int client_fd = accept4(server_fd_, nullptr, nullptr, SOCK_CLOEXEC);
  if (client_fd < 0) {
    if (errno != EAGAIN && errno != EWOULDBLOCK) {
      fprintf(stderr, "agnocast_daemon: accept4() failed: %s\n", strerror(errno));
    }
    return;
  }

  // Obtain the connecting process's PID from the kernel.
  // SO_PEERCRED is kernel-verified and cannot be spoofed by the client.
  ucred cred{};
  socklen_t cred_len = sizeof(cred);
  if (getsockopt(client_fd, SOL_SOCKET, SO_PEERCRED, &cred, &cred_len) < 0) {
    fprintf(stderr, "agnocast_daemon: getsockopt(SO_PEERCRED) failed: %s\n", strerror(errno));
    close(client_fd);
    return;
  }

  const int buf = AGNOCAST_DAEMON_SOCKET_BUF_SIZE;
  if (setsockopt(client_fd, SOL_SOCKET, SO_SNDBUF, &buf, sizeof(buf)) < 0) {
    fprintf(stderr, "agnocast_daemon: setsockopt(SO_SNDBUF) failed: %s\n", strerror(errno));
    close(client_fd);
    return;
  }
  if (setsockopt(client_fd, SOL_SOCKET, SO_RCVBUF, &buf, sizeof(buf)) < 0) {
    fprintf(stderr, "agnocast_daemon: setsockopt(SO_RCVBUF) failed: %s\n", strerror(errno));
    close(client_fd);
    return;
  }

  std::thread([this, client_fd, pid = cred.pid]() {
    handle_client(client_fd, pid);
    close(client_fd);
  }).detach();
}

// ============================================================
// Per-client request loop
// ============================================================

void SocketServer::handle_client(int client_fd, pid_t client_pid)
{
  std::vector<uint8_t> buf(kMaxRequestSize);

  while (!shutdown_requested_) {
    // For SOCK_SEQPACKET, one recv() delivers exactly one message (the full
    // packet sent by the client).  MSG_TRUNC would indicate buffer overflow.
    const ssize_t n = recv(client_fd, buf.data(), buf.size(), MSG_TRUNC);
    if (n == 0) break;  // client closed the connection
    if (n < 0) {
      if (errno == EINTR) continue;
      fprintf(stderr, "agnocast_daemon: recv() failed (pid=%d): %s\n", client_pid, strerror(errno));
      break;
    }
    if (static_cast<size_t>(n) > kMaxRequestSize) {
      // MSG_TRUNC caused truncation — reject the oversized message.
      fprintf(
        stderr, "agnocast_daemon: oversized message from pid=%d (%zd bytes)\n", client_pid, n);
      break;
    }
    if (static_cast<size_t>(n) < sizeof(RequestHeader)) {
      fprintf(
        stderr, "agnocast_daemon: message too short from pid=%d (%zd bytes)\n", client_pid, n);
      break;
    }

    RequestHeader hdr{};
    memcpy(&hdr, buf.data(), sizeof(hdr));

    const size_t expected = sizeof(RequestHeader) + hdr.payload_size;
    if (static_cast<size_t>(n) != expected) {
      fprintf(
        stderr,
        "agnocast_daemon: payload size mismatch from pid=%d "
        "(got %zd, expected %zu)\n",
        client_pid, n, expected);
      break;
    }

    const void * payload = (hdr.payload_size > 0) ? buf.data() + sizeof(RequestHeader) : nullptr;
    handlers_.dispatch(client_fd, client_pid, hdr, payload);
  }

  handlers_.on_client_disconnect(client_pid);
}
