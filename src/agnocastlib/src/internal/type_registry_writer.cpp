// Copyright 2026
// SPDX-License-Identifier: Apache-2.0

#include "agnocast/internal/type_registry_writer.hpp"

#include "agnocast/agnocast_utils.hpp"

#include <rclcpp/logging.hpp>

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <string>

namespace agnocast::internal
{

namespace
{
// Directories are world-readable so the daemon (possibly another user) can
// scan them; the per-process file is 0644.
constexpr mode_t kRegistryDirMode = 0755;
constexpr mode_t kRegistryFileMode = 0644;

// Tmpfs root. Default `/dev/shm` is world-writable (1777) so unelevated
// user processes can create entries; override via `AGNOCAST_TMPFS_DIR`
// for hardened containers. Test seam: `set_base_dir_for_test()`.
std::string g_base_dir = []() {
  const char * env = std::getenv("AGNOCAST_TMPFS_DIR");
  std::string root = (env != nullptr && *env != '\0') ? env : "/dev/shm";
  return root + "/agnocast_type_registry";
}();  // NOLINT(runtime/string)

bool ensure_dir(const std::string & path, mode_t mode)
{
  if (mkdir(path.c_str(), mode) == 0) {
    return true;
  }
  if (errno != EEXIST) {
    return false;
  }
  // The path exists but might be a regular file (or a symlink to one).
  // Confirm it's actually a directory so later operations don't fail with
  // a misleading errno.
  struct stat st = {};
  if (::stat(path.c_str(), &st) != 0) {
    return false;
  }
  return S_ISDIR(st.st_mode);
}
}  // namespace

TypeRegistryWriter & TypeRegistryWriter::instance()
{
  static TypeRegistryWriter inst;
  return inst;
}

void TypeRegistryWriter::set_base_dir_for_test(const std::string & dir)
{
  g_base_dir = dir;
}

std::string TypeRegistryWriter::current_path_for_test() const
{
  return path_;
}

void TypeRegistryWriter::ensure_open_locked()
{
  if (fd_ != -1) {
    return;
  }
  if (open_failed_warned_) {
    return;
  }

  uint64_t ns_inode = 0;
  try {
    ns_inode = get_self_ipc_ns_inode();
  } catch (const std::exception & e) {
    RCLCPP_WARN(
      rclcpp::get_logger("Agnocast"), "TypeRegistryWriter: failed to read IPC namespace inode: %s",
      e.what());
    open_failed_warned_ = true;
    return;
  }

  // Any failure here means cross-NS observability silently breaks for
  // this process, so log as ERROR (not a transient warning).
  if (!ensure_dir(g_base_dir, kRegistryDirMode)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("Agnocast"),
      "TypeRegistryWriter: mkdir '%s' failed: %s. Cross-NS observability will "
      "silently NOT work in this process. Override the root with "
      "AGNOCAST_TMPFS_DIR if /dev/shm is unavailable.",
      g_base_dir.c_str(), std::strerror(errno));
    open_failed_warned_ = true;
    return;
  }
  const std::string ns_dir = g_base_dir + "/" + std::to_string(ns_inode);
  if (!ensure_dir(ns_dir, kRegistryDirMode)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("Agnocast"),
      "TypeRegistryWriter: mkdir '%s' failed: %s. Check tmpfs free space.", ns_dir.c_str(),
      std::strerror(errno));
    open_failed_warned_ = true;
    return;
  }

  path_ = ns_dir + "/" + std::to_string(getpid()) + ".txt";
  fd_ = ::open(
    path_.c_str(), O_WRONLY | O_CREAT | O_APPEND | O_CLOEXEC,
    kRegistryFileMode);  // NOLINT(runtime/int)
  if (fd_ == -1) {
    RCLCPP_ERROR(
      rclcpp::get_logger("Agnocast"),
      "TypeRegistryWriter: open('%s') failed: %s. Check the parent directory "
      "permissions and the tmpfs free space.",
      path_.c_str(), std::strerror(errno));
    open_failed_warned_ = true;
    return;
  }

  // Register a one-shot atexit handler the first time we open the file
  // successfully. SIGKILL bypasses this; the daemon's `/proc/<pid>` sweep
  // handles those stale files (and also any case where registration fails).
  static bool atexit_registered = false;
  if (!atexit_registered && std::atexit(&TypeRegistryWriter::on_process_exit) == 0) {
    atexit_registered = true;
  }
}

void TypeRegistryWriter::register_type(
  const std::string & topic_name, const std::string & type_name, const std::string & role,
  const std::string & node_name)
{
  std::lock_guard<std::mutex> lock(mutex_);
  ensure_open_locked();
  if (fd_ == -1) {
    return;
  }

  std::string line;
  line.reserve(topic_name.size() + type_name.size() + role.size() + node_name.size() + 4);
  line.append(topic_name).push_back('\t');
  line.append(type_name).push_back('\t');
  line.append(role).push_back('\t');
  line.append(node_name).push_back('\n');

  // One file per process plus the mutex above keeps writers from interleaving.
  // We loop to drain short writes and retry on EINTR. A partial line is only
  // possible if a later `write()` fails outright (e.g. ENOSPC), in which case
  // the daemon's parser skips the unterminated tail.
  const char * data = line.data();
  size_t remaining = line.size();
  while (remaining > 0) {
    const ssize_t n = ::write(fd_, data, remaining);
    if (n > 0) {
      data += n;  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
      remaining -= static_cast<size_t>(n);
      continue;
    }
    if (n == -1 && errno == EINTR) {
      continue;
    }
    RCLCPP_WARN_ONCE(
      rclcpp::get_logger("Agnocast"),
      "TypeRegistryWriter: write to '%s' failed: %s. Further write failures will be silent.",
      path_.c_str(), std::strerror(errno));
    return;
  }
}

void TypeRegistryWriter::reset_for_test()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (fd_ != -1) {
    ::close(fd_);
    fd_ = -1;
  }
  path_.clear();
  open_failed_warned_ = false;
}

void TypeRegistryWriter::on_process_exit()
{
  auto & inst = TypeRegistryWriter::instance();
  std::lock_guard<std::mutex> lock(inst.mutex_);
  if (inst.fd_ != -1) {
    ::close(inst.fd_);
    inst.fd_ = -1;
  }
  if (!inst.path_.empty()) {
    // Best-effort. If unlink fails the daemon's `cleanup_dead_pids()` will
    // notice the dead pid and remove the file.
    (void)::unlink(inst.path_.c_str());
  }
}

}  // namespace agnocast::internal
