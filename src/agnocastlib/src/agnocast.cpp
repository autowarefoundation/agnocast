#include "agnocast/agnocast.hpp"

#include "agnocast/agnocast_ioctl.hpp"
#include "agnocast/agnocast_version.hpp"
#include "agnocast/bridge/agnocast_bridge_manager.hpp"

#include <ament_index_cpp/get_package_prefix.hpp>

#include <dlfcn.h>
#include <strings.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <span>
#include <string_view>
#include <vector>

extern "C" {
const char * agnocast_get_version()
{
  return agnocastlib::VERSION;
}
}

namespace agnocast
{

int agnocast_fd = -1;
std::vector<int> shm_fds;
std::mutex shm_fds_mtx;
std::mutex mmap_mtx;
// mmap_mtx serves two distinct purposes. Both require it to be held across the whole
// receive/take ioctl *and* the shared-memory mapping that follows it, so it must stay
// process-global and must not be narrowed to per-subscription or per-executor scope.
//
// (1) Prevents a race condition and segfault between two threads
// in a multithreaded executor using the same eventfd.
//
// Race Scenario:
// 1. Thread 1 (T1):
//    - Calls epoll_wait(), reads the eventfd, then ioctl(RECEIVE_CMD), initially obtaining
//      publisher info (PID, shared memory address `shm_addr`).
//    - Critical: OS context switch occurs *after* ioctl() but *before* T1 fully
//      processes/maps `shm_addr`.
// 2. Thread 2 (T2):
//    - Calls epoll_wait(), reads the eventfd, then ioctl(RECEIVE_CMD) on the same eventfd,
//      but does *not* receive publisher info (assuming it's already set up).
//    - Proceeds to a callback which attempts to use `shm_addr`, leading to a SEGFAULT.
//
// Root Cause: T2's callback uses `shm_addr` that T1 fetched but hadn't initialized/mapped yet.
// This mutex ensures atomicity for T1's critical section: from ioctl fetching publisher
// info through to completing shared memory setup.
//
// (2) Serializes RECEIVE_CMD/TAKE_CMD process-wide, which the kernel module depends on.
// agnocast_ioctl_receive_msg and agnocast_ioctl_take_msg take only a *read* lock on the topic, so
// the kernel lets receives on one topic run concurrently and relies on this mutex to keep two of
// them from overlapping within one process. That matters most under a Reentrant callback group,
// where one subscription's callback runs on several threads at once: two concurrent calls for the
// same subscriber would read the same sub_info->latest_received_entry_id and walk the same
// entries, and the loser would fail the ioctl with -EALREADY, which every call site below treats
// as fatal.
// RELEASE_SUB_REF_CMD deliberately stays outside this mutex, to keep the ipc_shared_ptr
// destructor path from contending on the receive fast path.

void * map_area(
  const pid_t pid, const uint64_t shm_addr, const uint64_t shm_size, const bool writable)
{
  const std::string shm_name = create_shm_name(pid);

  int oflag = writable ? O_CREAT | O_EXCL | O_RDWR : O_RDONLY;
  const int shm_mode = 0666;
  int shm_fd = shm_open(shm_name.c_str(), oflag, shm_mode);
  if (shm_fd == -1) {
    RCLCPP_ERROR(logger, "shm_open failed: %s", strerror(errno));
    close(agnocast_fd);
    return nullptr;
  }

  {
    std::lock_guard<std::mutex> lock(shm_fds_mtx);
    shm_fds.push_back(shm_fd);
  }

  auto cleanup_shm_fd = [&]() {
    {
      std::lock_guard<std::mutex> lock(shm_fds_mtx);
      shm_fds.erase(std::remove(shm_fds.begin(), shm_fds.end(), shm_fd), shm_fds.end());
    }
    close(shm_fd);
    if (writable) {
      shm_unlink(shm_name.c_str());
    }
  };

  if (writable) {
    if (ftruncate(shm_fd, static_cast<off_t>(shm_size)) == -1) {
      RCLCPP_ERROR(logger, "ftruncate failed: %s", strerror(errno));
      cleanup_shm_fd();
      close(agnocast_fd);
      return nullptr;
    }

    const int new_shm_mode = 0444;
    if (fchmod(shm_fd, new_shm_mode) == -1) {
      RCLCPP_ERROR(logger, "fchmod failed: %s", strerror(errno));
      cleanup_shm_fd();
      close(agnocast_fd);
      return nullptr;
    }
  }

  int prot = writable ? PROT_READ | PROT_WRITE : PROT_READ;
  void * ret = mmap(
    reinterpret_cast<void *>(shm_addr), shm_size, prot, MAP_SHARED | MAP_FIXED_NOREPLACE, shm_fd,
    0);

  if (ret == MAP_FAILED) {
    RCLCPP_ERROR(logger, "mmap failed: %s", strerror(errno));
    cleanup_shm_fd();
    close(agnocast_fd);
    return nullptr;
  }

  return ret;
}

void * map_writable_area(const pid_t pid, const uint64_t shm_addr, const uint64_t shm_size)
{
  return map_area(pid, shm_addr, shm_size, true);
}

void map_read_only_area(const pid_t pid, const uint64_t shm_addr, const uint64_t shm_size)
{
  if (map_area(pid, shm_addr, shm_size, false) == nullptr) {
    exit(EXIT_FAILURE);
  }
}

// Initializes the child allocator for bridge functionality.
// Note: This function must only be called in a forked child process before TLSF initialization.
// Calling it after initialization will result in double initialization.
void initialize_bridge_allocator(void * mempool_ptr, size_t mempool_size)
{
  void * handle = dlopen(nullptr, RTLD_NOW);
  if (handle == nullptr) {
    const char * err_msg = dlerror();
    throw std::runtime_error(
      std::string("dlopen failed: ") + (err_msg != nullptr ? err_msg : "Unknown"));
  }

  using InitFunc = bool (*)(void *, size_t);
  auto init_func = reinterpret_cast<InitFunc>(dlsym(handle, "init_child_allocator"));

  const char * dlsym_error = dlerror();
  if ((dlsym_error != nullptr) || (init_func == nullptr)) {
    dlclose(handle);
    throw std::runtime_error(
      std::string("dlsym 'init_child_allocator' failed: ") +
      (dlsym_error != nullptr ? dlsym_error : "Symbol is null"));
  }

  bool success = init_func(mempool_ptr, mempool_size);

  if (!success) {
    throw std::runtime_error("init_child_allocator returned false.");
  }
}

initialize_agnocast_result acquire_agnocast_resources_for_bridge()
{
  union ioctl_add_process_args add_process_args = {};
  add_process_args.role = PROCESS_ROLE_BRIDGE_MANAGER;
  add_process_args.domain_id = get_ros_domain_id();
  if (ioctl(agnocast_fd, AGNOCAST_ADD_PROCESS_CMD, &add_process_args) < 0) {
    throw std::runtime_error(std::string("AGNOCAST_ADD_PROCESS_CMD failed: ") + strerror(errno));
  }

  if (add_process_args.ret_bridge_daemon_exist) {
    close(agnocast_fd);
    exit(EXIT_SUCCESS);
  }

  void * mempool_ptr =
    map_writable_area(getpid(), add_process_args.ret_addr, add_process_args.ret_shm_size);

  if (mempool_ptr == nullptr) {
    throw std::runtime_error("map_writable_area failed.");
  }

  return {
    mempool_ptr,
    add_process_args.ret_shm_size,
  };
}

// The spawn lease is the exclusive right to fork one of the namespace-singleton daemons. The
// kernel module grants it to a single process per (namespace, role, domain), which is what keeps a
// launch of N processes to one daemon fork rather than N. Returns the lease fd, or -1 when a
// daemon already runs or another process is already forking one.
//
// The lease is an fd rather than a record keyed by pid so that nothing has to notice a failure:
// the forked child inherits it, and a child that dies before it registers drops the last reference
// and releases the lease, with no timeout to wait out.
int acquire_spawn_lease(const enum process_role role, const uint32_t domain_id)
{
  struct ioctl_acquire_spawn_lease_args args = {};
  args.role = role;
  args.domain_id = domain_id;
  if (ioctl(agnocast_fd, AGNOCAST_ACQUIRE_SPAWN_LEASE_CMD, &args) < 0) {
    RCLCPP_ERROR(logger, "AGNOCAST_ACQUIRE_SPAWN_LEASE_CMD failed: %s", strerror(errno));
    close(agnocast_fd);
    exit(EXIT_FAILURE);
  }
  return args.ret_acquired ? args.ret_lease_fd : -1;
}

// Takes -1 so the daemons that are spawned without a lease need no special case.
void release_spawn_lease(const int lease_fd)
{
  if (lease_fd >= 0) {
    close(lease_fd);
  }
}

void poll_for_unlink(const int lease_fd)
{
  // Register so the kernel module can tell a live daemon from a dead one. No domain_id: the
  // kernel module records this daemon as belonging to none.
  union ioctl_add_process_args add_process_args = {};
  add_process_args.role = PROCESS_ROLE_UNLINK_DAEMON;
  if (ioctl(agnocast_fd, AGNOCAST_ADD_PROCESS_CMD, &add_process_args) < 0) {
    RCLCPP_ERROR(logger, "AGNOCAST_ADD_PROCESS_CMD failed: %s", strerror(errno));
    close(agnocast_fd);
    exit(EXIT_FAILURE);
  }

  // The lease only had to cover the window between the fork and this registration; from here on
  // the registration itself is what tells a starting process that a daemon exists. Holding the
  // lease any longer would block a replacement during the window between this daemon
  // deregistering in agnocast_commit_exit_process() and actually dying.
  release_spawn_lease(lease_fd);

  // The lease makes this unreachable in practice, but the kernel module's decision is the
  // authoritative one and costs nothing to honour.
  if (add_process_args.ret_unlink_daemon_exist) {
    close(agnocast_fd);
    exit(EXIT_SUCCESS);
  }

  while (true) {
    sleep(1);

    struct ioctl_get_exit_process_args get_exit_process_args = {};
    do {
      if (ioctl(agnocast_fd, AGNOCAST_GET_EXIT_PROCESS_CMD, &get_exit_process_args) < 0) {
        RCLCPP_ERROR(logger, "AGNOCAST_GET_EXIT_PROCESS_CMD failed: %s", strerror(errno));
        close(agnocast_fd);
        exit(EXIT_FAILURE);
      }

      if (get_exit_process_args.ret_pid > 0) {
        const std::string shm_name = create_shm_name(get_exit_process_args.ret_pid);
        shm_unlink(shm_name.c_str());
      }
    } while (get_exit_process_args.ret_pid > 0);

    if (get_exit_process_args.ret_daemon_should_exit) {
      break;
    }
  }

  exit(0);
}

void poll_for_bridge_manager(const int lease_fd)
{
  try {
    const auto resources = acquire_agnocast_resources_for_bridge();
    // Registered, so the lease has done its job; see poll_for_unlink().
    release_spawn_lease(lease_fd);
    initialize_bridge_allocator(resources.mempool_ptr, resources.mempool_size);
    BridgeManager manager;
    manager.run();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger, "BridgeManager crashed: %s", e.what());
    exit(EXIT_FAILURE);
  }
  exit(0);
}

// For the forked child, which by convention here sticks to async-signal-safe calls: no
// RCLCPP_ERROR, no strerror().
void write_stderr_with_errno(const std::string_view msg, const int err)
{
  constexpr size_t buf_size = 256;
  std::array<char, buf_size> buf = {};
  const int len = snprintf(
    buf.data(), buf.size(), "%.*s (errno=%d)\n", static_cast<int>(msg.size()), msg.data(), err);
  if (len > 0) {
    const ssize_t written =
      write(STDERR_FILENO, buf.data(), std::min(static_cast<size_t>(len), buf.size() - 1));
    static_cast<void>(written);
  }
}

constexpr const char * DISCOVERY_AGENT_PACKAGE = "ros2agnocast_discovery_agent";
constexpr const char * DISCOVERY_AGENT_EXECUTABLE = "agnocast_discovery_agent";

std::string resolve_discovery_agent_path()
{
  std::string candidate;
  try {
    candidate = ament_index_cpp::get_package_prefix(DISCOVERY_AGENT_PACKAGE) + "/lib/" +
                DISCOVERY_AGENT_PACKAGE + "/" + DISCOVERY_AGENT_EXECUTABLE;
  } catch (const std::exception &) {
    // AMENT_PREFIX_PATH unset or empty, or the package is not indexed.
    return "";
  }
  // R_OK too: the agent is a shebang script, so the interpreter has to read it.
  return access(candidate.c_str(), R_OK | X_OK) == 0 ? candidate : "";
}

enum class claim_result { won, lost, failed };

claim_result claim_discovery_agent(const uint32_t domain_id)
{
  struct ioctl_add_discovery_agent_args args = {};
  args.domain_id = domain_id;
  if (ioctl(agnocast_fd, AGNOCAST_ADD_DISCOVERY_AGENT_CMD, &args) < 0) {
    write_stderr_with_errno(
      "[ERROR] [Agnocast] Failed to claim the discovery agent singleton", errno);
    return claim_result::failed;
  }
  return args.ret_owned_by_caller ? claim_result::won : claim_result::lost;
}

// Not `ros2 run`, which would Popen the agent as a grandchild under a different PID than the one
// that claimed the singleton. exec preserves the pid, so the agent's own claim hits the kmod's
// idempotent path and wins.
[[noreturn]] void exec_discovery_agent(const char * agent_path)
{
  // const_cast is safe: execv does not modify argv.
  // NOLINTBEGIN(cppcoreguidelines-pro-type-const-cast)
  std::array<char *, 3> argv = {
    const_cast<char *>(agent_path), const_cast<char *>("--exit-when-idle"), nullptr};
  // NOLINTEND(cppcoreguidelines-pro-type-const-cast)
  execv(agent_path, argv.data());
  // The kmod releases the claim on process exit, so the slot is not leaked.
  write_stderr_with_errno("[ERROR] [Agnocast] Failed to exec the discovery agent", errno);
  _exit(EXIT_FAILURE);
}

struct semver
{
  int major;
  int minor;
  int patch;
};

bool parse_semver(const char * version, struct semver * out_ver)
{
  if (version == nullptr || out_ver == nullptr) {
    return false;
  }

  out_ver->major = 0;
  out_ver->minor = 0;
  out_ver->patch = 0;

  std::string version_str(version);
  std::stringstream ss(version_str);

  int64_t major = 0;
  int64_t minor = 0;
  int64_t patch = 0;

  if (!(ss >> major) || ss.get() != '.') {
    return false;
  }

  if (!(ss >> minor) || ss.get() != '.') {
    return false;
  }

  if (!(ss >> patch)) {
    return false;
  }

  if (!ss.eof()) {
    char remaining = '\0';
    if (ss >> remaining) {
      return false;
    }
  }

  if (major < 0 || minor < 0 || patch < 0) {
    return false;
  }

  out_ver->major = static_cast<int>(major);
  out_ver->minor = static_cast<int>(minor);
  out_ver->patch = static_cast<int>(patch);

  return true;
}

bool compare_to_minor_version(const struct semver * v1, const struct semver * v2)
{
  if (v1 == nullptr || v2 == nullptr) {
    return false;
  }

  return (v1->major == v2->major && v1->minor == v2->minor);
}

bool compare_to_patch_version(const struct semver * v1, const struct semver * v2)
{
  if (v1 == nullptr || v2 == nullptr) {
    return false;
  }

  return (v1->major == v2->major && v1->minor == v2->minor && v1->patch == v2->patch);
}

bool is_version_consistent(
  const unsigned char * heaphook_version_ptr, const size_t heaphook_version_str_len,
  struct ioctl_get_version_args kmod_version)
{
  std::array<char, VERSION_BUFFER_LEN> heaphook_version_arr{};
  struct semver lib_ver
  {
  };
  struct semver heaphook_ver
  {
  };
  struct semver kmod_ver
  {
  };

  size_t copy_len = heaphook_version_str_len < (VERSION_BUFFER_LEN - 1) ? heaphook_version_str_len
                                                                        : (VERSION_BUFFER_LEN - 1);
  std::memcpy(heaphook_version_arr.data(), heaphook_version_ptr, copy_len);
  heaphook_version_arr[copy_len] = '\0';

  bool parse_lib_result = parse_semver(agnocastlib::VERSION, &lib_ver);
  bool parse_heaphook_result = parse_semver(heaphook_version_arr.data(), &heaphook_ver);
  bool parse_kmod_result =
    parse_semver(static_cast<const char *>(&kmod_version.ret_version[0]), &kmod_ver);

  if (!parse_lib_result || !parse_heaphook_result || !parse_kmod_result) {
    RCLCPP_ERROR(logger, "Failed to parse one or more version strings");
    return false;
  }

  if (!compare_to_patch_version(&lib_ver, &heaphook_ver)) {
    RCLCPP_ERROR(
      logger,
      "Agnocast Heaphook and Agnocastlib versions must match exactly: Major, Minor, and Patch "
      "versions must all be identical. (agnocast-heaphook(%d.%d.%d), agnocast(%d.%d.%d))",
      heaphook_ver.major, heaphook_ver.minor, heaphook_ver.patch, lib_ver.major, lib_ver.minor,
      lib_ver.patch);
    return false;
  }

  if (!compare_to_minor_version(&lib_ver, &kmod_ver)) {
    RCLCPP_ERROR(
      logger,
      "Agnocast Kernel Module and Agnocastlib must be compatible: Major and Minor versions must "
      "match. (agnocast-kmod(%d.%d.%d), agnocast(%d.%d.%d))",
      kmod_ver.major, kmod_ver.minor, kmod_ver.patch, lib_ver.major, lib_ver.minor, lib_ver.patch);
    return false;
  }

  return true;
}

// Opt-out for deployments that manage the discovery agent themselves. getenv()
// does not allocate, so this is safe before agnocast's allocator is ready.
bool discovery_agent_auto_fork_disabled()
{
  const char * v = getenv("AGNOCAST_NO_DISCOVERY_AGENT");
  return v != nullptr &&
         (strcmp(v, "1") == 0 || strcasecmp(v, "true") == 0 || strcasecmp(v, "yes") == 0);
}

// lease_fd is the spawn lease acquired by the caller, or -1 for a daemon that is not leased. The
// child inherits it and hands it back once it has registered; the parent must drop its own copy as
// soon as the fork returns, or the lease would outlive the child it is meant to track.
template <typename Func>
pid_t spawn_daemon_process(const int lease_fd, Func && func)
{
  auto fail = [](const char * err_fmt) {
    RCLCPP_ERROR(logger, err_fmt, strerror(errno));
    close(agnocast_fd);
    exit(EXIT_FAILURE);
  };

  pid_t pid = fork();
  if (pid < 0) {
    fail("fork failed: %s");
  }
  if (pid == 0) {
    agnocast::is_bridge_process = true;
    unsetenv("LD_PRELOAD");

    // Redirect stdio to /dev/null when stdout or stderr is an inherited pipe or socket. In that
    // case, a process may be reading from the pipe and waiting on it to close, which can cause
    // the process to hang because the daemon never closes it. Redirecting to /dev/null works around
    // this issue.
    struct stat st_out = {};
    struct stat st_err = {};
    if (fstat(STDOUT_FILENO, &st_out) < 0) {
      fail("fstat for stdout failed: %s");
    }
    if (fstat(STDERR_FILENO, &st_err) < 0) {
      fail("fstat for stderr failed: %s");
    }

    if (
      S_ISFIFO(st_out.st_mode) || S_ISFIFO(st_err.st_mode) || S_ISSOCK(st_out.st_mode) ||
      S_ISSOCK(st_err.st_mode)) {
      int devnull = open("/dev/null", O_RDWR);
      if (devnull < 0) {
        fail("Failed to open /dev/null: %s");
      }

      // Send the output to the terminal rather than discarding it. Must be opened before setsid(),
      // which drops the controlling terminal /dev/tty resolves against. stdin stays on /dev/null
      // because this is write-only, so reads see EOF rather than EBADF.
      const int tty = open("/dev/tty", O_WRONLY);
      const int out_fd = (tty >= 0) ? tty : devnull;

      if (dup2(devnull, STDIN_FILENO) < 0) {
        fail("dup2 for stdin failed: %s");
      }
      if (dup2(out_fd, STDOUT_FILENO) < 0) {
        fail("dup2 for stdout failed: %s");
      }
      if (dup2(out_fd, STDERR_FILENO) < 0) {
        fail("dup2 for stderr failed: %s");
      }
      if (out_fd != devnull) {
        close(out_fd);
      }
      close(devnull);
    }

    if (setsid() == -1) {
      fail("setsid failed: %s");
    }

    func(lease_fd);
    exit(0);
  }

  release_spawn_lease(lease_fd);
  return pid;
}

// NOTE: Avoid heap allocation inside initialize_agnocast. TLSF is not initialized yet.
struct initialize_agnocast_result initialize_agnocast(
  const unsigned char * heaphook_version_ptr, const size_t heaphook_version_str_len)
{
  if (agnocast_fd >= 0) {
    RCLCPP_ERROR(logger, "Agnocast is already open");
    exit(EXIT_FAILURE);
  }

  agnocast_fd = open("/dev/agnocast", O_RDWR);
  if (agnocast_fd < 0) {
    if (errno == ENOENT) {
      RCLCPP_ERROR(logger, "%s", AGNOCAST_DEVICE_NOT_FOUND_MSG);
    } else {
      RCLCPP_ERROR(logger, "Failed to open /dev/agnocast: %s", strerror(errno));
    }
    exit(EXIT_FAILURE);
  }

  struct ioctl_get_version_args get_version_args = {};
  if (ioctl(agnocast_fd, AGNOCAST_GET_VERSION_CMD, &get_version_args) < 0) {
    RCLCPP_ERROR(logger, "AGNOCAST_GET_VERSION_CMD failed: %s", strerror(errno));
    close(agnocast_fd);
    exit(EXIT_FAILURE);
  }

  if (!is_version_consistent(heaphook_version_ptr, heaphook_version_str_len, get_version_args)) {
    close(agnocast_fd);
    exit(EXIT_FAILURE);
  }

  // add_process_args is a union, so ADD_PROCESS overwrites domain_id with its ret_* fields.
  const uint32_t domain_id = get_ros_domain_id();
  if (domain_id == AGNOCAST_DOMAIN_ID_NONE) {
    RCLCPP_ERROR(logger, "ROS_DOMAIN_ID=%u is reserved by Agnocast", domain_id);
    close(agnocast_fd);
    exit(EXIT_FAILURE);
  }

  union ioctl_add_process_args add_process_args = {};
  add_process_args.role = PROCESS_ROLE_APPLICATION;
  add_process_args.domain_id = domain_id;
  if (ioctl(agnocast_fd, AGNOCAST_ADD_PROCESS_CMD, &add_process_args) < 0) {
    RCLCPP_ERROR(logger, "AGNOCAST_ADD_PROCESS_CMD failed: %s", strerror(errno));
    close(agnocast_fd);
    exit(EXIT_FAILURE);
  }

  auto bridge_mode = get_bridge_mode();

  // Create a shm_unlink daemon process if it doesn't exist in its ipc namespace.
  // ret_unlink_daemon_exist is only an early-out hint that saves an ioctl; the spawn lease is what
  // decides, so that N processes starting at once cost one fork in total rather than one each.
  if (!add_process_args.ret_unlink_daemon_exist) {
    const int lease_fd = acquire_spawn_lease(PROCESS_ROLE_UNLINK_DAEMON, AGNOCAST_DOMAIN_ID_NONE);
    if (lease_fd >= 0) {
      spawn_daemon_process(lease_fd, [](const int fd) { poll_for_unlink(fd); });
    }
  }

  if (bridge_mode == BridgeMode::On && !add_process_args.ret_bridge_daemon_exist) {
    const int lease_fd = acquire_spawn_lease(PROCESS_ROLE_BRIDGE_MANAGER, domain_id);
    if (lease_fd >= 0) {
      spawn_daemon_process(lease_fd, [](const int fd) { poll_for_bridge_manager(fd); });
    }
  }

  // The forked agent inherits this process's IPC namespace and ROS_DOMAIN_ID, and self-exits when
  // the scope empties. A missing agent is not fatal because the data plane does not depend on
  // the observer; a fork() failure still is, as for the other daemons spawned here.
  // ret_discovery_agent_exist is only an early-out hint.
  if (!add_process_args.ret_discovery_agent_exist && !discovery_agent_auto_fork_disabled()) {
    const std::string agent_path = resolve_discovery_agent_path();
    if (agent_path.empty()) {
      RCLCPP_WARN(
        logger,
        "The discovery agent executable was not found in AMENT_PREFIX_PATH, so it is not "
        "auto-started. Source the workspace that installs ros2agnocast_discovery_agent to enable "
        "Agnocast observability.");
    } else {
      // Not leased: the agent already claims its own slot from this child before exec, so a
      // second claim over the same singleton would only give it two sources of truth.
      spawn_daemon_process(-1, [domain_id, agent_path](const int /*lease_fd*/) {
        // Claiming here rather than in the agent keeps the launch O(1): N processes starting at
        // once cost one fork each, not N Python interpreters.
        switch (claim_discovery_agent(domain_id)) {
          case claim_result::won:
            exec_discovery_agent(agent_path.c_str());
          case claim_result::lost:
            _exit(EXIT_SUCCESS);
          case claim_result::failed:
            _exit(EXIT_FAILURE);
        }
      });
    }
  }

  void * mempool_ptr =
    map_writable_area(getpid(), add_process_args.ret_addr, add_process_args.ret_shm_size);
  if (mempool_ptr == nullptr) {
    close(agnocast_fd);
    exit(EXIT_FAILURE);
  }

  struct initialize_agnocast_result result = {};
  result.mempool_ptr = mempool_ptr;
  result.mempool_size = add_process_args.ret_shm_size;
  return result;
}

static void shutdown_agnocast()
{
  std::lock_guard<std::mutex> lock(shm_fds_mtx);
  for (int fd : shm_fds) {
    if (close(fd) == -1) {
      perror("[ERROR] [Agnocast] close shm_fd failed");
    }
  }
}

class Cleanup
{
public:
  Cleanup(const Cleanup &) = delete;
  Cleanup & operator=(const Cleanup &) = delete;
  Cleanup(Cleanup &&) = delete;
  Cleanup & operator=(Cleanup &&) = delete;

  Cleanup() = default;
  ~Cleanup() { shutdown_agnocast(); }
};

static Cleanup cleanup;

}  // namespace agnocast
