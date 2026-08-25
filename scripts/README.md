# scripts/

This directory contains shell scripts used for installing, building, running, testing, and releasing Agnocast. Scripts are organized by audience.

- **Top level** and [`sample_application/`](#sample_application) — user-facing (runtime setup and sample launches).
- [`dev/`](#dev--setup-build-lint) and [`test/`](#test--tests-and-coverage) — developer-facing (build/lint/test workflow).
- [`releases/`](#releases--maintainer-only) — maintainer-only (Debian/PPA release automation).

All scripts are intended to be invoked from the repository root unless noted otherwise.

## User-facing

| Script | Purpose |
|---|---|
| `dds_config.bash` | Apply CycloneDDS runtime settings (`net.core.rmem_max`, loopback multicast) required for Agnocast over CycloneDDS. Guarded by `/tmp/cycloneDDS_configured` so it runs only once per boot. |
| `run_daemon.bash` | Run the user-space daemon (`agnocast_daemon`) in the foreground. Required in **user daemon mode** (workspace built with `-DAGNOCAST_USE_DAEMON=ON`): must be running before any Agnocast process starts, in place of the kernel module. Build first with `agnocast_daemon`'s `make` or `dev/build_all_daemon.bash`. |
| `setup_thread_configurator.bash` | Grant `CAP_SYS_NICE` to `thread_configurator_node` and register library paths in `/etc/ld.so.conf.d/agnocast-cie.conf`. Required for Callback Isolated Executor. See the [integration guide](https://autowarefoundation.github.io/agnocast_doc/callback-isolated-executor/integration-guide/#step-2-set-up-the-thread-configurator). |
| `switch_kmod.bash` | Swap the host's `agnocast-kmod-v<ver>` to another version. For container-based setups where only the host-side kmod needs replacing; the kmod and in-container heaphook must share the same ioctl ABI version. |

### sample_application/

Each script is a thin wrapper that runs `source install/setup.bash` followed by the matching `ros2 launch agnocast_sample_application <name>.launch.xml`.

**Samples inheriting from `rclcpp::Node`:**

| Script | Launch file |
|---|---|
| `run_talker.bash` | `talker.launch.xml` |
| `run_listener.bash` | `listener.launch.xml` |
| `run_cie_talker.bash` | `cie_talker.launch.xml` (`CallbackIsolatedAgnocastExecutor`) |
| `run_cie_listener.bash` | `cie_listener.launch.xml` (`CallbackIsolatedAgnocastExecutor`) |
| `run_client.bash` | `client.launch.xml` (service client) |
| `run_server.bash` | `server.launch.xml` (service server) |

**Samples inheriting from `agnocast::Node` (instead of `rclcpp::Node`):**

| Script | Launch file |
|---|---|
| `run_no_rclcpp_talker.bash` | `no_rclcpp_talker.launch.xml` |
| `run_no_rclcpp_listener.bash` | `no_rclcpp_listener.launch.xml` |
| `run_no_rclcpp_take_listener.bash` | `no_rclcpp_take_listener.launch.xml` (polling-style subscription) |
| `run_no_rclcpp_pubsub.bash` | `no_rclcpp_pubsub.launch.xml` |
| `run_no_rclcpp_client.bash` | `no_rclcpp_client.launch.xml` |
| `run_no_rclcpp_server.bash` | `no_rclcpp_server.launch.xml` |
| `run_sim_time_timer.bash` | `sim_time_timer.launch.xml` (simulation time) |

## Developer-facing

### dev/ — setup, build, lint

| Script | Purpose |
|---|---|
| `dev/setup.bash` | Install build dependencies: `rosdep install --from-paths src`, and Rust toolchain `1.75.0` with `clippy`/`rustfmt`. Aborts if conflicting apt-installed `agnocast-*` packages are present. |
| `dev/build_all.bash` | Build the whole workspace: `colcon build`, `make` for `agnocast_kmod/`, `cargo build --release` for `agnocast_heaphook/`, then copy `libagnocast_heaphook.so` into `install/agnocastlib/lib/`. |
| `dev/build_all_daemon.bash` | Same as `build_all.bash` but for **user daemon mode**: builds the `agnocast_daemon` executable (instead of the kernel module) and passes `-DAGNOCAST_USE_DAEMON=ON` so `agnocastlib` and downstream packages use the Unix-socket client. Heaphook is still built. Start the daemon afterwards with `run_daemon.bash`. |
| `dev/run_checkpatch.bash` | Run the Linux kernel `checkpatch.pl` against C/H files under `agnocast_kmod/`. Auto-discovers `checkpatch.pl` from `PATH` or kernel headers; override with the `CHECKPATCH` env var. |

### test/ — tests and coverage

**Backend selection.** The end-to-end scripts below (`e2e_test_1to1`, `e2e_test_2to2`, `e2e_test_many_exit`, `e2e_test_stress`) work against either backend, selected via the `AGNOCAST_USE_DAEMON` environment variable:

- **Kernel module (default):** the script verifies `agnocast.ko` is loaded, as before.
- **User daemon** (`AGNOCAST_USE_DAEMON=1`): the script reuses a daemon already listening on `/tmp/agnocast_daemon.sock`, or starts one from `agnocast_daemon/build/agnocast_daemon` and stops it on exit. Build the workspace with `dev/build_all_daemon.bash` first, so `agnocastlib` actually uses the socket client.

The shared logic lives in `scripts/lib/agnocast_backend.bash` (`agnocast_require_backend`, `agnocast_start_daemon`, `agnocast_stop_daemon`).

| Script | Purpose |
|---|---|
| `test/test_and_create_report.bash` | Build with coverage flags, run `colcon test`, and generate the HTML coverage report for `agnocastlib`. Temporarily removes apt-installed `agnocast-heaphook` if present and restores it at the end. |
| `test/run_kunit.bash` | Build and insert `agnocast_kunit.ko`, parse pass/fail from dmesg, and generate the HTML coverage report for the kernel module. Requires a kernel with `CONFIG_KUNIT=y` and `CONFIG_GCOV_KERNEL=y`. |
| `test/run_requires_kernel_module_tests.bash` | Build `agnocast_e2e_test` and run `colcon test` filtered to the `requires_kernel_module` label. Kernel module must already be loaded. |
| `test/e2e_test_1to1.bash` | 1-publisher / 1-subscriber end-to-end test sweeping publisher type (agnocast/ros2), QoS depth, transient-local, take-subscription, and launch-order combinations. Options: `-s` single, `-c` continue-on-failure, `-p N` parallel workers. |
| `test/e2e_test_2to2.bash` | 2-publisher / 2-subscriber end-to-end test sweeping container layouts and `agno↔ros2` bridge modes. Options: `-s`, `-c`. |
| `test/e2e_test_many_exit.bash` | Spawn many agnocast talker processes and terminate them via `SIGINT` to exercise graceful-exit cleanup. |
| `test/e2e_test_stress.bash` | Run `e2e_test_1to1` and `e2e_test_2to2` under `stress-ng` CPU / VM / mqueue loads. |
| `test/test_rmmod_refcount.bash` | Verify `rmmod agnocast` is refused while `/dev/agnocast` is open and succeeds once the fd is closed. |
| `test/switch_kmod.bats` | [bats](https://github.com/bats-core/bats-core) test suite for `switch_kmod.bash` (9 cases). **Destructive** — swaps installed `agnocast-kmod-v*` packages and load/unloads the module. Requires `apt install bats` and a prior run of `switch_kmod_canonical_setup.bash`. Run: `sudo bats scripts/test/switch_kmod.bats`. |
| `test/switch_kmod_canonical_setup.bash` | One-time, idempotent setup for `switch_kmod.bats`: installs `agnocast-kmod-v${CANONICAL_VER:-2.3.5}` and caches the `.deb` for teardown recovery. |

### releases/ — maintainer-only

Release automation for the `agnocast-heaphook` and `agnocast-kmod` Debian packages on `ppa:t4-system-software/agnocast`. Requires `debuild`, `dput`, `backportpackage`, and `DEBEMAIL`/`DEBFULLNAME` exported.

| Script | Purpose |
|---|---|
| `releases/prepare_release_heaphook.bash` | Build a signed source package (`.dsc` / `.changes`) for `agnocast-heaphook` via `debuild -S -sa`. |
| `releases/prepare_release_kmod.bash` | Build a signed source package for `agnocast-kmod` (DKMS). Verifies `dh-dkms` is recorded in the generated `.dsc`. |
| `releases/release_heaphook.bash` | Upload the heaphook `.changes` to the PPA (noble) and backport to jammy via `backportpackage`. |
| `releases/release_kmod.bash` | Same flow for `agnocast-kmod`. Re-checks `dh-dkms` presence in the `.dsc` before upload. |
