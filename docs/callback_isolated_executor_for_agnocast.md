# Callback Isolated Executor for Agnocast

[`agnocast::CallbackIsolatedAgnocastExecutor`](../src/agnocastlib/include/agnocast/agnocast_callback_isolated_executor.hpp) and [`agnocast_component_container_cie`](../src/agnocastlib/src/agnocast_component_container_cie.cpp) are the Agnocast versions of the Callback Isolated Executor, which can handle both ROS 2 callbacks and Agnocast callbacks. For basic usage, please refer to the [tier4/callback_isolated_executor README](https://github.com/tier4/callback_isolated_executor/blob/main/README.md).

## Differences from the Original

### Package, Class, and Executable Names

| | Original | Agnocast Version |
|--|----------|------------------|
| Package | `callback_isolated_executor` | `agnocastlib` |
| Executor Class | `CallbackIsolatedExecutor` | `agnocast::CallbackIsolatedAgnocastExecutor` |
| Container Executable | `component_container_callback_isolated` | `agnocast_component_container_cie` |

### Features

#### Multiple ROS domain support

`cie_thread_configurator` can handle callback groups from multiple ROS domains. Use the `--domains` option to specify domain IDs you use:

```bash
ros2 run cie_thread_configurator thread_configurator_node --prerun --domains 0,1
```

#### RT Throttling

The `rt_throttling` feature configures the kernel's real-time scheduling bandwidth parameters (`sched_rt_period_us` and `sched_rt_runtime_us`). At startup, `cie_thread_configurator` validates that the current kernel values match the configuration and reports an error if they differ.

Since `/proc/sys/kernel/sched_rt_period_us` and `/proc/sys/kernel/sched_rt_runtime_us` can only be written by root (uid 0) — Linux capabilities such as `CAP_SYS_ADMIN` cannot bypass the `/proc/sys/` permission check — these values must be pre-configured via `/etc/sysctl.d/`:

```bash
# /etc/sysctl.d/99-rt-throttling.conf
kernel.sched_rt_period_us = 1000000
kernel.sched_rt_runtime_us = 950000
```

After creating the file, apply it immediately with:

```bash
sudo sysctl --system
```

The values will also be applied automatically on subsequent boots.
