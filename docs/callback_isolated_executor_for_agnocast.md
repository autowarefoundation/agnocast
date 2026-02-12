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

#### `agnocast_components_register_node` Macro

Instead of manually configuring component containers and launch files, you can use the `agnocast_components_register_node()` CMake macro (provided by the [`agnocast_components`](../src/agnocast_components) package) as a drop-in replacement for `rclcpp_components_register_node()`. This macro generates a standalone executable that uses the specified Agnocast executor.

```cmake
find_package(agnocast_components REQUIRED)

add_library(my_component SHARED src/my_node.cpp)
ament_target_dependencies(my_component rclcpp rclcpp_components agnocastlib ...)

agnocast_components_register_node(
  my_component
  PLUGIN "MyNode"
  EXECUTABLE my_node
  EXECUTOR CallbackIsolatedAgnocastExecutor
)
```

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
