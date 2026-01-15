# Agnocast::Node and rclcpp::Node Interface Comparison

## Executive Summary

While Agnocast can be applied incrementally on a per-topic basis to nodes that inherit from `rclcpp::Node`, we also provide `agnocast::Node` for users who seek further performance improvements.

`agnocast::Node` offers an API that is largely compatible with `rclcpp::Node`, allowing existing nodes to be migrated by simply replacing `rclcpp::Node` with `agnocast::Node`.
However, some APIs are not yet supported, and others are intentionally not planned to be supported.
This document summarizes those limitations.
Since `rclcpp::Node` is composed of ten modular node interfaces, this document organizes the API compatibility of `agnocast::Node` accordingly, one section per interface.

**Key Characteristics of agnocast::Node**:

- `agnocast::Node` is a node implementation that bypasses the RMW layer entirely (e.g., it does not create a DDS participant)
- When run as a standalone node (i.e., not loaded into a Component Container), nodes inheriting from `agnocast::Node` must be executed with Agnocast-only executors (i.e., `AgnocastOnlySingleThreadedExecutor`, `AgnocastOnlyMultiThreadedExecutor` or `AgnocastOnlyCallbackIsolatedExecutor`). In contrast, when such nodes are loaded into a Component Container, the container’s Agnocast-compatible executors—`SingleThreadedAgnocastExecutor`, `MultiThreadedAgnocastExecutor`, and `CallbackIsolatedAgnocastExecutor`—can also be used.

---

## 1. Overview of rclcpp::Node Interface Architecture

rclcpp::Node consists of 10 interface components:

1. **NodeBaseInterface** - Core node identity and callback management
2. **NodeTopicsInterface** - Publisher/Subscription management
3. **NodeParametersInterface** - Parameter management
4. **NodeGraphInterface** - ROS graph introspection
5. **NodeServicesInterface** - Service client/server management
6. **NodeTimersInterface** - Timer management
7. **NodeLoggingInterface** - Logging functionality
8. **NodeClockInterface** - Time and clock management
9. **NodeWaitablesInterface** - Custom waitable management
10. **NodeTimeSourceInterface** - Time source configuration

Each interface is accessible via getter methods such as `get_node_base_interface()`, `get_node_topics_interface()`, etc.

---

## 2. Detailed Interface Comparison

### 2.1 NodeBaseInterface

**Purpose**: Core node identity, context management, callback groups

**Important**: `agnocast::node_interfaces::NodeBase` **inherits from** `rclcpp::node_interfaces::NodeBaseInterface`.

| Feature | agnocast::Node | Support Level | Planned | Notes |
|---------|----------------|---------------|---------|-------|
| `get_name()` | ✓ | **Full Support** | - | |
| `get_namespace()` | ✓ | **Full Support** | - | |
| `get_fully_qualified_name()` | ✓ | **Full Support** | - | |
| `get_context()` | ✓ | **Full Support** | - | Returns rclcpp::Context::SharedPtr passed when using Component Container. Returns nullptr if not loaded in Component Container |
| `get_rcl_node_handle()` | ✗ | **Throws Exception** | No | Throws `std::runtime_error` because DDS is not used |
| `get_shared_rcl_node_handle()` | ✗ | **Throws Exception** | No | Throws `std::runtime_error` because DDS is not used |
| `create_callback_group()` | ✓ | **Full Support** | - | |
| `get_default_callback_group()` | ✓ | **Full Support** | - | |
| `callback_group_in_node()` | ✓ | **Full Support** | - | |
| `for_each_callback_group()` | ✓ | **Full Support** | - | |
| `get_notify_guard_condition()` | ✗ | **Throws Exception** | Yes | Not needed as agnocast uses epoll instead of condition variables, but planned for loading into Component Container |
| `get_associated_with_executor_atomic()` | ✓ | **Full Support** | - | |
| `resolve_topic_or_service_name()` | ✓ | **Full Support** | - | Used by NodeTopics and (future) NodeServices |
| `get_use_intra_process_default()` | ⚠ | **API Only** | No | Returns value while logging a warning. agnocast uses its own shared memory IPC, so rclcpp's intra_process_communication is not used. |
| `get_enable_topic_statistics_default()` | ⚠ | **API Only** | Yes | Returns value passed from NodeOptions. |

---

### 2.2 NodeTopicsInterface

**Purpose**: Publisher and Subscription management

| Feature | agnocast::Node | Support Level | Planned | Notes |
|---------|----------------|---------------|---------|-------|
| `resolve_topic_name()` | ✓ | **Full Support** | - | |
| `get_node_base_interface()` | ✓ | **Full Support** | - | |
| `create_publisher()` | ✗ | **Throws Exception** | No | Use `agnocast::create_publisher()` or `agnocast::Node::create_publisher()` |
| `add_publisher()` | ✗ | **Throws Exception** | No | Uses agnocast's own Publisher management |
| `create_subscription()` | ✗ | **Throws Exception** | No | Use `agnocast::create_subscription()` or `agnocast::Node::create_subscription()` |
| `add_subscription()` | ✗ | **Throws Exception** | No | Uses agnocast's own Subscription management |
| `get_node_timers_interface()` | ✗ | **Throws Exception** | TBD | Timer interface is not supported |

---

### 2.3 NodeParametersInterface

**Purpose**: Parameter declaration, access, and management

**Important**: `agnocast::node_interfaces::NodeParameters` **inherits from** `rclcpp::node_interfaces::NodeParametersInterface`.

| Feature | agnocast::Node | Support Level | Planned | Notes |
|---------|----------------|---------------|---------|-------|
| `declare_parameter(name, default_value)` | ✓ | **Full Support** | - | |
| `declare_parameter(name, type)` | ✓ | **Full Support** | - | |
| `undeclare_parameter()` | ✓ | **Full Support** | - | |
| `has_parameter()` | ✓ | **Full Support** | - | |
| `get_parameter(name)` | ✓ | **Full Support** | - | |
| `get_parameter(name, param&)` | ✓ | **Full Support** | - | |
| `get_parameters(names)` | ✓ | **Full Support** | - | |
| `get_parameter_overrides()` | ✓ | **Full Support** | - | |
| `set_parameters()` | ✓ | **Full Support** | - | Parameter events not triggered (see below) |
| `set_parameters_atomically()` | ✓ | **Full Support** | - | Parameter events not triggered (see below) |
| `get_parameters_by_prefix()` | ✓ | **Full Support** | - | |
| `describe_parameters()` | ✓ | **Full Support** | - | |
| `get_parameter_types()` | ✓ | **Full Support** | - | |
| `list_parameters()` | ✓ | **Full Support** | - | |
| `add_on_set_parameters_callback()` | ✓ | **Full Support** | - | |
| `remove_on_set_parameters_callback()` | ✓ | **Full Support** | - | |

**Other differences from rclcpp::NodeParameters**:

| Item | rclcpp::NodeParameters | agnocast::NodeParameters | Planned |
|------|------------------------|-------------------------|---------|
| Parameter Service | Creates `ParameterService` (optional) | None | Yes |
| Parameter Event Publishing | Publishes to `/parameter_events` | None | TBD |

---

### 2.4 NodeClockInterface

**Purpose**: Clock access

| Feature | agnocast::Node | Support Level | Planned | Notes |
|---------|----------------|---------------|---------|-------|
| `get_clock()` | ✓ | **Full Support** | - | |

---

### 2.5 NodeTimeSourceInterface

**Purpose**: Time source configuration for simulated time (`use_sim_time`)

| Feature | agnocast::Node | Support Level | Planned | Notes |
|---------|----------------|---------------|---------|-------|
| `use_sim_time` parameter | ✓ | **Full Support** | - | Declared at node construction |
| `/clock` subscription | ✓ | **Full Support** | - | Subscribes when `use_sim_time:=true` |
| ROS time override | ✓ | **Full Support** | - | Uses `rcl_*_ros_time_override()` functions |

**Unimplemented Features** (compared to rclcpp::TimeSource):

| Feature | Impact | Notes |
|---------|--------|-------|
| Multiple clocks | Low | `agnocast::Node` uses single clock; rarely needed |
| Message caching | Low | Only matters when attaching clocks after `/clock` messages arrive |
| Dynamic parameter change | Low | Typically `use_sim_time` is set at launch time and not changed at runtime (e.g., Autoware's logging_simulation) |

---

### 2.6 Other Interfaces

The following interfaces are all **unsupported**. agnocast::Node does not implement these interfaces.

| Interface | Support Status | Planned | Notes |
|-----------|---------------|---------|-------|
| NodeGraphInterface | Unsupported | No | DDS is not used |
| NodeLoggingInterface | Unsupported | TBD | `get_logger()` is provided as a direct method |
| NodeServicesInterface | Unsupported | TBD | Uses agnocast's own service functionality |
| NodeTimersInterface | Unsupported | Yes | |
| NodeWaitablesInterface | Unsupported | TBD | |

---

## 3. Argument Parsing and Name Resolution

### 3.1 Command Line Argument Support

agnocast uses `rcl_parse_arguments()` to parse command line arguments.
This provides the same argument parsing functionality as rcl.

| Argument Type | agnocast | Support Level | Planned | Notes |
|---------------|----------|---------------|---------|-------|
| `--ros-args` | ✓ | **Full Support** | - | ROS arguments start marker |
| `-r topic:=new_topic` | ✓ | **Full Support** | - | Topic name remapping |
| `-r __ns:=/namespace` | ✓ | **Full Support** | - | Change node namespace |
| `-r __node:=name` | ✓ | **Full Support** | - | Change node name |
| `-p param:=value` | ✓ | **Full Support** | - | Set parameter value |
| `--params-file file.yaml` | ✓ | **Full Support** | - | Load parameters from YAML file |
| `--` (end marker) | ✓ | **Full Support** | - | ROS arguments end marker |
| `-r node:old:=new` | ✓ | **Full Support** | - | Node-specific remapping |
| `--log-level` | ✗ | **Unsupported** | TBD | Set log level |
| `--enable-rosout-logs` | ✗ | **Unsupported** | TBD | Enable logging to rosout |
| `--disable-external-lib-logs` | ✗ | **Unsupported** | TBD | Disable external library logs |
| `--disable-stdout-logs` | ✗ | **Unsupported** | TBD | Disable stdout logging |
| `-e` (enclave) | ✗ | **Unsupported** | TBD | Specify security enclave |

### 3.2 Parameter Override Resolution

agnocast resolves parameter overrides with `resolve_parameter_overrides()`.
This provides equivalent functionality to rclcpp's `rclcpp::detail::resolve_parameter_overrides()`.

**Priority Order** (highest priority first):

1. `parameter_overrides` (from NodeOptions::parameter_overrides())
2. `local_args` (from NodeOptions::arguments())
3. `global_args` (from command line)

### 3.3 Topic Name Resolution

| Feature | agnocast | Support Level | Planned |
|---------|----------|---------------|---------|
| Private topic (`~topic`) | ✓ | **Full Support** | - |
| Relative topic (`topic`) | ✓ | **Full Support** | - |
| Absolute topic (`/topic`) | ✓ | **Full Support** | - |
| Substitution (`{node}`) | ✓ | **Full Support** | - |
| Substitution (`{ns}`, `{namespace}`) | ✓ | **Full Support** | - |
| Topic remapping | ✓ | **Full Support** | - |
| Service remapping | ⚠ | **Unused** | Yes |

---

## 4. Node Construction Patterns

### 4.1 Constructors

**agnocast::Node** provides the following two constructors, same as rclcpp::Node:

```cpp
// Constructor 1: Node name only (NodeOptions optional)
explicit Node(
  const std::string & node_name,
  const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

// Constructor 2: Node name + namespace (NodeOptions optional)
explicit Node(
  const std::string & node_name,
  const std::string & namespace_,
  const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
```

### 4.2 Direct API Comparison between rclcpp::Node and agnocast::Node

The following tables compare methods that are **directly defined** in each class.
"Directly defined" means methods defined in the class itself, not those accessible only via interfaces.

#### Basic Information & Logging

| API | rclcpp::Node | agnocast::Node | Notes |
|-----|:------------:|:--------------:|-------|
| `get_name()` | ✓ | ✓ | |
| `get_namespace()` | ✓ | ✓ | |
| `get_fully_qualified_name()` | ✓ | ✓ | |
| `get_logger()` | ✓ | ✓ | |

#### Callback Groups

| API | rclcpp::Node | agnocast::Node | Notes |
|-----|:------------:|:--------------:|-------|
| `create_callback_group()` | ✓ | ✓ | |
| `for_each_callback_group()` | ✓ | ✓ | |

#### Publisher/Subscription

| API | rclcpp::Node | agnocast::Node | Notes |
|-----|:------------:|:--------------:|-------|
| `create_publisher<MessageT>()` | ✓ | ✓ | Return type differs (rclcpp::Publisher vs agnocast::Publisher) |
| `create_subscription<MessageT>()` | ✓ | ✓ | Return type differs (rclcpp::Subscription vs agnocast::Subscription) |
| `create_generic_publisher()` | ✓ | ✗ | |
| `create_generic_subscription()` | ✓ | ✗ | |

#### Parameters

| API | rclcpp::Node | agnocast::Node | Notes |
|-----|:------------:|:--------------:|-------|
| `declare_parameter()` | ✓ | ✓ | |
| `declare_parameters()` | ✓ | ✗ | |
| `undeclare_parameter()` | ✓ | ✓ | |
| `has_parameter()` | ✓ | ✓ | |
| `get_parameter()` | ✓ | ✓ | |
| `get_parameter_or()` | ✓ | ✗ | |
| `get_parameters()` | ✓ | ✓ | |
| `set_parameter()` | ✓ | ✓ | |
| `set_parameters()` | ✓ | ✓ | |
| `set_parameters_atomically()` | ✓ | ✓ | |
| `describe_parameter()` | ✓ | ✓ | |
| `describe_parameters()` | ✓ | ✓ | |
| `get_parameter_types()` | ✓ | ✓ | |
| `list_parameters()` | ✓ | ✓ | |
| `add_on_set_parameters_callback()` | ✓ | ✓ | |
| `remove_on_set_parameters_callback()` | ✓ | ✓ | |

#### Timers, Services, Clients

| API | rclcpp::Node | agnocast::Node | Notes |
|-----|:------------:|:--------------:|-------|
| `create_wall_timer()` | ✓ | ✗ | |
| `create_client<ServiceT>()` | ✓ | ✗ | Use `agnocast::create_client()` |
| `create_service<ServiceT>()` | ✓ | ✗ | Use `agnocast::create_service()` |

#### Graph API (ROS Network Discovery)

| API | rclcpp::Node | agnocast::Node |
|-----|:------------:|:--------------:|
| `get_node_names()` | ✓ | ✗ |
| `get_topic_names_and_types()` | ✓ | ✗ |
| `get_service_names_and_types()` | ✓ | ✗ |
| `get_service_names_and_types_by_node()` | ✓ | ✗ |
| `count_publishers()` | ✓ | ✗ |
| `count_subscribers()` | ✓ | ✗ |
| `get_publishers_info_by_topic()` | ✓ | ✗ |
| `get_subscriptions_info_by_topic()` | ✓ | ✗ |
| `get_graph_event()` | ✓ | ✗ |
| `wait_for_graph_change()` | ✓ | ✗ |

#### Time & Clock

| API | rclcpp::Node | agnocast::Node | Notes |
|-----|:------------:|:--------------:|-------|
| `get_clock()` | ✓ | ✓ | |
| `now()` | ✓ | ✓ | |

#### Node Interface Access

| API | rclcpp::Node | agnocast::Node | Notes |
|-----|:------------:|:--------------:|-------|
| `get_node_base_interface()` | ✓ | ✓ | |
| `get_node_topics_interface()` | ✓ | ✓ | |
| `get_node_parameters_interface()` | ✓ | ✓ | |
| `get_node_clock_interface()` | ✓ | ✗ | |
| `get_node_graph_interface()` | ✓ | ✗ | |
| `get_node_logging_interface()` | ✓ | ✗ | |
| `get_node_timers_interface()` | ✓ | ✗ | |
| `get_node_services_interface()` | ✓ | ✗ | |
| `get_node_waitables_interface()` | ✓ | ✗ | |
| `get_node_time_source_interface()` | ✓ | ✗ | |

#### Sub-nodes & Namespaces

| API | rclcpp::Node | agnocast::Node | Notes |
|-----|:------------:|:--------------:|-------|
| `get_sub_namespace()` | ✓ | ✗ | Sub-nodes not supported |
| `get_effective_namespace()` | ✓ | ✗ | Sub-nodes not supported |
| `create_sub_node()` | ✓ | ✗ | Sub-nodes not supported |

#### Other

| API | rclcpp::Node | agnocast::Node | Notes |
|-----|:------------:|:--------------:|-------|
| `get_node_options()` | ✓ | ✗ | |

### 4.3 Behavioral Differences

| Aspect | agnocast | Notes |
|--------|----------|-------|
| Global context required | ✗ (Optional) | Works without agnocast::init() |
| NodeOptions support | ✓ | Supports parameter_overrides, context, arguments, etc. |
| Sub-nodes | ✗ | agnocast does not support sub-nodes |
| Lifecycle nodes | ✗ | Not applicable |

---

## 5. Composable Node Considerations

`agnocast::Node` implements `rclcpp::node_interfaces::NodeBaseInterface`, so it can be loaded as a Composable Node into a Component Container.

**Note**: While `agnocast::Node` can be loaded into a Component Container, the Component Container itself becomes a DDS participant. Therefore, the performance maximization benefits—which are the primary purpose of using `agnocast::Node`—cannot be fully realized when using a Component Container.

---

## 6. Dependencies on rcl/rclcpp

agnocast::Node uses the following rcl/rclcpp functions, data structures, and classes:

**rcl Functions**:

- `rcl_parse_arguments()` - Command line argument parsing
- `rcl_arguments_copy()` - Argument copying
- `rcl_expand_topic_name()` - Topic name expansion
- `rcl_remap_topic_name()` - Topic remapping
- `rcl_remap_node_name()` - Node name remapping
- `rcl_remap_node_namespace()` - Namespace remapping
- `rcl_arguments_get_param_overrides()` - Get parameter overrides
- `rcl_arguments_fini()` - Argument cleanup
- `rcl_get_default_allocator()` - Get default allocator
- `rcl_enable_ros_time_override()` - Enable ROS time override for simulated time
- `rcl_disable_ros_time_override()` - Disable ROS time override
- `rcl_set_ros_time_override()` - Set ROS time override value

**rcl Data Structures**:

- `rcl_arguments_t` - Parsed arguments

**rclcpp Functions**:

- `rclcpp::detail::declare_qos_parameters()` - Declares QoS-related parameters and applies overrides. This function only requires `NodeParametersInterface` and internally calls `declare_parameter`/`get_parameter`, so it works with `agnocast::Node`. When `QosOverridingOptions` is specified, QoS policies are automatically applied from parameters using the naming convention `qos_overrides.<topic>.<entity>.<policy>` (e.g., `qos_overrides./my_topic.subscription.durability`), without requiring explicit parameter declaration in user code.
- `rclcpp::exceptions::throw_from_rcl_error()` - Throw exception from rcl error

**rclcpp Classes/Interfaces**:

- `rclcpp::Context` - Context management
- `rclcpp::CallbackGroup` - Callback group management
- `rclcpp::Logger` - Logging
- `rclcpp::Parameter` / `rclcpp::ParameterValue` - Parameter management
- `rclcpp::QoS` - QoS configuration
- `rclcpp::QosOverridingOptions` - QoS override configuration via parameters
- `rclcpp::NodeOptions` - Node construction options
- `rclcpp::node_interfaces::NodeBaseInterface` - Node base interface (inherited)
- `rclcpp::node_interfaces::NodeTopicsInterface` - Node topics interface (inherited)
- `rclcpp::node_interfaces::NodeParametersInterface` - Node parameters interface (inherited)
- `rclcpp::node_interfaces::ParameterMutationRecursionGuard` - RAII guard to prevent recursive parameter modifications from within callbacks
- `rclcpp::node_interfaces::ParameterInfo` - Parameter value and descriptor storage
- `rclcpp::node_interfaces::OnSetParametersCallbackHandle` - Handle for parameter set callbacks
- `rclcpp::node_interfaces::NodeClockInterface` - Node clock interface (inherited)
- `rclcpp::node_interfaces::NodeTimeSourceInterface` - Node time source interface (inherited)
- `rclcpp::Clock` - Clock management
- `rclcpp::Time` - Time representation

**Message Types**:

- `rosgraph_msgs::msg::Clock` - Clock message for simulated time
