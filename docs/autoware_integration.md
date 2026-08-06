## How to integrate Agnocast to Autoware

Basically, take a look at the first Autoware integration Pull Request <https://github.com/tier4/autoware.universe_tmp-agnocast/pull/2>

There are two steps for Agnocast to work with Autoware:

### Step 1: Add Agnocast dependencies

For CMakeLists.txt (`target_library` should be replaced with the corresponding target):

```c++
find_package(agnocastlib REQUIRED)
...
ament_target_dependencies(target_library agnocastlib)
target_include_directories(target_library PRIVATE
  ${agnocastlib_INCLUDE_DIRS}
)
```

For launch.xml:

```xml
<env name="LD_PRELOAD" value="libagnocast_heaphook.so"/>
```

For packages.xml:

```xml
<depend>agnocastlib</depend>
```

### Step 2: Replace ROS 2 APIs for Agnocast APIs

The declarations and initializations should be replaced like the following:

```c++
// rclcpp::Publisher<MessageType>::SharedPtr message_pub_;
// message_pub_ = this->create_publisher<MessageType>("/topic_name", rclcpp::QoS{x});

std::shared_ptr<agnocast::Publisher<MessageType>> message_pub_;
message_pub_ = agnocast::create_publisher<MessageType>("/topic_name", rclcpp::QoS{x});
```

```c++
// rclcpp::Subscription<MessageType>::SharedPtr> message_sub_;
// message_sub_ = node_.create_subscription<MessageType>("/topic_name", rclcpp::QoS{x}, callback);

std::shared_ptr<agnocast::Subscription<MessageType>> message_sub_;
message_sub_ = agnocast::create_subscription<MessageType>("/topic_name", rclcpp::QoS{x}, callback);
```

### Keeping a `MessageT::ConstSharedPtr` callback signature

A subscription callback may take the plain ROS 2 `MessageT::ConstSharedPtr` instead of an
`agnocast::ipc_shared_ptr`, so a callback whose signature cannot be changed — a virtual method
overridden downstream, a pluginlib boundary, a third-party callback — needs no edit at all:

```c++
// void on_message(const MessageType::ConstSharedPtr & msg);  // signature is fixed
message_sub_ = agnocast::create_subscription<MessageType>(
  "/topic_name", rclcpp::QoS{x},
  [this](const MessageType::ConstSharedPtr & msg) { on_message(msg); });
```

The payload is not copied: the pointer aliases the shared-memory message and shares ownership with
it. Three things to know before using it:

- **It costs one extra heap allocation per message** (the `std::shared_ptr` control block). Prefer
  `agnocast::ipc_shared_ptr` on hot paths; this form is for the boundaries where you cannot.
- **It may be retained beyond the callback, but it pins one shared-memory entry** for as long as it
  lives. The publisher's entry count can then exceed its QoS depth, and caching messages in an
  unbounded container can exhaust the process mempool (see [shared_memory.md](shared_memory.md)).
- **It must not outlive the `Subscription` that delivered it.** Since members are destroyed in
  reverse declaration order, declare the subscription first so it is destroyed last:

  ```c++
  class Foo : public rclcpp::Node {
    std::shared_ptr<agnocast::Subscription<MessageType>> message_sub_;  // destroyed last
    MessageType::ConstSharedPtr latest_msg_;                            // destroyed first
  };
  ```

`MessageT::SharedPtr` (non-const) and `MessageT::UniquePtr` are rejected at compile time: the
message lives in shared memory that other subscribers read concurrently, and it must not be freed
with `operator delete`.

### Other tips

- Until the subscription callback thread is integrated into ROS 2 executor, all callback functions should be guarded with mutex lock.
- Although Agnocast already has `get_subscription_count()` API, it is not still complete. There is an issue ticket <https://github.com/autowarefoundation/agnocast/issues/181>.
- Agnocast does not support `publish_if_subscribed()` API yet. There is an issue ticket <https://github.com/autowarefoundation/agnocast/issues/164>.
