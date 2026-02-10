# Message Filters User Guide

## Feature Support

The following table shows the support status of agnocast message_filters compared to ROS 2 [message_filters](https://github.com/ros2/message_filters).

### Filters

| Filter | Supported | Description |
|---|---|---|
| `Subscriber` | Yes | Wraps an agnocast subscription and feeds messages into the filter chain |
| `PassThrough` | Yes | Re-signals every incoming message unchanged |
| `Synchronizer` | Yes | Policy-based multi-message synchronizer (2–9 inputs) |
| `TimeSynchronizer` | No | Convenience wrapper for `Synchronizer<ExactTime>` |
| `Cache` | No | Time-indexed message history |
| `Chain` | No | Sequential filter pipeline |
| `TimeSequencer` | No | Delays and reorders messages by timestamp |

### Sync Policies

| Policy | Supported | Description |
|---|---|---|
| `ExactTime` | Yes | Matches messages with exactly the same timestamp |
| `ApproximateTime` | Yes | Matches messages with approximately the same timestamp |
| `ApproximateEpsilonTime` | No | Variant of `ApproximateTime` with an epsilon bound |
| `LatestTime` | No | Matches the latest available message from each topic |

## Synchronizer

### Overview

The Synchronizer is a message filter that takes in messages from multiple agnocast topics and outputs them only when it has received a message on each topic with a matching or approximately matching timestamp. This is useful when you need to process data from multiple sensors or sources that publish at different rates.

The Synchronizer is parameterized by a **sync policy** that determines how messages are matched:

| Policy | Description |
|---|---|
| `ExactTime` | Matches messages with exactly the same timestamp |
| `ApproximateTime` | Matches messages with approximately the same timestamp |

The Synchronizer supports 2 to 9 input topics.

## Prerequisites

### TimeStamp Trait

Both sync policies require extracting a `rclcpp::Time` from each message. You must specialize the `message_filters::message_traits::TimeStamp` trait for every message type used with the Synchronizer. Messages with a `header.stamp` field (e.g., `sensor_msgs::msg::Image`, `geometry_msgs::msg::PoseStamped`) are supported out of the box.

## Basic Usage

The typical usage pattern is:

1. Create `Subscriber` objects for each input topic.
2. Create a `Synchronizer` with a sync policy and connect the subscribers.
3. Register a callback that receives the synchronized messages.

### Example: Synchronizing Two Topics with ExactTime

```cpp
#include <agnocast/message_filters/subscriber.hpp>
#include <agnocast/message_filters/synchronizer.hpp>
#include <agnocast/message_filters/sync_policies/exact_time.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

using Image = sensor_msgs::msg::Image;
using CameraInfo = sensor_msgs::msg::CameraInfo;

// Define the policy and synchronizer types
using SyncPolicy = agnocast::message_filters::sync_policies::ExactTime<Image, CameraInfo>;
using Sync = agnocast::message_filters::Synchronizer<SyncPolicy>;

class MyNode : public rclcpp::Node
{
public:
  MyNode() : Node("my_node")
  {
    // 1. Create subscribers
    image_sub_.subscribe(this, "camera/image");
    info_sub_.subscribe(this, "camera/camera_info");

    // 2. Create synchronizer (queue_size = 10)
    sync_ = std::make_unique<Sync>(SyncPolicy(10), image_sub_, info_sub_);

    // 3. Register callback
    sync_->registerCallback(&MyNode::callback, this);
  }

private:
  void callback(
    const agnocast::ipc_shared_ptr<Image const> & image,
    const agnocast::ipc_shared_ptr<CameraInfo const> & info)
  {
    // Process synchronized messages
  }

  agnocast::message_filters::Subscriber<Image> image_sub_;
  agnocast::message_filters::Subscriber<CameraInfo> info_sub_;
  std::unique_ptr<Sync> sync_;
};
```

### Example: Synchronizing Two Topics with ApproximateTime

```cpp
#include <agnocast/message_filters/sync_policies/approximate_time.hpp>

using SyncPolicy = agnocast::message_filters::sync_policies::ApproximateTime<Image, CameraInfo>;
using Sync = agnocast::message_filters::Synchronizer<SyncPolicy>;

// In constructor:
sync_ = std::make_unique<Sync>(SyncPolicy(10), image_sub_, info_sub_);
sync_->registerCallback(&MyNode::callback, this);
```

## Sync Policies

### ExactTime Policy

**Header**: `<agnocast/message_filters/sync_policies/exact_time.hpp>`

**Namespace**: `agnocast::message_filters::sync_policies`

The ExactTime policy matches messages from different topics that have exactly the same timestamp. This is suitable when publishers are guaranteed to stamp messages with identical timestamps (e.g., a single driver node that publishes image and camera info simultaneously).

#### Constructor

```cpp
explicit ExactTime(uint32_t queue_size);
```

- `queue_size`: Maximum number of incomplete message sets to buffer. When exceeded, the oldest sets are dropped.

#### Algorithm

ExactTime maintains an internal map from timestamp to a tuple of messages:

```text
std::map<rclcpp::Time, Tuple>
```

1. When a message arrives on channel `i`, its timestamp is extracted and the message is stored in the tuple at that timestamp.
2. After each insertion, the tuple is checked for completeness: do all channels have a message?
3. If complete, the user callback is invoked with the full set, the tuple is removed, and all tuples with earlier timestamps are dropped (since they can never be completed in order).
4. If the number of buffered tuples exceeds `queue_size`, the oldest tuples are evicted.

#### Drop Callbacks

You can register a callback that is invoked whenever incomplete tuples are discarded:

```cpp
sync.getPolicy()->registerDropCallback(&MyNode::dropCallback, this);
```

The drop callback has the same signature as the main callback.

---

### ApproximateTime Policy

**Header**: `<agnocast/message_filters/sync_policies/approximate_time.hpp>`

**Namespace**: `agnocast::message_filters::sync_policies`

The ApproximateTime policy matches messages whose timestamps are close but not necessarily identical. This is the more commonly used policy in practice, since independent publishers rarely produce exactly matching timestamps.

#### Constructor

```cpp
explicit ApproximateTime(uint32_t queue_size);
```

- `queue_size`: Maximum number of messages to buffer per topic. Must be greater than 0; a value of at least 2 is recommended since a queue size of 1 tends to drop many messages.

#### Configuration

| Method | Description | Default |
|---|---|---|
| `setAgePenalty(double)` | Controls the trade-off between temporal tightness and recency. Higher values prefer newer message sets even if slightly less tightly clustered. Must be >= 0. | 0.1 |
| `setMaxIntervalDuration(rclcpp::Duration)` | Rejects candidate sets whose time span exceeds this duration. | Effectively unlimited |
| `setInterMessageLowerBound(int i, rclcpp::Duration)` | Declares the known minimum interval between consecutive messages on topic `i`. Enables earlier optimality proofs, reducing latency. | 0 (no bound) |

Configuration is set on the policy object before or after constructing the Synchronizer:

```cpp
SyncPolicy policy(10);
policy.setAgePenalty(0.5);
policy.setMaxIntervalDuration(rclcpp::Duration(1, 0));  // 1 second max

Sync sync(policy, sub0, sub1);

// Or configure via getPolicy() after construction:
sync.getPolicy()->setInterMessageLowerBound(0, rclcpp::Duration(0, 33333333));  // ~30 Hz
sync.getPolicy()->setInterMessageLowerBound(1, rclcpp::Duration(0, 100000000)); // ~10 Hz
```

Please see [ApproximateTime Algorithm](./approximate_time_algorithm.md) for details.

## Callback Signatures

The synchronized callback can use any of these parameter types, and they can be mixed:

```cpp
// By const reference (recommended)
void cb(const agnocast::ipc_shared_ptr<MsgA const> & a,
        const agnocast::ipc_shared_ptr<MsgB const> & b);

// By value
void cb(agnocast::ipc_shared_ptr<MsgA const> a,
        agnocast::ipc_shared_ptr<MsgB const> b);

// As MessageEvent (provides receipt time metadata)
void cb(const agnocast::message_filters::MessageEvent<MsgA const> & a,
        const agnocast::message_filters::MessageEvent<MsgB const> & b);
```

Callbacks can be registered as:

```cpp
// Free function or lambda
sync.registerCallback(myFreeFunction);
sync.registerCallback([](const auto & a, const auto & b) { /* ... */ });

// Bound via std::bind
sync.registerCallback(std::bind(&MyClass::callback, this, std::placeholders::_1, std::placeholders::_2));

// Member function pointer (recommended for class methods)
sync.registerCallback(&MyClass::callback, this);
```

## Subscriber

The `agnocast::message_filters::Subscriber<M>` class wraps an agnocast subscription and feeds messages into the Synchronizer.

```cpp
// Construct with node and topic
agnocast::message_filters::Subscriber<Image> sub(node_ptr, "camera/image");

// Construct with raw node pointer
agnocast::message_filters::Subscriber<Image> sub(this, "camera/image");

// Construct with QoS
agnocast::message_filters::Subscriber<Image> sub(this, "camera/image", rmw_qos_profile_sensor_data);

// Construct with SubscriptionOptions
agnocast::message_filters::Subscriber<Image> sub(this, "camera/image", rmw_qos_profile_default, options);

// Default construct, then subscribe later
agnocast::message_filters::Subscriber<Image> sub;
sub.subscribe(this, "camera/image");
```

When using `agnocast::Node` instead of `rclcpp::Node`, specify the node type as the second template parameter:

```cpp
agnocast::message_filters::Subscriber<Image, agnocast::Node> sub(node_ptr, "camera/image");
```

## PassThrough

**Header**: `<agnocast/message_filters/pass_through.hpp>`

The `PassThrough` filter re-signals every incoming message unchanged. It can be connected to any upstream filter or fed messages manually.

```cpp
#include <agnocast/message_filters/pass_through.hpp>

// Connect to an existing filter (e.g., Subscriber)
agnocast::message_filters::Subscriber<Image> sub(this, "camera/image");
agnocast::message_filters::PassThrough<Image> pass(sub);

// Or connect later
agnocast::message_filters::PassThrough<Image> pass;
pass.connectInput(sub);

// Register downstream callback
pass.registerCallback(&MyNode::callback, this);
```
