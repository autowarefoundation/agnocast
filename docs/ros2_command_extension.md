## How to use ros2 command for Agnocast

Currently, Agnocast supports the following `ros2` commands:

- `ros2 topic list`
- `ros2 topic info /topic_name`
- `ros2 node list`
- `ros2 node info /node_name`

### Topic List

To list all topics including Agnocast, use `ros2 topic list_agnocast`.

If a topic is Agnocast enabled, it will be shown with a "(Agnocast enabled)" suffix.

```bash
$ ros2 topic list_agnocast
/topic_name1
/topic_name2 (Agnocast enabled)
/topic_name3
```

If you want to get only Agnocast related topics, use `ros2 topic list_agnocast | grep Agnocast`:

```bash
$ ros2 topic list_agnocast | grep Agnocast
/topic_name2 (Agnocast enabled)
```

If an Agnocast topic is bridged to ROS 2 publishers or subscribers, it is shown with the "(Agnocast enabled, bridged)" suffix.

```bash
$ ros2 topic list_agnocast
/topic_name1 (Agnocast enabled)
/topic_name2 (Agnocast enabled, bridged)
/topic_name3
```

This does not necessarily mean that a bridge process is currently running. If there is only one Agnocast publisher and no ROS 2 subscriber, the topic will be displayed simply as "(Agnocast enabled)" rather than including the "bridged" suffix. The "bridged" status indicates that communication has been successfully established between Agnocast and ROS 2.

#### Table 1: Pub/Sub situations and display names

| pub | sub | bridge | display |
| :--- | :--- | :--- | :--- |
| rclcpp::publisher | rclcpp::subscription | off / standard / performance | /my_topic |
| agnocast::publisher | rclcpp::subscription | off | /my_topic (WARN: Agnocast and ROS 2 endpoints exist but bridge is not active) |
| agnocast::publisher | rclcpp::subscription | standard / performance | /my_topic (Agnocast enabled, bridged) |
| rclcpp::publisher | agnocast::subscription | off | /my_topic (WARN: Agnocast and ROS 2 endpoints exist but bridge is not active) |
| rclcpp::publisher | agnocast::subscription | standard / performance | /my_topic (Agnocast enabled, bridged) |
| agnocast::publisher | agnocast::subscription | off / standard / performance | /my_topic (Agnocast enabled) |
| agnocast::publisher | non | off / standard / performance | /my_topic (Agnocast enabled) |
| non | agnocast::subscription | off / standard / performance | /my_topic (Agnocast enabled) |

#### Notes

- If an Agnocast topic and a ROS 2 topic share the same name but have different message types, the status will still be displayed as (Agnocast enabled, bridged). However, in this case, the actual communication will not be established.

### Topic Info

To show the topic info including Agnocast, use `ros2 topic info_agnocast /topic_name`.

```bash
$ ros2 topic info_agnocast /my_topic
Type: agnocast_sample_interfaces/msg/DynamicSizeArray
ROS 2 Publisher count: 0
Agnocast Publisher count: 1
ROS 2 Subscription count: 0
Agnocast Subscription count: 1
```

For more details, use `--verbose` or `-v`.

```bash
$ ros2 topic info_agnocast /my_topic -v
Type: agnocast_sample_interfaces/msg/DynamicSizeArray

ROS 2 Publisher count: 0
Agnocast Publisher count: 1

Node name: talker_node
Node namespace: /
Topic type: agnocast_sample_interfaces/msg/DynamicSizeArray
Endpoint type: PUBLISHER (Agnocast enabled)
QoS profile:
  History (Depth): KEEP_LAST (1)
  Durability: VOLATILE

ROS 2 Subscription count: 0
Agnocast Subscription count: 1

Node name: listener_node
Node namespace: /
Topic type: agnocast_sample_interfaces/msg/DynamicSizeArray
Endpoint type: SUBSCRIPTION (Agnocast enabled)
QoS profile:
  History (Depth): KEEP_LAST (1)
  Durability: VOLATILE
```

#### Debug Mode

By default, `ros2 topic info_agnocast` hides internal bridge nodes and endpoints to provide a cleaner view. To include these internal details, use the `--debug` or `-d` flag.

```bash
$ ros2 topic info_agnocast /my_topic -d
Type: agnocast_sample_interfaces/msg/DynamicSizeArray
ROS 2 Publisher count: 1
Agnocast Publisher count: 2
ROS 2 Subscription count: 1
Agnocast Subscription count: 2
```

You can combine `-d` with `-v` for full verbose output including bridge node details:

```bash
$ ros2 topic info_agnocast /my_topic -v -d
Type: agnocast_sample_interfaces/msg/DynamicSizeArray

ROS 2 Publisher count: 1
Agnocast Publisher count: 2

Node name: agnocast_bridge_node_86050
Node namespace: /
Topic type: agnocast_sample_interfaces/msg/DynamicSizeArray
Endpoint type: PUBLISHER
GID: 01.10.b2.d2.ef.55.13.0d.74.cc.0c.e6.00.00.16.03.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: RELIABLE
  History (Depth): KEEP_LAST (10)
  Durability: TRANSIENT_LOCAL
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite

Node name: agnocast_bridge_node_86050
Node namespace: /
Topic type: agnocast_sample_interfaces/msg/DynamicSizeArray
Endpoint type: PUBLISHER (Agnocast enabled)
QoS profile:
  History (Depth): KEEP_LAST (10)
  Durability: TRANSIENT_LOCAL

Node name: talker_node
Node namespace: /
Topic type: agnocast_sample_interfaces/msg/DynamicSizeArray
Endpoint type: PUBLISHER (Agnocast enabled)
QoS profile:
  History (Depth): KEEP_LAST (1)
  Durability: VOLATILE

ROS 2 Subscription count: 1
Agnocast Subscription count: 2

Node name: agnocast_bridge_node_86050
Node namespace: /
Topic type: agnocast_sample_interfaces/msg/DynamicSizeArray
Endpoint type: SUBSCRIPTION
GID: 01.10.b2.d2.ef.55.13.0d.74.cc.0c.e6.00.00.15.04.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: RELIABLE
  History (Depth): KEEP_LAST (1)
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite

Node name: listener_node
Node namespace: /
Topic type: agnocast_sample_interfaces/msg/DynamicSizeArray
Endpoint type: SUBSCRIPTION (Agnocast enabled)
QoS profile:
  History (Depth): KEEP_LAST (1)
  Durability: VOLATILE

Node name: agnocast_bridge_node_86050
Node namespace: /
Topic type: agnocast_sample_interfaces/msg/DynamicSizeArray
Endpoint type: SUBSCRIPTION (Agnocast enabled)
QoS profile:
  History (Depth): KEEP_LAST (1)
  Durability: VOLATILE
```

### Node List

To list all nodes including those implemented with Agnocast, use `ros2 node list_agnocast`.

Standard ROS 2 nodes are displayed normally, while `agnocast::Node` instances are marked with the "(Agnocast enabled)" suffix.

```bash
$ ros2 node list_agnocast
/ros2_talker_node
/agnocast_listener_node (Agnocast enabled)
```

#### Debug Mode

By default, `ros2 node list_agnocast` hides internal bridge nodes and endpoints to provide a cleaner view. To include these internal details, use the `--debug` or `-d` flag.

```bash
$ ros2 node list_agnocast -d
/ros2_talker_node
/agnocast_bridge_node_86050
/agnocast_listener_node (Agnocast enabled)
```

#### Notes

Detection of agnocast::Node instances depends on the presence of Agnocast-enabled endpoints. A node without at least one Agnocast publisher or subscriber will be omitted from the output.

### Node Info

To show the node info including Agnocast, use `ros2 node info_agnocast /node_name`.

```bash
$ ros2 node info_agnocast /listener_node
  Subscribers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /my_topic: agnocast_sample_interfaces/msg/DynamicSizeArray (Agnocast enabled, bridged)
  Publishers:
    /my_topic2: agnocast_sample_interfaces/msg/DynamicSizeArray (Agnocast enabled)
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
  Service Servers:
    /listener_node/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /listener_node/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /listener_node/get_parameters: rcl_interfaces/srv/GetParameters
    /listener_node/list_parameters: rcl_interfaces/srv/ListParameters
    /listener_node/set_parameters: rcl_interfaces/srv/SetParameters
    /listener_node/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Service Clients:
  Action Servers:
  Action Clients:
```

Similar to `ros2 topic list_agnocast`, the (Agnocast enabled, bridged) suffix in the node info indicates that communication has been successfully established between Agnocast and ROS 2 for that specific topic.

If a topic is Agnocast-enabled but not currently bridged (e.g., there is no corresponding ROS 2 publisher/subscriber or the bridge process is not active), it will be displayed simply as (Agnocast enabled). This allows you to verify the connectivity status of each topic directly from the node's perspective.
