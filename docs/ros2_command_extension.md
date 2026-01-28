## How to use ros2 command for Agnocast

Currently, Agnocast supports the following `ros2` commands:

- `ros2 topic list`
- `ros2 topic info /topic_name`
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

### Topic Debug Mode

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

### Node Info

To show the node info including Agnocast, use `ros2 node info_agnocast /node_name`.

```bash
$ ros2 node info_agnocast /listener_node
  Subscribers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /my_topic: agnocast_sample_interfaces/msg/DynamicSizeArray (Agnocast enabled)
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

### Node Debug Mode

Use the `--debug` or `-d` flag to show additional information about bridged endpoints. When enabled, topics that are connected through a bridge node will display "(Agnocast enabled, bridged)".

```bash
$ ros2 node info_agnocast /listener_node -d
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
