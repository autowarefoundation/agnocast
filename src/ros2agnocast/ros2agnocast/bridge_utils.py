"""Shared utilities for A2R bridge activation.

Used by both :mod:`ros2agnocast._bridged_ros2cli` (CLI bridge commands) and
:mod:`ros2agnocast._a2r_bridge_activator` (background activator for bag recording etc.).
"""

from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def load_msg_class(type_name: str):
    """Load and return the ROS message class for type_name.

    Returns the message class on success, or None if the class cannot be
    loaded (error is printed to stdout).
    """
    try:
        from rosidl_runtime_py.utilities import get_message
        return get_message(type_name)
    except Exception as e:
        print(
            "ERROR: Could not load message class for type '%s': %s" % (type_name, e))
        return None


def dummy_sub_qos() -> QoSProfile:
    """QoS profile for a dummy ROS 2 subscription used to trigger A2R bridges."""
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def create_dummy_subscription(node, topic_name: str, msg_type):
    """Create and return a dummy ROS 2 subscription to trigger the A2R bridge.

    The subscription only needs to exist; no spinning is required to keep it alive.
    """
    return node.create_subscription(msg_type, topic_name, lambda _msg: None, dummy_sub_qos())
