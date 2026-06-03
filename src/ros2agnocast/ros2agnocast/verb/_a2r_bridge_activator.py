import os
import threading

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.logging import LoggingSeverity
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from ros2agnocast.discovery import GOSSIP_TOPIC, gossip_qos
from ros2agnocast.verb._bridged_ros2cli import load_msg_class
from ros2agnocast_discovery_msgs.msg import AgnocastDaemonState


class A2rBridgeActivator:
    """Watch /_agnocast_discovery and spawn dummy ROS2 subscriptions to activate A2R bridges.

    One dummy subscription per unique topic_name is created on first discovery and kept
    alive for the lifetime of this object, so that:

    - The A2R bridge for each Agnocast publisher topic is started immediately on discovery.
    - Subscriptions survive publisher restarts (e.g. bag recording is not interrupted).

    Uses a private rclpy context to avoid interfering with the caller's default context.

    Usage::

        with A2rBridgeActivator() as activator:
            # ... record or otherwise consume Agnocast topics via ROS2 ...
    """

    def __init__(self, log_level: str = 'info') -> None:
        self._ctx = rclpy.Context()
        self._node = None
        self._thread = None
        self._lock = threading.Lock()
        self._active_subs: dict = {}  # topic_name -> Subscription
        self._log_level = log_level

    def start(self) -> None:
        """Initialize rclpy context, create node, subscribe to gossip, and start spin thread."""
        rclpy.init(context=self._ctx)
        self._node = rclpy.create_node(
            '_ros2agnocast_a2r_activator_%d' % os.getpid(),
            context=self._ctx,
        )
        severity = LoggingSeverity[self._log_level.upper()]
        self._node.get_logger().set_level(severity)
        self._node.create_subscription(
            AgnocastDaemonState,
            GOSSIP_TOPIC,
            self._on_discovery,
            gossip_qos(),
        )
        self._thread = threading.Thread(
            target=self._spin,
            daemon=True,
            name='a2r_bridge_activator',
        )
        self._thread.start()

    def _spin(self) -> None:
        executor = SingleThreadedExecutor(context=self._ctx)
        executor.add_node(self._node)
        try:
            executor.spin()
        except Exception:
            pass
        finally:
            executor.remove_node(self._node)
            try:
                self._node.destroy_node()
            except Exception:
                pass

    def _on_discovery(self, msg: AgnocastDaemonState) -> None:
        with self._lock:
            for topic in msg.topics:
                if topic.publishers and topic.topic_name not in self._active_subs:
                    self._spawn_subscription(topic.topic_name, topic.type_name)

    def _spawn_subscription(self, topic_name: str, type_name: str) -> None:
        """Create a dummy ROS2 subscription to trigger the A2R bridge. Called with _lock held."""
        msg_type = load_msg_class(type_name)
        if msg_type is None:
            return
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        sub = self._node.create_subscription(msg_type, topic_name, lambda _msg: None, qos)
        self._active_subs[topic_name] = sub
        self._node.get_logger().debug("A2R bridge activated: '%s' (%s)" % (topic_name, type_name))

    def stop(self) -> None:
        """Shut down the gossip spin thread and release resources."""
        rclpy.try_shutdown(context=self._ctx)
        if self._thread is not None:
            self._thread.join(timeout=5.0)

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *_):
        self.stop()
