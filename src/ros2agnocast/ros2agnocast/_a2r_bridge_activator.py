import os
import sys
import threading
import time
from typing import Callable

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.logging import LoggingSeverity

from ros2agnocast.bridge_utils import create_dummy_subscription, load_msg_class
from ros2agnocast.discovery import GOSSIP_TOPIC, gossip_qos
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

    BRIDGE_CHECK_INTERVAL = 2.0   # seconds between diagnostic checks
    BRIDGE_SPAWN_TIMEOUT = 5.0    # seconds to wait before warning about missing bridge

    def __init__(
        self,
        log_level: str = 'info',
        should_activate: Callable[[str, str], bool] = lambda topic_name, type_name: True,
    ) -> None:
        # should_activate(topic_name, type_name) -> bool
        # Called for each discovered topic to decide whether to activate its A2R bridge.
        self._should_activate = should_activate
        self._ctx = rclpy.Context()
        self._node = None
        self._thread = None
        self._gossip_sub = None
        self._timer = None
        self._active_subs: dict = {}             # topic_name -> Subscription
        self._awaiting_bridge_topics: dict = {}   # topic_name -> monotonic time, until confirmed or timed out
        self._log_level = log_level

    def start(self) -> None:
        """Initialize rclpy context, create node, subscribe to gossip, and start spin thread."""
        rclpy.init(context=self._ctx)
        self._node = rclpy.create_node(
            '_ros2agnocast_a2r_activator_%d' % os.getpid(),
            context=self._ctx,
        )
        severity = getattr(LoggingSeverity, self._log_level.upper(), LoggingSeverity.INFO)
        self._node.get_logger().set_level(severity)
        self._gossip_sub = self._node.create_subscription(
            AgnocastDaemonState,
            GOSSIP_TOPIC,
            self._on_discovery,
            gossip_qos(),
        )
        self._timer = self._node.create_timer(self.BRIDGE_CHECK_INTERVAL, self._check_bridges)
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
        except Exception as e:
            if rclpy.ok(context=self._ctx):
                self._node.get_logger().error(
                    f'A2rBridgeActivator spin thread exited with error: {e}')
        finally:
            executor.remove_node(self._node)
            try:
                self._node.destroy_node()
            except Exception:
                pass

    def _on_discovery(self, msg: AgnocastDaemonState) -> None:
        # No lock needed: both _on_discovery and _check_bridges are called from the same
        # SingleThreadedExecutor spin thread, so they never execute concurrently.
        for topic in msg.topics:
            if topic.topic_name.startswith('/AGNOCAST_SRV_'):
                continue
            try:
                if not self._should_activate(topic.topic_name, topic.type_name):
                    continue
            except Exception as e:
                self._node.get_logger().warning(
                    "should_activate raised an exception for topic '%s' (%s): %s; activating anyway"
                    % (topic.topic_name, topic.type_name, e)
                )
            if topic.publishers and topic.topic_name not in self._active_subs:
                self._spawn_subscription(topic.topic_name, topic.type_name)

    def _spawn_subscription(self, topic_name: str, type_name: str) -> None:
        """Create a dummy ROS2 subscription to trigger the A2R bridge."""
        msg_type = load_msg_class(type_name)
        if msg_type is None:
            return
        sub = create_dummy_subscription(self._node, topic_name, msg_type)
        self._active_subs[topic_name] = sub
        self._awaiting_bridge_topics[topic_name] = time.monotonic()
        self._node.get_logger().debug("A2R bridge requested: '%s' (%s)" % (topic_name, type_name))

    def _check_bridges(self) -> None:
        """Timer callback: warn if A2R bridge publishers have not appeared within the timeout."""
        now = time.monotonic()
        to_remove = []
        for topic_name, created_at in self._awaiting_bridge_topics.items():
            if self._node.count_publishers(topic_name) > 0:
                to_remove.append(topic_name)
            elif now - created_at >= self.BRIDGE_SPAWN_TIMEOUT:
                self._node.get_logger().warning(
                    "A2R bridge has not been created for topic '%s' "
                    "(%d seconds elapsed since subscription was created)"
                    % (topic_name, int(now - created_at))
                )
                to_remove.append(topic_name)

        for topic_name in to_remove:
            self._awaiting_bridge_topics.pop(topic_name, None)

    def stop(self) -> None:
        """Shut down the gossip spin thread and release resources."""
        rclpy.try_shutdown(context=self._ctx)
        if self._thread is not None:
            self._thread.join(timeout=5.0)
            if self._thread.is_alive():
                print(
                    '[WARN] [ros2agnocast]: A2rBridgeActivator spin thread did not stop within 5 seconds',
                    file=sys.stderr,
                )

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *_):
        self.stop()
