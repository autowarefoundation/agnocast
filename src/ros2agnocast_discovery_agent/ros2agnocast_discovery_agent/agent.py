"""Per-IPC-namespace discovery agent.

Reads the local Agnocast state via the existing NS-scoped ioctl wrapper
(libagnocast_ioctl_wrapper.so) and publishes it as AgnocastDaemonState on
``/_agnocast_discovery`` so other namespaces and ECUs running ros2agnocast
tooling can observe and make bridge generation decisions.

One daemon process is intended to run per IPC namespace. Lifecycle is the
user's responsibility (systemd unit, ros2 launch include, container
entrypoint, etc.).
"""

import ctypes
import importlib.metadata
import logging
import os
import socket
import sys
import uuid

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, LivelinessPolicy, QoSProfile, ReliabilityPolicy
from rclpy.duration import Duration

from ros2agnocast_discovery_msgs.msg import (
    AgnocastDaemonState,
    AgnocastEndpoint,
    AgnocastTopic,
)


GOSSIP_TOPIC = '/_agnocast_discovery'
SCHEMA_VERSION = 1
PUBLISH_INTERVAL_SEC = 1.0
LIVELINESS_LEASE_SEC = 30.0
# Preferred over /etc/machine-id: avoids the systemd dep and the
# baked-into-image case where every ECU collides on one host_uuid.
BOOT_ID_PATH = '/proc/sys/kernel/random/boot_id'
SELF_IPC_NS_PATH = '/proc/self/ns/ipc'
# Kept separate from TOPIC_NAME_BUFFER_SIZE because the C side (`agnocast_kmod`)
# defines NODE_NAME_BUFFER_SIZE and TOPIC_NAME_BUFFER_SIZE as independent
# constants — they happen to share the value 256 today.
NODE_NAME_BUFFER_SIZE = 256
TOPIC_NAME_BUFFER_SIZE = 256
# How long a remote daemon's last snapshot may sit in _remote_states without
# being refreshed before we drop it. Matches the gossip Liveliness lease so
# DDS-side liveliness loss and local prune happen on the same timescale.
REMOTE_STATE_STALE_SEC = 30.0


class TopicInfoRet(ctypes.Structure):
    """Mirror of ``struct topic_info_ret`` in agnocast_ioctl.hpp."""

    _fields_ = [
        ('node_name', ctypes.c_char * NODE_NAME_BUFFER_SIZE),
        ('qos_depth', ctypes.c_uint32),
        ('qos_is_transient_local', ctypes.c_bool),
        ('qos_is_reliable', ctypes.c_bool),
        ('is_bridge', ctypes.c_bool),
    ]


def _load_ioctl_wrapper():
    """Load libagnocast_ioctl_wrapper.so and set argtypes for the symbols we use."""
    lib = ctypes.CDLL('libagnocast_ioctl_wrapper.so')

    lib.get_agnocast_topics.argtypes = [ctypes.POINTER(ctypes.c_int)]
    lib.get_agnocast_topics.restype = ctypes.POINTER(ctypes.POINTER(ctypes.c_char))
    lib.free_agnocast_topics.argtypes = [
        ctypes.POINTER(ctypes.POINTER(ctypes.c_char)),
        ctypes.c_int,
    ]
    lib.free_agnocast_topics.restype = None

    lib.get_agnocast_sub_nodes.argtypes = [ctypes.c_char_p, ctypes.POINTER(ctypes.c_int)]
    lib.get_agnocast_sub_nodes.restype = ctypes.POINTER(TopicInfoRet)
    lib.get_agnocast_pub_nodes.argtypes = [ctypes.c_char_p, ctypes.POINTER(ctypes.c_int)]
    lib.get_agnocast_pub_nodes.restype = ctypes.POINTER(TopicInfoRet)
    lib.free_agnocast_topic_info_ret.argtypes = [ctypes.POINTER(TopicInfoRet)]
    lib.free_agnocast_topic_info_ret.restype = None

    return lib


def _ioctl_to_endpoint(info: TopicInfoRet) -> AgnocastEndpoint:
    """Convert one ``topic_info_ret`` row to an AgnocastEndpoint msg.

    ``pid`` is best-effort 0 because the existing ioctl does not expose it; the
    bridge decider may fill it via ``/proc`` walk when routing bridge requests.
    """
    ep = AgnocastEndpoint()
    ep.node_name = info.node_name.decode('utf-8', errors='replace')
    ep.pid = 0
    ep.qos_depth = info.qos_depth
    ep.qos_is_transient_local = info.qos_is_transient_local
    ep.qos_is_reliable = info.qos_is_reliable
    ep.is_bridge = info.is_bridge
    return ep


def read_local_topics(lib) -> list:
    """Snapshot the current namespace's Agnocast topics via the ioctl wrapper.

    Returns a list of AgnocastTopic msgs. The ioctl returns only the caller's
    IPC namespace, so the daemon process just being inside that namespace is
    sufficient to scope the result.
    """
    topic_count = ctypes.c_int()
    topic_names_ptr = lib.get_agnocast_topics(ctypes.byref(topic_count))
    topics = []
    if not topic_names_ptr:
        return topics

    try:
        for i in range(topic_count.value):
            topic_name_b = ctypes.cast(topic_names_ptr[i], ctypes.c_char_p).value
            topic_name = topic_name_b.decode('utf-8', errors='replace')

            agnocast_topic = AgnocastTopic()
            agnocast_topic.topic_name = topic_name
            # type_name is best-effort empty here; resolved by future work
            # (procfs/topic_info exposing message_type, or kmod ioctl extension)
            agnocast_topic.type_name = ''
            agnocast_topic.domain_id = 0
            agnocast_topic.publishers = _collect_endpoints(lib.get_agnocast_pub_nodes, lib, topic_name_b)
            agnocast_topic.subscribers = _collect_endpoints(lib.get_agnocast_sub_nodes, lib, topic_name_b)
            topics.append(agnocast_topic)
    finally:
        lib.free_agnocast_topics(topic_names_ptr, topic_count.value)

    return topics


def _collect_endpoints(getter, lib, topic_name_b: bytes) -> list:
    count = ctypes.c_int()
    array = getter(topic_name_b, ctypes.byref(count))
    endpoints = []
    if not array:
        return endpoints
    try:
        for i in range(count.value):
            endpoints.append(_ioctl_to_endpoint(array[i]))
    finally:
        lib.free_agnocast_topic_info_ret(array)
    return endpoints


def _read_host_uuid() -> str:
    """Return the boot UUID; fall back to a random UUID with a WARN log."""
    try:
        with open(BOOT_ID_PATH) as fp:
            return str(uuid.UUID(fp.read().strip()))
    except (OSError, ValueError) as exc:
        fallback = str(uuid.uuid4())
        logging.warning(
            'agnocast_discovery_agent: failed to read %s (%s); '
            'falling back to random host_uuid=%s',
            BOOT_ID_PATH, exc, fallback)
        return fallback


def _read_ipc_ns_inode() -> int:
    """Return the inode number of the daemon's own IPC namespace."""
    return os.stat(SELF_IPC_NS_PATH).st_ino


def _gossip_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        # AUTOMATIC: rclpy (Humble) does not expose ``assert_liveliness`` on
        # Publisher, so MANUAL_BY_TOPIC cannot be driven reliably from Python.
        # AUTOMATIC keeps the publisher alive as long as the participant is up.
        liveliness=LivelinessPolicy.AUTOMATIC,
        liveliness_lease_duration=Duration(seconds=LIVELINESS_LEASE_SEC),
    )


def _read_agent_version() -> str:
    """Return this package's installed version, or '' if running from source."""
    try:
        return importlib.metadata.version('ros2agnocast_discovery_agent')
    except importlib.metadata.PackageNotFoundError:
        return ''


class DiscoveryAgent(Node):
    """rclpy Node that publishes the local Agnocast state every PUBLISH_INTERVAL_SEC.

    Also subscribes to its own gossip topic so the bridge decider can observe
    remote namespace state; the callback records the latest snapshot keyed by
    ``(host_uuid, ipc_ns_inode)``.
    """

    def __init__(self):
        self._host_uuid = _read_host_uuid()
        self._host_hostname = socket.gethostname()
        self._ipc_ns_inode = _read_ipc_ns_inode()
        # Multiple agents may run on one ECU (one per IPC NS); disambiguate.
        host_short = self._host_uuid.replace('-', '')[:8]
        # Force use_sim_time=False so the 1 Hz timer + clock reads are
        # wall-clock regardless of /clock; sim-time playback would otherwise
        # stretch the publish cadence past the liveliness lease window.
        super().__init__(
            f'agnocast_discovery_agent_{host_short}_{self._ipc_ns_inode}',
            parameter_overrides=[Parameter('use_sim_time', value=False)],
        )
        self._lib = _load_ioctl_wrapper()
        self._agnocast_version = _read_agent_version()
        # Independent wall-clock for prune / receive timestamps so they stay
        # consistent even if a future change accidentally enables sim time.
        self._clock = Clock(clock_type=ClockType.SYSTEM_TIME)

        qos = _gossip_qos()
        self._pub = self.create_publisher(AgnocastDaemonState, GOSSIP_TOPIC, qos)
        self._sub = self.create_subscription(
            AgnocastDaemonState, GOSSIP_TOPIC, self._on_remote_state, qos)
        self._remote_states = {}

        self._timer = self.create_timer(PUBLISH_INTERVAL_SEC, self._on_tick)

        self.get_logger().info(
            f'agnocast_discovery_agent up: host_uuid={self._host_uuid} '
            f'hostname={self._host_hostname} ipc_ns_inode={self._ipc_ns_inode} '
            f'version={self._agnocast_version}')

    def _on_tick(self) -> None:
        self._prune_stale_remote_states()
        self.publish_snapshot()

    def _prune_stale_remote_states(
            self, now_sec: float | None = None,
            stale_after_sec: float = REMOTE_STATE_STALE_SEC) -> None:
        """Drop ``_remote_states`` entries whose local arrival time is too old.

        Mirrors the DDS Liveliness lease so dead-peer snapshots don't leak
        memory. Cross-ECU clocks aren't synced, so local arrival time is
        the freshness reference rather than the publisher's stamp.
        """
        if now_sec is None:
            now_sec = self._clock.now().nanoseconds / 1e9
        stale_keys = [
            key for key, (_msg, received_at) in self._remote_states.items()
            if now_sec - received_at > stale_after_sec
        ]
        for key in stale_keys:
            del self._remote_states[key]

    def publish_snapshot(self) -> None:
        """Build and publish the current local AgnocastDaemonState."""
        msg = self.build_state()
        self._pub.publish(msg)

    def build_state(self) -> AgnocastDaemonState:
        msg = AgnocastDaemonState()
        msg.schema_version = SCHEMA_VERSION
        msg.agnocast_version = self._agnocast_version
        msg.host_uuid = self._host_uuid
        msg.host_hostname = self._host_hostname
        msg.ipc_ns_inode = self._ipc_ns_inode
        msg.topics = read_local_topics(self._lib)
        return msg

    def _on_remote_state(self, msg: AgnocastDaemonState) -> None:
        if msg.host_uuid == self._host_uuid and msg.ipc_ns_inode == self._ipc_ns_inode:
            return
        received_at = self._clock.now().nanoseconds / 1e9
        self._remote_states[(msg.host_uuid, msg.ipc_ns_inode)] = (msg, received_at)

    @property
    def remote_states(self) -> dict:
        """Map of ``(host_uuid, ipc_ns_inode)`` to ``(msg, received_at_sec)``."""
        return self._remote_states


def main(argv=None) -> int:
    rclpy.init(args=argv)
    node = DiscoveryAgent()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
