"""Per-IPC-namespace discovery agent.

Reads the local Agnocast state via the existing NS-scoped ioctl wrapper
(libagnocast_ioctl_wrapper.so) and publishes it as AgnocastDaemonState on
``/_agnocast_discovery`` so other namespaces and ECUs running ros2agnocast
tooling can observe and make bridge generation decisions.

One daemon process is intended to run per (IPC namespace, ROS_DOMAIN_ID):
gossip is published on the agent's DDS domain, so a namespace shared by
launches in different domains needs one agent per domain. To make duplicate
launches (e.g. one node spawning the agent and another ``ros2 launch``
session also trying to spawn one) safe, the agent acquires an
``flock(2)``-based singleton lock on a per-(namespace, domain) file before
starting; subsequent instances in the same (namespace, domain) detect the
held lock and exit cleanly (code 0), leaving exactly one live agent per
(namespace, domain). Emitting a single agent per launch tree is the launch
side's responsibility; this lock is the cross-invocation safety net for when
separate launches target the same (namespace, domain).
"""

import ctypes
from dataclasses import dataclass
from enum import Enum
import fcntl
import importlib.metadata
import logging
import os
import socket
import sys
from typing import IO
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

from . import bridge_decider
from .type_registry import TypeRegistryReader


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


def _ioctl_to_endpoint(
        info: TopicInfoRet, topic_name: str, role: str,
        registry: TypeRegistryReader | None = None) -> AgnocastEndpoint:
    """Convert one ``topic_info_ret`` row to an AgnocastEndpoint msg.

    ``pid`` is looked up from the tmpfs type registry (written by agnocastlib
    at Publisher/Subscription construction time) using
    ``(topic_name, role, node_name)`` as the join key. When no match is found
    the field stays 0, which lets the rest of the pipeline degrade gracefully
    (the gossip publication still flows; just no pid for that endpoint).
    """
    ep = AgnocastEndpoint()
    ep.node_name = info.node_name.decode('utf-8', errors='replace')
    ep.pid = 0
    if registry is not None:
        entry = registry.lookup(topic_name, role, ep.node_name)
        if entry is not None:
            ep.pid = entry.pid
    ep.qos_depth = info.qos_depth
    ep.qos_is_transient_local = info.qos_is_transient_local
    ep.qos_is_reliable = info.qos_is_reliable
    ep.is_bridge = info.is_bridge
    return ep


def read_local_topics(lib, registry: TypeRegistryReader | None = None) -> list:
    """Snapshot the current namespace's Agnocast topics via the ioctl wrapper.

    Returns a list of AgnocastTopic msgs. The ioctl returns only the caller's
    IPC namespace, so the daemon process just being inside that namespace is
    sufficient to scope the result. The optional ``registry`` argument
    supplies the type names and pids that the ioctl does not expose.
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
            agnocast_topic.type_name = ''
            agnocast_topic.domain_id = 0
            agnocast_topic.publishers = _collect_endpoints(
                lib.get_agnocast_pub_nodes, lib, topic_name_b, topic_name, 'pub', registry)
            agnocast_topic.subscribers = _collect_endpoints(
                lib.get_agnocast_sub_nodes, lib, topic_name_b, topic_name, 'sub', registry)
            # Type name comes from the tmpfs registry; any registered
            # endpoint on this topic carries the same type (ROS 2
            # invariant), so the first non-empty one wins.
            if registry is not None:
                resolved = _resolve_topic_type(agnocast_topic, registry)
                if resolved:
                    agnocast_topic.type_name = resolved
            topics.append(agnocast_topic)
    finally:
        lib.free_agnocast_topics(topic_names_ptr, topic_count.value)

    return topics


def _resolve_topic_type(
        agnocast_topic: AgnocastTopic, registry: TypeRegistryReader) -> str:
    for ep in agnocast_topic.publishers:
        entry = registry.lookup(agnocast_topic.topic_name, 'pub', ep.node_name)
        if entry is not None and entry.type_name:
            return entry.type_name
    for ep in agnocast_topic.subscribers:
        entry = registry.lookup(agnocast_topic.topic_name, 'sub', ep.node_name)
        if entry is not None and entry.type_name:
            return entry.type_name
    return ''


def _collect_endpoints(
        getter, lib, topic_name_b: bytes, topic_name: str, role: str,
        registry: TypeRegistryReader | None = None) -> list:
    count = ctypes.c_int()
    array = getter(topic_name_b, ctypes.byref(count))
    endpoints = []
    if not array:
        return endpoints
    try:
        for i in range(count.value):
            endpoints.append(_ioctl_to_endpoint(array[i], topic_name, role, registry))
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


def _read_ros_domain_id() -> int:
    """Return ROS_DOMAIN_ID from the environment (0 if unset, empty, or
    unparsable), matching ROS 2's default and the kmod's get_ros_domain_id."""
    raw = os.environ.get('ROS_DOMAIN_ID')
    if not raw:
        return 0
    try:
        return int(raw)
    except ValueError:
        return 0


def _singleton_lock_path(ipc_ns_inode: int, domain_id: int) -> str:
    """Return the singleton lock-file path for this (IPC namespace, domain).

    Keyed by both the IPC namespace and ROS_DOMAIN_ID: gossip is published on
    the agent's DDS domain, so launches sharing a namespace but using different
    domains each need their own agent and must not deduplicate against each
    other.

    Co-located with the rest of Agnocast tmpfs state so a container overriding
    ``AGNOCAST_TMPFS_DIR`` keeps the lock under the same root.
    """
    root = os.environ.get('AGNOCAST_TMPFS_DIR') or '/dev/shm'
    return os.path.join(root, f'agnocast_discovery_agent_{ipc_ns_inode}_d{domain_id}.lock')


class LockStatus(Enum):
    ACQUIRED = 'acquired'   # we got the lock
    HELD = 'held'           # another agent in this NS holds it — caller should idle
    ERROR = 'error'         # could not even open the lock file — caller should exit non-zero


@dataclass
class SingletonLockAttempt:
    """Result of trying to acquire the per-NS singleton lock.

    ``file`` is set only when ``status == ACQUIRED`` — the caller keeps the
    reference alive so the ``flock(2)`` outlives the spin loop.
    """
    status: LockStatus
    file: IO | None = None


def _try_acquire_singleton_lock(ipc_ns_inode: int, domain_id: int) -> SingletonLockAttempt:
    """Try to take an exclusive ``flock(2)`` on the per-(IPC namespace, domain) lock file."""
    lock_path = _singleton_lock_path(ipc_ns_inode, domain_id)
    try:
        fd = os.open(lock_path, os.O_RDWR | os.O_CREAT | os.O_CLOEXEC, 0o644)
    except OSError as e:
        sys.stderr.write(
            f'agnocast_discovery_agent: cannot open singleton lock {lock_path}: {e}\n')
        return SingletonLockAttempt(LockStatus.ERROR)
    lock_file = os.fdopen(fd, 'r+')
    try:
        fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        # Contention: another agent in this NS holds the lock.
        lock_file.close()
        return SingletonLockAttempt(LockStatus.HELD)
    except OSError as e:
        # Real failure (permission denied, ENOTSUP on the FS, etc.) — surface
        # it rather than masking it as "already running".
        sys.stderr.write(
            f'agnocast_discovery_agent: flock({lock_path}) failed: {e}\n')
        lock_file.close()
        return SingletonLockAttempt(LockStatus.ERROR)
    return SingletonLockAttempt(LockStatus.ACQUIRED, file=lock_file)


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

    Also subscribes to its own gossip topic and caches the latest snapshot per
    ``(host_uuid, ipc_ns_inode)``; each tick the bridge decider diffs that
    against the local state and issues cross-NS bridge requests.
    """

    def __init__(self, registry: TypeRegistryReader | None = None):
        self._host_uuid = _read_host_uuid()
        self._host_hostname = socket.gethostname()
        self._ipc_ns_inode = _read_ipc_ns_inode()
        self._domain_id = _read_ros_domain_id()
        # Multiple agents may run on one ECU (one per (IPC NS, domain)); disambiguate.
        host_short = self._host_uuid.replace('-', '')[:8]
        # Force use_sim_time=False so the 1 Hz timer + clock reads are
        # wall-clock regardless of /clock; sim-time playback would otherwise
        # stretch the publish cadence past the liveliness lease window.
        super().__init__(
            f'agnocast_discovery_agent_{host_short}_{self._ipc_ns_inode}_d{self._domain_id}',
            parameter_overrides=[Parameter('use_sim_time', value=False)],
        )
        self._lib = _load_ioctl_wrapper()
        self._agnocast_version = _read_agent_version()
        # Independent wall-clock for prune / receive timestamps so they stay
        # consistent even if a future change accidentally enables sim time.
        self._clock = Clock(clock_type=ClockType.SYSTEM_TIME)
        self._registry = registry if registry is not None else TypeRegistryReader(
            self._ipc_ns_inode, logger=self.get_logger())

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
        self._registry.rebuild()
        self._registry.cleanup_dead_pids()
        self._prune_stale_remote_states()
        snapshot = self.publish_snapshot()
        self._dispatch_bridge_requests(snapshot)

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

    def publish_snapshot(self) -> AgnocastDaemonState:
        """Build and publish the current local AgnocastDaemonState."""
        msg = self.build_state()
        self._pub.publish(msg)
        return msg

    def _dispatch_bridge_requests(self, local_state: AgnocastDaemonState) -> None:
        if not self._remote_states:
            return
        remote_states = {key: msg for key, (msg, _received_at) in self._remote_states.items()}
        requests = bridge_decider.decide_bridges(local_state, remote_states)
        if requests:
            bridge_decider.dispatch_requests(requests, logger=self.get_logger())

    def build_state(self) -> AgnocastDaemonState:
        msg = AgnocastDaemonState()
        msg.schema_version = SCHEMA_VERSION
        msg.agnocast_version = self._agnocast_version
        msg.host_uuid = self._host_uuid
        msg.host_hostname = self._host_hostname
        msg.ipc_ns_inode = self._ipc_ns_inode
        msg.topics = read_local_topics(self._lib, self._registry)
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
    # Take the per-IPC-namespace singleton lock before any DDS / ioctl work.
    # If another agent holds it, exit cleanly (0).
    #
    # A duplicate that exits early *could* make launch_test treat it as a
    # crashed node and tear down the launch tree. That's not a risk here: the
    # launch emits one agent per tree, so a held lock always means a separate
    # launch/run — never a same-tree sibling — which exiting can't affect.
    ipc_ns_inode = _read_ipc_ns_inode()
    domain_id = _read_ros_domain_id()
    lock_attempt = _try_acquire_singleton_lock(ipc_ns_inode, domain_id)
    if lock_attempt.status == LockStatus.HELD:
        sys.stderr.write(
            'agnocast_discovery_agent: another instance is already running in this '
            f'IPC namespace (inode={ipc_ns_inode}, domain={domain_id}); exiting.\n')
        return 0
    if lock_attempt.status == LockStatus.ERROR:
        # Don't masquerade as "already running" — propagate failure so
        # supervisors can detect it.
        return 1

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
        # Keep `lock_attempt` referenced through shutdown so the flock
        # outlives the spin() loop.
        del lock_attempt
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
