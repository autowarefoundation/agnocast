"""Shared helper for subscribing to /_agnocast_discovery and aggregating gossip.

The local NS is excluded from gossip so the CLI does not double-count
what it already reads via ioctl. Staleness is bounded by the publisher's
DDS Liveliness lease; no separate freshness filter.
"""

import argparse
import os
import sys
import time
import uuid

import rclpy
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, HistoryPolicy, LivelinessPolicy, QoSProfile, ReliabilityPolicy

from ros2agnocast_discovery_msgs.msg import AgnocastDaemonState


GOSSIP_TOPIC = '/_agnocast_discovery'
DEFAULT_COLLECT_TIMEOUT_SEC = 2.0
LIVELINESS_LEASE_SEC = 30.0
BOOT_ID_PATH = '/proc/sys/kernel/random/boot_id'
SELF_IPC_NS_PATH = '/proc/self/ns/ipc'


def add_gossip_timeout_arg(parser) -> None:
    """Add the hidden, transitional ``--gossip-timeout`` flag to a verb's parser.

    Suppressed from ``--help`` because the plan is to replace the fixed
    wait with a ros2-daemon-driven stop condition. Use
    :func:`warn_if_gossip_timeout_overridden` in ``main()`` to nudge any
    operator who passes it explicitly.
    """
    parser.add_argument(
        '--gossip-timeout',
        type=float,
        default=DEFAULT_COLLECT_TIMEOUT_SEC,
        help=argparse.SUPPRESS)


def warn_if_gossip_timeout_overridden(args) -> None:
    """Print a stderr WARN when ``--gossip-timeout`` is set to a non-default."""
    if args.gossip_timeout != DEFAULT_COLLECT_TIMEOUT_SEC:
        print(
            'WARNING: --gossip-timeout is unsupported and will be '
            'removed once ros2-daemon-driven discovery lands.',
            file=sys.stderr)


def gossip_qos() -> QoSProfile:
    """QoS profile for the gossip subscription.

    Must match ``ros2agnocast_discovery_agent.agent._gossip_qos`` on every
    field except depth (per-endpoint) so DDS accepts the match.
    """
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=64,
        liveliness=LivelinessPolicy.AUTOMATIC,
        liveliness_lease_duration=Duration(seconds=LIVELINESS_LEASE_SEC),
    )


def _local_host_uuid() -> str:
    """Return the host's boot UUID (matches what the discovery agent stamps)."""
    try:
        with open(BOOT_ID_PATH) as fp:
            return str(uuid.UUID(fp.read().strip()))
    except (OSError, ValueError):
        return ''


def _local_ipc_ns_inode() -> int:
    """Return the IPC namespace inode of the calling process."""
    return os.stat(SELF_IPC_NS_PATH).st_ino


def collect_announcements(
    node,
    timeout_sec: float = DEFAULT_COLLECT_TIMEOUT_SEC,
) -> tuple:
    """Collect remote gossip snapshots.

    Returns ``(remote_snapshots, saw_local)``:

    * ``remote_snapshots`` is one latest message per remote ``(host_uuid,
      ipc_ns_inode)``.
    * ``saw_local`` is ``True`` if a snapshot from the caller's own NS was
      received during the wait — used by :func:`warn_if_no_announcements`
      to distinguish "remote agents are silent" from "we are the only
      agent in the domain" (a valid single-NS steady state).
    """
    snapshots = {}
    saw_local = False
    local_host_uuid = _local_host_uuid()
    local_ipc_ns_inode = _local_ipc_ns_inode()

    def on_msg(msg: AgnocastDaemonState) -> None:
        nonlocal saw_local
        if msg.host_uuid == local_host_uuid and msg.ipc_ns_inode == local_ipc_ns_inode:
            saw_local = True
            return
        snapshots[(msg.host_uuid, msg.ipc_ns_inode)] = msg

    spin_node = _resolve_spin_node(node)
    sub = spin_node.create_subscription(
        AgnocastDaemonState, GOSSIP_TOPIC, on_msg, gossip_qos())
    try:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(spin_node, timeout_sec=0.05)
    finally:
        spin_node.destroy_subscription(sub)

    return list(snapshots.values()), saw_local


def _resolve_spin_node(node):
    """Return the underlying ``rclpy.Node`` if ``node`` is a ``NodeStrategy``.

    ``NodeStrategy`` is a ros2cli dispatcher, not an ``rclpy.Node``; passing
    it to executor APIs like ``rclpy.spin_once`` is outside its intended use.
    """
    direct = getattr(node, 'direct_node', None)
    if direct is None:
        return node
    return getattr(direct, 'node', direct)


def warn_if_no_announcements(
    node, snapshots: list, saw_local: bool, timeout_sec: float,
) -> None:
    """Best-effort stderr hint when no gossip arrived; does not change exit code.

    ``saw_local`` (from :func:`collect_announcements`) plus the visible
    publisher count let us suppress only the genuinely benign case: exactly
    one publisher is visible and we received its snapshot — i.e. our own
    local agent is the only one in the domain. With 2+ visible publishers
    and no remote snapshot, at least one remote agent is silent, which is
    still worth warning about.
    """
    if timeout_sec <= 0:
        return
    if snapshots:
        return

    spin_node = _resolve_spin_node(node)
    try:
        publishers = spin_node.get_publishers_info_by_topic(GOSSIP_TOPIC)
    except Exception:
        publishers = []

    domain_id = os.environ.get('ROS_DOMAIN_ID', '0')

    if not publishers:
        print(
            f'WARNING: no /_agnocast_discovery publisher visible in '
            f'ROS_DOMAIN_ID={domain_id} within {timeout_sec:.1f}s. Common '
            'causes: (1) no discovery agent running on this domain (start one '
            'with `ros2 run ros2agnocast_discovery_agent discovery_agent`); '
            '(2) RMW mismatch between agent and CLI. Pass '
            '`--gossip-timeout 0` to skip this check.',
            file=sys.stderr)
        return

    if saw_local and len(publishers) == 1:
        # The single visible publisher is our own NS agent; no remote
        # agents exist to report on. Expected single-NS steady state.
        return

    print(
        f'WARNING: /_agnocast_discovery has {len(publishers)} publisher(s) '
        f'visible but no snapshot was received in {timeout_sec:.1f}s. Common '
        'causes: (1) the ros2 daemon was started before `install/setup.bash` '
        'was sourced and is missing the discovery msg package on its '
        'PYTHONPATH (try `ros2 daemon stop && ros2 daemon start`); '
        '(2) QoS / type mismatch on the publisher side. Pass '
        '`--gossip-timeout 0` to skip this check.',
        file=sys.stderr)


def all_topic_names(snapshots: list) -> set:
    """Union of topic names across all snapshots."""
    return {topic.topic_name for snap in snapshots for topic in snap.topics}


def all_nodes(snapshots: list) -> set:
    """Union of node names across all snapshots."""
    nodes = set()
    for snap in snapshots:
        for topic in snap.topics:
            for endpoint in topic.publishers:
                nodes.add(endpoint.node_name)
            for endpoint in topic.subscribers:
                nodes.add(endpoint.node_name)
    return nodes


def topic_endpoints(snapshots: list, topic_name: str) -> tuple:
    """Return (publishers, subscribers) for ``topic_name``; caller dedups."""
    publishers = []
    subscribers = []
    for snap in snapshots:
        for topic in snap.topics:
            if topic.topic_name != topic_name:
                continue
            publishers.extend(topic.publishers)
            subscribers.extend(topic.subscribers)
    return publishers, subscribers


def topics_of_node(snapshots: list, node_name: str) -> tuple:
    """Return (pub topics, sub topics) for ``node_name`` as {topic_name, type_name} dicts."""
    pubs, subs = [], []
    for snap in snapshots:
        for topic in snap.topics:
            for endpoint in topic.publishers:
                if endpoint.node_name == node_name:
                    pubs.append({'topic_name': topic.topic_name,
                                 'type_name': topic.type_name})
            for endpoint in topic.subscribers:
                if endpoint.node_name == node_name:
                    subs.append({'topic_name': topic.topic_name,
                                 'type_name': topic.type_name})
    return pubs, subs


def gossip_has_bridge_endpoint(snapshots: list, topic_name: str) -> tuple:
    """Return (has_pub_bridge, has_sub_bridge) seen on remote NSes via gossip."""
    has_pub_bridge = False
    has_sub_bridge = False
    for snap in snapshots:
        for topic in snap.topics:
            if topic.topic_name != topic_name:
                continue
            for endpoint in topic.publishers:
                if endpoint.is_bridge:
                    has_pub_bridge = True
            for endpoint in topic.subscribers:
                if endpoint.is_bridge:
                    has_sub_bridge = True
    return has_pub_bridge, has_sub_bridge
