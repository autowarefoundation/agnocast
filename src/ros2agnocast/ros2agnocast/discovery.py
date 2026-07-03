"""Subscribe to /_agnocast_discovery; fall back to ioctl when no agent gossips.

Staleness is bounded by the publisher's DDS Liveliness lease.
"""

import argparse
import sys
import time

import rclpy
from rclpy.duration import Duration
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, LivelinessPolicy, QoSProfile, ReliabilityPolicy,
)

from ros2agnocast._ioctl import self_ns_snapshot

from ros2agnocast_discovery_msgs.msg import AgnocastDaemonState, AgnocastEndpoint


GOSSIP_TOPIC = '/_agnocast_discovery'
DEFAULT_COLLECT_TIMEOUT_SEC = 2.0
LIVELINESS_LEASE_SEC = 30.0


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
    """Emit a stderr WARN when ``--gossip-timeout`` is set to a non-default."""
    if args.gossip_timeout != DEFAULT_COLLECT_TIMEOUT_SEC:
        print(
            'WARNING: --gossip-timeout is unsupported and will be '
            'removed once ros2-daemon-driven discovery lands.',
            file=sys.stderr)


def gossip_qos() -> QoSProfile:
    """Profile for the gossip subscription.

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


def collect_announcements(
    node,
    timeout_sec: float = DEFAULT_COLLECT_TIMEOUT_SEC,
) -> list[AgnocastDaemonState]:
    """Collect one latest snapshot per ``(host_uuid, ipc_ns_inode)`` from gossip.

    Returns one ``AgnocastDaemonState`` per gossiping agent, or ``[]`` when no
    agent is reachable within ``timeout_sec``.
    """
    snapshots = {}

    def on_msg(msg: AgnocastDaemonState) -> None:
        snapshots[(msg.host_uuid, msg.ipc_ns_inode)] = msg

    spin_node = _resolve_spin_node(node)
    sub = spin_node.create_subscription(
        AgnocastDaemonState, GOSSIP_TOPIC, on_msg, gossip_qos())
    try:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(spin_node, timeout_sec=0.05)
            try:
                publishers = spin_node.get_publishers_info_by_topic(GOSSIP_TOPIC)
            except Exception:
                publishers = []
            # Return as soon as every visible publisher has reported, rather
            # than waiting out timeout_sec (now just an upper bound).
            # One agent = one publisher = one (host_uuid, ipc_ns_inode) snapshot,
            # so the publisher count is how many snapshots to expect.
            if publishers and len(snapshots) >= len(publishers):
                break
    finally:
        spin_node.destroy_subscription(sub)

    return list(snapshots.values())


def collect_announcements_with_fallback(
    node,
    timeout_sec: float = DEFAULT_COLLECT_TIMEOUT_SEC,
) -> tuple[list[AgnocastDaemonState], bool]:
    """Gossip first, then ioctl fallback. Returns ``(snapshots, used_fallback)``.

    - ``used_fallback`` is False when at least one agent gossiped; ``snapshots``
      is then every reachable agent's state.
    - Otherwise it is True and ``snapshots`` is the local-NS ioctl view: a
      single-element ``[state]`` whose ``topics`` may be empty (kmod absent or
      nothing registered), or ``[]`` only if the ioctl wrapper library itself
      cannot be loaded.
    """
    snapshots = collect_announcements(node, timeout_sec)
    if snapshots:
        return snapshots, False
    fallback = self_ns_snapshot()
    return ([fallback] if fallback is not None else []), True


def _resolve_spin_node(node):
    """Return the underlying ``rclpy.Node`` if ``node`` is a ``NodeStrategy``.

    ``NodeStrategy`` is a ros2cli dispatcher, not an ``rclpy.Node``; passing
    it to executor APIs like ``rclpy.spin_once`` is outside its intended use.
    """
    direct = getattr(node, 'direct_node', None)
    if direct is None:
        return node
    return getattr(direct, 'node', direct)


def warn_if_using_fallback(
    node, used_fallback: bool, timeout_sec: float, snapshots: list[AgnocastDaemonState]
) -> None:
    """Best-effort stderr note when gossip was unavailable and ioctl fallback ran.

    ``snapshots`` is the fallback result from ``collect_announcements_with_fallback``.
    The local ioctl fallback returns a ``[state]`` with empty ``topics`` (not ``[]``)
    when the kmod is absent or nothing is registered, so "no Agnocast here" is
    detected by the absence of topics, not by an empty list.
    """
    if not used_fallback or timeout_sec <= 0:
        return

    spin_node = _resolve_spin_node(node)
    try:
        publishers = spin_node.get_publishers_info_by_topic(GOSSIP_TOPIC)
    except Exception:
        publishers = []

    if not publishers:
        # No agent is gossiping. Only worth a note when this namespace actually
        # has Agnocast endpoints to show: ros2 CLI runs constantly in namespaces
        # without Agnocast, where "no agent" would be misleading noise.
        if not any(snap.topics for snap in snapshots):
            return
        print(
            'NOTE: no /_agnocast_discovery agent visible; showing local '
            'NS only via ioctl. Start one with '
            '`ros2 run ros2agnocast_discovery_agent discovery_agent` to '
            'see other NSes / ECUs.',
            file=sys.stderr)
    else:
        print(
            f'WARNING: /_agnocast_discovery has {len(publishers)} publisher(s) '
            f'visible but no snapshot received in {timeout_sec:.1f}s; falling '
            'back to ioctl (local NS only). Try '
            '`ros2 daemon stop && ros2 daemon start` (the ros2 daemon may be '
            'missing the discovery msg package on its PYTHONPATH).',
            file=sys.stderr)


def all_topic_names(snapshots: list[AgnocastDaemonState]) -> set[str]:
    """Union of topic names across all snapshots."""
    return {topic.topic_name for snap in snapshots for topic in snap.topics}


def all_nodes(snapshots: list[AgnocastDaemonState]) -> set[str]:
    """Union of node names across all snapshots."""
    nodes = set()
    for snap in snapshots:
        for topic in snap.topics:
            for endpoint in topic.publishers:
                nodes.add(endpoint.node_name)
            for endpoint in topic.subscribers:
                nodes.add(endpoint.node_name)
    return nodes


def topic_endpoints(
    snapshots: list[AgnocastDaemonState], topic_name: str
) -> tuple[list[AgnocastEndpoint], list[AgnocastEndpoint]]:
    """Return (publishers, subscribers) for ``topic_name`` across all snapshots."""
    publishers = []
    subscribers = []
    for snap in snapshots:
        for topic in snap.topics:
            if topic.topic_name != topic_name:
                continue
            publishers.extend(topic.publishers)
            subscribers.extend(topic.subscribers)
    return publishers, subscribers


def topics_of_node(
    snapshots: list[AgnocastDaemonState], node_name: str
) -> tuple[list[dict[str, str]], list[dict[str, str]]]:
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


# Bridge-label states for the CLI verbs. Wording is shared so list/info stay
# consistent.
BRIDGE_ENABLED = 'enabled'
BRIDGE_BRIDGED = 'bridged'
BRIDGE_WARN = 'warn'
BRIDGE_LABEL_TEXT = {
    BRIDGE_ENABLED: 'Agnocast enabled',
    BRIDGE_BRIDGED: 'Agnocast enabled, bridged',
    BRIDGE_WARN: 'WARN: one or more necessary bridges are not running',
}


def collect_bridge_roles(
    snapshots: list[AgnocastDaemonState],
) -> dict[str, list[tuple[bool, bool, bool, bool]]]:
    """Map ``topic_name -> per-NS bridge roles`` in a single pass over snapshots.

    Each value is a list with one entry per NS that has a real (non-bridge)
    endpoint on that topic: ``(real_pub, real_sub, inbound_bridge,
    outbound_bridge)``. A bridge endpoint is directional: an inbound bridge
    (ROS 2 -> Agnocast) is a bridge publisher; an outbound bridge
    (Agnocast -> ROS 2) is a bridge subscriber.

    Building this once lets the verbs label every topic with an O(1) lookup
    instead of re-scanning the snapshots per topic.
    """
    roles = {}
    for snap in snapshots:
        for topic in snap.topics:
            real_pub = any(not ep.is_bridge for ep in topic.publishers)
            real_sub = any(not ep.is_bridge for ep in topic.subscribers)
            if not (real_pub or real_sub):
                continue
            inbound_bridge = any(ep.is_bridge for ep in topic.publishers)
            outbound_bridge = any(ep.is_bridge for ep in topic.subscribers)
            roles.setdefault(topic.topic_name, []).append(
                (real_pub, real_sub, inbound_bridge, outbound_bridge))
    return roles


def bridge_label_from_roles(ns_roles: list, ros2_has_pub: bool, ros2_has_sub: bool) -> str:
    """Classify a topic's bridge status from its per-NS roles.

    Returns ``BRIDGE_ENABLED`` / ``BRIDGE_BRIDGED`` / ``BRIDGE_WARN``. A bridge
    is *expected* only where data must actually cross a boundary: a real
    publisher needs an outbound bridge only if a consumer exists elsewhere
    (another NS or ROS 2), and a real subscriber needs an inbound bridge only
    if a producer exists elsewhere. So WARN fires only when such a bridge is
    expected but missing (e.g. a talker and listener in different NSes with no
    bridges); BRIDGED means every expected bridge is present; ENABLED means no
    bridging is expected (a lone endpoint, or producer/consumer in the same NS).

    ``ns_roles`` comes from :func:`collect_bridge_roles` (empty if the topic has
    no real Agnocast endpoint). ``ros2_has_pub`` / ``ros2_has_sub`` are whether
    the topic has a ROS 2 (DDS) publisher / subscriber in the caller's NS — the
    only NS whose DDS view the CLI can see.
    """
    real_pub_nses = sum(1 for r in ns_roles if r[0])
    real_sub_nses = sum(1 for r in ns_roles if r[1])

    bridge_expected = False
    bridge_missing = False
    for real_pub, real_sub, inbound_bridge, outbound_bridge in ns_roles:
        # "elsewhere" excludes this NS — an intra-NS producer/consumer pair
        # talks directly without a bridge.
        consumer_elsewhere = ros2_has_sub or real_sub_nses - (1 if real_sub else 0) > 0
        producer_elsewhere = ros2_has_pub or real_pub_nses - (1 if real_pub else 0) > 0
        if real_pub and consumer_elsewhere:
            bridge_expected = True
            bridge_missing |= not outbound_bridge
        if real_sub and producer_elsewhere:
            bridge_expected = True
            bridge_missing |= not inbound_bridge

    if not bridge_expected:
        return BRIDGE_ENABLED
    return BRIDGE_WARN if bridge_missing else BRIDGE_BRIDGED
