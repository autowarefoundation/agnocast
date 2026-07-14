"""Unit tests for ros2agnocast.discovery helpers.

These tests do not require DDS; they exercise the projection helpers with
hand-built AgnocastDaemonState messages.
"""

from unittest.mock import MagicMock, patch

from ros2agnocast.discovery import (
    _resolve_spin_node,
    all_nodes,
    all_topic_names,
    BRIDGE_BRIDGED,
    BRIDGE_ENABLED,
    bridge_label_from_roles,
    BRIDGE_WARN,
    collect_announcements_with_fallback,
    collect_bridge_roles,
    topic_endpoints,
    topics_of_node,
    warn_if_using_fallback,
)

from ros2agnocast_discovery_msgs.msg import (
    AgnocastDaemonState,
    AgnocastEndpoint,
    AgnocastTopic,
)


def _endpoint(node_name: str, *, is_bridge: bool = False, qos_depth: int = 10) -> AgnocastEndpoint:
    ep = AgnocastEndpoint()
    ep.node_name = node_name
    ep.pid = 0
    ep.qos_depth = qos_depth
    ep.qos_is_transient_local = False
    ep.qos_is_reliable = True
    ep.is_bridge = is_bridge
    return ep


def _topic(topic_name: str, *, pubs=None, subs=None, type_name: str = '') -> AgnocastTopic:
    topic = AgnocastTopic()
    topic.topic_name = topic_name
    topic.type_name = type_name
    topic.domain_id = 0
    topic.publishers = pubs or []
    topic.subscribers = subs or []
    return topic


def _state(host: str, ipc_ns: int, *, topics=None) -> AgnocastDaemonState:
    state = AgnocastDaemonState()
    state.schema_version = 1
    state.agnocast_version = ''
    state.host_uuid = host
    state.host_hostname = host
    state.ipc_ns_inode = ipc_ns
    state.topics = topics or []
    return state


def test_all_topic_names_unions_across_snapshots():
    snap_a = _state('a', 1, topics=[_topic('/foo'), _topic('/bar')])
    snap_b = _state('b', 2, topics=[_topic('/bar'), _topic('/baz')])
    assert all_topic_names([snap_a, snap_b]) == {'/foo', '/bar', '/baz'}


def test_all_nodes_includes_both_publishers_and_subscribers():
    pub = _endpoint('/talker')
    sub = _endpoint('/listener')
    snap = _state('a', 1, topics=[_topic('/foo', pubs=[pub], subs=[sub])])
    assert all_nodes([snap]) == {'/talker', '/listener'}


def test_topic_endpoints_filters_by_topic_name():
    pub1 = _endpoint('/talker_a')
    pub2 = _endpoint('/talker_b')
    sub = _endpoint('/listener')
    snap_a = _state('a', 1, topics=[_topic('/foo', pubs=[pub1])])
    snap_b = _state('b', 2, topics=[_topic('/foo', subs=[sub]), _topic('/bar', pubs=[pub2])])
    pubs, subs = topic_endpoints([snap_a, snap_b], '/foo')
    assert [p.node_name for p in pubs] == ['/talker_a']
    assert [s.node_name for s in subs] == ['/listener']
    assert topic_endpoints([snap_a, snap_b], '/missing') == ([], [])


def test_topics_of_node_collects_pub_and_sub_topics():
    pub = _endpoint('/talker')
    sub = _endpoint('/talker')  # same node also subscribes elsewhere
    snap = _state('a', 1, topics=[
        _topic('/foo', pubs=[pub], type_name='std_msgs/msg/String'),
        _topic('/bar', subs=[sub]),
    ])
    pubs, subs = topics_of_node([snap], '/talker')
    assert pubs == [{'topic_name': '/foo', 'type_name': 'std_msgs/msg/String'}]
    assert subs == [{'topic_name': '/bar', 'type_name': ''}]


# Role tuples are (real_pub, real_sub, inbound_bridge, outbound_bridge), one per NS.
# A bridge is only "expected" where a producer and consumer sit on opposite
# sides of a boundary, so signatures take ros2_has_pub / ros2_has_sub.

def test_bridge_label_from_roles_enabled_single_ns_no_ros2():
    """Talker + listener in one NS, no ROS 2: nothing to bridge."""
    assert bridge_label_from_roles(
        [(True, True, False, False)], ros2_has_pub=False, ros2_has_sub=False) == BRIDGE_ENABLED


def test_bridge_label_from_roles_warn_cross_ns_without_bridges():
    """Talker in NS-A, listener in NS-B, no bridges: TetsuKawa's case → WARN."""
    ns_roles = [(True, False, False, False), (False, True, False, False)]
    assert bridge_label_from_roles(
        ns_roles, ros2_has_pub=False, ros2_has_sub=False) == BRIDGE_WARN


def test_bridge_label_from_roles_bridged_cross_ns_with_both_bridges():
    """NS-A talker + outbound bridge, NS-B listener + inbound bridge → bridged."""
    ns_roles = [(True, False, False, True), (False, True, True, False)]
    assert bridge_label_from_roles(
        ns_roles, ros2_has_pub=False, ros2_has_sub=False) == BRIDGE_BRIDGED


def test_bridge_label_from_roles_warn_when_one_ns_missing_its_bridge():
    """NS-A is bridged but NS-B's listener lacks an inbound bridge → WARN."""
    ns_roles = [(True, False, False, True), (False, True, False, False)]
    assert bridge_label_from_roles(
        ns_roles, ros2_has_pub=False, ros2_has_sub=False) == BRIDGE_WARN


def test_bridge_label_from_roles_pub_needs_outbound_only_if_consumer_exists():
    """A talker + a ROS 2 subscriber needs an outbound bridge; a ROS 2 publisher does not."""
    # Consumer on ROS 2 → outbound bridge expected, missing → WARN.
    assert bridge_label_from_roles(
        [(True, False, False, False)], ros2_has_pub=False, ros2_has_sub=True) == BRIDGE_WARN
    # Only a ROS 2 publisher (no consumer anywhere) → no bridge expected → enabled.
    assert bridge_label_from_roles(
        [(True, False, False, False)], ros2_has_pub=True, ros2_has_sub=False) == BRIDGE_ENABLED
    # Outbound bridge present for the consumer case → bridged.
    assert bridge_label_from_roles(
        [(True, False, False, True)], ros2_has_pub=False, ros2_has_sub=True) == BRIDGE_BRIDGED


def test_collect_bridge_roles_extracts_real_and_bridge_roles_per_ns():
    """One pass maps each topic to per-NS (real_pub, real_sub, inbound, outbound)."""
    outbound = _endpoint('/agnocast_bridge_node_a', is_bridge=True)  # bridge subscriber
    snap_a = _state('a', 1, topics=[
        _topic('/foo', pubs=[_endpoint('/talker')], subs=[outbound])])
    snap_b = _state('b', 2, topics=[_topic('/foo', subs=[_endpoint('/listener')])])
    # A topic whose only endpoint is a bridge contributes no NS entry.
    snap_c = _state('c', 3, topics=[
        _topic('/bridge_only', pubs=[_endpoint('/agnocast_bridge_node_c', is_bridge=True)])])

    roles = collect_bridge_roles([snap_a, snap_b, snap_c])
    assert roles['/foo'] == [(True, False, False, True), (False, True, False, False)]
    assert '/bridge_only' not in roles


def test_resolve_spin_node_returns_node_as_is_when_not_nodestrategy():
    plain = MagicMock(spec=[])
    assert _resolve_spin_node(plain) is plain


def test_resolve_spin_node_unwraps_nodestrategy_to_underlying_node():
    inner_node = MagicMock(spec=[])
    direct_node = MagicMock()
    direct_node.node = inner_node
    strategy = MagicMock()
    strategy.direct_node = direct_node
    assert _resolve_spin_node(strategy) is inner_node


def _plain_node(publishers=None):
    """Build a mock that _resolve_spin_node treats as a plain rclpy.Node (no NodeStrategy)."""
    node = MagicMock(spec=['get_publishers_info_by_topic'])
    node.get_publishers_info_by_topic.return_value = publishers or []
    return node


def test_warn_if_using_fallback_silent_when_no_fallback_or_timeout_zero(capsys):
    warn_if_using_fallback(_plain_node(), used_fallback=False, timeout_sec=2.0, snapshots=[])
    warn_if_using_fallback(_plain_node(), used_fallback=True, timeout_sec=0, snapshots=[])
    assert capsys.readouterr().err == ''


def test_warn_if_using_fallback_says_no_agent_when_dds_sees_no_publisher(capsys):
    # Local Agnocast state exists (a topic), but no agent is gossiping.
    warn_if_using_fallback(
        _plain_node(publishers=[]), used_fallback=True, timeout_sec=2.0,
        snapshots=[_state('h', 1, topics=[_topic('/chatter')])])
    err = capsys.readouterr().err
    assert 'NOTE' in err
    assert 'no /_agnocast_discovery agent visible' in err


def test_warn_if_using_fallback_silent_when_no_publisher_and_no_local_state(capsys):
    # No agent and no local Agnocast: stay quiet rather than nag the many ros2 CLI
    # invocations in namespaces without Agnocast. The ioctl fallback yields a single
    # state with empty ``topics`` (not an empty list) when nothing is registered, so
    # emptiness is judged by topics, not list length.
    warn_if_using_fallback(
        _plain_node(publishers=[]), used_fallback=True, timeout_sec=2.0,
        snapshots=[_state('h', 1)])
    warn_if_using_fallback(
        _plain_node(publishers=[]), used_fallback=True, timeout_sec=2.0, snapshots=[])
    assert capsys.readouterr().err == ''


def test_warn_if_using_fallback_says_qos_or_pythonpath_when_publisher_visible(capsys):
    warn_if_using_fallback(
        _plain_node(publishers=[MagicMock()]), used_fallback=True, timeout_sec=2.0,
        snapshots=[])
    err = capsys.readouterr().err
    assert 'WARNING' in err
    assert 'falling back to ioctl' in err


def test_collect_announcements_with_fallback_uses_ioctl_when_gossip_empty():
    """When `collect_announcements` returns nothing, fall back to the synthetic snapshot."""
    fake_state = _state('host-x', 999, topics=[_topic('/local_topic')])
    with patch('ros2agnocast.discovery.collect_announcements', return_value=[]), \
         patch('ros2agnocast.discovery.self_ns_snapshot', return_value=fake_state):
        snapshots, used_fallback = collect_announcements_with_fallback(
            _plain_node(), timeout_sec=1.0)
    assert used_fallback is True
    assert len(snapshots) == 1
    assert snapshots[0].ipc_ns_inode == 999


def test_collect_announcements_with_fallback_returns_gossip_when_present():
    snap = _state('host-y', 1, topics=[_topic('/x')])
    with patch('ros2agnocast.discovery.collect_announcements', return_value=[snap]), \
         patch('ros2agnocast.discovery.self_ns_snapshot') as fallback_mock:
        snapshots, used_fallback = collect_announcements_with_fallback(
            _plain_node(), timeout_sec=1.0)
    assert used_fallback is False
    assert snapshots == [snap]
    fallback_mock.assert_not_called()


def test_collect_announcements_with_fallback_empty_when_ioctl_unavailable():
    with patch('ros2agnocast.discovery.collect_announcements', return_value=[]), \
         patch('ros2agnocast.discovery.self_ns_snapshot', return_value=None):
        snapshots, used_fallback = collect_announcements_with_fallback(
            _plain_node(), timeout_sec=1.0)
    assert used_fallback is True
    assert snapshots == []
