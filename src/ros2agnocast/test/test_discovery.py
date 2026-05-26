"""Unit tests for ros2agnocast.discovery helpers.

These tests do not require DDS; they exercise the projection helpers with
hand-built AgnocastDaemonState messages.
"""

from unittest.mock import MagicMock

from ros2agnocast_discovery_msgs.msg import (
    AgnocastDaemonState,
    AgnocastEndpoint,
    AgnocastTopic,
)

from ros2agnocast.discovery import (
    _resolve_spin_node,
    all_nodes,
    all_topic_names,
    gossip_has_bridge_endpoint,
    topic_endpoints,
    topics_of_node,
    warn_if_no_announcements,
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
    # Unknown topic returns empty tuples (not None) so callers can extend(...).
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


def test_gossip_has_bridge_endpoint_flags_pub_and_sub_independently():
    """One bridge pub + non-bridge sub on the same topic → (True, False)."""
    bridge_pub = _endpoint('/agnocast_bridge_node_xxx', is_bridge=True)
    sub = _endpoint('/listener')
    snap = _state('a', 1, topics=[_topic('/foo', pubs=[bridge_pub], subs=[sub])])
    assert gossip_has_bridge_endpoint([snap], '/foo') == (True, False)


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


def test_warn_if_no_announcements_silent_when_snapshots_present(capsys):
    snap = _state('a', 1)
    warn_if_no_announcements(_plain_node(), [snap], saw_local=False, timeout_sec=2.0)
    assert capsys.readouterr().err == ''


def test_warn_if_no_announcements_silent_when_timeout_zero(capsys):
    warn_if_no_announcements(_plain_node(), [], saw_local=False, timeout_sec=0)
    assert capsys.readouterr().err == ''


def test_warn_if_no_announcements_says_no_publisher_when_dds_sees_none(capsys):
    warn_if_no_announcements(
        _plain_node(publishers=[]), [], saw_local=False, timeout_sec=2.0)
    err = capsys.readouterr().err
    assert 'WARNING' in err
    assert 'no /_agnocast_discovery publisher visible' in err
    assert 'ROS_DOMAIN_ID' in err


def test_warn_if_no_announcements_says_qos_or_pythonpath_when_publisher_visible(capsys):
    warn_if_no_announcements(
        _plain_node(publishers=[MagicMock()]), [], saw_local=False, timeout_sec=2.0)
    err = capsys.readouterr().err
    assert 'WARNING' in err
    assert 'publisher(s) visible but no snapshot' in err


def test_warn_if_no_announcements_silent_when_only_local_publisher_visible(capsys):
    """Single-NS steady state: 1 publisher visible and it is our own."""
    warn_if_no_announcements(
        _plain_node(publishers=[MagicMock()]), [], saw_local=True, timeout_sec=2.0)
    assert capsys.readouterr().err == ''


def test_warn_if_no_announcements_warns_when_remote_publisher_is_silent(capsys):
    """Cross-NS / cross-ECU: more publishers visible than the local one we received.

    Even if `saw_local=True`, the second publisher must be remote and
    failed to deliver — operators want to hear about that.
    """
    warn_if_no_announcements(
        _plain_node(publishers=[MagicMock(), MagicMock()]),
        [], saw_local=True, timeout_sec=2.0)
    err = capsys.readouterr().err
    assert 'WARNING' in err
    assert 'publisher(s) visible but no snapshot' in err
