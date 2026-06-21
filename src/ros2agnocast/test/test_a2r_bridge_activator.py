"""Unit tests for A2rBridgeActivator bridge-diagnostic logic.

These tests exercise _on_discovery and _check_bridges without starting
rclpy or DDS.  The rclpy node is replaced by a MagicMock so that the
logic under test can be driven in pure Python.
"""

from unittest.mock import MagicMock, patch

from ros2agnocast._a2r_bridge_activator import A2rBridgeActivator
from ros2agnocast_discovery_msgs.msg import AgnocastDaemonState, AgnocastEndpoint, AgnocastTopic


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_activator() -> A2rBridgeActivator:
    """Return an A2rBridgeActivator with a mock node (no rclpy required)."""
    act = A2rBridgeActivator()
    act._node = MagicMock()
    return act


def _endpoint(node_name: str = '/pub') -> AgnocastEndpoint:
    ep = AgnocastEndpoint()
    ep.node_name = node_name
    ep.pid = 1
    ep.qos_depth = 1
    ep.qos_is_transient_local = False
    ep.qos_is_reliable = False
    ep.is_bridge = False
    return ep


def _topic(topic_name: str, *, pubs=None, type_name: str = 'std_msgs/msg/String') -> AgnocastTopic:
    t = AgnocastTopic()
    t.topic_name = topic_name
    t.type_name = type_name
    t.domain_id = 0
    t.publishers = pubs if pubs is not None else []
    t.subscribers = []
    return t


def _state(*topics: AgnocastTopic) -> AgnocastDaemonState:
    msg = AgnocastDaemonState()
    msg.schema_version = 1
    msg.agnocast_version = ''
    msg.host_uuid = 'test'
    msg.host_hostname = 'test'
    msg.ipc_ns_inode = 0
    msg.topics = list(topics)
    return msg


# ---------------------------------------------------------------------------
# _on_discovery tests
# ---------------------------------------------------------------------------

def test_on_discovery_adds_topic_to_awaiting():
    act = _make_activator()
    with patch('ros2agnocast._a2r_bridge_activator.load_msg_class', return_value=MagicMock()):
        act._on_discovery(_state(_topic('/chatter', pubs=[_endpoint()])))
    assert '/chatter' in act._awaiting_bridge_topics


def test_on_discovery_skips_agnocast_srv_topics():
    act = _make_activator()
    with patch('ros2agnocast._a2r_bridge_activator.load_msg_class', return_value=MagicMock()):
        act._on_discovery(_state(_topic('/AGNOCAST_SRV_foo', pubs=[_endpoint()])))
    assert '/AGNOCAST_SRV_foo' not in act._awaiting_bridge_topics


def test_on_discovery_skips_topics_without_publishers():
    act = _make_activator()
    with patch('ros2agnocast._a2r_bridge_activator.load_msg_class', return_value=MagicMock()):
        act._on_discovery(_state(_topic('/chatter', pubs=[])))
    assert '/chatter' not in act._awaiting_bridge_topics


def test_on_discovery_does_not_duplicate_subscription():
    act = _make_activator()
    with patch('ros2agnocast._a2r_bridge_activator.load_msg_class', return_value=MagicMock()):
        act._on_discovery(_state(_topic('/chatter', pubs=[_endpoint()])))
        act._on_discovery(_state(_topic('/chatter', pubs=[_endpoint()])))
    assert act._node.create_subscription.call_count == 1


def test_on_discovery_skips_when_load_msg_class_returns_none():
    act = _make_activator()
    with patch('ros2agnocast._a2r_bridge_activator.load_msg_class', return_value=None):
        act._on_discovery(_state(_topic('/chatter', pubs=[_endpoint()])))
    assert '/chatter' not in act._awaiting_bridge_topics


def test_on_discovery_skips_topic_when_should_activate_returns_false():
    act = _make_activator()
    act._should_activate = lambda topic_name, type_name: False
    with patch('ros2agnocast._a2r_bridge_activator.load_msg_class', return_value=MagicMock()):
        act._on_discovery(_state(_topic('/chatter', pubs=[_endpoint()])))
    assert '/chatter' not in act._active_subs
    assert '/chatter' not in act._awaiting_bridge_topics


def test_on_discovery_activates_only_matching_topics_based_on_should_activate():
    """should_activate selectively activates topics: only allowed topics get a subscription."""
    act = _make_activator()
    act._should_activate = lambda topic_name, type_name: topic_name == '/allowed'
    with patch('ros2agnocast._a2r_bridge_activator.load_msg_class', return_value=MagicMock()):
        act._on_discovery(_state(
            _topic('/allowed', pubs=[_endpoint()]),
            _topic('/blocked', pubs=[_endpoint()]),
        ))
    assert '/allowed' in act._awaiting_bridge_topics
    assert '/blocked' not in act._awaiting_bridge_topics


def test_on_discovery_activates_when_should_activate_raises():
    act = _make_activator()
    act._should_activate = MagicMock(side_effect=RuntimeError('oops'))
    with patch('ros2agnocast._a2r_bridge_activator.load_msg_class', return_value=MagicMock()):
        act._on_discovery(_state(_topic('/chatter', pubs=[_endpoint()])))
    act._node.get_logger.return_value.warning.assert_called_once()
    assert '/chatter' in act._awaiting_bridge_topics


# ---------------------------------------------------------------------------
# _check_bridges tests
# ---------------------------------------------------------------------------

def test_check_bridges_removes_confirmed_topic_silently():
    act = _make_activator()
    act._awaiting_bridge_topics['/chatter'] = 0.0  # created long ago
    act._node.count_publishers.return_value = 1

    with patch('ros2agnocast._a2r_bridge_activator.time') as mock_time:
        mock_time.monotonic.return_value = act.BRIDGE_SPAWN_TIMEOUT + 1.0
        act._check_bridges()

    assert '/chatter' not in act._awaiting_bridge_topics
    act._node.get_logger.return_value.warning.assert_not_called()


def test_check_bridges_warns_and_removes_timed_out_topic():
    act = _make_activator()
    act._awaiting_bridge_topics['/chatter'] = 0.0
    act._node.count_publishers.return_value = 0

    with patch('ros2agnocast._a2r_bridge_activator.time') as mock_time:
        mock_time.monotonic.return_value = act.BRIDGE_SPAWN_TIMEOUT + 1.0
        act._check_bridges()

    assert '/chatter' not in act._awaiting_bridge_topics
    act._node.get_logger.return_value.warning.assert_called_once()
    warning_msg = act._node.get_logger.return_value.warning.call_args[0][0]
    assert '/chatter' in warning_msg


def test_check_bridges_does_not_warn_before_timeout():
    act = _make_activator()
    act._awaiting_bridge_topics['/chatter'] = 0.0
    act._node.count_publishers.return_value = 0

    with patch('ros2agnocast._a2r_bridge_activator.time') as mock_time:
        mock_time.monotonic.return_value = act.BRIDGE_SPAWN_TIMEOUT - 1.0
        act._check_bridges()

    assert '/chatter' in act._awaiting_bridge_topics
    act._node.get_logger.return_value.warning.assert_not_called()


def test_check_bridges_handles_multiple_topics_independently():
    act = _make_activator()
    act._awaiting_bridge_topics['/ok'] = 0.0
    act._awaiting_bridge_topics['/missing'] = 0.0
    act._awaiting_bridge_topics['/pending'] = 999.0  # created "recently"

    def count_publishers(topic_name):
        return 1 if topic_name == '/ok' else 0

    act._node.count_publishers.side_effect = count_publishers

    with patch('ros2agnocast._a2r_bridge_activator.time') as mock_time:
        mock_time.monotonic.return_value = 1000.0
        act._check_bridges()

    assert '/ok' not in act._awaiting_bridge_topics       # confirmed -> removed
    assert '/missing' not in act._awaiting_bridge_topics  # timed out -> removed with warning
    assert '/pending' in act._awaiting_bridge_topics      # still within timeout -> kept

    warning_msg = act._node.get_logger.return_value.warning.call_args[0][0]
    assert '/missing' in warning_msg
    assert act._node.get_logger.return_value.warning.call_count == 1
