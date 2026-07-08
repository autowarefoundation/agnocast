"""Unit tests for the discovery agent's ioctl-to-msg conversion and gossip wiring.

These tests do not require the kmod or DDS; they exercise the conversion
helpers directly with a mock ctypes library.
"""

import ctypes
import sys
import threading
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from rclpy.executors import ExternalShutdownException

from ros2agnocast_discovery_agent.agent import (
    DiscoveryAgent,
    EXIT_WHEN_IDLE_ENV,
    EXIT_WHEN_IDLE_FLAG,
    IdleExitTracker,
    NODE_NAME_BUFFER_SIZE,
    TopicInfoRet,
    _exit_when_idle_enabled,
    _ioctl_to_endpoint,
    _read_host_uuid,
    read_local_topics,
)


def _make_info(node_name: str, qos_depth: int = 10,
               qos_is_transient_local: bool = False,
               qos_is_reliable: bool = True,
               is_bridge: bool = False) -> TopicInfoRet:
    info = TopicInfoRet()
    encoded = node_name.encode('utf-8')
    info.node_name = encoded + b'\x00' * (NODE_NAME_BUFFER_SIZE - len(encoded))
    info.qos_depth = qos_depth
    info.qos_is_transient_local = qos_is_transient_local
    info.qos_is_reliable = qos_is_reliable
    info.is_bridge = is_bridge
    return info


def test_ioctl_to_endpoint_copies_all_fields():
    info = _make_info(
        '/talker_node',
        qos_depth=7,
        qos_is_transient_local=True,
        qos_is_reliable=False,
        is_bridge=True,
    )
    ep = _ioctl_to_endpoint(info, '/chatter', 'pub')
    assert ep.node_name == '/talker_node'
    # No registry provided => pid stays 0.
    assert ep.pid == 0
    assert ep.qos_depth == 7
    assert ep.qos_is_transient_local is True
    assert ep.qos_is_reliable is False
    assert ep.is_bridge is True


def test_ioctl_to_endpoint_handles_short_name():
    info = _make_info('/x')
    ep = _ioctl_to_endpoint(info, '/chatter', 'pub')
    assert ep.node_name == '/x'


def test_ioctl_to_endpoint_fills_pid_from_registry():
    """When a registry entry matches (topic, role, node), the pid is filled."""
    from ros2agnocast_discovery_agent.type_registry import RegistryEntry

    class FakeRegistry:
        def lookup(self, topic, role, node):
            if (topic, role, node) == ('/chatter', 'pub', '/talker_node'):
                return RegistryEntry(pid=4242, type_name='std_msgs/msg/Int32')
            return None

    info = _make_info('/talker_node')
    ep = _ioctl_to_endpoint(info, '/chatter', 'pub', FakeRegistry())
    assert ep.pid == 4242


def _make_mock_lib(topic_to_endpoints: dict, topic_domains: dict | None = None) -> MagicMock:
    """Build a ctypes-flavoured mock that returns the given topic data.

    ``topic_domains`` optionally maps a topic name to its domain_id (default 0);
    the mock fills the wrapper-owned domain array returned via the out-pointer.
    """
    lib = MagicMock()
    topic_domains = topic_domains or {}

    topic_names = list(topic_to_endpoints.keys())

    name_storage = []
    for name in topic_names:
        buf = ctypes.create_string_buffer(name.encode('utf-8'))
        name_storage.append(buf)

    char_pp = (ctypes.POINTER(ctypes.c_char) * len(name_storage))(
        *(ctypes.cast(b, ctypes.POINTER(ctypes.c_char)) for b in name_storage))

    # Keep the domain arrays alive for the duration of the call (the wrapper would
    # own this memory; here the mock does).
    domain_storage = []

    def get_topics(count_ptr, domain_ids_out):
        count_ptr._obj.value = len(topic_names)
        arr = (ctypes.c_uint32 * len(topic_names))(
            *(topic_domains.get(name, 0) for name in topic_names))
        domain_storage.append(arr)
        domain_ids_out[0] = ctypes.cast(arr, ctypes.POINTER(ctypes.c_uint32))
        return char_pp

    lib.get_agnocast_topics = MagicMock(side_effect=get_topics)
    lib.free_agnocast_topics = MagicMock()
    lib.free_agnocast_topic_domains = MagicMock()

    def make_endpoints_getter(direction):
        def getter(topic_name_b, count_ptr, domain_id):
            name = topic_name_b.decode('utf-8')
            infos = topic_to_endpoints.get(name, {}).get(direction, [])
            count_ptr._obj.value = len(infos)
            if not infos:
                return ctypes.POINTER(TopicInfoRet)()
            arr = (TopicInfoRet * len(infos))(*infos)
            return ctypes.cast(arr, ctypes.POINTER(TopicInfoRet))
        return getter

    lib.get_agnocast_pub_nodes = MagicMock(side_effect=make_endpoints_getter('pub'))
    lib.get_agnocast_sub_nodes = MagicMock(side_effect=make_endpoints_getter('sub'))
    lib.free_agnocast_topic_info_ret = MagicMock()

    return lib


def test_read_local_topics_combines_pub_and_sub():
    pub_info = _make_info('/talker_node', qos_depth=3)
    sub_info = _make_info('/listener_node', qos_depth=5)

    lib = _make_mock_lib({
        '/chatter': {
            'pub': [pub_info],
            'sub': [sub_info],
        },
    })

    topics = read_local_topics(lib)
    assert len(topics) == 1
    topic = topics[0]
    assert topic.topic_name == '/chatter'
    # No registry passed => type stays empty.
    assert topic.type_name == ''
    assert topic.domain_id == 0
    assert len(topic.publishers) == 1
    assert topic.publishers[0].node_name == '/talker_node'
    assert topic.publishers[0].qos_depth == 3
    assert len(topic.subscribers) == 1
    assert topic.subscribers[0].node_name == '/listener_node'


def test_read_local_topics_stamps_domain_id():
    """The real domain_id from the ioctl is stamped onto the gossip topic."""
    pub_info = _make_info('/talker_node')
    lib = _make_mock_lib(
        {'/chatter': {'pub': [pub_info], 'sub': []}},
        topic_domains={'/chatter': 7})

    topics = read_local_topics(lib)
    assert len(topics) == 1
    assert topics[0].domain_id == 7
    # The per-topic domain is forwarded to the endpoint queries.
    _name, _count, domain_arg = lib.get_agnocast_pub_nodes.call_args.args
    assert domain_arg == 7


def test_read_local_topics_filters_by_own_domain():
    """A per-(NS, domain) agent reports only its own domain's topics."""
    lib = _make_mock_lib(
        {
            '/chatter': {'pub': [_make_info('/talker_d1')], 'sub': []},
            '/mrm': {'pub': [_make_info('/talker_d3')], 'sub': []},
        },
        topic_domains={'/chatter': 1, '/mrm': 3})

    topics = read_local_topics(lib, own_domain_id=1)
    assert [t.topic_name for t in topics] == ['/chatter']
    assert topics[0].domain_id == 1


def test_read_local_topics_resolves_type_from_registry():
    """A registry entry for any endpoint on the topic populates `type_name`."""
    from ros2agnocast_discovery_agent.type_registry import RegistryEntry

    pub_info = _make_info('/talker_node')
    lib = _make_mock_lib({'/chatter': {'pub': [pub_info], 'sub': []}})

    class FakeRegistry:
        def lookup(self, topic, role, node):
            if (topic, role, node) == ('/chatter', 'pub', '/talker_node'):
                return RegistryEntry(pid=99, type_name='std_msgs/msg/Int32')
            return None

    topics = read_local_topics(lib, FakeRegistry())
    assert len(topics) == 1
    assert topics[0].type_name == 'std_msgs/msg/Int32'
    assert topics[0].publishers[0].pid == 99


def test_read_local_topics_falls_back_to_subscriber_type_when_pub_missing():
    """If the registry only knows the subscriber side, `type_name` still resolves.

    Mirrors the case where the publisher process exited (or was on another NS
    and its registry file isn't local) but a local subscriber is still active.
    """
    from ros2agnocast_discovery_agent.type_registry import RegistryEntry

    pub_info = _make_info('/talker_node')
    sub_info = _make_info('/listener_node')
    lib = _make_mock_lib({'/chatter': {'pub': [pub_info], 'sub': [sub_info]}})

    class SubOnlyRegistry:
        def lookup(self, topic, role, node):
            if (topic, role, node) == ('/chatter', 'sub', '/listener_node'):
                return RegistryEntry(pid=77, type_name='std_msgs/msg/Int32')
            return None

    topics = read_local_topics(lib, SubOnlyRegistry())
    assert len(topics) == 1
    assert topics[0].type_name == 'std_msgs/msg/Int32'
    assert topics[0].subscribers[0].pid == 77
    assert topics[0].publishers[0].pid == 0  # publisher unknown to the registry


def test_read_local_topics_returns_empty_when_no_topics():
    lib = _make_mock_lib({})
    assert read_local_topics(lib) == []


def test_read_local_topics_handles_topic_without_subscribers():
    pub_info = _make_info('/orphan_pub_node')
    lib = _make_mock_lib({
        '/orphan_topic': {'pub': [pub_info], 'sub': []},
    })
    topics = read_local_topics(lib)
    assert len(topics) == 1
    assert topics[0].topic_name == '/orphan_topic'
    assert len(topics[0].publishers) == 1
    assert topics[0].subscribers == []


def test_read_host_uuid_returns_uuid_string():
    host_uuid = _read_host_uuid()
    assert isinstance(host_uuid, str)
    assert len(host_uuid) > 0
    # Either a valid UUID from /proc/sys/kernel/random/boot_id or a random
    # fallback; both should parse as UUIDs.
    import uuid
    uuid.UUID(host_uuid)


# ---------------------------------------------------------------------------
# Singleton claim (kmod-arbitrated) + idle-exit gate
# ---------------------------------------------------------------------------


def test_read_ros_domain_id_parses_env(monkeypatch):
    from ros2agnocast_discovery_agent.agent import _read_ros_domain_id
    monkeypatch.delenv('ROS_DOMAIN_ID', raising=False)
    assert _read_ros_domain_id() == 0
    monkeypatch.setenv('ROS_DOMAIN_ID', '7')
    assert _read_ros_domain_id() == 7
    monkeypatch.setenv('ROS_DOMAIN_ID', '')   # unset-equivalent -> default 0
    assert _read_ros_domain_id() == 0
    monkeypatch.setenv('ROS_DOMAIN_ID', 'abc')  # unparsable -> default 0
    assert _read_ros_domain_id() == 0
    monkeypatch.setenv('ROS_DOMAIN_ID', '-5')  # negative would wrap in uint32 -> default 0
    assert _read_ros_domain_id() == 0
    monkeypatch.setenv('ROS_DOMAIN_ID', str(2 ** 32))  # above uint32 max -> default 0
    assert _read_ros_domain_id() == 0
    monkeypatch.setenv('ROS_DOMAIN_ID', str(2 ** 32 - 1))  # uint32 max -> accepted as-is
    assert _read_ros_domain_id() == 2 ** 32 - 1


class _FakeSingletonLib:
    """Stand-in for the ioctl wrapper: agnocast_discovery_agent_register returns a set code."""

    def __init__(self, register_ret):
        self._register_ret = register_ret
        self.register_calls = []

    def agnocast_discovery_agent_register(self, domain_id):
        self.register_calls.append(domain_id)
        return self._register_ret


def test_main_exits_cleanly_when_claim_is_lost(monkeypatch):
    """A duplicate (register -> 1) returns 0 promptly, before any DDS / rclpy bring-up.

    Runs main() in a thread so a regression to a blocking wait is caught as a non-returning
    thread rather than hanging the run: a lost claim means another agent owns this (ns, domain),
    so main must return 0 without spinning.
    """
    from ros2agnocast_discovery_agent import agent
    fake = _FakeSingletonLib(register_ret=1)
    monkeypatch.setattr(agent, '_load_ioctl_wrapper', lambda: fake)

    result = {}
    t = threading.Thread(target=lambda: result.__setitem__('rc', agent.main(argv=[])))
    t.start()
    t.join(timeout=5.0)
    assert not t.is_alive(), 'main() did not return on a lost claim (blocked instead of exiting?)'
    assert result['rc'] == 0
    assert fake.register_calls  # it actually attempted the claim


def test_main_returns_error_when_claim_ioctl_fails(monkeypatch):
    """A register ioctl error (-1) propagates as a non-zero exit, not a silent 0."""
    from ros2agnocast_discovery_agent import agent
    fake = _FakeSingletonLib(register_ret=-1)
    monkeypatch.setattr(agent, '_load_ioctl_wrapper', lambda: fake)
    assert agent.main(argv=[]) == 1


def test_main_returns_error_when_wrapper_unavailable(monkeypatch):
    """A missing library or symbol (version skew) exits 1 cleanly, not with a traceback."""
    from ros2agnocast_discovery_agent import agent

    def _raise_oserror():
        raise OSError('libagnocast_ioctl_wrapper.so: cannot open shared object file')
    monkeypatch.setattr(agent, '_load_ioctl_wrapper', _raise_oserror)
    assert agent.main(argv=[]) == 1

    # spec=[] makes any attribute access (the missing symbol) raise AttributeError.
    monkeypatch.setattr(agent, '_load_ioctl_wrapper', lambda: MagicMock(spec=[]))
    assert agent.main(argv=[]) == 1


def _idle_gate_self(should_exit_ret, commit_ret):
    """Build a duck-typed DiscoveryAgent for exercising _maybe_exit_when_idle without rclpy/kmod.

    threshold=1 makes the idle tracker fire on the first idle tick, so one call reaches the gate.
    """
    lib = MagicMock()
    lib.agnocast_discovery_agent_should_exit.return_value = should_exit_ret
    lib.agnocast_discovery_agent_commit_exit.return_value = commit_ret
    return SimpleNamespace(
        _lib=lib, _domain_id=0, _ipc_ns_inode=1,
        _idle_tracker=IdleExitTracker(threshold=1), get_logger=lambda: MagicMock())


def test_idle_exit_commits_then_exits_when_domain_stays_empty():
    """Grace period elapsed and the kmod commit succeeds -> the agent exits."""
    fake_self = _idle_gate_self(should_exit_ret=1, commit_ret=1)
    with pytest.raises(ExternalShutdownException):
        DiscoveryAgent._maybe_exit_when_idle(fake_self)
    fake_self._lib.agnocast_discovery_agent_commit_exit.assert_called_once_with(0)


def test_idle_exit_vetoed_keeps_running_when_process_races_in():
    """The commit is vetoed (a process started during the grace period) -> keep running."""
    fake_self = _idle_gate_self(should_exit_ret=1, commit_ret=0)
    DiscoveryAgent._maybe_exit_when_idle(fake_self)  # must not raise
    fake_self._lib.agnocast_discovery_agent_commit_exit.assert_called_once_with(0)


def test_idle_exit_never_commits_on_query_error():
    """A should_exit error counts as not-idle: the gate is never reached, so we never exit."""
    fake_self = _idle_gate_self(should_exit_ret=-1, commit_ret=1)
    DiscoveryAgent._maybe_exit_when_idle(fake_self)  # must not raise
    fake_self._lib.agnocast_discovery_agent_commit_exit.assert_not_called()


# --- idle-exit (opt-in auto-fork cleanup) -----------------------------------

def test_idle_exit_tracker_fires_after_threshold():
    tracker = IdleExitTracker(threshold=3)
    assert tracker.update(True) is False
    assert tracker.update(True) is False
    assert tracker.update(True) is True  # third consecutive idle tick


def test_idle_exit_tracker_resets_on_activity():
    tracker = IdleExitTracker(threshold=3)
    tracker.update(True)
    tracker.update(True)
    assert tracker.update(False) is False  # a busy tick resets the count
    assert tracker.update(True) is False   # counting restarts from zero
    assert tracker.update(True) is False
    assert tracker.update(True) is True


def test_idle_exit_tracker_threshold_floor():
    # A non-positive threshold is clamped to 1, so a single idle tick fires.
    assert IdleExitTracker(threshold=0).update(True) is True


def test_exit_when_idle_enabled_via_env(monkeypatch):
    monkeypatch.setattr(sys, 'argv', ['discovery_agent'])  # no CLI flag
    monkeypatch.delenv(EXIT_WHEN_IDLE_ENV, raising=False)
    assert _exit_when_idle_enabled() is False
    for truthy in ('1', 'true', 'TRUE', 'yes'):
        monkeypatch.setenv(EXIT_WHEN_IDLE_ENV, truthy)
        assert _exit_when_idle_enabled() is True
    for falsy in ('0', 'false', '', 'no'):
        monkeypatch.setenv(EXIT_WHEN_IDLE_ENV, falsy)
        assert _exit_when_idle_enabled() is False


def test_exit_when_idle_enabled_via_cli_flag(monkeypatch):
    # The auto-fork passes the flag as an argv literal (no env set in the child).
    monkeypatch.delenv(EXIT_WHEN_IDLE_ENV, raising=False)
    monkeypatch.setattr(sys, 'argv', ['discovery_agent', EXIT_WHEN_IDLE_FLAG])
    assert _exit_when_idle_enabled() is True
