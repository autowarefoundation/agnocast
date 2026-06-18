"""Unit tests for the discovery agent's ioctl-to-msg conversion and gossip wiring.

These tests do not require the kmod or DDS; they exercise the conversion
helpers directly with a mock ctypes library.
"""

import ctypes
import threading
from unittest.mock import MagicMock

import pytest

from ros2agnocast_discovery_agent.agent import (
    NODE_NAME_BUFFER_SIZE,
    TopicInfoRet,
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
# Singleton lock
# ---------------------------------------------------------------------------


def test_singleton_lock_path_honors_tmpfs_dir(monkeypatch, tmp_path):
    monkeypatch.setenv('AGNOCAST_TMPFS_DIR', str(tmp_path))
    from ros2agnocast_discovery_agent.agent import _singleton_lock_path
    assert _singleton_lock_path(42, 3) == str(tmp_path / 'agnocast_discovery_agent_42_d3.lock')


def test_singleton_lock_path_defaults_to_dev_shm(monkeypatch):
    monkeypatch.delenv('AGNOCAST_TMPFS_DIR', raising=False)
    from ros2agnocast_discovery_agent.agent import _singleton_lock_path
    assert _singleton_lock_path(42, 0) == '/dev/shm/agnocast_discovery_agent_42_d0.lock'


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


def test_acquire_singleton_lock_succeeds_when_free(monkeypatch, tmp_path):
    monkeypatch.setenv('AGNOCAST_TMPFS_DIR', str(tmp_path))
    from ros2agnocast_discovery_agent.agent import LockStatus, _try_acquire_singleton_lock
    attempt = _try_acquire_singleton_lock(123, 0)
    assert attempt.status == LockStatus.ACQUIRED
    attempt.file.close()


def test_acquire_singleton_lock_blocks_second_attempt(monkeypatch, tmp_path):
    """A second acquire in the same (NS, domain) reports HELD while the first is alive."""
    monkeypatch.setenv('AGNOCAST_TMPFS_DIR', str(tmp_path))
    from ros2agnocast_discovery_agent.agent import LockStatus, _try_acquire_singleton_lock
    first = _try_acquire_singleton_lock(456, 0)
    assert first.status == LockStatus.ACQUIRED
    assert _try_acquire_singleton_lock(456, 0).status == LockStatus.HELD
    first.file.close()
    # After releasing, a new acquire succeeds.
    third = _try_acquire_singleton_lock(456, 0)
    assert third.status == LockStatus.ACQUIRED
    third.file.close()


def test_acquire_singleton_lock_independent_per_ipc_ns(monkeypatch, tmp_path):
    """Different IPC NS inodes get independent locks."""
    monkeypatch.setenv('AGNOCAST_TMPFS_DIR', str(tmp_path))
    from ros2agnocast_discovery_agent.agent import LockStatus, _try_acquire_singleton_lock
    lock_a = _try_acquire_singleton_lock(111, 0)
    lock_b = _try_acquire_singleton_lock(222, 0)
    assert lock_a.status == LockStatus.ACQUIRED
    assert lock_b.status == LockStatus.ACQUIRED
    lock_a.file.close()
    lock_b.file.close()


def test_acquire_singleton_lock_independent_per_domain(monkeypatch, tmp_path):
    """Same IPC NS but different ROS_DOMAIN_ID get independent locks, so two
    launches sharing a namespace in different domains each run their own agent."""
    monkeypatch.setenv('AGNOCAST_TMPFS_DIR', str(tmp_path))
    from ros2agnocast_discovery_agent.agent import LockStatus, _try_acquire_singleton_lock
    lock_d1 = _try_acquire_singleton_lock(111, 1)
    lock_d3 = _try_acquire_singleton_lock(111, 3)
    assert lock_d1.status == LockStatus.ACQUIRED
    assert lock_d3.status == LockStatus.ACQUIRED
    lock_d1.file.close()
    lock_d3.file.close()


def test_acquire_singleton_lock_reports_error_on_unwritable_dir(monkeypatch):
    """When the lock-file directory is not writable we report ERROR (distinct
    from HELD) so the caller can surface a non-zero exit code."""
    monkeypatch.setenv('AGNOCAST_TMPFS_DIR', '/nonexistent_path_for_agnocast_test')
    from ros2agnocast_discovery_agent.agent import LockStatus, _try_acquire_singleton_lock
    assert _try_acquire_singleton_lock(789, 0).status == LockStatus.ERROR


def test_main_exits_promptly_when_another_agent_holds_the_lock(monkeypatch, tmp_path):
    """A duplicate (lock already held) returns 0 promptly instead of idling.

    Runs main() in a thread so a regression to the old ``signal.pause()``
    (which would block forever here) is caught as a non-returning thread
    rather than hanging the whole test run. main() returns before any DDS /
    ioctl bring-up, so this needs neither the kmod nor rclpy.
    """
    monkeypatch.setenv('AGNOCAST_TMPFS_DIR', str(tmp_path))
    from ros2agnocast_discovery_agent.agent import (
        LockStatus, _read_ipc_ns_inode, _read_ros_domain_id,
        _try_acquire_singleton_lock, main)

    holder = _try_acquire_singleton_lock(_read_ipc_ns_inode(), _read_ros_domain_id())
    assert holder.status == LockStatus.ACQUIRED
    try:
        result = {}
        t = threading.Thread(target=lambda: result.__setitem__('rc', main(argv=[])))
        t.start()
        t.join(timeout=5.0)
        assert not t.is_alive(), 'main() did not return (regressed to signal.pause()?)'
        assert result['rc'] == 0
    finally:
        holder.file.close()
