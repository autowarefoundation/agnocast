"""Unit tests for the discovery agent's ioctl-to-msg conversion and gossip wiring.

These tests do not require the kmod or DDS; they exercise the conversion
helpers directly with a mock ctypes library.
"""

import ctypes
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
    ep = _ioctl_to_endpoint(info)
    assert ep.node_name == '/talker_node'
    assert ep.pid == 0  # best-effort empty
    assert ep.qos_depth == 7
    assert ep.qos_is_transient_local is True
    assert ep.qos_is_reliable is False
    assert ep.is_bridge is True


def test_ioctl_to_endpoint_handles_short_name():
    info = _make_info('/x')
    ep = _ioctl_to_endpoint(info)
    assert ep.node_name == '/x'


def _make_mock_lib(topic_to_endpoints: dict) -> MagicMock:
    """Build a ctypes-flavoured mock that returns the given topic data."""
    lib = MagicMock()

    topic_names = list(topic_to_endpoints.keys())

    name_storage = []
    for name in topic_names:
        buf = ctypes.create_string_buffer(name.encode('utf-8'))
        name_storage.append(buf)

    char_pp = (ctypes.POINTER(ctypes.c_char) * len(name_storage))(
        *(ctypes.cast(b, ctypes.POINTER(ctypes.c_char)) for b in name_storage))

    def get_topics(count_ptr):
        count_ptr._obj.value = len(topic_names)
        return char_pp

    lib.get_agnocast_topics = MagicMock(side_effect=get_topics)
    lib.free_agnocast_topics = MagicMock()

    def make_endpoints_getter(direction):
        def getter(topic_name_b, count_ptr):
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
    assert topic.type_name == ''  # best-effort empty
    assert topic.domain_id == 0
    assert len(topic.publishers) == 1
    assert topic.publishers[0].node_name == '/talker_node'
    assert topic.publishers[0].qos_depth == 3
    assert len(topic.subscribers) == 1
    assert topic.subscribers[0].node_name == '/listener_node'


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
