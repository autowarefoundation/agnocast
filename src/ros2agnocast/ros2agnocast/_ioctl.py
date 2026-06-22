"""ioctl fallback: synthesise an AgnocastDaemonState for the caller's NS."""

import ctypes
import os
from typing import Optional
import uuid

from ros2agnocast_discovery_msgs.msg import (
    AgnocastDaemonState,
    AgnocastEndpoint,
    AgnocastTopic,
)


BOOT_ID_PATH = '/proc/sys/kernel/random/boot_id'
SELF_IPC_NS_PATH = '/proc/self/ns/ipc'


class _SnapshotTopicRet(ctypes.Structure):
    """Mirror of ``struct snapshot_topic_ret`` in agnocast.h / agnocast_ioctl.hpp."""

    _fields_ = [
        ('topic_name', ctypes.c_char * 256),
        ('publisher_num', ctypes.c_uint32),
        ('subscriber_num', ctypes.c_uint32),
        ('ros2_publisher_num', ctypes.c_uint32),
        ('ros2_subscriber_num', ctypes.c_uint32),
    ]


class _SnapshotEndpointRet(ctypes.Structure):
    """Mirror of ``struct snapshot_endpoint_ret``. ``pid`` is pid_t (== int32)."""

    _fields_ = [
        ('node_name', ctypes.c_char * 256),
        ('pid', ctypes.c_int32),
        ('qos_depth', ctypes.c_uint32),
        ('qos_is_transient_local', ctypes.c_bool),
        ('qos_is_reliable', ctypes.c_bool),
        ('is_bridge', ctypes.c_bool),
    ]


def _load_lib() -> Optional[ctypes.CDLL]:
    """Return the configured ioctl wrapper, or None on absent kmod."""
    try:
        lib = ctypes.CDLL('libagnocast_ioctl_wrapper.so')
    except OSError:
        return None
    lib.get_agnocast_discovery_snapshot.argtypes = [
        ctypes.POINTER(ctypes.POINTER(_SnapshotTopicRet)), ctypes.POINTER(ctypes.c_int),
        ctypes.POINTER(ctypes.POINTER(_SnapshotEndpointRet)), ctypes.POINTER(ctypes.c_int),
    ]
    lib.get_agnocast_discovery_snapshot.restype = ctypes.c_int
    lib.free_agnocast_discovery_snapshot.argtypes = [
        ctypes.POINTER(_SnapshotTopicRet), ctypes.POINTER(_SnapshotEndpointRet)]
    lib.free_agnocast_discovery_snapshot.restype = None
    return lib


def _endpoint_from_snapshot(ep_ret: _SnapshotEndpointRet) -> AgnocastEndpoint:
    ep = AgnocastEndpoint()
    ep.node_name = ep_ret.node_name.decode('utf-8').rstrip('\x00')
    ep.pid = ep_ret.pid
    ep.qos_depth = ep_ret.qos_depth
    ep.qos_is_transient_local = ep_ret.qos_is_transient_local
    ep.qos_is_reliable = ep_ret.qos_is_reliable
    ep.is_bridge = ep_ret.is_bridge
    return ep


def _local_host_uuid() -> str:
    try:
        with open(BOOT_ID_PATH) as fp:
            return str(uuid.UUID(fp.read().strip()))
    except (OSError, ValueError):
        return ''


def _local_ipc_ns_inode() -> int:
    return os.stat(SELF_IPC_NS_PATH).st_ino


def self_ns_snapshot() -> Optional[AgnocastDaemonState]:
    """Build a snapshot from ioctl, or None if the wrapper is unavailable."""
    lib = _load_lib()
    if lib is None:
        return None

    state = AgnocastDaemonState()
    state.schema_version = 1
    state.agnocast_version = ''
    state.host_uuid = _local_host_uuid()
    state.host_hostname = ''
    state.ipc_ns_inode = _local_ipc_ns_inode()
    state.topics = []

    # One ioctl returns every topic plus its pub/sub endpoints. The endpoint array is laid out
    # in topic order as "publishers then subscribers", sliced via publisher_num / subscriber_num.
    topics_ptr = ctypes.POINTER(_SnapshotTopicRet)()
    topic_count = ctypes.c_int()
    endpoints_ptr = ctypes.POINTER(_SnapshotEndpointRet)()
    endpoint_count = ctypes.c_int()
    rc = lib.get_agnocast_discovery_snapshot(
        ctypes.byref(topics_ptr), ctypes.byref(topic_count),
        ctypes.byref(endpoints_ptr), ctypes.byref(endpoint_count))
    if rc != 0 or topic_count.value == 0:
        # Error or empty namespace: best-effort empty snapshot (matches old behaviour).
        return state

    try:
        cursor = 0
        for i in range(topic_count.value):
            topic_ret = topics_ptr[i]
            topic = AgnocastTopic()
            topic.topic_name = topic_ret.topic_name.decode('utf-8').rstrip('\x00')
            topic.type_name = ''
            topic.domain_id = 0
            topic.publishers = [
                _endpoint_from_snapshot(endpoints_ptr[cursor + j])
                for j in range(topic_ret.publisher_num)]
            cursor += topic_ret.publisher_num
            topic.subscribers = [
                _endpoint_from_snapshot(endpoints_ptr[cursor + j])
                for j in range(topic_ret.subscriber_num)]
            cursor += topic_ret.subscriber_num
            state.topics.append(topic)
    finally:
        lib.free_agnocast_discovery_snapshot(topics_ptr, endpoints_ptr)
    return state
