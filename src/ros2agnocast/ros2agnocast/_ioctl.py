"""ioctl fallback: synthesise an AgnocastDaemonState for the caller's NS."""

from contextlib import contextmanager
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


class _TopicInfoRet(ctypes.Structure):
    _fields_ = [
        ('node_name', ctypes.c_char * 256),
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
    lib.get_agnocast_topics.argtypes = [ctypes.POINTER(ctypes.c_int)]
    lib.get_agnocast_topics.restype = ctypes.POINTER(ctypes.POINTER(ctypes.c_char))
    lib.free_agnocast_topics.argtypes = [
        ctypes.POINTER(ctypes.POINTER(ctypes.c_char)), ctypes.c_int]
    lib.free_agnocast_topics.restype = None
    lib.get_agnocast_sub_nodes.argtypes = [ctypes.c_char_p, ctypes.POINTER(ctypes.c_int)]
    lib.get_agnocast_sub_nodes.restype = ctypes.POINTER(_TopicInfoRet)
    lib.get_agnocast_pub_nodes.argtypes = [ctypes.c_char_p, ctypes.POINTER(ctypes.c_int)]
    lib.get_agnocast_pub_nodes.restype = ctypes.POINTER(_TopicInfoRet)
    lib.free_agnocast_topic_info_ret.argtypes = [ctypes.POINTER(_TopicInfoRet)]
    lib.free_agnocast_topic_info_ret.restype = None
    return lib


def _endpoint_from_topic_info_ret(tir: _TopicInfoRet) -> AgnocastEndpoint:
    ep = AgnocastEndpoint()
    ep.node_name = tir.node_name.decode('utf-8').rstrip('\x00')
    ep.pid = 0
    ep.qos_depth = tir.qos_depth
    ep.qos_is_transient_local = tir.qos_is_transient_local
    ep.qos_is_reliable = tir.qos_is_reliable
    ep.is_bridge = tir.is_bridge
    return ep


@contextmanager
def _endpoints(lib, fn, topic_name: str):
    """Yield a list[AgnocastEndpoint] for one topic; free the ctypes buffer on exit."""
    count = ctypes.c_int()
    arr = fn(topic_name.encode('utf-8'), ctypes.byref(count))
    try:
        if not arr:
            yield []
            return
        yield [_endpoint_from_topic_info_ret(arr[i]) for i in range(count.value)]
    finally:
        if arr:
            lib.free_agnocast_topic_info_ret(arr)


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

    topic_count = ctypes.c_int()
    topic_array = lib.get_agnocast_topics(ctypes.byref(topic_count))
    try:
        for i in range(topic_count.value):
            tname = ctypes.cast(topic_array[i], ctypes.c_char_p).value.decode('utf-8')
            t = AgnocastTopic()
            t.topic_name = tname
            t.type_name = ''
            t.domain_id = 0
            with _endpoints(lib, lib.get_agnocast_pub_nodes, tname) as pubs:
                t.publishers = pubs
            with _endpoints(lib, lib.get_agnocast_sub_nodes, tname) as subs:
                t.subscribers = subs
            state.topics.append(t)
    finally:
        if topic_count.value != 0:
            lib.free_agnocast_topics(topic_array, topic_count)
    return state
