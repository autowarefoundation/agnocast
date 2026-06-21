"""Decide and dispatch cross-namespace bridge requests for the discovery agent.

Each tick the agent compares the local Agnocast state with the remote
snapshots gathered over gossip. When a topic has an Agnocast endpoint locally
and the opposite-role endpoint in another namespace, a bridge is needed here
so the two reach each other through ROS 2 (DDS):

  * local publisher  + remote subscriber -> A2R bridge (publish to DDS)
  * local subscriber + remote publisher  -> R2A bridge (reinject from DDS)

The request is sent as ``MqMsgDaemonBridge`` to the per-namespace bridge_manager
MQ. The struct layout is mirrored here so the daemon stays decoupled from
libagnocast's C++ headers; ``agnocast_mq.hpp`` owns the source of truth and a
test asserts the size stays in sync.
"""

import ctypes
from dataclasses import dataclass
import errno
import os
import struct
from typing import Iterable, Optional

TOPIC_NAME_BUFFER_SIZE = 256
MESSAGE_TYPE_BUFFER_SIZE = 256

# char topic_name[256]; char type_name[256]; uint32 direction; uint32 qos_depth;
# bool qos_is_transient_local; bool qos_is_reliable; + 2 bytes tail padding so
# the total matches sizeof(MqMsgDaemonBridge) == 524 on the C++ side.
_MSG_PACK_FORMAT = '=256s256sIIBB2x'

DIRECTION_ROS2_TO_AGNOCAST = 0
DIRECTION_AGNOCAST_TO_ROS2 = 1

# One bridge_manager per IPC namespace listens on this MQ.
_PERFORMANCE_MQ_NAME = '/agnocast_daemon_bridge_perf'

# librt mq_* loaded lazily to keep the daemon's deps at "rclpy + stdlib".
_librt = None


def _load_librt():
    global _librt
    if _librt is not None:
        return _librt
    lib = ctypes.CDLL('librt.so.1', use_errno=True)
    lib.mq_open.argtypes = [ctypes.c_char_p, ctypes.c_int]
    lib.mq_open.restype = ctypes.c_int
    lib.mq_send.argtypes = [ctypes.c_int, ctypes.c_char_p, ctypes.c_size_t, ctypes.c_uint]
    lib.mq_send.restype = ctypes.c_int
    lib.mq_close.argtypes = [ctypes.c_int]
    lib.mq_close.restype = ctypes.c_int
    _librt = lib
    return _librt


@dataclass(frozen=True)
class BridgeRequest:
    topic_name: str
    type_name: str
    direction: int
    qos_depth: int
    qos_is_transient_local: bool
    qos_is_reliable: bool


def serialize_request(req: BridgeRequest) -> bytes:
    topic = req.topic_name.encode('utf-8')[: TOPIC_NAME_BUFFER_SIZE - 1]
    type_name = req.type_name.encode('utf-8')[: MESSAGE_TYPE_BUFFER_SIZE - 1]
    return struct.pack(
        _MSG_PACK_FORMAT,
        topic,
        type_name,
        req.direction,
        req.qos_depth,
        1 if req.qos_is_transient_local else 0,
        1 if req.qos_is_reliable else 0,
    )


def _resolve_types(local_state, remote_states) -> dict:
    """Resolve each topic's message type, preferring local then any remote.

    A bridge is deduped per ``(topic, direction)``, so its type must be resolved
    per-topic across local + *all* remotes: the remote that supplies the
    opposite-role endpoint may lack the type while another snapshot has it.
    """
    types = {t.topic_name: t.type_name for t in local_state.topics if t.type_name}
    for remote in remote_states.values():
        for t in remote.topics:
            if t.type_name:
                types.setdefault(t.topic_name, t.type_name)
    return types


def decide_bridges(local_state, remote_states) -> list:
    """Return the bridge requests this namespace should issue this tick.

    ``remote_states`` maps ``(host_uuid, ipc_ns_inode)`` to AgnocastDaemonState.
    Requests are collapsed to one per ``(topic, direction)``.
    """
    requests = {}

    local_by_topic = {t.topic_name: t for t in local_state.topics}
    types = _resolve_types(local_state, remote_states)

    for (host_uuid, ipc_ns_inode), remote in remote_states.items():
        if host_uuid == local_state.host_uuid and ipc_ns_inode == local_state.ipc_ns_inode:
            continue
        for remote_topic in remote.topics:
            local_topic = local_by_topic.get(remote_topic.topic_name)
            if local_topic is None:
                continue

            local_pubs = [p for p in local_topic.publishers if not p.is_bridge]
            local_subs = [s for s in local_topic.subscribers if not s.is_bridge]
            remote_pubs = [p for p in remote_topic.publishers if not p.is_bridge]
            remote_subs = [s for s in remote_topic.subscribers if not s.is_bridge]

            type_name = types.get(local_topic.topic_name)
            if not type_name:
                continue

            if local_pubs and remote_subs:
                pub = local_pubs[0]
                key = (local_topic.topic_name, DIRECTION_AGNOCAST_TO_ROS2)
                requests.setdefault(key, BridgeRequest(
                    topic_name=local_topic.topic_name,
                    type_name=type_name,
                    direction=DIRECTION_AGNOCAST_TO_ROS2,
                    qos_depth=pub.qos_depth,
                    qos_is_transient_local=pub.qos_is_transient_local,
                    qos_is_reliable=pub.qos_is_reliable,
                ))

            if local_subs and remote_pubs:
                sub = local_subs[0]
                key = (local_topic.topic_name, DIRECTION_ROS2_TO_AGNOCAST)
                requests.setdefault(key, BridgeRequest(
                    topic_name=local_topic.topic_name,
                    type_name=type_name,
                    direction=DIRECTION_ROS2_TO_AGNOCAST,
                    qos_depth=sub.qos_depth,
                    qos_is_transient_local=sub.qos_is_transient_local,
                    qos_is_reliable=sub.qos_is_reliable,
                ))

    return list(requests.values())


def _performance_mq_name() -> str:
    name = _PERFORMANCE_MQ_NAME
    domain_id = os.environ.get('ROS_DOMAIN_ID')
    if domain_id:
        name += '_d' + domain_id
    return name


def send_request(mq_name: str, payload: bytes) -> Optional[str]:
    """Send ``payload`` to ``mq_name``; return an error string or None.

    O_NONBLOCK keeps a full or absent queue from stalling the daemon: the
    request is re-issued idempotently next tick.
    """
    lib = _load_librt()
    fd = lib.mq_open(mq_name.encode('utf-8'), os.O_WRONLY | os.O_NONBLOCK)
    if fd == -1:
        err = ctypes.get_errno()
        if err == errno.ENOENT:
            return None
        return f'mq_open({mq_name}): {os.strerror(err)}'
    try:
        if lib.mq_send(fd, payload, len(payload), 0) == -1:
            err = ctypes.get_errno()
            if err == errno.EAGAIN:
                return None
            return f'mq_send({mq_name}): {os.strerror(err)}'
    finally:
        lib.mq_close(fd)
    return None


def dispatch_requests(requests: Iterable[BridgeRequest], logger=None) -> None:
    """Deliver each request to the per-namespace bridge_manager MQ.

    The MQ is absent until a bridge_manager is up; ``send_request`` skips
    ENOENT/EAGAIN so a missing or full queue never stalls the daemon, and the
    request is re-issued idempotently next tick.
    """
    perf_mq = _performance_mq_name()
    for req in requests:
        err = send_request(perf_mq, serialize_request(req))
        if err is not None and logger is not None:
            logger.warn('daemon bridge dispatch failed: %s', err)
