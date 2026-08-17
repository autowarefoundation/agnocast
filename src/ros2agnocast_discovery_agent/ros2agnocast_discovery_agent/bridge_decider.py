"""Decide and dispatch cross-namespace bridge requests for the discovery agent.

Each tick the agent compares the local Agnocast state with the remote
snapshots gathered over gossip. When a topic has an Agnocast endpoint locally
and the opposite-role endpoint in another namespace, a bridge is needed here
so the two reach each other through ROS 2 (DDS):

  * local publisher  + remote subscriber -> A2R bridge (publish to DDS)
  * local subscriber + remote publisher  -> R2A bridge (reinject from DDS)

The request is sent as a ``BridgeMsg`` (type=DaemonPubSub) to the per-namespace
bridge_manager over an abstract-namespace UNIX domain socket
(``\\0agnocast_bridge_manager_<ipc_ns_inode>[_d<domain>]``).
The struct layout is mirrored here so the daemon stays decoupled from
libagnocast's C++ headers; ``agnocast_bridge_msg.hpp`` owns the source of
truth and a test asserts the size stays in sync.
"""

from dataclasses import dataclass
import errno
import os
import socket
import struct
from typing import Iterable, Optional

TOPIC_NAME_BUFFER_SIZE = 256
MESSAGE_TYPE_BUFFER_SIZE = 256

# BridgeMsgType::DaemonPubSub discriminator value (matches the C++ enum).
_BRIDGE_MSG_TYPE_DAEMON_PUBSUB = 2

# BridgeMsg wire format for a DaemonPubSub-variant message (528 bytes total).
# The C++ BridgeMsg is `uint32_t type` + union { pubsub | service | daemon_pubsub }.
# All payload variants are 4-byte aligned so no padding precedes the union.
# Senders transmit only the bytes for the active variant, so a DaemonPubSub
# message is 4 (tag) + 524 (BridgeMsgDaemonPubSubPayload) = 528 bytes.
#
#   uint32 type                             [0..3]   = _BRIDGE_MSG_TYPE_DAEMON_PUBSUB
#   BridgeMsgDaemonPubSubPayload at union offset 4..527:
#     char[256] topic_name                  [4..259]
#     char[256] type_name                   [260..515]
#     uint32    direction                   [516..519]
#     uint32    qos_depth                   [520..523]
#     bool      qos_is_transient_local      [524]
#     bool      qos_is_reliable             [525]
#     2 bytes   padding                     [526..527]
#
# Must stay in sync with bridge_msg_wire_size<BridgeMsgDaemonPubSubPayload>() == 528.
_MSG_PACK_FORMAT = '=I256s256sIIBB2x'

DIRECTION_ROS2_TO_AGNOCAST = 0
DIRECTION_AGNOCAST_TO_ROS2 = 1

_BRIDGE_UDS_BASE = 'agnocast_bridge_manager'


@dataclass(frozen=True)
class BridgeRequest:
    topic_name: str
    type_name: str
    direction: int
    qos_depth: int
    qos_is_transient_local: bool
    qos_is_reliable: bool
    # Selects the target bridge_manager's UDS (one manager per domain); not part
    # of the wire payload, since that manager already runs in this domain.
    domain_id: int = 0


def serialize_request(req: BridgeRequest) -> bytes:
    topic = req.topic_name.encode('utf-8')[: TOPIC_NAME_BUFFER_SIZE - 1]
    type_name = req.type_name.encode('utf-8')[: MESSAGE_TYPE_BUFFER_SIZE - 1]
    return struct.pack(
        _MSG_PACK_FORMAT,
        _BRIDGE_MSG_TYPE_DAEMON_PUBSUB,
        topic,
        type_name,
        req.direction,
        req.qos_depth,
        1 if req.qos_is_transient_local else 0,
        1 if req.qos_is_reliable else 0,
    )


def _resolve_types(local_state, remote_states) -> dict:
    """Resolve each ``(topic, domain)``'s message type, preferring local then any remote.

    The type must be resolved across local + *all* remotes: the remote that
    supplies the opposite-role endpoint may lack the type while another snapshot
    has it.
    """
    types = {
        (t.topic_name, t.domain_id): t.type_name for t in local_state.topics if t.type_name}
    for remote in remote_states.values():
        for t in remote.topics:
            if t.type_name:
                types.setdefault((t.topic_name, t.domain_id), t.type_name)
    return types


def decide_bridges(local_state, remote_states) -> list:
    """Return the bridge requests this namespace should issue this tick.

    ``remote_states`` maps ``(host_uuid, ipc_ns_inode)`` to AgnocastDaemonState.
    Topics match only within the same domain (a bridge never crosses domains;
    cross-domain relaying is the external domain_bridge's job), and requests are
    collapsed to one per ``(topic, domain, direction)``.
    """
    requests = {}

    local_by_topic = {(t.topic_name, t.domain_id): t for t in local_state.topics}
    types = _resolve_types(local_state, remote_states)

    for (host_uuid, ipc_ns_inode), remote in remote_states.items():
        if host_uuid == local_state.host_uuid and ipc_ns_inode == local_state.ipc_ns_inode:
            continue
        for remote_topic in remote.topics:
            local_topic = local_by_topic.get((remote_topic.topic_name, remote_topic.domain_id))
            if local_topic is None:
                continue

            local_pubs = [p for p in local_topic.publishers if not p.is_bridge]
            local_subs = [s for s in local_topic.subscribers if not s.is_bridge]
            remote_pubs = [p for p in remote_topic.publishers if not p.is_bridge]
            remote_subs = [s for s in remote_topic.subscribers if not s.is_bridge]

            domain_id = local_topic.domain_id
            type_name = types.get((local_topic.topic_name, domain_id))
            if not type_name:
                continue

            if local_pubs and remote_subs:
                pub = local_pubs[0]
                key = (local_topic.topic_name, domain_id, DIRECTION_AGNOCAST_TO_ROS2)
                requests.setdefault(key, BridgeRequest(
                    topic_name=local_topic.topic_name,
                    type_name=type_name,
                    direction=DIRECTION_AGNOCAST_TO_ROS2,
                    qos_depth=pub.qos_depth,
                    qos_is_transient_local=pub.qos_is_transient_local,
                    qos_is_reliable=pub.qos_is_reliable,
                    domain_id=domain_id,
                ))

            if local_subs and remote_pubs:
                sub = local_subs[0]
                key = (local_topic.topic_name, domain_id, DIRECTION_ROS2_TO_AGNOCAST)
                requests.setdefault(key, BridgeRequest(
                    topic_name=local_topic.topic_name,
                    type_name=type_name,
                    direction=DIRECTION_ROS2_TO_AGNOCAST,
                    qos_depth=sub.qos_depth,
                    qos_is_transient_local=sub.qos_is_transient_local,
                    qos_is_reliable=sub.qos_is_reliable,
                    domain_id=domain_id,
                ))

    return list(requests.values())


def _bridge_uds_addr(ipc_ns_inode: int, domain_id: int) -> str:
    name = '\x00' + _BRIDGE_UDS_BASE + '_' + str(ipc_ns_inode)
    if domain_id:
        name += '_d' + str(domain_id)
    return name


def send_request(uds_addr: str, payload: bytes) -> Optional[str]:
    """Send ``payload`` to ``uds_addr``; return an error string or None.

    Transient failures (bridge_manager not yet bound, receiver buffer full)
    are swallowed since the request is re-issued idempotently next tick.
    """
    transient_errnos = (
        errno.ECONNREFUSED,
        errno.ENOENT,
        errno.EAGAIN,
        errno.EWOULDBLOCK,
        errno.ENOBUFS,
    )
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    sock.setblocking(False)
    try:
        try:
            sock.sendto(payload, uds_addr)
        except OSError as e:
            if e.errno in transient_errnos:
                return None
            return f'sendto({uds_addr!r}): {os.strerror(e.errno) if e.errno else str(e)}'
    finally:
        sock.close()
    return None


def dispatch_requests(
        requests: Iterable[BridgeRequest], ipc_ns_inode: int, logger=None) -> None:
    """Deliver each request to the per-namespace bridge_manager UDS.

    Each request goes to the manager that owns its (IPC namespace, domain).
    The listener UDS is absent until that bridge_manager is up;
    ``send_request`` swallows ECONNREFUSED/ENOENT so a missing peer never
    stalls the daemon, and the request is re-issued idempotently next tick.
    """
    for req in requests:
        err = send_request(
            _bridge_uds_addr(ipc_ns_inode, req.domain_id), serialize_request(req))
        if err is not None and logger is not None:
            logger.warn('daemon bridge dispatch failed: %s', err)
