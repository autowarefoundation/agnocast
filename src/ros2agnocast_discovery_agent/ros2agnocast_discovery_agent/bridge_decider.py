"""Decide and dispatch cross-namespace bridge requests for the discovery agent.

Each tick the agent compares the local Agnocast state with the remote
snapshots gathered over gossip. When a topic has an Agnocast endpoint locally
and the opposite-role endpoint in another namespace, a bridge is needed here
so the two reach each other through ROS 2 (DDS):

  * local publisher  + remote subscriber -> A2R bridge (publish to DDS)
  * local subscriber + remote publisher  -> R2A bridge (reinject from DDS)

Agnocast services ride on an internal request topic, so the same comparison decides them; see
``decide_service_bridges``, which emits type=DaemonService instead.

The domain bridge rule config is a second, gossip-independent source of requests.

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
SERVICE_NAME_BUFFER_SIZE = 256

# BridgeMsgType discriminator values (match the C++ enum).
_BRIDGE_MSG_TYPE_DAEMON_PUBSUB = 2
_BRIDGE_MSG_TYPE_DAEMON_SERVICE = 3

# Only the request topic identifies a service; response topics are per-client.
SRV_REQUEST_PREFIX = '/AGNOCAST_SRV_REQUEST'
SRV_RESPONSE_PREFIX = '/AGNOCAST_SRV_RESPONSE'

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

# BridgeMsg wire format for a DaemonService-variant message (260 bytes total). It always means
# "lease R2A for this service".
#
#   uint32 type                             [0..3]   = _BRIDGE_MSG_TYPE_DAEMON_SERVICE
#   BridgeMsgDaemonServicePayload at union offset 4..259:
#     char[256] service_name                [4..259]
#
# Must stay in sync with bridge_msg_wire_size<BridgeMsgDaemonServicePayload>() == 260.
_SERVICE_MSG_PACK_FORMAT = '=I256s'

DIRECTION_ROS2_TO_AGNOCAST = 0
DIRECTION_AGNOCAST_TO_ROS2 = 1

_BRIDGE_UDS_BASE = 'agnocast_bridge_manager'


@dataclass(frozen=True)
class ServiceBridgeRequest:
    service_name: str
    domain_id: int = 0


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


def serialize_service_request(req: ServiceBridgeRequest) -> bytes:
    name = req.service_name.encode('utf-8')[: SERVICE_NAME_BUFFER_SIZE - 1]
    return struct.pack(_SERVICE_MSG_PACK_FORMAT, _BRIDGE_MSG_TYPE_DAEMON_SERVICE, name)


def _service_name_of(topic_name: str) -> Optional[str]:
    """Return the service a request topic belongs to, or None if it is not one."""
    if not topic_name.startswith(SRV_REQUEST_PREFIX):
        return None
    service_name = topic_name[len(SRV_REQUEST_PREFIX):]
    # A bare prefix names no service; anything else must be an absolute service name.
    return service_name if service_name.startswith('/') else None


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
            # Handled by decide_service_bridges(), not as plain pub/sub.
            if remote_topic.topic_name.startswith(
                    (SRV_REQUEST_PREFIX, SRV_RESPONSE_PREFIX)):
                continue

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


def decide_service_bridges(local_state, remote_states) -> list:
    """Return the service bridge requests this namespace should issue this tick.

    Roles come from the request topic, which every Agnocast service subscribes to and every
    Agnocast client publishes on: a local subscriber means the service lives here, and only that
    side is leased. The client side's A2R comes up on the ordinary demand rule once the leased R2A
    publishes the ROS 2 service it was waiting for, and must not be forced ahead of it --
    ``agnocast_service_bridge.hpp`` has the argument.

    Requests are collapsed to one per ``(service, domain)``, and matched within a domain only, as
    for pub/sub.
    """
    requests = {}

    local_by_topic = {(t.topic_name, t.domain_id): t for t in local_state.topics}

    for (host_uuid, ipc_ns_inode), remote in remote_states.items():
        if host_uuid == local_state.host_uuid and ipc_ns_inode == local_state.ipc_ns_inode:
            continue
        for remote_topic in remote.topics:
            service_name = _service_name_of(remote_topic.topic_name)
            if service_name is None:
                continue

            local_topic = local_by_topic.get((remote_topic.topic_name, remote_topic.domain_id))
            if local_topic is None:
                continue

            domain_id = local_topic.domain_id

            # Counting a bridge's own endpoints would keep every lease alive off its own bridge.
            local_subs = [s for s in local_topic.subscribers if not s.is_bridge]
            remote_pubs = [p for p in remote_topic.publishers if not p.is_bridge]

            if local_subs and remote_pubs:
                key = (service_name, domain_id)
                requests.setdefault(
                    key, ServiceBridgeRequest(service_name=service_name, domain_id=domain_id))

    return list(requests.values())


# At ~1 Hz, long enough that the ordinary startup gap -- the agent is forked by the first
# Agnocast process, so it ticks before that process has created its publishers -- passes at info.
UNFORCED_WARN_AFTER_TICKS = 30


def _note_unforced(logger, reported, cell, reason, seen) -> None:
    """Report why a rule forced nothing, escalating only once the gap persists.

    Both "no local topic" and "type not known" are the normal state at startup, so warning on the
    first tick would cry wolf. The first sighting is info; the gap still standing after
    ``UNFORCED_WARN_AFTER_TICKS`` warns once, and nothing in between is logged.

    The count is per cell per tick: ``seen`` holds the cells already noted in this pass, since two
    rules can name one cell. It counts consecutive unforced ticks, not how long one reason has
    held, so a gap that alternates between reasons still escalates; a changed reason is worth an
    info line, not a restart. A cell that alternates between forced and unforced does restart,
    since forcing clears the entry, and so stays at info.
    """
    if logger is None:
        return
    topic, domain = cell
    text = f'no cross-domain bridge forced for {topic}@{domain}: {reason}'
    if cell in seen:
        return
    seen.add(cell)

    seen_reason, ticks = reported.get(cell, (None, 0))
    ticks += 1
    reported[cell] = (reason, ticks)
    if ticks == UNFORCED_WARN_AFTER_TICKS:
        # The warn carries the current reason, so a change on this tick loses nothing.
        logger.warn(f'{text} (unforced for {ticks} ticks)')
    elif seen_reason != reason:
        logger.info(text)


def _note_forced(logger, reported, cell) -> None:
    """Close the loop when a rule starts forcing, so a reported gap does not just stop being said."""
    if reported.pop(cell, None) is not None and logger is not None:
        topic, domain = cell
        logger.info(f'cross-domain bridge now forced for {topic}@{domain}')


def decide_domain_rule_bridges(
        local_state, rules, domain_id=None, logger=None, reported=None) -> list:
    """Return the A2R requests implied by the registered domain bridge rules.

    Only the ``from`` side is forced: there domain_bridge waits for a DDS
    publisher while the A2R bridge waits for a DDS subscriber, so neither starts.
    The ``to`` side is left to the ordinary on-demand check.

    Forcing is unconditional: gossip never crosses domains, so there is no
    evidence here of a subscriber in ``to_domain``.

    A rule that forces nothing is reported through ``logger``, with ``reported`` (a caller-owned
    dict) suppressing the repeat every tick. Skipping in silence would reproduce the symptom this
    forcing exists to remove: a topic that does not flow, with no trace of why.

    ``domain_id`` is the caller's own domain. A rule whose ``from`` side belongs to another one is
    not this agent's to force and is skipped without a word -- ``bidirectional`` splits an entry
    into a tuple per direction, so exactly one of each pair is always someone else's.
    """
    requests = {}
    # Cells already noted this pass, so two rules naming one cell count as one tick.
    seen = set()
    # Normalised once so the helpers never branch on it; a caller that omits it simply gets no
    # counting across ticks.
    if reported is None:
        reported = {}
    local_by_topic = {(t.topic_name, t.domain_id): t for t in local_state.topics}

    for from_topic, _to_topic, from_domain, to_domain in rules:
        if from_domain == to_domain:
            continue

        if domain_id is not None and from_domain != domain_id:
            continue

        cell = (from_topic, from_domain)
        local_topic = local_by_topic.get(cell)
        if local_topic is None:
            _note_unforced(logger, reported, cell, 'no local topic in this domain', seen)
            continue
        if not local_topic.type_name:
            # The type comes from the tmpfs registry: empty means the topic has not registered
            # yet, which is ordinary at startup, or that the join failed.
            _note_unforced(logger, reported, cell, 'the topic type is not known yet', seen)
            continue

        local_pubs = [p for p in local_topic.publishers if not p.is_bridge]
        if not local_pubs:
            _note_unforced(logger, reported, cell, 'no local publisher other than a bridge', seen)
            continue

        _note_forced(logger, reported, cell)

        pub = local_pubs[0]
        key = (from_topic, from_domain, DIRECTION_AGNOCAST_TO_ROS2)
        requests.setdefault(key, BridgeRequest(
            topic_name=from_topic,
            type_name=local_topic.type_name,
            direction=DIRECTION_AGNOCAST_TO_ROS2,
            qos_depth=pub.qos_depth,
            qos_is_transient_local=pub.qos_is_transient_local,
            qos_is_reliable=pub.qos_is_reliable,
            domain_id=from_domain,
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


def dispatch_service_requests(
        requests: Iterable[ServiceBridgeRequest], ipc_ns_inode: int, logger=None) -> None:
    """Deliver each service request to the per-namespace bridge_manager UDS.

    Same delivery contract as ``dispatch_requests``: best-effort, re-issued idempotently every
    tick, and a missing peer never stalls the daemon.
    """
    for req in requests:
        err = send_request(
            _bridge_uds_addr(ipc_ns_inode, req.domain_id), serialize_service_request(req))
        if err is not None and logger is not None:
            logger.warn(f'daemon service bridge dispatch failed: {err}')


def dispatch_requests(
        requests: Iterable[BridgeRequest], ipc_ns_inode: int, logger=None) -> None:
    """Deliver each request to the per-namespace bridge_manager UDS.

    Each request goes to the manager that owns its (IPC namespace, domain).
    The listener UDS is absent until that bridge_manager is up;
    ``send_request`` swallows ECONNREFUSED/ENOENT so a missing peer never
    stalls the daemon, and the request is re-issued idempotently next tick.
    """
    # decide_bridges and decide_domain_rule_bridges can both ask for the same bridge.
    for req in dict.fromkeys(requests):
        err = send_request(
            _bridge_uds_addr(ipc_ns_inode, req.domain_id), serialize_request(req))
        if err is not None and logger is not None:
            logger.warn(f'daemon bridge dispatch failed: {err}')
