"""Unit tests for the bridge decider.

These need neither the kmod, DDS, nor a POSIX MQ: ``decide_bridges`` is pure
logic, and the wire format is checked against the hand-built byte layout that
mirrors a Daemon-variant ``BridgeMsg`` (4-byte tag + 524-byte payload = 528
bytes) in ``agnocast_bridge_msg.hpp``.
"""

import struct

from ros2agnocast_discovery_agent.bridge_decider import (
    BridgeRequest,
    decide_bridges,
    DIRECTION_AGNOCAST_TO_ROS2,
    DIRECTION_ROS2_TO_AGNOCAST,
    dispatch_requests,
    serialize_request,
)
from ros2agnocast_discovery_msgs.msg import (
    AgnocastDaemonState,
    AgnocastEndpoint,
    AgnocastTopic,
)


def _endpoint(node, depth=10, transient=False, reliable=True, is_bridge=False):
    ep = AgnocastEndpoint()
    ep.node_name = node
    ep.qos_depth = depth
    ep.qos_is_transient_local = transient
    ep.qos_is_reliable = reliable
    ep.is_bridge = is_bridge
    return ep


def _topic(name, type_name='std_msgs/msg/Int32', pubs=None, subs=None, domain=0):
    t = AgnocastTopic()
    t.topic_name = name
    t.type_name = type_name
    t.domain_id = domain
    t.publishers = pubs or []
    t.subscribers = subs or []
    return t


def _state(host_uuid='HOST', ipc_ns=111, topics=None):
    s = AgnocastDaemonState()
    s.schema_version = 1
    s.host_uuid = host_uuid
    s.ipc_ns_inode = ipc_ns
    s.topics = topics or []
    return s


def test_decide_emits_a2r_when_local_pub_remote_sub():
    local = _state(topics=[_topic('/x', pubs=[_endpoint('/pub')])])
    remote = _state(host_uuid='OTHER', ipc_ns=222,
                    topics=[_topic('/x', subs=[_endpoint('/sub')])])

    reqs = decide_bridges(local, {('OTHER', 222): remote})
    assert len(reqs) == 1
    assert reqs[0].topic_name == '/x'
    assert reqs[0].direction == DIRECTION_AGNOCAST_TO_ROS2
    assert reqs[0].type_name == 'std_msgs/msg/Int32'


def test_decide_emits_r2a_when_local_sub_remote_pub():
    local = _state(topics=[_topic('/x', subs=[_endpoint('/sub')])])
    remote = _state(host_uuid='OTHER', ipc_ns=222,
                    topics=[_topic('/x', pubs=[_endpoint('/pub')])])

    reqs = decide_bridges(local, {('OTHER', 222): remote})
    assert len(reqs) == 1
    assert reqs[0].direction == DIRECTION_ROS2_TO_AGNOCAST


def test_decide_emits_both_directions_when_both_sides_have_both():
    local = _state(topics=[_topic('/x', pubs=[_endpoint('/lp')], subs=[_endpoint('/ls')])])
    remote = _state(host_uuid='OTHER', ipc_ns=222,
                    topics=[_topic('/x', pubs=[_endpoint('/rp')], subs=[_endpoint('/rs')])])

    reqs = decide_bridges(local, {('OTHER', 222): remote})
    directions = sorted(r.direction for r in reqs)
    assert directions == sorted([DIRECTION_ROS2_TO_AGNOCAST, DIRECTION_AGNOCAST_TO_ROS2])


def test_decide_skips_bridge_only_endpoints():
    local = _state(topics=[_topic('/x', pubs=[_endpoint('/lp', is_bridge=True)])])
    remote = _state(host_uuid='OTHER', ipc_ns=222,
                    topics=[_topic('/x', subs=[_endpoint('/rs')])])

    assert decide_bridges(local, {('OTHER', 222): remote}) == []


def test_decide_skips_self_namespace():
    local = _state(topics=[_topic('/x', pubs=[_endpoint('/lp')], subs=[_endpoint('/ls')])])
    assert decide_bridges(local, {('HOST', 111): local}) == []


def test_decide_skips_when_no_topic_overlap():
    local = _state(topics=[_topic('/x', pubs=[_endpoint('/lp')])])
    remote = _state(host_uuid='OTHER', ipc_ns=222,
                    topics=[_topic('/y', subs=[_endpoint('/rs')])])
    assert decide_bridges(local, {('OTHER', 222): remote}) == []


def test_decide_skips_when_type_unknown_on_both_sides():
    local = _state(topics=[_topic('/x', type_name='', pubs=[_endpoint('/lp')])])
    remote = _state(host_uuid='OTHER', ipc_ns=222,
                    topics=[_topic('/x', type_name='', subs=[_endpoint('/rs')])])
    assert decide_bridges(local, {('OTHER', 222): remote}) == []


def test_decide_uses_remote_type_when_local_missing():
    local = _state(topics=[_topic('/x', type_name='', pubs=[_endpoint('/lp')])])
    remote = _state(host_uuid='OTHER', ipc_ns=222,
                    topics=[_topic('/x', type_name='std_msgs/msg/String',
                                   subs=[_endpoint('/rs')])])
    reqs = decide_bridges(local, {('OTHER', 222): remote})
    assert len(reqs) == 1
    assert reqs[0].type_name == 'std_msgs/msg/String'


def test_decide_uses_type_from_other_remote_when_matching_remote_missing():
    # The remote that supplies the opposite-role endpoint (sub) has no type,
    # but another remote snapshot for the same topic does -> still bridged.
    local = _state(topics=[_topic('/x', type_name='', pubs=[_endpoint('/lp')])])
    remote_sub = _state(host_uuid='A', ipc_ns=1,
                        topics=[_topic('/x', type_name='', subs=[_endpoint('/rs')])])
    remote_type = _state(host_uuid='B', ipc_ns=2,
                         topics=[_topic('/x', type_name='std_msgs/msg/String')])

    reqs = decide_bridges(local, {('A', 1): remote_sub, ('B', 2): remote_type})
    assert len(reqs) == 1
    assert reqs[0].type_name == 'std_msgs/msg/String'


def test_decide_skips_when_only_same_role_present():
    # Topic matches but the opposite-role peer is absent (both sides only
    # publish, no subscriber anywhere) -> nothing to bridge.
    local = _state(topics=[_topic('/x', pubs=[_endpoint('/lp')])])
    remote = _state(host_uuid='OTHER', ipc_ns=222,
                    topics=[_topic('/x', pubs=[_endpoint('/rp')])])
    assert decide_bridges(local, {('OTHER', 222): remote}) == []


def test_decide_skips_cross_domain_match():
    # Same topic name but different domains must not be bridged: isolation holds
    # and cross-domain relaying is the external domain_bridge's job.
    local = _state(topics=[_topic('/x', pubs=[_endpoint('/lp')], domain=0)])
    remote = _state(host_uuid='OTHER', ipc_ns=222,
                    topics=[_topic('/x', subs=[_endpoint('/rs')], domain=1)])
    assert decide_bridges(local, {('OTHER', 222): remote}) == []


def test_decide_matches_within_same_nonzero_domain():
    local = _state(topics=[_topic('/x', pubs=[_endpoint('/lp')], domain=5)])
    remote = _state(host_uuid='OTHER', ipc_ns=222,
                    topics=[_topic('/x', subs=[_endpoint('/rs')], domain=5)])
    reqs = decide_bridges(local, {('OTHER', 222): remote})
    assert len(reqs) == 1
    assert reqs[0].direction == DIRECTION_AGNOCAST_TO_ROS2
    assert reqs[0].domain_id == 5


def test_decide_collapses_duplicates_across_remotes():
    local = _state(topics=[_topic('/x', pubs=[_endpoint('/lp')])])
    remote_a = _state(host_uuid='A', ipc_ns=1, topics=[_topic('/x', subs=[_endpoint('/sa')])])
    remote_b = _state(host_uuid='B', ipc_ns=2, topics=[_topic('/x', subs=[_endpoint('/sb')])])

    reqs = decide_bridges(local, {('A', 1): remote_a, ('B', 2): remote_b})
    assert len(reqs) == 1
    assert reqs[0].direction == DIRECTION_AGNOCAST_TO_ROS2


def test_serialize_matches_cpp_struct_size():
    req = BridgeRequest('/x', 'std_msgs/msg/Int32', DIRECTION_AGNOCAST_TO_ROS2,
                        10, False, True)
    msg = serialize_request(req)
    assert len(msg) == 528
    # First uint32 must be BridgeMsgType::DaemonPubSub (=2)
    (tag,) = struct.unpack_from('=I', msg, 0)
    assert tag == 2


def test_serialize_nul_terminates_truncated_topic():
    req = BridgeRequest('/' + 'a' * 1000, 'T', DIRECTION_AGNOCAST_TO_ROS2, 10, False, True)
    msg = serialize_request(req)
    payload = msg[4:]
    assert payload[255] == 0


def test_serialize_packs_direction_qos_at_expected_offsets():
    req = BridgeRequest('/x', 'T', DIRECTION_ROS2_TO_AGNOCAST, 7, True, True)
    msg = serialize_request(req)
    payload = msg[4:]
    direction, depth = struct.unpack_from('=II', payload, 512)
    transient, reliable = struct.unpack_from('=BB', payload, 520)
    assert (direction, depth, transient, reliable) == (DIRECTION_ROS2_TO_AGNOCAST, 7, 1, 1)


def test_dispatch_targets_per_namespace_uds(monkeypatch):
    from ros2agnocast_discovery_agent import bridge_decider as bd
    sent = []
    monkeypatch.setattr(bd, 'send_request', lambda addr, payload: sent.append(addr) or None)

    req = BridgeRequest('/x', 'T', DIRECTION_AGNOCAST_TO_ROS2, 1, False, True)
    dispatch_requests([req], ipc_ns_inode=12345)

    assert sent == ['\x00agnocast_bridge_manager_12345']


def test_dispatch_routes_to_per_domain_uds(monkeypatch):
    from ros2agnocast_discovery_agent import bridge_decider as bd
    sent = []
    monkeypatch.setattr(bd, 'send_request', lambda addr, payload: sent.append(addr) or None)

    dispatch_requests([
        BridgeRequest('/a', 'T', DIRECTION_AGNOCAST_TO_ROS2, 1, False, True, domain_id=0),
        BridgeRequest('/b', 'T', DIRECTION_AGNOCAST_TO_ROS2, 1, False, True, domain_id=5),
    ], ipc_ns_inode=12345)

    assert sent == [
        '\x00agnocast_bridge_manager_12345',
        '\x00agnocast_bridge_manager_12345_d5',
    ]
