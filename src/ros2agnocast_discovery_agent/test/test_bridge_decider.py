"""Unit tests for the bridge decider.

These need neither the kmod, DDS, nor the bridge UDS: ``decide_bridges`` is pure
logic, and the wire format is checked against the hand-built byte layout that
mirrors a Daemon-variant ``BridgeMsg`` (4-byte tag + 524-byte payload = 528
bytes) in ``agnocast_bridge_msg.hpp``.
"""

import struct
from unittest.mock import MagicMock

from ros2agnocast_discovery_agent.bridge_decider import (
    BridgeRequest,
    UNFORCED_WARN_AFTER_TICKS,
    decide_bridges,
    decide_domain_rule_bridges,
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


def test_domain_rule_forces_a2r_on_the_from_side():
    local = _state(topics=[_topic('/x', pubs=[_endpoint('/lp')], domain=1)])
    reqs = decide_domain_rule_bridges(local, [('/x', '/x', 1, 2)])
    assert len(reqs) == 1
    assert reqs[0].topic_name == '/x'
    assert reqs[0].direction == DIRECTION_AGNOCAST_TO_ROS2
    assert reqs[0].domain_id == 1


def test_domain_rule_leaves_the_to_side_to_the_on_demand_path():
    # The publisher matters: without it this would return [] for want of one rather than for the
    # cell lookup. The domain filter is a separate guard, covered by the test above.
    local = _state(topics=[_topic('/x', pubs=[_endpoint('/lp')], domain=2)])
    assert decide_domain_rule_bridges(local, [('/x', '/x', 1, 2)]) == []


def test_domain_rule_ignores_a_rule_belonging_to_another_domain():
    """bidirectional splits an entry per direction, so one of each pair is always someone else's."""
    logger = MagicMock()
    reported = {}
    # Both domains present, so without the guard the 2->1 tuple would force a second request.
    local = _state(topics=[
        _topic('/x', pubs=[_endpoint('/lp')], domain=1),
        _topic('/x', pubs=[_endpoint('/rp')], domain=2)])
    rules = [('/x', '/x', 1, 2), ('/x', '/x', 2, 1)]

    reqs = decide_domain_rule_bridges(
        local, rules, domain_id=1, logger=logger, reported=reported)

    assert len(reqs) == 1
    assert reqs[0].domain_id == 1
    # The 2->1 tuple is the domain-2 agent's to force: no line, and no entry left behind.
    logger.info.assert_not_called()
    logger.warn.assert_not_called()
    assert reported == {}


def test_domain_rule_reports_each_reason_it_forced_nothing():
    """Skipping in silence is the symptom this forcing exists to remove."""
    logger = MagicMock()
    for topics in (
            [],
            [_topic('/x', type_name='', pubs=[_endpoint('/lp')], domain=1)],
            [_topic('/x', pubs=[_endpoint('/b', is_bridge=True)], domain=1)]):
        decide_domain_rule_bridges(
            _state(topics=topics), [('/x', '/x', 1, 2)], domain_id=1, logger=logger, reported={})

    reasons = ' '.join(str(c) for c in logger.info.call_args_list)
    assert logger.info.call_count == 3
    assert 'no local topic' in reasons
    assert 'type is not known' in reasons
    assert 'no local publisher' in reasons
    # A fresh dict each time, so nothing has persisted: the startup gap stays at info.
    logger.warn.assert_not_called()


def test_domain_rule_reports_one_reason_once_across_ticks():
    """The decider runs every tick; an unchanged reason must not be repeated."""
    logger = MagicMock()
    reported = {}
    local = _state(topics=[])
    for _ in range(3):
        decide_domain_rule_bridges(
            local, [('/x', '/x', 1, 2)], domain_id=1, logger=logger, reported=reported)
    assert logger.info.call_count == 1
    logger.warn.assert_not_called()


def test_domain_rule_warns_once_after_the_reason_persists():
    """The agent ticks before the first publisher exists, so only a lasting gap is a warning."""
    logger = MagicMock()
    reported = {}
    local = _state(topics=[])
    rules = [('/x', '/x', 1, 2)]
    for _ in range(UNFORCED_WARN_AFTER_TICKS - 1):
        decide_domain_rule_bridges(local, rules, domain_id=1, logger=logger, reported=reported)
    logger.warn.assert_not_called()

    decide_domain_rule_bridges(local, rules, domain_id=1, logger=logger, reported=reported)
    assert logger.warn.call_count == 1
    assert f'unforced for {UNFORCED_WARN_AFTER_TICKS} ticks' in str(logger.warn.call_args)

    for _ in range(5):
        decide_domain_rule_bridges(local, rules, domain_id=1, logger=logger, reported=reported)
    assert logger.warn.call_count == 1
    assert logger.info.call_count == 1


def test_domain_rule_counts_one_tick_when_two_rules_name_one_cell():
    """Two rules can name one cell, and the warn text states the count, so it must not double."""
    logger = MagicMock()
    reported = {}
    local = _state(topics=[])
    rules = [('/x', '/x', 1, 2), ('/x', '/y', 1, 3)]
    for _ in range(UNFORCED_WARN_AFTER_TICKS - 1):
        decide_domain_rule_bridges(local, rules, domain_id=1, logger=logger, reported=reported)

    logger.warn.assert_not_called()
    assert reported[('/x', 1)][1] == UNFORCED_WARN_AFTER_TICKS - 1
    assert logger.info.call_count == 1


def test_domain_rule_keeps_counting_when_the_reason_changes():
    """A fault that alternates is still a fault; resetting would leave it at info forever."""
    logger = MagicMock()
    reported = {}
    rules = [('/x', '/x', 1, 2)]
    absent = _state(topics=[])
    untyped = _state(topics=[_topic('/x', type_name='', pubs=[_endpoint('/lp')], domain=1)])

    for i in range(UNFORCED_WARN_AFTER_TICKS):
        state = absent if i % 2 == 0 else untyped
        decide_domain_rule_bridges(state, rules, domain_id=1, logger=logger, reported=reported)

    assert reported[('/x', 1)][1] == UNFORCED_WARN_AFTER_TICKS
    assert logger.warn.call_count == 1


def test_domain_rule_stays_quiet_when_a_rule_forces_from_the_start():
    """`now forced` is the other half of a reported gap, not a line for every forcing rule."""
    logger = MagicMock()
    reported = {}
    local = _state(topics=[_topic('/x', pubs=[_endpoint('/lp')], domain=1)])

    for _ in range(3):
        assert decide_domain_rule_bridges(
            local, [('/x', '/x', 1, 2)], domain_id=1, logger=logger, reported=reported)

    logger.info.assert_not_called()
    logger.warn.assert_not_called()


def test_domain_rule_says_when_forcing_starts_and_can_report_again():
    """A reported gap that closes is said so, and a later one is reported afresh."""
    logger = MagicMock()
    reported = {}
    absent = _state(topics=[])
    present = _state(topics=[_topic('/x', pubs=[_endpoint('/lp')], domain=1)])
    rules = [('/x', '/x', 1, 2)]

    decide_domain_rule_bridges(absent, rules, logger=logger, reported=reported)
    assert decide_domain_rule_bridges(present, rules, logger=logger, reported=reported)
    decide_domain_rule_bridges(absent, rules, logger=logger, reported=reported)

    messages = ' '.join(str(c) for c in logger.info.call_args_list)
    assert logger.info.call_count == 3
    assert 'now forced' in messages


def test_domain_rule_request_carries_the_topic_type_and_publisher_qos():
    """The forced request is what bridge_manager builds from, so its payload matters."""
    pub = _endpoint('/lp', depth=7, transient=True)
    local = _state(topics=[_topic('/x', pubs=[pub], domain=1, type_name='pkg/msg/Thing')])
    reqs = decide_domain_rule_bridges(local, [('/x', '/x', 1, 2)])
    assert len(reqs) == 1
    assert reqs[0].type_name == 'pkg/msg/Thing'
    assert reqs[0].qos_depth == 7
    assert reqs[0].qos_is_transient_local is True


def test_domain_rule_collapses_two_rules_naming_the_same_cell():
    """A bidirectional pair and a duplicate entry both land on the same (topic, from_domain)."""
    local = _state(topics=[_topic('/x', pubs=[_endpoint('/lp')], domain=1)])
    reqs = decide_domain_rule_bridges(local, [('/x', '/x', 1, 2), ('/x', '/y', 1, 3)])
    assert len(reqs) == 1


def test_domain_rule_forces_the_reverse_leg_of_a_bidirectional_rule():
    # parse_domain_bridge_config emits the reverse tuple for 'bidirectional: true'. This is the
    # half that turns it into a request: a publisher in the to_domain is forced by that tuple.
    local = _state(topics=[_topic('/x', pubs=[_endpoint('/lp')], domain=2)])
    reqs = decide_domain_rule_bridges(local, [('/x', '/x', 1, 2), ('/x', '/x', 2, 1)])
    assert len(reqs) == 1
    assert reqs[0].domain_id == 2
    assert reqs[0].direction == DIRECTION_AGNOCAST_TO_ROS2


def test_domain_rule_uses_the_from_side_name_when_renamed():
    local = _state(topics=[_topic('/x', pubs=[_endpoint('/lp')], domain=1)])
    reqs = decide_domain_rule_bridges(local, [('/x', '/y', 1, 2)])
    assert len(reqs) == 1
    assert reqs[0].topic_name == '/x'


def test_domain_rule_skips_when_no_local_publisher():
    local = _state(topics=[_topic('/x', subs=[_endpoint('/ls')], domain=1)])
    assert decide_domain_rule_bridges(local, [('/x', '/x', 1, 2)]) == []


def test_domain_rule_skips_bridge_only_publisher():
    local = _state(topics=[_topic('/x', pubs=[_endpoint('/lp', is_bridge=True)], domain=1)])
    assert decide_domain_rule_bridges(local, [('/x', '/x', 1, 2)]) == []


def test_domain_rule_skips_when_type_unknown():
    local = _state(topics=[_topic('/x', type_name='', pubs=[_endpoint('/lp')], domain=1)])
    assert decide_domain_rule_bridges(local, [('/x', '/x', 1, 2)]) == []


def test_domain_rule_skips_rule_that_does_not_cross_domains():
    local = _state(topics=[_topic('/x', pubs=[_endpoint('/lp')], domain=1)])
    assert decide_domain_rule_bridges(local, [('/x', '/x', 1, 1)]) == []


def test_domain_rule_carries_publisher_qos():
    local = _state(topics=[
        _topic('/x', pubs=[_endpoint('/lp', depth=3, transient=True, reliable=False)], domain=1)])
    reqs = decide_domain_rule_bridges(local, [('/x', '/x', 1, 2)])
    assert (reqs[0].qos_depth, reqs[0].qos_is_transient_local, reqs[0].qos_is_reliable) == \
        (3, True, False)


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


def test_dispatch_sends_one_datagram_per_distinct_request(monkeypatch):
    from ros2agnocast_discovery_agent import bridge_decider as bd
    sent = []
    monkeypatch.setattr(bd, 'send_request', lambda addr, payload: sent.append(addr) or None)

    req = BridgeRequest('/x', 'T', DIRECTION_AGNOCAST_TO_ROS2, 1, False, True)
    dispatch_requests([req, req], ipc_ns_inode=12345)

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
