"""Unit tests for the --spin-time handling in topic_list_agnocast.

The verb waits for two things: the ROS 2 graph (elapsed time after its own
participant is created) and the Agnocast gossip. ``--spin-time`` is the budget
for both, and the two overlap. These tests drive ``main()`` with a fake clock so
the wait is observable without a discovery agent, a kernel module or DDS:

- The budget is measured the way ros2cli measures its own ``--spin-time``: from
  when the participant exists, not from process start. Node setup is therefore
  added to the run, not subtracted from the wait. ``FakeNodeStrategy`` charges
  the clock for what ros2cli would spend, including the ``spin_time`` it is
  handed, so a verb that let ros2cli spin as well would show up as a shrunken
  window.
- Whatever gossip leaves unused is slept off before the ROS 2 graph query, but
  only when no ros2 daemon is serving that query.

Assertions look at the total time slept, not at individual sleep calls, so that
skipping a zero-length sleep stays a free implementation choice.
"""

import argparse

from unittest.mock import patch

from ros2cli.node.direct import DEFAULT_TIMEOUT as ROS2_DEFAULT_SPIN_TIME

import pytest

from ros2agnocast.verb import topic_list_agnocast as tl

ROS2_TOPICS = [('/ros2_only_topic', ['std_msgs/msg/String'])]
START = 1000.0


class FakeClock:
    def __init__(self):
        self.now = START
        self.slept = []

    def monotonic(self):
        return self.now

    def sleep(self, duration):
        self.slept.append(duration)
        self.advance(duration)

    def advance(self, duration):
        """Time passing without a sleep, as in spinning or node construction."""
        self.now += duration

    @property
    def elapsed(self):
        return self.now - START

    @property
    def waited(self):
        return sum(self.slept)


class FakeNodeStrategy:
    """Stand-in for ros2cli's NodeStrategy, charging the clock as ros2cli would."""

    def __init__(self, args, *, clock, daemon_node, construction_cost):
        self.daemon_node = daemon_node
        clock.advance(construction_cost)  # rclpy.init, create_node, spawn_daemon
        clock.advance(args.spin_time)     # DirectNode's own fixed spin

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False

    def get_publishers_info_by_topic(self, name):
        return []

    def get_subscriptions_info_by_topic(self, name):
        return []


def _run_main(spin_time=None, gossip_elapsed=0.0, daemon=False, construction_cost=0.0):
    """Run ``main()`` on a fake clock.

    Returns ``(clock, gossip_timeout)``, where ``gossip_timeout`` is the window
    the verb handed to the gossip collection.
    """
    clock = FakeClock()
    gossip_timeout = []

    def fake_node_strategy(args):
        return FakeNodeStrategy(
            args, clock=clock, daemon_node=object() if daemon else None,
            construction_cost=construction_cost)

    def fake_collect(node, timeout_sec):
        gossip_timeout.append(timeout_sec)
        clock.advance(gossip_elapsed)
        return [], True

    args = argparse.Namespace(debug=False)
    args.spin_time = ROS2_DEFAULT_SPIN_TIME if spin_time is None else spin_time

    with patch.object(tl, 'time', clock), \
            patch.object(tl, 'NodeStrategy', fake_node_strategy), \
            patch.object(tl, 'collect_announcements_with_fallback', fake_collect), \
            patch.object(tl, 'warn_if_using_fallback', lambda *a, **k: None), \
            patch.object(tl, 'get_topic_names_and_types', lambda node: ROS2_TOPICS):
        tl.ListAgnocastVerb().main(args=args)

    return clock, gossip_timeout[0]


def test_spin_time_defaults_to_the_ros2_value():
    parser = argparse.ArgumentParser()
    tl.ListAgnocastVerb().add_arguments(parser, 'list_agnocast')
    assert parser.parse_args([]).spin_time == ROS2_DEFAULT_SPIN_TIME


def test_gossip_gets_the_whole_spin_time():
    """Nothing may be spent before it, ros2cli's own spin included."""
    _, gossip_timeout = _run_main(spin_time=3.0)
    assert gossip_timeout == pytest.approx(3.0)


def test_node_setup_is_added_to_the_run_not_taken_out_of_the_wait():
    """A slow ``spawn_daemon`` must not shrink the window to nothing.

    That would drop the cross-NS view without the operator ever asking for a
    shorter wait.
    """
    clock, gossip_timeout = _run_main(
        spin_time=3.0, construction_cost=0.4, gossip_elapsed=3.0)
    assert gossip_timeout == pytest.approx(3.0)
    assert clock.elapsed == pytest.approx(3.4)

    _, gossip_timeout = _run_main(spin_time=0.5, construction_cost=0.8)
    assert gossip_timeout == pytest.approx(0.5)


def test_time_gossip_leaves_unused_is_slept_off():
    """Without a daemon the ROS 2 graph query must see the full spin time."""
    clock, _ = _run_main(spin_time=3.0, gossip_elapsed=0.2)
    assert clock.waited == pytest.approx(2.8)
    assert clock.elapsed == pytest.approx(3.0)


def test_gossip_overshooting_its_window_does_not_wait_again():
    """The gossip loop can overrun its deadline by a tick; never sleep negative."""
    clock, _ = _run_main(spin_time=1.0, gossip_elapsed=1.3)
    assert clock.waited == pytest.approx(0.0)
    assert clock.elapsed == pytest.approx(1.3)


def test_a_daemon_answers_the_graph_query_so_nothing_is_slept_off():
    """The daemon's node has been discovering all along; waiting cannot help."""
    clock, _ = _run_main(spin_time=3.0, gossip_elapsed=0.2, daemon=True)
    assert clock.waited == pytest.approx(0.0)
    assert clock.elapsed == pytest.approx(0.2)


def test_no_wait_at_all_for_zero_or_negative_spin_time():
    for spin_time in (0.0, -1.0):
        clock, gossip_timeout = _run_main(spin_time=spin_time)
        assert gossip_timeout == pytest.approx(0.0)
        assert clock.elapsed == pytest.approx(0.0)


def test_ros2_topics_are_still_listed(capsys):
    _run_main(spin_time=1.0)
    assert capsys.readouterr().out.splitlines() == ['/ros2_only_topic']
