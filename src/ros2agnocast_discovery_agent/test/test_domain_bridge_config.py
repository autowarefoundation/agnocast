"""Unit tests for parsing the domain_bridge YAML into kmod rule tuples.

These exercise the pure parser only; no kmod, DDS, or file I/O is involved.
"""

from ros2agnocast_discovery_agent.domain_bridge_config import parse_domain_bridge_config


def test_top_level_domains_apply_to_each_topic():
    text = """
from_domain: 1
to_domain: 2
topics:
  chatter:
    type: std_msgs/msg/String
"""
    assert parse_domain_bridge_config(text) == [('chatter', 1, 2)]


def test_per_topic_domains_override_top_level():
    text = """
from_domain: 1
to_domain: 2
topics:
  chatter:
    type: std_msgs/msg/String
  special:
    from_domain: 3
    to_domain: 4
"""
    rules = parse_domain_bridge_config(text)
    assert ('chatter', 1, 2) in rules
    assert ('special', 3, 4) in rules


def test_topic_without_resolvable_domain_pair_is_skipped():
    text = """
topics:
  chatter:
    type: std_msgs/msg/String
"""
    assert parse_domain_bridge_config(text) == []


def test_empty_or_topicless_config_yields_no_rules():
    assert parse_domain_bridge_config('') == []
    assert parse_domain_bridge_config('topics:') == []
