"""Unit tests for locating, loading, and parsing the domain_bridge YAMLs.

These exercise path resolution, the loader, and the pure parser; no kmod or DDS
is involved.
"""

import pytest

from ros2agnocast_discovery_agent import domain_bridge_config
from ros2agnocast_discovery_agent.domain_bridge_config import parse_domain_bridge_config


def test_top_level_domains_apply_to_each_topic():
    text = """
from_domain: 1
to_domain: 2
topics:
  chatter:
    type: std_msgs/msg/String
"""
    assert parse_domain_bridge_config(text) == ([('/chatter', '/chatter', 1, 2)], [])


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
    rules, skipped = parse_domain_bridge_config(text)
    assert ('/chatter', '/chatter', 1, 2) in rules
    assert ('/special', '/special', 3, 4) in rules
    assert skipped == []


def test_remap_sets_the_target_topic_name():
    # The `remap` field becomes the to_topic; the source name stays the from_topic.
    text = """
from_domain: 1
to_domain: 2
topics:
  /in_sub/chatter:
    type: std_msgs/msg/String
    remap: /chatter
"""
    assert parse_domain_bridge_config(text) == ([('/in_sub/chatter', '/chatter', 1, 2)], [])


def test_absent_remap_reuses_the_source_name():
    text = """
from_domain: 1
to_domain: 2
topics:
  chatter:
    type: std_msgs/msg/String
"""
    assert parse_domain_bridge_config(text) == ([('/chatter', '/chatter', 1, 2)], [])


def test_non_string_topic_key_without_remap_is_coerced():
    # A non-string YAML key (here an integer) with no `remap` must not trip the remap
    # string check; both names default to the coerced source name. The name itself is
    # not asserted: rcl rejects a key like this, so domain_bridge never bridges it.
    text = """
from_domain: 1
to_domain: 2
topics:
  123:
"""
    (rule,), skipped = parse_domain_bridge_config(text)
    from_topic, to_topic, from_id, to_id = rule
    assert from_topic == to_topic
    assert (from_id, to_id) == (1, 2)
    assert skipped == []


def test_non_string_remap_raises():
    text = """
from_domain: 1
to_domain: 2
topics:
  chatter:
    remap: [not, a, string]
"""
    with pytest.raises((ValueError, TypeError)):
        parse_domain_bridge_config(text)


def test_relative_and_absolute_topic_keys_yield_the_same_rule():
    relative = """
from_domain: 1
to_domain: 2
topics:
  chatter:
    type: std_msgs/msg/String
    remap: renamed
"""
    absolute = """
from_domain: 1
to_domain: 2
topics:
  /chatter:
    type: std_msgs/msg/String
    remap: /renamed
"""
    assert parse_domain_bridge_config(relative) == parse_domain_bridge_config(absolute)
    assert parse_domain_bridge_config(relative) == ([('/chatter', '/renamed', 1, 2)], [])


def test_topic_without_resolvable_domain_pair_is_reported_as_skipped():
    text = """
topics:
  chatter:
    type: std_msgs/msg/String
"""
    rules, skipped = parse_domain_bridge_config(text)
    assert rules == []
    assert skipped == ['/chatter']


def test_empty_or_topicless_config_yields_no_rules():
    assert parse_domain_bridge_config('') == ([], [])
    assert parse_domain_bridge_config('topics:') == ([], [])


def test_non_integer_domain_raises():
    # A non-integer domain raises; the agent catches this and runs without rules
    # rather than crashing at startup.
    text = """
from_domain: not_a_number
to_domain: 2
topics:
  chatter:
    type: std_msgs/msg/String
"""
    with pytest.raises(ValueError):
        parse_domain_bridge_config(text)


def test_out_of_range_domain_raises():
    # uint32 overflow would wrap silently at the ioctl boundary, so reject it.
    text = """
from_domain: 1
to_domain: 4294967296
topics:
  chatter:
    type: std_msgs/msg/String
"""
    with pytest.raises(ValueError):
        parse_domain_bridge_config(text)


def test_negative_domain_raises():
    text = """
from_domain: -1
to_domain: 2
topics:
  chatter:
    type: std_msgs/msg/String
"""
    with pytest.raises(ValueError):
        parse_domain_bridge_config(text)


# Malformed structure must raise a caught exception (ValueError/TypeError), not
# AttributeError, so the daemon runs without rules instead of crashing.

def test_non_mapping_root_raises():
    with pytest.raises((ValueError, TypeError)):
        parse_domain_bridge_config('- a\n- b\n')


def test_non_mapping_topics_raises():
    with pytest.raises((ValueError, TypeError)):
        parse_domain_bridge_config('topics:\n  - chatter\n  - special\n')


def test_non_mapping_topic_spec_raises():
    with pytest.raises((ValueError, TypeError)):
        parse_domain_bridge_config('from_domain: 1\nto_domain: 2\ntopics:\n  chatter: oops\n')


def test_null_topic_spec_with_top_level_domains_is_used():
    # `chatter:` with no body is a None spec; it should fall back to the
    # top-level domains, not crash.
    text = """
from_domain: 1
to_domain: 2
topics:
  chatter:
"""
    assert parse_domain_bridge_config(text) == ([('/chatter', '/chatter', 1, 2)], [])


def test_bidirectional_topic_yields_both_directions():
    text = """
from_domain: 1
to_domain: 2
topics:
  /chatter:
    bidirectional: true
"""
    rules, skipped = parse_domain_bridge_config(text)
    assert rules == [('/chatter', '/chatter', 1, 2), ('/chatter', '/chatter', 2, 1)]
    assert skipped == []


def test_bidirectional_accepts_the_yaml11_forms_yaml_cpp_resolves():
    # PyYAML leaves a bare 'y'/'n' as a string while yaml-cpp reads it as a bool. Rejecting them
    # would refuse a config the external domain_bridge node runs.
    # PyYAML already resolves 'yes'/'no'/'on'/'off'; only these reach the string branch.
    for text, expected_len in (('y', 2), ('n', 1), ('Y', 2), ('N', 1), ("'y'", 2), ("' y '", 2)):
        doc = f'from_domain: 1\nto_domain: 2\ntopics:\n  chatter:\n    bidirectional: {text}\n'
        rules, _ = parse_domain_bridge_config(doc)
        assert len(rules) == expected_len, text


def test_bidirectional_reverse_leg_swaps_only_the_domains():
    """Mirrors the external node, whose reverse leg keeps the source and remap names."""
    text = """
from_domain: 1
to_domain: 2
topics:
  /chatter:
    remap: /renamed
    bidirectional: true
"""
    rules, _skipped = parse_domain_bridge_config(text)
    assert rules == [('/chatter', '/renamed', 1, 2), ('/chatter', '/renamed', 2, 1)]


def test_bidirectional_false_yields_one_direction():
    text = """
from_domain: 1
to_domain: 2
topics:
  /chatter:
    bidirectional: false
"""
    rules, _skipped = parse_domain_bridge_config(text)
    assert rules == [('/chatter', '/chatter', 1, 2)]


def test_non_boolean_bidirectional_is_rejected():
    text = """
from_domain: 1
to_domain: 2
topics:
  /chatter:
    bidirectional: yes please
"""
    with pytest.raises(ValueError):
        parse_domain_bridge_config(text)


def test_resolve_config_paths_prefers_the_env_var(monkeypatch):
    monkeypatch.setenv(domain_bridge_config.CONFIG_ENV, '/somewhere/else.yaml')
    assert domain_bridge_config.resolve_config_paths() == (['/somewhere/else.yaml'], True)


def test_resolve_config_paths_splits_the_env_var_on_the_separator(monkeypatch):
    monkeypatch.setenv(domain_bridge_config.CONFIG_ENV, '/a.yaml:/b.yaml:/c.yaml')
    assert domain_bridge_config.resolve_config_paths() == (
        ['/a.yaml', '/b.yaml', '/c.yaml'], True)


def _patch_default(monkeypatch, tmp_path):
    """Point the default path into tmp_path and return it, drop-in directory included."""
    default = tmp_path / 'domain_bridge.yaml'
    monkeypatch.setattr(domain_bridge_config, 'DEFAULT_CONFIG_PATH', str(default))
    return default


def test_resolve_config_paths_falls_back_to_the_default(monkeypatch, tmp_path):
    monkeypatch.delenv(domain_bridge_config.CONFIG_ENV, raising=False)
    default = _patch_default(monkeypatch, tmp_path)
    assert domain_bridge_config.resolve_config_paths() == ([str(default)], False)


@pytest.mark.parametrize('value', [':', '::'])
def test_resolve_config_paths_treats_an_empty_listing_as_unset(monkeypatch, tmp_path, value):
    """A cleared or separator-only variable must not resolve to a path of ''."""
    monkeypatch.setenv(domain_bridge_config.CONFIG_ENV, value)
    default = _patch_default(monkeypatch, tmp_path)
    assert domain_bridge_config.resolve_config_paths() == ([str(default)], False)


def test_the_drop_ins_are_read_after_the_default_file(monkeypatch, tmp_path):
    monkeypatch.delenv(domain_bridge_config.CONFIG_ENV, raising=False)
    default = _patch_default(monkeypatch, tmp_path)
    default.write_text('topics:\n')
    drop_in_dir = tmp_path / 'domain_bridge.d'
    drop_in_dir.mkdir()
    (drop_in_dir / '10-lidar.yaml').write_text('topics:\n')

    assert domain_bridge_config.resolve_config_paths() == (
        [str(default), str(drop_in_dir / '10-lidar.yaml')], False)


def test_the_drop_ins_are_read_in_name_order_without_the_default_file(monkeypatch, tmp_path):
    monkeypatch.delenv(domain_bridge_config.CONFIG_ENV, raising=False)
    _patch_default(monkeypatch, tmp_path)
    drop_in_dir = tmp_path / 'domain_bridge.d'
    drop_in_dir.mkdir()
    for name in ('20-lidar.yaml', '10-base.yaml'):
        (drop_in_dir / name).write_text('topics:\n')

    paths, from_env = domain_bridge_config.resolve_config_paths()

    assert paths == [str(drop_in_dir / '10-base.yaml'), str(drop_in_dir / '20-lidar.yaml')]
    assert from_env is False


def test_a_drop_in_that_is_not_yaml_is_ignored(monkeypatch, tmp_path):
    monkeypatch.delenv(domain_bridge_config.CONFIG_ENV, raising=False)
    default = _patch_default(monkeypatch, tmp_path)
    drop_in_dir = tmp_path / 'domain_bridge.d'
    drop_in_dir.mkdir()
    (drop_in_dir / 'base.yaml.bak').write_text('topics:\n')

    assert domain_bridge_config.resolve_config_paths() == ([str(default)], False)


def test_the_env_var_wins_over_the_drop_in_directory(monkeypatch, tmp_path):
    listed = tmp_path / 'listed.yaml'
    listed.write_text('topics:\n')
    monkeypatch.setenv(domain_bridge_config.CONFIG_ENV, str(listed))
    _patch_default(monkeypatch, tmp_path)
    drop_in_dir = tmp_path / 'domain_bridge.d'
    drop_in_dir.mkdir()
    (drop_in_dir / '10-base.yaml').write_text('topics:\n')

    assert domain_bridge_config.resolve_config_paths() == ([str(listed)], True)


def _write(tmp_path, name, text):
    path = tmp_path / name
    path.write_text(text)
    return str(path)


def test_load_accumulates_rules_from_every_config_in_order(tmp_path):
    first = _write(tmp_path, 'a.yaml', 'from_domain: 1\nto_domain: 2\ntopics:\n  chatter:\n')
    second = _write(tmp_path, 'b.yaml', 'from_domain: 3\nto_domain: 4\ntopics:\n  image:\n')

    results = domain_bridge_config.load_domain_bridge_rules([first, second])

    assert [r.path for r in results] == [first, second]
    assert [rule for r in results for rule in r.rules] == [
        ('/chatter', '/chatter', 1, 2), ('/image', '/image', 3, 4)]
    assert all(r.error is None for r in results)


def test_load_keeps_the_domain_defaults_local_to_each_config(tmp_path):
    """Matching domain_bridge, where 'from_domain'/'to_domain' never leak between files."""
    with_defaults = _write(
        tmp_path, 'a.yaml', 'from_domain: 1\nto_domain: 2\ntopics:\n  chatter:\n')
    without = _write(tmp_path, 'b.yaml', 'topics:\n  image:\n')

    results = domain_bridge_config.load_domain_bridge_rules([with_defaults, without])

    assert results[1].rules == []
    assert results[1].skipped == ['/image']


def test_load_reports_a_broken_config_without_dropping_the_others(tmp_path):
    broken = _write(tmp_path, 'a.yaml', 'topics: [not, a, mapping]\n')
    good = _write(tmp_path, 'b.yaml', 'from_domain: 1\nto_domain: 2\ntopics:\n  chatter:\n')

    results = domain_bridge_config.load_domain_bridge_rules([broken, good])

    assert isinstance(results[0].error, ValueError)
    assert results[0].rules == []
    assert results[1].error is None
    assert results[1].rules == [('/chatter', '/chatter', 1, 2)]


def test_load_carries_a_missing_config_as_file_not_found(tmp_path):
    """The agent logs an absent default at info and an absent listed path at warn."""
    (result,) = domain_bridge_config.load_domain_bridge_rules([str(tmp_path / 'absent.yaml')])

    assert isinstance(result.error, FileNotFoundError)
