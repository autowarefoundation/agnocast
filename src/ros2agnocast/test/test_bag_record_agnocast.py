"""Unit tests for _make_should_activate in bag_record_agnocast.

CLI args are simulated with argparse.Namespace. _make_should_activate reads all
attributes via getattr(args, attr, default), so setting only the relevant kwargs
and leaving others absent is equivalent to running on a version that does not
define them (e.g. Humble vs Jazzy). No version-specific helpers are needed.
"""

import argparse

from ros2agnocast.verb.bag_record_agnocast import _make_should_activate

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

ANY_TYPE = 'std_msgs/msg/String'


def _args(**kwargs) -> argparse.Namespace:
    """Return a Namespace with only the specified attributes set."""
    return argparse.Namespace(**kwargs)


# ---------------------------------------------------------------------------
# -a / --all, --all-topics
# ---------------------------------------------------------------------------

class TestAllFlag:
    def test_all_activates_any_topic(self):
        f = _make_should_activate(_args(all=True))
        assert f('/anything', ANY_TYPE) is True

    def test_all_topics_activates_any_topic(self):
        f = _make_should_activate(_args(all_topics=True))
        assert f('/anything', ANY_TYPE) is True

    def test_no_all_flag_does_not_activate_unmatched_topic(self):
        f = _make_should_activate(_args(topics=['/foo']))
        assert f('/bar', ANY_TYPE) is False


# ---------------------------------------------------------------------------
# --topics
# ---------------------------------------------------------------------------

class TestTopicsFlag:
    def test_topic_in_list_is_activated(self):
        f = _make_should_activate(_args(topics=['/foo', '/bar']))
        assert f('/foo', ANY_TYPE) is True
        assert f('/bar', ANY_TYPE) is True

    def test_topic_not_in_list_is_not_activated(self):
        f = _make_should_activate(_args(topics=['/foo']))
        assert f('/other', ANY_TYPE) is False


# ---------------------------------------------------------------------------
# [topics..] positional
# In Jazzy, positional topics land in args.topics_positional.
# In Humble, they land in args.topics (same dest as --topics).
# ---------------------------------------------------------------------------

class TestPositionalTopics:
    def test_topics_positional_is_activated(self):
        f = _make_should_activate(_args(topics_positional=['/my_topic']))
        assert f('/my_topic', ANY_TYPE) is True
        assert f('/other', ANY_TYPE) is False

    def test_multiple_topics_positional(self):
        f = _make_should_activate(_args(topics_positional=['/foo', '/bar']))
        assert f('/foo', ANY_TYPE) is True
        assert f('/bar', ANY_TYPE) is True
        assert f('/baz', ANY_TYPE) is False

    def test_topics_used_as_positional_dest(self):
        # Humble stores positional topics in args.topics
        f = _make_should_activate(_args(topics=['/my_topic']))
        assert f('/my_topic', ANY_TYPE) is True
        assert f('/other', ANY_TYPE) is False


# ---------------------------------------------------------------------------
# -e / --regex
# ---------------------------------------------------------------------------

class TestRegex:
    def test_regex_match_is_activated(self):
        f = _make_should_activate(_args(regex=r'^/sensor'))
        assert f('/sensor/imu', ANY_TYPE) is True
        assert f('/sensor/camera', ANY_TYPE) is True

    def test_regex_no_match_is_not_activated(self):
        f = _make_should_activate(_args(regex=r'^/sensor'))
        assert f('/cmd_vel', ANY_TYPE) is False

    def test_alternation_regex(self):
        # Alternation pattern: matches either branch
        f = _make_should_activate(_args(regex=r'^/hoge/(.*)|/fuga/(.*)'))
        assert f('/hoge/foo/bar', ANY_TYPE) is True
        assert f('/foo/hoge/bar', ANY_TYPE) is False  # ^/hoge/ is anchored
        assert f('/fuga/foo/bar', ANY_TYPE) is True
        assert f('/foo/fuga/bar', ANY_TYPE) is True   # /fuga/ is unanchored
        assert f('/other', ANY_TYPE) is False
        assert f('/hoge_extra', ANY_TYPE) is False

    def test_topics_positional_and_regex_are_ored(self):
        # topics_positional and regex are OR-ed: either match activates the bridge
        f = _make_should_activate(_args(topics_positional=['/explicit'], regex=r'^/sensor'))
        assert f('/explicit', ANY_TYPE) is True
        assert f('/sensor/imu', ANY_TYPE) is True
        assert f('/unrelated', ANY_TYPE) is False


# ---------------------------------------------------------------------------
# Relaxed default: no positive criteria -> True
# ---------------------------------------------------------------------------

class TestRelaxedDefault:
    def test_no_criteria_returns_true(self):
        # No --all, --topics, --regex: should not happen with valid CLI, but default True
        f = _make_should_activate(_args())
        assert f('/any_topic', ANY_TYPE) is True
