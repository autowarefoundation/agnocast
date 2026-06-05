import re

from ros2bag.verb.record import RecordVerb
from ros2agnocast._a2r_bridge_activator import A2rBridgeActivator


def _make_should_activate(args):
    """Build a topic filter callable from recording CLI args.

    Returns a should_activate(topic_name, type_name) -> bool callable.
    Errs on the side of activating bridges: returns True when uncertain.

    Supported options:
        -a / --all, --all-topics, --topics, [topics..] (positional), -e / --regex

    NOTE: Some options are only defined in certain ROS 2 versions and may not exist
    as attributes on `args`. For example, --all-topics is available in Jazzy and later
    but not in Humble. getattr(..., default) is used throughout to handle this safely.
    """
    # `all` exists in all versions; `all_topics` is Jazzy+ only (not present in Humble).
    all_flag = getattr(args, 'all', False) or getattr(args, 'all_topics', False)
    # Both belong to a mutually exclusive group, so combining them is safe.
    topics = set(getattr(args, 'topics', None) or []) | set(getattr(args, 'topics_positional', None) or [])
    regex_str = getattr(args, 'regex', None) or ''
    regex = None
    if regex_str:
        try:
            regex = re.compile(regex_str)
        except re.error:
            # Invalid regex: avoid runtime exceptions inside discovery callbacks.
            # Fall back to the relaxed default behaviour (activate all when no other criteria).
            regex_str = ''
            regex = None

    def should_activate(topic_name: str, type_name: str) -> bool:
        if all_flag:
            return True

        if topic_name in topics:
            return True

        if regex is not None and regex.search(topic_name):
            return True

        # No positive criteria at all -> relaxed default
        if not topics and not regex_str:
            return True

        # Topic does not match any inclusion criterion
        return False

    return should_activate


class BagRecordAgnocastVerb(RecordVerb):
    """Record ROS data to a bag, with automatic A2R bridge activation for Agnocast topics."""

    def main(self, *, args):
        # NOTE: Agnocast service is not supported.
        should_activate = _make_should_activate(args)
        with A2rBridgeActivator(log_level=args.log_level, should_activate=should_activate):
            return super().main(args=args)
