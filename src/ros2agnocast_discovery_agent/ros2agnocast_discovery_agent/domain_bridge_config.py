"""Parse a ROS 2 ``domain_bridge`` YAML into kmod rule tuples.

The same YAML drives both the external ``domain_bridge`` node (cross-ECU, via DDS)
and the daemon's kmod rule injection that opens same-IPC-namespace zero-copy
cross-domain delivery. Only the topic name and domain pair matter here;
``type`` and other fields are ignored.
"""
import yaml

# Operators point the daemon at the config by setting this to the YAML path.
CONFIG_ENV = 'AGNOCAST_DOMAIN_BRIDGE_CONFIG'

# Domain ids cross the ioctl boundary as ctypes.c_uint32, so an out-of-range
# value would wrap silently; reject it here instead.
_UINT32_MAX = 0xFFFFFFFF


def _as_domain_id(value):
    """Coerce a YAML domain value to a uint32, raising ``ValueError`` if invalid."""
    domain = int(value)  # ValueError on non-numeric, TypeError on a list/dict
    if not 0 <= domain <= _UINT32_MAX:
        raise ValueError(f'domain id {domain} out of range [0, {_UINT32_MAX}]')
    return domain


def parse_domain_bridge_config(text):
    """Return ``(rules, skipped)``.

    ``rules`` is a list of ``(topic_name, from_domain, to_domain)`` tuples.
    ``skipped`` lists the topic names dropped for lack of a resolvable domain
    pair, so the caller can surface them instead of dropping them silently.
    ``from_domain`` / ``to_domain`` are taken from the top level and may be
    overridden per topic.

    Raises ``ValueError`` / ``TypeError`` on a structurally malformed document
    (non-mapping root, ``topics``, or topic spec) or an out-of-range domain id.
    The daemon catches both and runs without rules rather than crashing.
    """
    doc = yaml.safe_load(text) or {}
    if not isinstance(doc, dict):
        raise ValueError('domain bridge config root must be a mapping')

    topics = doc.get('topics')
    if topics is None:
        topics = {}
    if not isinstance(topics, dict):
        raise ValueError("'topics' must be a mapping")

    default_from = doc.get('from_domain')
    default_to = doc.get('to_domain')

    rules = []
    skipped = []
    for topic_name, spec in topics.items():
        if spec is None:
            spec = {}
        elif not isinstance(spec, dict):
            raise ValueError(f'spec for topic {topic_name!r} must be a mapping')
        from_domain = spec.get('from_domain', default_from)
        to_domain = spec.get('to_domain', default_to)
        if from_domain is None or to_domain is None:
            skipped.append(str(topic_name))
            continue
        rules.append((str(topic_name), _as_domain_id(from_domain), _as_domain_id(to_domain)))
    return rules, skipped


def load_domain_bridge_rules(path):
    """Read and parse the ``domain_bridge`` YAML at ``path``; return ``(rules, skipped)``."""
    with open(path, encoding='utf-8') as f:
        return parse_domain_bridge_config(f.read())
