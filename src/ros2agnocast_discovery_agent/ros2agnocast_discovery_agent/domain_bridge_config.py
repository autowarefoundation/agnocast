"""Parse ROS 2 ``domain_bridge`` YAMLs into kmod rule tuples.

Each rule is ``(from_topic, to_topic, from_domain, to_domain)``. The same YAMLs
drive both the external ``domain_bridge`` node (cross-ECU, via DDS) and the kmod
rule injection that opens same-IPC-namespace zero-copy cross-domain delivery. The
topic name, its ``remap`` target, and the domain pair matter here; ``type`` and
other fields are ignored.
"""
from collections import namedtuple
import glob
import os

import yaml

# Operators point the daemon at the config by setting this to the YAML path.
CONFIG_ENV = 'AGNOCAST_DOMAIN_BRIDGE_CONFIG'

# The agent is exec'd by an application process and inherits *its* environment, so
# an env var set only where the registration tool runs never reaches the agent.
DEFAULT_CONFIG_PATH = '/etc/agnocast/domain_bridge.yaml'

# Several configs are listed the way PATH lists directories.
CONFIG_PATH_SEP = ':'

# Domain ids cross the ioctl boundary as ctypes.c_uint32, so an out-of-range
# value would wrap silently; reject it here instead.
_UINT32_MAX = 0xFFFFFFFF


def default_config_dir():
    """Return the drop-in directory beside ``DEFAULT_CONFIG_PATH`` (``/etc/agnocast/domain_bridge.d``).

    Derived, not a constant, so moving the default path in a test moves the directory too.
    """
    return os.path.splitext(DEFAULT_CONFIG_PATH)[0] + '.d'


def _default_config_paths():
    """Return the configs at the default location: the main file, then its ``.d`` drop-ins.

    The layout systemd and sysctl use for a configuration file and its drop-in directory, but
    not their precedence: a later file only adds. Two rules that pair one cell differently are
    a configuration error the kmod rejects, not something an order resolves.

    Names ``DEFAULT_CONFIG_PATH`` when neither exists, so the caller can say where a config
    would go.
    """
    paths = [DEFAULT_CONFIG_PATH] if os.path.isfile(DEFAULT_CONFIG_PATH) else []
    paths += sorted(glob.glob(os.path.join(default_config_dir(), '*.yaml')))
    return paths or [DEFAULT_CONFIG_PATH]


def resolve_config_paths():
    """Return ``(paths, from_env)`` for the configs every consumer should read, in order."""
    listed = os.environ.get(CONFIG_ENV, '')
    paths = [path for path in listed.split(CONFIG_PATH_SEP) if path]
    return (paths, True) if paths else (_default_config_paths(), False)


def _as_domain_id(value):
    """Coerce a YAML domain value to a uint32, raising ``ValueError`` if invalid."""
    domain = int(value)  # ValueError on non-numeric, TypeError on a list/dict
    if not 0 <= domain <= _UINT32_MAX:
        raise ValueError(f'domain id {domain} out of range [0, {_UINT32_MAX}]')
    return domain


# yaml-cpp resolves the YAML 1.1 bool set, PyYAML only part of it: 'yes'/'no'/'on'/'off' come
# through as bools, but a bare 'y'/'n' stays a string. Accepting them here keeps a config the
# external domain_bridge node runs from being rejected wholesale on this side.
_YAML11_TRUE = frozenset(('y', 'yes', 'true', 'on'))
_YAML11_FALSE = frozenset(('n', 'no', 'false', 'off'))


def _as_bool(value, field, topic_name):
    """Coerce a YAML scalar to a bool, raising ``ValueError`` otherwise.

    Accepts every spelling yaml-cpp resolves, and is deliberately looser about case and
    surrounding space: a value this takes and yaml-cpp rejects fails loudly in the external node,
    while the reverse would silently disable the rules on this side alone.
    """
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        text = value.strip().lower()
        if text in _YAML11_TRUE:
            return True
        if text in _YAML11_FALSE:
            return False
    raise ValueError(f"'{field}' for topic {topic_name!r} must be a boolean")


def _as_topic_name(value):
    """Coerce a YAML topic key or ``remap`` target to the absolute name domain_bridge resolves it to.

    ``domain_bridge`` resolves a relative name against the root, so ``my_topic`` and
    ``/my_topic`` name the same topic there. Matching that here keeps a shared config
    from meaning two different topics to the external node and to this parser.
    """
    name = str(value)
    return name if name.startswith('/') else '/' + name


def parse_domain_bridge_config(text):
    """Return ``(rules, skipped)``.

    ``rules`` is a list of ``(from_topic, to_topic, from_domain, to_domain)``
    tuples. ``to_topic`` is the per-topic ``remap`` target (same ``domain_bridge``
    field the external node honors), or the source name when ``remap`` is absent.
    A ``bidirectional`` topic yields two tuples, one per direction.
    ``skipped`` lists the topic names dropped for lack of a resolvable domain
    pair, so the caller can surface them instead of dropping them silently.
    Topic names are returned absolute, matching how ``domain_bridge`` resolves
    the same keys.
    ``from_domain`` / ``to_domain`` are taken from the top level and may be
    overridden per topic.

    Raises ``ValueError`` / ``TypeError`` on a structurally malformed document
    (non-mapping root, ``topics``, or topic spec), a non-string ``remap``, or an
    out-of-range domain id. The caller catches these and skips the config rather
    than crashing.
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
            skipped.append(_as_topic_name(topic_name))
            continue
        # Default to the source name (coerced like from_topic below), so a non-string YAML key
        # without a remap doesn't trip the "'remap' must be a string" check.
        remap = spec.get('remap', str(topic_name))
        if not isinstance(remap, str):
            raise ValueError(f"'remap' for topic {topic_name!r} must be a string")
        from_topic = _as_topic_name(topic_name)
        to_topic = _as_topic_name(remap)
        bidirectional = _as_bool(spec.get('bidirectional', False), 'bidirectional', topic_name)
        from_id = _as_domain_id(from_domain)
        to_id = _as_domain_id(to_domain)
        rules.append((from_topic, to_topic, from_id, to_id))
        if bidirectional:
            # The external node's reverse leg swaps the domain ids and nothing else, keeping the
            # source name on the subscribe side and the remap on the publish side.
            rules.append((from_topic, to_topic, to_id, from_id))
    return rules, skipped


ConfigResult = namedtuple('ConfigResult', ('path', 'rules', 'skipped', 'error'))


def load_domain_bridge_rules(paths):
    """Read and parse each ``domain_bridge`` YAML in ``paths``; return a ``ConfigResult`` each.

    Rules from separate files accumulate, the way the external node merges several configs:
    ``topics`` add up, while ``from_domain`` / ``to_domain`` stay local to the file setting them.

    An error is carried in the result rather than raised, so one unreadable config does not
    take the readable ones with it.
    """
    results = []
    for path in paths:
        try:
            with open(path, encoding='utf-8') as f:
                rules, skipped = parse_domain_bridge_config(f.read())
        except (OSError, yaml.YAMLError, ValueError, TypeError) as e:
            results.append(ConfigResult(path, [], [], e))
            continue
        results.append(ConfigResult(path, rules, skipped, None))
    return results
