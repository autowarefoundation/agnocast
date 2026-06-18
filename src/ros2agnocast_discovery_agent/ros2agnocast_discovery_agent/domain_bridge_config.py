"""Parse a ROS 2 ``domain_bridge`` YAML into the (topic, from_domain, to_domain)
rules the Agnocast daemon registers with the kmod.

The same YAML drives both the external ``domain_bridge`` node (cross-ECU, Case 8)
and the daemon's kmod rule injection that opens same-namespace zero-copy
cross-domain delivery (Case 2). Only the topic name and domain pair matter here;
``type`` and other fields are ignored.
"""
import yaml

# Operators point the daemon at the config by setting this to the YAML path.
CONFIG_ENV = 'AGNOCAST_DOMAIN_BRIDGE_CONFIG'


def parse_domain_bridge_config(text):
    """Return a list of ``(topic_name, from_domain, to_domain)`` tuples.

    ``from_domain`` / ``to_domain`` are taken from the top level and may be
    overridden per topic. Entries without a resolvable domain pair are skipped.
    """
    doc = yaml.safe_load(text) or {}
    default_from = doc.get('from_domain')
    default_to = doc.get('to_domain')

    rules = []
    for topic_name, spec in (doc.get('topics') or {}).items():
        spec = spec or {}
        from_domain = spec.get('from_domain', default_from)
        to_domain = spec.get('to_domain', default_to)
        if from_domain is None or to_domain is None:
            continue
        rules.append((str(topic_name), int(from_domain), int(to_domain)))
    return rules


def load_domain_bridge_rules(path):
    """Read and parse the ``domain_bridge`` YAML at ``path``."""
    with open(path, encoding='utf-8') as f:
        return parse_domain_bridge_config(f.read())
