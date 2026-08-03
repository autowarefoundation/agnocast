"""Register Agnocast domain bridge rules with the kernel module.

Reads a ROS 2 ``domain_bridge`` YAML and registers each
``(from_topic, to_topic, from_domain, to_domain)`` rule through the ioctl wrapper
(``to_topic`` is the per-topic ``remap`` target, or the source name if absent).

Run this once, before any application node for the bridged topics starts: the
kmod rejects a rule once an endpoint exists in either domain. The tool is
standalone and idempotent (the kmod folds duplicate rules), so it can run from
a boot-time one-shot, a launch file, or by hand. It is independent of the
discovery agent, which is observability-only and never registers rules.
"""
import argparse
import ctypes
import os
import sys

import yaml

from . import domain_bridge_config


def _load_add_rule_symbol():
    """Load the ioctl wrapper and return the bound add_agnocast_domain_bridge_rule."""
    lib = ctypes.CDLL('libagnocast_ioctl_wrapper.so')
    lib.add_agnocast_domain_bridge_rule.argtypes = [
        ctypes.c_char_p, ctypes.c_char_p, ctypes.c_uint32, ctypes.c_uint32]
    lib.add_agnocast_domain_bridge_rule.restype = ctypes.c_int
    return lib.add_agnocast_domain_bridge_rule


def main(argv=None) -> int:
    """Register every rule in the config; return non-zero if any is rejected."""
    parser = argparse.ArgumentParser(
        description='Register Agnocast domain bridge rules with the kernel module.')
    parser.add_argument(
        '--config',
        default=os.environ.get(domain_bridge_config.CONFIG_ENV),
        help='path to the domain_bridge YAML '
             f'(default: ${domain_bridge_config.CONFIG_ENV})')
    args = parser.parse_args(argv)

    if not args.config:
        parser.error(
            f'no config given; pass --config or set {domain_bridge_config.CONFIG_ENV}')

    try:
        rules, skipped = domain_bridge_config.load_domain_bridge_rules(args.config)
    except (OSError, yaml.YAMLError, ValueError, TypeError) as e:
        print(f'error: cannot load {args.config}: {e}', file=sys.stderr)
        return 1

    for topic in skipped:
        print(f'warning: skipping {topic}: no from_domain/to_domain resolved '
              '(set them at the top level or on the topic)', file=sys.stderr)

    add_rule = _load_add_rule_symbol()
    failures = 0
    for from_topic, to_topic, from_domain, to_domain in rules:
        label = f'{from_topic}@{from_domain} -> {to_topic}@{to_domain}'
        if add_rule(
                from_topic.encode('utf-8'), to_topic.encode('utf-8'),
                from_domain, to_domain) == 0:
            print(f'registered: {label}')
            continue
        # The wrapper prints the specific errno to stderr just above; the usual
        # cause is that an endpoint already exists, since a rule must precede
        # every node in either domain.
        failures += 1
        print(f'error: failed to register {label}', file=sys.stderr)

    if failures:
        print(f'error: {failures} of {len(rules)} rule(s) rejected', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
