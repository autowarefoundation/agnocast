"""Register Agnocast domain bridge rules with the kernel module.

Unsupported: the kmod cross-domain zero-copy path is incomplete; relay between ROS
domains with the external ``domain_bridge`` node instead.

Reads one or more ROS 2 ``domain_bridge`` YAMLs and registers each
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
import sys

from . import domain_bridge_config

# Only registration is unsupported -- the agent reads the same YAML to force the A2R bridge the
# external domain_bridge node needs.
UNSUPPORTED_NOTICE = (
    'registering Agnocast domain bridge rules (kmod cross-domain zero-copy) is incomplete and '
    'unsupported; use the external domain_bridge node instead')


def _load_add_rule_symbol():
    """Load the ioctl wrapper and return the bound add_agnocast_domain_bridge_rule."""
    lib = ctypes.CDLL('libagnocast_ioctl_wrapper.so')
    lib.add_agnocast_domain_bridge_rule.argtypes = [
        ctypes.c_char_p, ctypes.c_char_p, ctypes.c_uint32, ctypes.c_uint32]
    lib.add_agnocast_domain_bridge_rule.restype = ctypes.c_int
    return lib.add_agnocast_domain_bridge_rule


def main(argv=None) -> int:
    """Register every rule in the configs; return non-zero if any rule or config is rejected."""
    parser = argparse.ArgumentParser(
        description='Register Agnocast domain bridge rules with the kernel module. '
                    f'Unsupported: {UNSUPPORTED_NOTICE}.')
    parser.add_argument(
        '--config',
        nargs='+',
        default=domain_bridge_config.resolve_config_paths()[0],
        help='paths to the domain_bridge YAMLs, applied in order '
             f'(default: ${domain_bridge_config.CONFIG_ENV}, a '
             f"'{domain_bridge_config.CONFIG_PATH_SEP}'-separated list, "
             f'else {domain_bridge_config.DEFAULT_CONFIG_PATH} and '
             f'{domain_bridge_config.default_config_dir()}/*.yaml)')
    args = parser.parse_args(argv)

    # Before the configs are read, so an operator sees it even on a run that registers nothing.
    print(f'warning: {UNSUPPORTED_NOTICE}', file=sys.stderr)

    rules = []
    unreadable = 0
    for result in domain_bridge_config.load_domain_bridge_rules(args.config):
        if result.error is not None:
            print(f'error: cannot load {result.path}: {result.error}', file=sys.stderr)
            unreadable += 1
            continue
        rules.extend(result.rules)
        for topic in result.skipped:
            print(f'warning: skipping {topic}: no from_domain/to_domain resolved '
                  '(set them at the top level or on the topic)', file=sys.stderr)

    failures = 0
    # Nothing to register: leave the wrapper unloaded so an unreadable config is what gets
    # reported, not a missing library.
    if rules:
        add_rule = _load_add_rule_symbol()
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
    return 1 if failures or unreadable else 0


if __name__ == '__main__':
    sys.exit(main())
