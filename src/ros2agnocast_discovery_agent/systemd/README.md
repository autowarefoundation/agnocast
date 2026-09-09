# Registering domain bridge rules at boot

> **Unsupported.** Registering rules with the kernel module — the kmod
> cross-domain zero-copy path — is incomplete and not supported. Use the
> external `domain_bridge` node to relay between ROS domains. The tool below
> still works, but it warns on every run and the kernel module warns once per
> module load. The discovery agent's use of the same file, described below, is
> separate and supported.

Agnocast domain bridge rules must be registered **before any publisher or
subscriber for the bridged topics exists** — the kernel module rejects a rule
once a domain has allocated endpoint ids. Registration is therefore a one-time
boot step, independent of the discovery agent (which is observability-only and
never registers rules).

`register_domain_bridge` (a console script in this package) reads ROS 2
`domain_bridge` YAMLs and registers each rule:

```bash
ros2 run ros2agnocast_discovery_agent register_domain_bridge
# reads /etc/agnocast/domain_bridge.yaml; override with --config or
# AGNOCAST_DOMAIN_BRIDGE_CONFIG
```

Several configs can be listed, applied in the order given, the way `domain_bridge`
itself takes several positional arguments: `--config a.yaml b.yaml`, or a
`:`-separated `AGNOCAST_DOMAIN_BRIDGE_CONFIG`. `topics` accumulate across files,
while `from_domain` / `to_domain` stay local to the file that sets them — the same
merge `domain_bridge` performs. Unlike `domain_bridge`, which aborts on the first
unreadable file, a bad file here is reported and skipped so the rest still apply.

Drop-ins need no environment variable: every `*.yaml` in
`/etc/agnocast/domain_bridge.d/` is read in name order (`10-base.yaml`,
`20-lidar.yaml`), after `/etc/agnocast/domain_bridge.yaml` when that exists — the
layout systemd and sysctl use, but not their precedence: a later file only adds,
it never overrides an earlier one. Two files that bridge the same topic and domain
to different places are a configuration error, which the kernel module rejects.

The discovery agent reads the same files, to force the A2R bridge that a topic
split across both an IPC namespace and a ROS domain needs (without it,
`domain_bridge` waits for a DDS publisher while the A2R bridge waits for a DDS
subscriber, and the topic never flows). The agent is `execv`'d from an
application process, so it only ever sees the default path or an environment
variable exported to that process — **not** a `--config` argument passed here.

Keep the config at `/etc/agnocast/domain_bridge.yaml`, or export
`AGNOCAST_DOMAIN_BRIDGE_CONFIG` to the applications as well. Registering with
`--config` alone leaves the rules in the kmod but the forcing off, which the
agent reports only in its own log.

Run it once, ordered so that it:

1. runs **after** the Agnocast kernel module is loaded (so `/dev/agnocast` exists),
2. runs **after** the filesystem holding the config is mounted, and
3. completes **before** any application node for the bridged topics starts.

`agnocast-domain-bridge.service.example` is a reference systemd one-shot that
expresses exactly this ordering (`After=` the kmod, `RequiresMountsFor=` the
config, `Before=` your application target). Agnocast does not ship an installed
unit or assume systemd — an init script or a container entrypoint that
satisfies the same ordering works just as well.

The tool is idempotent (the kmod folds duplicate rules) and exits non-zero if
any rule is rejected, so a misordering — a node started first — fails loudly
instead of silently leaving topics unbridged.
