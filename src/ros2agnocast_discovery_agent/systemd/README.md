# Registering domain bridge rules at boot

Agnocast domain bridge rules must be registered **before any publisher or
subscriber for the bridged topics exists** — the kernel module rejects a rule
once a domain has allocated endpoint ids. Registration is therefore a one-time
boot step, independent of the discovery agent (which is observability-only and
never registers rules).

`register_domain_bridge` (a console script in this package) reads a ROS 2
`domain_bridge` YAML and registers each rule:

```bash
ros2 run ros2agnocast_discovery_agent register_domain_bridge
# reads /etc/agnocast/domain_bridge.yaml; override with --config or
# AGNOCAST_DOMAIN_BRIDGE_CONFIG
```

The discovery agent reads the same file, to force the A2R bridge that a topic
split across both an IPC namespace and a ROS domain needs (without it,
`domain_bridge` waits for a DDS publisher while the A2R bridge waits for a DDS
subscriber, and the topic never flows). The agent is `execv`'d from an
application process, so it only ever sees the default path or an environment
variable exported to that process — **not** a `--config` argument passed here.

Keep the file at `/etc/agnocast/domain_bridge.yaml`, or export
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
