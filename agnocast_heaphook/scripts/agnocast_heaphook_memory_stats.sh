#!/bin/sh
# Periodically prints the shared heap usage of every process started with
# AGNOCAST_HEAPHOOK_MEMORY_STATS set.
#
# current: bytes allocated now, max: peak of current,
# hwm: peak span from the lowest to the highest allocated address (allocated + external fragmentation).
# name: the full ROS node name (/namespace/node) from the process's command line, or the executable
# name for processes without one. Bridge processes forked by agnocast get a "[bridge]" suffix.
# Each line starts with an ISO 8601 timestamp with milliseconds and UTC offset
# (e.g. 2026-10-21T16:04:08.123+09:00), so logs can span days and time zone changes.
#
# Usage: agnocast_heaphook_memory_stats.sh [interval_seconds]   (default: 1)

interval=${1:-1}

# Sets $name for the pid in $1. Spawns processes, so the result is cached per pid.
lookup_name() {
  args=$(tr '\0' '\n' < "/proc/$1/cmdline" 2>/dev/null)
  node=$(printf '%s\n' "$args" | sed -n 's/^__node:=//p' | head -n 1)
  ns=$(printf '%s\n' "$args" | sed -n 's/^__ns:=//p' | head -n 1)
  if [ -n "$node" ]; then
    case $ns in
      '' | /) name=/$node ;;
      *) name=$ns/$node ;;
    esac
  else
    name=$(printf '%s\n' "$args" | head -n 1)
    name=${name##*/}
  fi
  # Keep the field free of whitespace so the line stays parseable.
  name=$(printf '%s' "${name:-?}" | tr -s '[:space:]' '_')
}

sweep=0
while true; do
  sweep=$((sweep + 1))
  # One timestamp per sweep: all pids of a sweep share it, so they can be summed per timestamp.
  now=$(date +%FT%T.%3N%:z)
  for f in /dev/shm/agnocast_heaphook_*; do
    [ -e "$f" ] || continue
    pid=${f##*_}
    # Skip files left behind by processes that did not exit cleanly.
    kill -0 "$pid" 2>/dev/null || continue
    set -- $(od -An -t u8 -N24 "$f")

    # Refresh the cached name if the pid was not seen in the previous sweep, so a reused pid
    # does not inherit the name of the process that had it before.
    eval "last_seen=\${seen_$pid:-0} name=\${name_$pid-}"
    if [ "$last_seen" -ne $((sweep - 1)) ] || [ -z "$name" ]; then
      lookup_name "$pid"
      eval "name_$pid=\$name"
    fi
    eval "seen_$pid=$sweep"
    # The bridge renames itself after forking, so check this every time (read is a builtin).
    comm=
    read -r comm < "/proc/$pid/comm" 2>/dev/null
    case $comm in
      agno_pbr_*) label="$name[bridge]" ;;
      *) label=$name ;;
    esac

    printf '%s pid=%-8s current=%-12s max=%-12s hwm=%-12s name=%s\n' \
      "$now" "$pid" "$1" "$2" "$3" "$label"
  done
  sleep "$interval"
done
