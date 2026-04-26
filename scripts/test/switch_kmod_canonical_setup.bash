#!/bin/bash
#
# Bring the host into the canonical state expected by switch_kmod.bats:
# agnocast-kmod-v${CANONICAL_VER} installed and loaded. Also stashes the
# canonical .deb under switch_kmod_fixtures/ so the bats teardown has a
# local fallback if apt cannot fetch it later.
#
# Run once before the first `sudo bats switch_kmod.bats`:
#     bash scripts/test/switch_kmod_canonical_setup.bash
#
# This helper itself does not need root — `apt-get download` does not
# acquire the dpkg lock, and `switch_kmod.bash` handles its own privilege
# escalation via sudo. You will be prompted for your sudo password once
# by the inner script.

set -euo pipefail

CANONICAL_VER="${CANONICAL_VER:-2.3.3}"

here="$(cd "$(dirname "$0")" && pwd)"
switch_script="$(cd "${here}/.." && pwd)/switch_kmod.bash"
fixtures_dir="${here}/switch_kmod_fixtures"

mkdir -p "${fixtures_dir}"

# Stash the canonical .deb locally; needed by teardown as a last-resort
# recovery path when apt is unreachable.
if ! ls "${fixtures_dir}/agnocast-kmod-v${CANONICAL_VER}"_*.deb >/dev/null 2>&1; then
	echo "Downloading agnocast-kmod-v${CANONICAL_VER}.deb into ${fixtures_dir}"
	(cd "${fixtures_dir}" && apt-get download "agnocast-kmod-v${CANONICAL_VER}")
fi

echo y | "${switch_script}" "${CANONICAL_VER}"

echo "Canonical state established: agnocast-kmod-v${CANONICAL_VER}"
