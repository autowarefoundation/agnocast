#!/usr/bin/env bats
#
# Tests for scripts/switch_kmod.bash.
#
# DESTRUCTIVE: these tests touch real /var/lib/dkms/agnocast, purge and
# reinstall agnocast-kmod-v* packages, and load/unload the kernel module.
# Every test is expected to leave the host in the canonical state (defined
# below) via teardown; do not run this suite on a machine you care about.
#
# Usage:
#     # One-time canonical setup (also stashes a .deb for recovery):
#     sudo bash scripts/test/switch_kmod_canonical_setup.bash
#
#     # Run:
#     sudo bats scripts/test/switch_kmod.bats
#
# Required versions (overridable via env):
#     CANONICAL_VER   version the host is restored to between tests (default 2.3.3)
#     UPGRADE_VER     version used as the swap target (default 2.1.2)

CANONICAL_VER="${CANONICAL_VER:-2.3.3}"
UPGRADE_VER="${UPGRADE_VER:-2.1.2}"

setup_file() {
	if [ "$(id -u)" -ne 0 ]; then
		echo "ERROR: must run as root (sudo bats ${BATS_TEST_FILENAME})" >&2
		return 1
	fi

	SCRIPT_DIR="$(cd "$(dirname "${BATS_TEST_FILENAME}")/.." && pwd)"
	SCRIPT="${SCRIPT_DIR}/switch_kmod.bash"
	FIXTURES_DIR="$(cd "$(dirname "${BATS_TEST_FILENAME}")" && pwd)/switch_kmod_fixtures"
	export SCRIPT SCRIPT_DIR FIXTURES_DIR CANONICAL_VER UPGRADE_VER

	if ! _is_canonical; then
		echo "ERROR: host is not in canonical state (agnocast-kmod-v${CANONICAL_VER} installed and loaded)." >&2
		echo "       Run: sudo bash $(dirname "${BATS_TEST_FILENAME}")/switch_kmod_canonical_setup.bash" >&2
		return 1
	fi
}

setup() {
	if ! _is_canonical; then
		echo "ERROR: host is not in canonical state before test '${BATS_TEST_NAME}' (agnocast-kmod-v${CANONICAL_VER} installed and loaded)." >&2
		echo "       Run: sudo bash $(dirname "${BATS_TEST_FILENAME}")/switch_kmod_canonical_setup.bash" >&2
		return 1
	fi
}

teardown() {
	_is_canonical && return 0
	if ! _restore_canonical; then
		echo "WARNING: failed to restore canonical state after test '${BATS_TEST_NAME}'" >&2
		return 1
	fi
}

_is_canonical() {
	dpkg-query -W -f='${Status}' "agnocast-kmod-v${CANONICAL_VER}" 2>/dev/null \
		| grep -q "install ok installed" \
		&& lsmod | awk '{print $1}' | grep -qx agnocast
}

_restore_canonical() {
	modprobe -r agnocast 2>/dev/null || true

	local installed
	installed=$(dpkg-query -W -f='${Package} ${Status}\n' 'agnocast-kmod-v*' 2>/dev/null \
		| awk '$NF == "installed" {print $1}' || true)
	if [ -n "${installed}" ]; then
		# shellcheck disable=SC2086
		apt-get purge -y ${installed} >/dev/null
	fi

	rm -rf /var/lib/dkms/agnocast
	rm -f "/lib/modules/$(uname -r)/updates/dkms/agnocast.ko" \
	      "/lib/modules/$(uname -r)/updates/dkms/agnocast.ko.zst"
	depmod -a >/dev/null 2>&1 || true

	if ! apt-get install -y "agnocast-kmod-v${CANONICAL_VER}" >/dev/null 2>&1; then
		local deb
		deb=$(ls "${FIXTURES_DIR}/agnocast-kmod-v${CANONICAL_VER}"_*.deb 2>/dev/null | head -n 1)
		[ -n "${deb}" ] || return 1
		dpkg -i "${deb}" >/dev/null 2>&1 || return 1
	fi
	modprobe agnocast
}

# ------------------------------------------------------------------ T1 --

@test "T1: rejects invocation with zero arguments" {
	run bash "${SCRIPT}"
	[ "${status}" -eq 1 ]
	[[ "${output}" == *"Usage:"* ]]
}

# ------------------------------------------------------------------ T2 --

@test "T2: pre-flight bails on unknown version before any destructive action" {
	run bash -c "echo y | '${SCRIPT}' 99.99.99"
	[ "${status}" -eq 1 ]
	[[ "${output}" == *"not available"* ]]
	_is_canonical
}

# ------------------------------------------------------------------ T3 --

@test "T3: aborts when confirmation prompt is declined" {
	run bash -c "echo n | '${SCRIPT}' '${UPGRADE_VER}'"
	[ "${status}" -eq 1 ]
	[[ "${output}" == *"Cancelled"* ]]
	_is_canonical
}

# ------------------------------------------------------------------ T4 --

@test "T4: exits when a prerequisite command is missing (dkms)" {
	local fake_bin bash_path
	fake_bin=$(mktemp -d)
	bash_path=$(command -v bash)
	local cmd
	for cmd in apt-get dpkg modprobe lsmod; do
		ln -s "$(command -v "${cmd}")" "${fake_bin}/${cmd}"
	done
	run env PATH="${fake_bin}" "${bash_path}" "${SCRIPT}" "${UPGRADE_VER}"
	rm -rf "${fake_bin}"
	[ "${status}" -eq 1 ]
	[[ "${output}" == *"'dkms' not found"* ]]
}

# ------------------------------------------------------------------ T5 --

@test "T5: early-exits when target is already installed and loaded" {
	run bash -c "echo y | '${SCRIPT}' '${CANONICAL_VER}'"
	[ "${status}" -eq 0 ]
	[[ "${output}" == *"Nothing to do"* ]]
}

# ------------------------------------------------------------------ T6 --

@test "T6: switches from canonical to upgrade version end-to-end" {
	run bash -c "echo y | '${SCRIPT}' '${UPGRADE_VER}'"
	[ "${status}" -eq 0 ]
	[[ "${output}" == *"Agnocast installed! v${UPGRADE_VER}"* ]]
	[[ "${output}" == *"Done."* ]]
	dpkg-query -W -f='${Status}' "agnocast-kmod-v${UPGRADE_VER}" | grep -q "install ok installed"
	run dpkg-query -W -f='${Status}' "agnocast-kmod-v${CANONICAL_VER}"
	[[ "${output}" != *"install ok installed"* ]]
	lsmod | awk '{print $1}' | grep -qx agnocast
}

# ------------------------------------------------------------------ T7 --

@test "T7: reports holding PID via fuser when unload fails" {
	# Hold /dev/agnocast open to bump the module refcount. The FD is inherited
	# by the child shell spawned by `run`, so modprobe -r will fail.
	exec 7<>/dev/agnocast
	run bash -c "echo y | '${SCRIPT}' '${UPGRADE_VER}'"
	exec 7>&-

	[ "${status}" -eq 1 ]
	[[ "${output}" == *"failed to unload agnocast"* ]]
	[[ "${output}" == *"Processes currently using /dev/agnocast"* ]]
	_is_canonical
}

# ------------------------------------------------------------------ T8 --

@test "T8: step 3 removes orphan unversioned .ko file" {
	modprobe -r agnocast
	local installed
	installed=$(dpkg-query -W -f='${Package} ${Status}\n' 'agnocast-kmod-v*' 2>/dev/null \
		| awk '$NF == "installed" {print $1}' || true)
	if [ -n "${installed}" ]; then
		# shellcheck disable=SC2086
		apt-get purge -y ${installed} >/dev/null
	fi
	rm -rf /var/lib/dkms/agnocast

	local orphan="/lib/modules/$(uname -r)/updates/dkms/agnocast.ko.zst"
	mkdir -p "$(dirname "${orphan}")"
	touch "${orphan}"

	run bash -c "echo y | '${SCRIPT}' '${UPGRADE_VER}'"
	[ "${status}" -eq 0 ]
	[[ "${output}" == *"Removing orphan unversioned module"* ]]
}

# ------------------------------------------------------------------ T9 --

@test "T9: step 3 cleans up orphan kernel-* symlinks pointing nowhere" {
	mkdir -p /var/lib/dkms/agnocast
	ln -sfn /nonexistent-target-for-test /var/lib/dkms/agnocast/kernel-orphan-x86_64

	run bash -c "echo y | '${SCRIPT}' '${UPGRADE_VER}'"
	[ "${status}" -eq 0 ]
	[ ! -L /var/lib/dkms/agnocast/kernel-orphan-x86_64 ]
}
