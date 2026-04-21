#!/bin/bash

# Switch the installed agnocast-kmod to a specified version.
#
# Usage:
#   ./scripts/switch_kmod.bash <version>
#
# Example:
#   ./scripts/switch_kmod.bash 2.3.3
#
# Intended for setups where autoware runs inside a container (so heaphook
# is switched by swapping containers) and only the host-side kmod needs
# to be swapped. The kmod and the heaphook inside the container must share
# the same ioctl ABI version.
#
# This script:
#   1. Unloads the currently loaded agnocast kernel module.
#   2. Purges every installed agnocast-kmod-v* package.
#   3. Removes leftover DKMS entries for agnocast.
#   4. Installs agnocast-kmod-v<version> from apt.
#   5. Loads the new module and prints the version info from dmesg.

set -euo pipefail

if [ $# -ne 1 ]; then
	echo "Usage: $0 <version>"
	echo "Example: $0 2.3.3"
	exit 1
fi

target_version="$1"

if ! [[ "${target_version}" =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
	echo "Error: version must be in full X.Y.Z form (e.g., 2.3.3); got '${target_version}'." >&2
	echo "       Short forms like '2.3' or 'v2.3.3' are not accepted." >&2
	exit 1
fi

target_package="agnocast-kmod-v${target_version}"

# --- Prerequisites ---------------------------------------------------------

for cmd in apt-get dpkg dkms modprobe lsmod; do
	if ! command -v "$cmd" >/dev/null 2>&1; then
		echo "Error: '$cmd' not found. This script must run on the host with apt/dkms available."
		exit 1
	fi
done

echo "Target: ${target_package}"

# Early-exit if the target package is already installed and the module is
# currently loaded — in that case there is nothing to switch.
if dpkg-query -W -f='${Status}' "${target_package}" 2>/dev/null | grep -q "install ok installed" \
	&& lsmod | awk '{print $1}' | grep -qx agnocast; then
	loaded_ver=$(sudo dmesg | grep -E "Agnocast installed! v" | tail -n 1 \
		| sed -n 's/.*Agnocast installed! v\([^[:space:]]*\).*/\1/p' || true)
	if [ "${loaded_ver}" = "${target_version}" ]; then
		echo "${target_package} is already installed and loaded. Nothing to do."
		exit 0
	fi
fi

echo ""
echo "WARNING: Make sure every container using agnocast is stopped before continuing."
echo "         The heaphook inside your container must match version v${target_version}."
read -rp "Proceed? [y/N] " answer
if ! [[ ${answer:0:1} =~ y|Y ]]; then
	echo "Cancelled."
	exit 1
fi

# Prime sudo credentials up-front so the 5 steps below don't stall on a
# password prompt mid-flow.
sudo -v

# --- Pre-flight: verify the target package is reachable --------------------
#
# Done BEFORE any destructive action so that a typo in the version or a
# missing apt source does not leave the host with no kmod installed.

echo "Pre-flight: verify ${target_package} is available in apt"
sudo apt-get update
if ! apt-cache show "${target_package}" >/dev/null 2>&1; then
	echo "Error: ${target_package} is not available in any configured apt source."
	echo "       Check the version string and your /etc/apt/sources.list*."
	exit 1
fi
echo "  OK."

# --- Step 1: Unload current module -----------------------------------------

echo "[1/5] Unload agnocast kernel module"

if lsmod | awk '{print $1}' | grep -qx agnocast; then
	if ! sudo modprobe -r agnocast; then
		echo "  Error: failed to unload agnocast. A process may still be holding /dev/agnocast."
		if command -v fuser >/dev/null 2>&1 && [ -e /dev/agnocast ]; then
			echo "  Processes currently using /dev/agnocast:"
			sudo fuser -v /dev/agnocast || echo "  (fuser reported no users; module refcount may be held by another path)"
		else
			echo "  (fuser not available; install 'psmisc' to see which PIDs hold /dev/agnocast)"
		fi
		echo "         Stop all agnocast users (containers, ROS nodes) and retry."
		exit 1
	fi
	echo "  Unloaded."
else
	echo "  Not loaded. Skipping."
fi

# --- Step 2: Purge existing agnocast-kmod-v* packages ----------------------

echo "[2/5] Purge existing agnocast-kmod-v* packages"

installed_pkgs=$(dpkg-query -W -f='${Package} ${Status}\n' 'agnocast-kmod-v*' 2>/dev/null \
	| awk '$NF == "installed" {print $1}' || true)

if [ -n "$installed_pkgs" ]; then
	# shellcheck disable=SC2086
	sudo apt-get purge -y $installed_pkgs
	echo "  Purged: $installed_pkgs"
else
	echo "  None installed. Skipping."
fi

# --- Step 3: Remove leftover DKMS entries and orphan modules ---------------

echo "[3/5] Remove leftover DKMS entries and orphan modules"

# DKMS may have been left with broken entries (missing source dir) by old
# packages, in which case `dkms remove` fails. Fall back to deleting the
# state directory directly.
if [ -d /var/lib/dkms/agnocast ]; then
	for ver_dir in /var/lib/dkms/agnocast/*/; do
		[ -d "$ver_dir" ] || continue
		ver=$(basename "$ver_dir")
		# Skip per-kernel build directories (e.g. kernel-6.8.0-XX-generic-x86_64);
		# `dkms remove agnocast/<ver> --all` cleans those up as a side effect.
		[[ "$ver" == kernel-* ]] && continue
		echo "  Removing agnocast/${ver}"
		if ! sudo dkms remove "agnocast/${ver}" --all 2>/dev/null; then
			sudo rm -rf "$ver_dir"
		fi
	done
	# Remove any kernel-* build dirs still left over (e.g. if `dkms remove`
	# failed above and we fell back to rm -rf of the version dir only).
	sudo rm -rf /var/lib/dkms/agnocast/kernel-*
	sudo rmdir /var/lib/dkms/agnocast 2>/dev/null || true
else
	echo "  No DKMS entries."
fi

# Old unversioned installs may leave /lib/modules/<kver>/updates/dkms/agnocast.ko*
# behind, which blocks the new DKMS install with "already installed
# (unversioned module)".
kver=$(uname -r)
orphan_dir="/lib/modules/${kver}/updates/dkms"
if sudo test -e "${orphan_dir}/agnocast.ko" || sudo test -e "${orphan_dir}/agnocast.ko.zst"; then
	echo "  Removing orphan unversioned module in ${orphan_dir}"
	sudo rm -f "${orphan_dir}/agnocast.ko" "${orphan_dir}/agnocast.ko.zst"
	sudo depmod -a
fi

# --- Step 4: Install target version ----------------------------------------

echo "[4/5] Install ${target_package}"

sudo apt-get install -y "${target_package}"

# --- Step 5: Load and verify -----------------------------------------------

echo "[5/5] Load agnocast and verify"

# Snapshot dmesg line count so we can isolate messages emitted by THIS
# modprobe (avoids mistaking a stale 'Agnocast installed!' line from an
# earlier load for the current one).
dmesg_before=$(sudo dmesg | wc -l)

sudo modprobe agnocast
echo "  Loaded."

new_dmesg=$(sudo dmesg | tail -n +$((dmesg_before + 1)))
install_line=$(echo "$new_dmesg" | grep -E "Agnocast installed! v" | tail -n 1 || true)

if [ -z "$install_line" ]; then
	echo "  Error: did not see 'Agnocast installed! v...' in dmesg after modprobe."
	echo "  Recent agnocast-related kernel messages:"
	echo "$new_dmesg" | grep -i agnocast || true
	exit 1
fi

loaded_version=$(echo "$install_line" | sed -n 's/.*Agnocast installed! v\([^[:space:]]*\).*/\1/p')
echo "  ${install_line}"

if [ "$loaded_version" != "$target_version" ]; then
	echo "  Error: loaded version (${loaded_version}) does not match target (${target_version})."
	exit 1
fi

echo ""
echo "Done. Installed and loaded: ${target_package} (v${loaded_version})"
echo ""
echo "=========================================================================="
echo "  NEXT STEP — verify kmod / heaphook / agnocastlib versions"
echo "=========================================================================="
echo ""
echo "  In the environment where your ROS 2 / Agnocast app will actually run"
echo "  (inside the container for container-based setups, or on this host"
echo "  for direct setups), run:"
echo ""
echo "      source <your-workspace>/install/setup.bash   # must include ros2agnocast"
echo "      ros2 agnocast -v"
echo ""
echo "=========================================================================="
