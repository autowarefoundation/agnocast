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
target_package="agnocast-kmod-v${target_version}"

# --- Prerequisites ---------------------------------------------------------

for cmd in apt-get dpkg dkms modprobe lsmod; do
	if ! command -v "$cmd" >/dev/null 2>&1; then
		echo "Error: '$cmd' not found. This script must run on the host with apt/dkms available."
		exit 1
	fi
done

echo "Target: ${target_package}"
echo ""
echo "WARNING: Make sure every container using agnocast is stopped before continuing."
echo "         The heaphook inside your container must match version v${target_version}."
read -rp "Proceed? [y/N] " answer
if ! [[ ${answer:0:1} =~ y|Y ]]; then
	echo "Cancelled."
	exit 1
fi

# --- Step 1: Unload current module -----------------------------------------

echo "[1/5] Unload agnocast kernel module"

if lsmod | awk '{print $1}' | grep -qx agnocast; then
	if ! sudo modprobe -r agnocast; then
		echo "  Error: failed to unload agnocast. A process may still be holding /dev/agnocast."
		echo "         Stop all agnocast users (containers, ROS nodes) and retry."
		exit 1
	fi
	echo "  Unloaded."
else
	echo "  Not loaded. Skipping."
fi

# --- Step 2: Purge existing agnocast-kmod packages -------------------------

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

# --- Step 3: Remove leftover DKMS entries ----------------------------------

echo "[3/5] Remove leftover DKMS entries for agnocast"

dkms_versions=$(dkms status agnocast 2>/dev/null \
	| awk -F'[/,:]' '/^agnocast/ {gsub(/ /,"",$2); print $2}' \
	| sort -u || true)

if [ -n "$dkms_versions" ]; then
	while IFS= read -r ver; do
		[ -z "$ver" ] && continue
		echo "  Removing agnocast/${ver}"
		sudo dkms remove "agnocast/${ver}" --all || true
	done <<<"$dkms_versions"
else
	echo "  No DKMS entries. Skipping."
fi

# --- Step 4: Install target version ----------------------------------------

echo "[4/5] Install ${target_package}"

sudo apt-get update
sudo apt-get install -y "${target_package}"

# --- Step 5: Load and verify -----------------------------------------------

echo "[5/5] Load agnocast and verify"

sudo modprobe agnocast
echo "  Loaded."
echo ""
echo "  Recent dmesg lines mentioning agnocast:"
sudo dmesg | grep -i agnocast | tail -n 5 || true

echo ""
echo "Done. Installed: ${target_package}"
