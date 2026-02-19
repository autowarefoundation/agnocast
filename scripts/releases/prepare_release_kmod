#!/bin/bash
set -eux

if ! command -v debuild &> /dev/null; then
  echo "Error: debuild is not installed. Install it with: sudo apt install devscripts"
  exit 1
fi

if ! dpkg -s dh-dkms &> /dev/null; then
  echo "Error: dh-dkms is required. Install it with: sudo apt install dh-dkms"
  exit 1
fi

VERSION="2.2.0"
RELEASE=$(dpkg-parsechangelog -l agnocast_kmod/debian/changelog -S Version | cut -d'-' -f2)
PKG_NAME="agnocast-kmod-v${VERSION}-${VERSION}"

# Clean up old artifacts
rm -rf ${PKG_NAME} agnocast-kmod-v${VERSION}_${VERSION}*

# Copy source into the package directory and remove stale debian/files
cp -r agnocast_kmod ${PKG_NAME}
rm -f ${PKG_NAME}/debian/files

# Create orig tarball (exclude debian/ so dpkg-source generates a proper debian diff)
tar --exclude=debian --exclude=.git -czf agnocast-kmod-v${VERSION}_${VERSION}.orig.tar.gz ${PKG_NAME}

# Build signed source package (.dsc, .changes, and .debian.tar.xz land in the current directory)
cd ${PKG_NAME}
debuild -S -sa
cd ..

# Clean up the build directory
rm -rf ${PKG_NAME}

# Verify dh-dkms is in the .dsc
if ! grep -q "dh-dkms" "agnocast-kmod-v${VERSION}_${VERSION}-${RELEASE}.dsc"; then
  echo "FATAL: dh-dkms is missing from the .dsc file!"
  exit 1
fi

echo "Success: .dsc contains dh-dkms"
