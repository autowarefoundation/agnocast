#!/bin/bash
set -eux

if ! command -v debuild &> /dev/null; then
  echo "Error: debuild is not installed. Install it with: sudo apt install devscripts"
  exit 1
fi

VERSION="2.2.0"

cp -r agnocast_heaphook agnocast-heaphook-v${VERSION}-${VERSION}
rm -f agnocast-heaphook-v${VERSION}-${VERSION}/Cargo.lock
rm -rf agnocast-heaphook-v${VERSION}-${VERSION}/target
tar czf agnocast-heaphook-v${VERSION}_${VERSION}.orig.tar.gz agnocast-heaphook-v${VERSION}-${VERSION}

cd agnocast-heaphook-v${VERSION}-${VERSION}
debuild -S -sa

# When re-upload with a next release number
# debuild -S -sd

cd ..
rm -rf agnocast-heaphook-v${VERSION}-${VERSION}
