#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

CHECKPATCH=/usr/src/linux-headers-"$(uname -r)"/scripts/checkpatch.pl

if [[ ! -x "$CHECKPATCH" ]]; then
  echo "Error: checkpatch.pl not found at $CHECKPATCH" >&2
  exit 1
fi

IGNORE="LEADING_SPACE,POINTER_LOCATION,SUSPECT_CODE_INDENT,CODE_INDENT"
IGNORE="$IGNORE,LINE_SPACING,BRACES,OPEN_BRACE,SPLIT_STRING,BLOCK_COMMENT_STYLE"
IGNORE="$IGNORE,TRAILING_STATEMENTS,LINUX_VERSION_CODE"

KMOD_DIR="$REPO_ROOT/agnocast_kmod"
TARGET_FILES=(
  "$KMOD_DIR"/agnocast_main.c
  "$KMOD_DIR"/agnocast_init.c
  "$KMOD_DIR"/agnocast_exit.c
  "$KMOD_DIR"/agnocast_ioctl.c
  "$KMOD_DIR"/agnocast_internal.c
  "$KMOD_DIR"/agnocast_memory_allocator.c
  "$KMOD_DIR"/agnocast.h
  "$KMOD_DIR"/agnocast_internal.h
  "$KMOD_DIR"/agnocast_memory_allocator.h
)

echo "Running checkpatch.pl on agnocast_kmod files..."
echo "Ignored checks: $IGNORE"
echo ""

"$CHECKPATCH" --no-tree -f --show-types --ignore="$IGNORE" "${TARGET_FILES[@]}"
