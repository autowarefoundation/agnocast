#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

CHECKPATCH="${CHECKPATCH:-}"
if [[ -z "$CHECKPATCH" ]]; then
  if CHECKPATCH_PATH="$(command -v checkpatch.pl 2>/dev/null)"; then
    CHECKPATCH="$CHECKPATCH_PATH"
  else
    for candidate in \
      "/usr/src/linux-headers-$(uname -r)/scripts/checkpatch.pl" \
      "/lib/modules/$(uname -r)/build/scripts/checkpatch.pl"
    do
      if [[ -x "$candidate" ]]; then
        CHECKPATCH="$candidate"
        break
      fi
    done
  fi
fi

if [[ ! -x "$CHECKPATCH" ]]; then
  echo "Error: checkpatch.pl not found. Set CHECKPATCH to its full path or ensure checkpatch.pl is available in PATH." >&2
  exit 1
fi

IGNORE="LEADING_SPACE,POINTER_LOCATION,SUSPECT_CODE_INDENT,CODE_INDENT"
IGNORE="$IGNORE,LINE_SPACING,BRACES,OPEN_BRACE,SPLIT_STRING,BLOCK_COMMENT_STYLE"
IGNORE="$IGNORE,TRAILING_STATEMENTS,LINUX_VERSION_CODE"

KMOD_DIR="$REPO_ROOT/agnocast_kmod"
mapfile -d '' TARGET_FILES < <(find "$KMOD_DIR" -type f \( -name '*.c' -o -name '*.h' \) -print0 | sort -z)
if [[ ${#TARGET_FILES[@]} -eq 0 ]]; then
  echo "Error: no C/H source files found under $KMOD_DIR" >&2
  exit 1
fi

echo "Running checkpatch.pl on agnocast_kmod files..."
echo "Ignored checks: $IGNORE"
echo ""

"$CHECKPATCH" --no-tree -f --show-types --ignore="$IGNORE" "${TARGET_FILES[@]}"
