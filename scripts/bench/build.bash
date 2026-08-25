#!/usr/bin/env bash
set -euo pipefail
#
# Build the benchmark binaries against one or both Agnocast backends.
#
#   scripts/build.bash              # both backends
#   scripts/build.bash kmod
#   scripts/build.bash daemon
#
# Each backend gets an isolated colcon build/install tree under ws/<backend>/,
# because agnocastlib selects its backend with a compile definition that
# propagates to every downstream package.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=common.bash
source "${SCRIPT_DIR}/common.bash"

BACKENDS=()
CLEAN=false
BUILD_TYPE=Release

while [[ $# -gt 0 ]]; do
  case "$1" in
  kmod | daemon) BACKENDS+=("$1"); shift ;;
  --clean) CLEAN=true; shift ;;
  --debug) BUILD_TYPE=RelWithDebInfo; shift ;;
  -h | --help) sed -n '3,14p' "${BASH_SOURCE[0]}"; exit 0 ;;
  *) die "unknown option: $1" ;;
  esac
done
[[ ${#BACKENDS[@]} -gt 0 ]] || BACKENDS=(kmod daemon)

[[ -d "${AGNOCAST_ROOT}/src/agnocastlib" ]] ||
  die "AGNOCAST_ROOT does not look like an Agnocast checkout: ${AGNOCAST_ROOT}"

[[ -n "${ROS_DISTRO:-}" ]] || die "ROS is not sourced. Run: source /opt/ros/humble/setup.bash"

# --- heaphook: identical for both backends, so build it once -----------------

HEAPHOOK_SRC="${AGNOCAST_ROOT}/agnocast_heaphook/target/release/libagnocast_heaphook.so"
echo "==> Building agnocast_heaphook"
(cd "${AGNOCAST_ROOT}/agnocast_heaphook" && cargo build --release)
[[ -f "${HEAPHOOK_SRC}" ]] || die "heaphook build produced no ${HEAPHOOK_SRC}"

# --- per-backend workspaces --------------------------------------------------

build_backend() {
  local backend="$1"
  local ws build install extra_cmake=()

  ws="$(ws_dir "${backend}")"
  build="$(build_dir "${backend}")"
  install="$(install_dir "${backend}")"

  echo ""
  echo "============================================================"
  echo " Building backend: ${backend} ($(backend_label "${backend}"))"
  echo "============================================================"

  if [[ "${CLEAN}" == "true" ]]; then
    rm -rf "${build}" "${install}"
  fi
  mkdir -p "${ws}"

  if [[ "${backend}" == "kmod" ]]; then
    echo "==> Building agnocast kernel module"
    (cd "${AGNOCAST_ROOT}/agnocast_kmod" && make)
  else
    echo "==> Building agnocast_daemon"
    # The daemon needs AGNOCAST_BENCH_TIMING too: without it the responses carry
    # no timing trailer and the client can only report the round trip as a whole.
    (cd "${AGNOCAST_ROOT}/agnocast_daemon" && make CMAKE_ARGS=-DAGNOCAST_BENCH_TIMING=ON)
    extra_cmake+=(-DAGNOCAST_USE_DAEMON=ON)
  fi

  # AGNOCAST_BENCH_TIMING enables get_last_{publish,receive}_ipc_ns(), which is
  # how the two backends' metadata round trips are compared.
  echo "==> colcon build (${backend})"
  colcon build \
    --base-paths "${AGNOCAST_ROOT}/src" \
    --build-base "${build}" \
    --install-base "${install}" \
    --packages-up-to agnocast_bench_publisher agnocast_bench_subscriber \
    --symlink-install \
    --cmake-args \
    "-DCMAKE_BUILD_TYPE=${BUILD_TYPE}" \
    -DAGNOCAST_BENCH_TIMING=ON \
    "${extra_cmake[@]}"

  install -D -m 0755 "${HEAPHOOK_SRC}" "$(heaphook_lib "${backend}")"
  echo "==> ${backend}: install tree ready at ${install}"
}

for backend in "${BACKENDS[@]}"; do
  build_backend "${backend}"
done

echo ""
echo "============================================================"
echo " Build complete: ${BACKENDS[*]}"
echo "============================================================"
for backend in "${BACKENDS[@]}"; do
  echo "  ${backend}: $(pub_bin "${backend}")"
done
