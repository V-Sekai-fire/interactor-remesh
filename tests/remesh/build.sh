#!/bin/sh
# Build remesh.elf and run its gates through the sandbox host's probe: the
# sphere remesh and simplify (half a voxel; planted 2 mm fails), the surface
# transfer (0.01 mm; planted 0.5 mm tangential shift fails) and the alpha
# cull (exactly half a split plane kept; a planted opaque texture fails) and
# the LOD chain (measured sag within 2x the reported error; a planted 10x
# under-report fails) and the avatar pipeline (atlas, attribute transfer and
# the shared error budget, each with its own planted defect) and the cluster
# LOD (every cut closed and within twice its threshold; a planted 10x
# under-report fails).
#
#   SANDBOX_API=<sandbox-api> MESHOPTIMIZER_DIR=<meshoptimizer> \
#   RV64_TOOLCHAIN=<riscv64-sysroot>/toolchain.cmake \
#   PROBE=<sbhost_probe> HOST_LIB=<sandbox_host library> tests/remesh/build.sh
#
# clang and lld come from pixi: pixi exec -s clangxx -s lld -s cmake -s ninja -- tests/remesh/build.sh
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/../.." && pwd)
OUT="$ROOT/build/remesh"
cmake -S "$ROOT/guest/remesh" -B "$OUT" -G Ninja -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_TOOLCHAIN_FILE="$RV64_TOOLCHAIN" -DSANDBOX_API="$SANDBOX_API" -DMESHOPTIMIZER_DIR="$MESHOPTIMIZER_DIR"
cmake --build "$OUT"
echo "built $OUT/remesh"
[ -n "$PROBE" ] || exit 0
# Each gate must PASS clean and FAIL with its planted defect.
check() { # name clean-arg planted-arg
  clean=$("$PROBE" "$HOST_LIB" call "$OUT/remesh" "$1" "$2" | grep '^rc')
  planted=$("$PROBE" "$HOST_LIB" call "$OUT/remesh" "$1" "$3" | grep '^rc')
  echo "$clean"
  echo "$planted"
  case "$clean" in *"result PASS"*) ;; *) echo "$1: clean run did not pass"; exit 1;; esac
  case "$planted" in *"result FAIL"*) ;; *) echo "$1: planted control did not fail"; exit 1;; esac
}
check gate i:0 i:2000
check gate_transfer i:0 i:500
check gate_alpha i:0 i:1
check gate_lod i:1 i:10
check gate_avatar i:0 i:1
check gate_avatar i:0 i:2
check gate_avatar i:0 i:3
check gate_clod i:1 i:10
echo "gates: PASS, controls: FAIL as planted"
