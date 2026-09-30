#!/bin/sh
# Build remesh.elf and run its sphere gate through the sandbox host's probe.
# The gate remeshes and simplifies a 0.1 m sphere, whose surface is analytic,
# and passes within half a voxel; the planted 2 mm control must fail.
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
clean=$("$PROBE" "$HOST_LIB" call "$OUT/remesh" gate i:0 | grep '^rc')
planted=$("$PROBE" "$HOST_LIB" call "$OUT/remesh" gate i:2000 | grep '^rc')
echo "$clean"
echo "$planted"
case "$clean" in *"result PASS"*) ;; *) echo "gate: clean sphere did not pass"; exit 1;; esac
case "$planted" in *"result FAIL"*) ;; *) echo "gate: planted 2 mm control did not fail"; exit 1;; esac
echo "gate: PASS, control: FAIL as planted"
