#!/usr/bin/env bash
# Configure, build and run all tests. Meant to run inside the container:
#
#   scripts/podman.sh run scripts/build_and_test.sh        # from the host
#   scripts/build_and_test.sh                              # inside the container
#
# Extra arguments are passed to the CMake configure step, e.g.
#   scripts/build_and_test.sh -DCMAKE_BUILD_TYPE=Debug
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."
BUILD_DIR="${BUILD_DIR:-build}"

cmake -S . -B "$BUILD_DIR" -G Ninja "$@"
cmake --build "$BUILD_DIR"
ctest --test-dir "$BUILD_DIR" --output-on-failure
