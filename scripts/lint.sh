#!/usr/bin/env bash
# Lint C++ sources with clang-tidy (rules in .clang-tidy). Run inside the container:
#
#   scripts/podman.sh run scripts/lint.sh     # from the host
#   scripts/lint.sh                           # inside the container: every .cc file
#   scripts/lint.sh FILE.cc...                # only these files
#
# Headers are checked through the .cc files that include them. clang-tidy needs
# build/compile_commands.json, so CMake is (re)configured first; that is fast.
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."
BUILD_DIR="${BUILD_DIR:-build}"

if (($# > 0)); then
    files=("$@")
else
    mapfile -t files < <(git ls-files --cached --others --exclude-standard -- '*.cc')
fi
((${#files[@]} > 0)) || exit 0

cmake -S . -B "$BUILD_DIR" -G Ninja >/dev/null

# A .cc file that no CMake target compiles yet has no compile command; clang-tidy
# would guess the flags and report bogus errors, so skip it with a note.
in_db=()
for f in "${files[@]}"; do
    if grep -q "\"file\": \"$PWD/$f\"" "$BUILD_DIR/compile_commands.json"; then
        in_db+=("$f")
    else
        echo "lint: skipping $f (not compiled by any CMake target yet)" >&2
    fi
done
((${#in_db[@]} > 0)) || exit 0

printf '%s\n' "${in_db[@]}" | xargs -P "$(nproc)" -n 1 clang-tidy --quiet -p "$BUILD_DIR"
