#!/usr/bin/env bash
# Format C++ sources with clang-format (Google style, 120 columns, 4-space indent, see .clang-format).
#
#   scripts/format.sh                 format every tracked and new C++ file in place
#   scripts/format.sh --check         only report files that need formatting (exit 1 if any)
#   scripts/format.sh [--check] FILE...   limit to the given files
#
# Works on the host if its clang-format has the pinned version, otherwise run it
# in the container: scripts/podman.sh run scripts/format.sh
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."

check=0
if [[ "${1:-}" == "--check" ]]; then
    check=1
    shift
fi

if (($# > 0)); then
    files=("$@")
else
    mapfile -t files < <(git ls-files --cached --others --exclude-standard -- '*.h' '*.cc')
fi
((${#files[@]} > 0)) || exit 0

if ((check)); then
    clang-format --dry-run --Werror "${files[@]}"
else
    clang-format -i "${files[@]}"
fi
