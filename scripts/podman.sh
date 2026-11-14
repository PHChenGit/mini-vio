#!/usr/bin/env bash
# Helper for the Mini-VIO Podman environment.
#
#   scripts/podman.sh build          build the image
#   scripts/podman.sh start          start the long-running dev container
#   scripts/podman.sh enter          open a shell in the running container
#   scripts/podman.sh stop           stop and remove the dev container
#   scripts/podman.sh run <cmd...>   run one command in a throwaway container
#
# The repo is mounted at /workspace (read-write), the EuRoC dataset at
# /data/euroc (read-only). Override the dataset path with EUROC_DIR.
set -euo pipefail

IMAGE="${MINI_VIO_IMAGE:-localhost/mini-vio:dev}"
CONTAINER="${MINI_VIO_CONTAINER:-mini-vio}"
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
EUROC_DIR="${EUROC_DIR:-$HOME/Documents/datasets/EuRoC}"

mount_args=(
    --userns=keep-id            # files written to /workspace stay owned by you
    -v "$REPO_DIR:/workspace"
    -v "$EUROC_DIR:/data/euroc:ro"
    -w /workspace
)

case "${1:-}" in
    build)
        podman build -t "$IMAGE" -f "$REPO_DIR/Containerfile" "$REPO_DIR"
        ;;
    start)
        if podman container exists "$CONTAINER"; then
            podman start "$CONTAINER" >/dev/null
        else
            podman run -d --name "$CONTAINER" "${mount_args[@]}" "$IMAGE" sleep infinity >/dev/null
        fi
        echo "Container '$CONTAINER' is running. Enter it with: scripts/podman.sh enter"
        ;;
    enter)
        podman exec -it -w /workspace "$CONTAINER" bash
        ;;
    stop)
        podman rm -f "$CONTAINER" >/dev/null
        echo "Container '$CONTAINER' removed."
        ;;
    run)
        shift
        podman run --rm "${mount_args[@]}" "$IMAGE" "$@"
        ;;
    *)
        sed -n '2,11p' "${BASH_SOURCE[0]}"
        exit 1
        ;;
esac
