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
# The dev container also runs sshd: `ssh -p 7777 ubuntu@<host>` with a key from
# the host's ~/.ssh/authorized_keys. Override the port with REMOTE_PORT.
set -euo pipefail

IMAGE="${MINI_VIO_IMAGE:-localhost/mini-vio:dev}"
CONTAINER="${MINI_VIO_CONTAINER:-mini-vio}"
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
EUROC_DIR="${EUROC_DIR:-$HOME/Documents/datasets/EuRoC}"
REMOTE_PORT="${REMOTE_PORT:-7777}"

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
            echo "Create container '$CONTAINER'"
            podman run -d --name "$CONTAINER" "${mount_args[@]}" \
                -p "0.0.0.0:${REMOTE_PORT}:22" \
                -v "$HOME/.ssh/authorized_keys:/home/ubuntu/.ssh/authorized_keys:ro" \
                --init "$IMAGE" sleep infinity >/dev/null   # --init reaps finished sshd sessions
        fi
        # sshd needs root; the container's default user stays `ubuntu` for enter.
        podman exec -u root "$CONTAINER" sh -c \
            'kill -0 "$(cat /run/sshd.pid 2>/dev/null)" 2>/dev/null || { mkdir -p /run/sshd && /usr/sbin/sshd -E /var/log/sshd.log -o LogLevel=VERBOSE; }'
        echo "Container '$CONTAINER' is running. Enter it with: scripts/podman.sh enter"
        echo "or over SSH: ssh -p ${REMOTE_PORT} ubuntu@$(uname -n)"
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
