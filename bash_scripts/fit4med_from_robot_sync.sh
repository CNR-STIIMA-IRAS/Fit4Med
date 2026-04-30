#!/usr/bin/env bash
set -euo pipefail

DEST="$HOME/fit4med_ws/src/"
SRC="fit4med@192.168.1.1:/home/fit4med/fit4med_ws/src/"

RSYNC_OPTS=(
  -av
  --update
  --exclude='.git/'
)

if [[ "${1:-}" == "--dry-run" ]]; then
  RSYNC_OPTS+=(--dry-run)
fi

rsync "${RSYNC_OPTS[@]}" "$SRC" "$DEST"