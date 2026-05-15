#!/usr/bin/env bash

# Exit on error, undefined var, or failed pipeline
set -euo pipefail

# -----------------------------
# Default configuration
# -----------------------------

DEFAULT_HOST="192.168.1.1"
REMOTE_HOST="$DEFAULT_HOST"
REMOTE_USER="fit4med"

DEST="$HOME/fit4med_ws/src/"
REMOTE_PATH="/home/fit4med/fit4med_ws/src/"

# -----------------------------
# Parse arguments
# -----------------------------

DRY_RUN=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --dry-run)
      DRY_RUN=true
      shift
      ;;
    --host)
      REMOTE_HOST="$2"
      shift 2
      ;;
    *)
      echo "[ERROR] Unknown option: $1"
      echo "Usage: $0 [--dry-run] [--host <ip>]"
      exit 1
      ;;
  esac
done

# Build source string

SRC="${REMOTE_USER}@${REMOTE_HOST}:${REMOTE_PATH}"

# -----------------------------
# rsync options
# -----------------------------

RSYNC_OPTS=(
  -av
  --update
  --exclude='.git/'
)

if [[ "$DRY_RUN" == true ]]; then
  echo "[INFO] Running in DRY-RUN mode (no changes will be applied)"
  RSYNC_OPTS+=(--dry-run)
fi

# -----------------------------
# Pre-checks
# -----------------------------

echo "[INFO] Using remote host: $REMOTE_HOST"

# Ping check
echo "[INFO] Checking connectivity (ping)..."
if ping -c 1 -W 2 "$REMOTE_HOST" >/dev/null 2>&1; then
  echo "[OK] Host reachable"
else
  echo "[ERROR] Host not reachable: $REMOTE_HOST"
  exit 1
fi

# SSH checkecho "[INFO] Checking SSH access..."
if ssh -o BatchMode=yes -o ConnectTimeout=5 "${REMOTE_USER}@${REMOTE_HOST}" "exit" >/dev/null 2>&1; then
  echo "[OK] SSH key authentication works"
else
  echo "[WARN] Key auth failed, trying password login..."
  if ssh -o ConnectTimeout=5 "${REMOTE_USER}@${REMOTE_HOST}" "exit"; then
    echo "[OK] SSH access via password verified"
  else
    echo "[ERROR] SSH connection failed"
    exit 1
  fi
fi
# -----------------------------
# Execution
# -----------------------------

echo "[INFO] Starting synchronization..."
echo "[INFO] Source:      $SRC"
echo "[INFO] Destination: $DEST"
echo "[INFO] rsync options: ${RSYNC_OPTS[*]}"

rsync "${RSYNC_OPTS[@]}" "$SRC" "$DEST"

# -----------------------------
# Done
# -----------------------------

echo "[INFO] Sync completed successfully ✅"
