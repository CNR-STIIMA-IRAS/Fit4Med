#!/usr/bin/env bash

FIT4MED_DEFAULT_HOST="10.2.15.217"
FIT4MED_REMOTE_USER="fit4med"
FIT4MED_LOCAL_PATH="$HOME/fit4med_ws/src/"
FIT4MED_REMOTE_PATH="/home/fit4med/fit4med_ws/src/"

fit4med_robot_sync_usage() {
  local script_name="$1"

  cat <<USAGE
Usage: ${script_name} [--dry-run] [--host <ip>] [--folder <relative/path>] [--skip-ssh-check]

Options:
  --dry-run            Show what would change without applying it.
  --host <ip>          Remote host IP address (default: ${FIT4MED_DEFAULT_HOST}).
  --folder <path>      Sync only one folder, relative to ${FIT4MED_LOCAL_PATH}.
  --path <path>        Alias for --folder.
  --skip-ssh-check     Skip the SSH preflight check.
  --no-ssh-check       Alias for --skip-ssh-check.
  --ssh-check          Run the SSH preflight check (default).
  -h, --help           Show this help message.
USAGE
}

fit4med_robot_sync_parse_args() {
  SCRIPT_NAME="$1"
  shift

  DRY_RUN=false
  CHECK_SSH=true
  REMOTE_HOST="$FIT4MED_DEFAULT_HOST"
  SELECTED_FOLDER=""

  while [[ $# -gt 0 ]]; do
    case "$1" in
      --dry-run)
        DRY_RUN=true
        shift
        ;;
      --host)
        if [[ $# -lt 2 || -z "$2" || "$2" == --* ]]; then
          echo "[ERROR] --host requires an IP address"
          fit4med_robot_sync_usage "$SCRIPT_NAME"
          exit 1
        fi
        REMOTE_HOST="$2"
        shift 2
        ;;
      --host=*)
        REMOTE_HOST="${1#*=}"
        if [[ -z "$REMOTE_HOST" ]]; then
          echo "[ERROR] --host requires an IP address"
          fit4med_robot_sync_usage "$SCRIPT_NAME"
          exit 1
        fi
        shift
        ;;
      --folder|--path)
        if [[ $# -lt 2 || -z "$2" || "$2" == --* ]]; then
          echo "[ERROR] $1 requires a folder path relative to ${FIT4MED_LOCAL_PATH}"
          fit4med_robot_sync_usage "$SCRIPT_NAME"
          exit 1
        fi
        fit4med_robot_sync_set_selected_folder "$2"
        shift 2
        ;;
      --folder=*|--path=*)
        fit4med_robot_sync_set_selected_folder "${1#*=}"
        shift
        ;;
      --skip-ssh-check|--no-ssh-check)
        CHECK_SSH=false
        shift
        ;;
      --ssh-check)
        CHECK_SSH=true
        shift
        ;;
      -h|--help)
        fit4med_robot_sync_usage "$SCRIPT_NAME"
        exit 0
        ;;
      *)
        echo "[ERROR] Unknown option: $1"
        fit4med_robot_sync_usage "$SCRIPT_NAME"
        exit 1
        ;;
    esac
  done
}

fit4med_robot_sync_set_selected_folder() {
  local folder="$1"

  if [[ -n "$SELECTED_FOLDER" ]]; then
    echo "[ERROR] Only one --folder/--path option can be used"
    fit4med_robot_sync_usage "$SCRIPT_NAME"
    exit 1
  fi

  while [[ "$folder" == ./* ]]; do
    folder="${folder#./}"
  done

  while [[ "$folder" == */ && "$folder" != "/" ]]; do
    folder="${folder%/}"
  done

  if [[ -z "$folder" || "$folder" == "." ]]; then
    echo "[ERROR] --folder requires a folder path relative to ${FIT4MED_LOCAL_PATH}"
    fit4med_robot_sync_usage "$SCRIPT_NAME"
    exit 1
  fi

  if [[ "$folder" == /* || "$folder" == "~"* || "$folder" == ".." || "$folder" == ../* || "$folder" == */.. || "$folder" == */../* || "$folder" == *"//"* ]]; then
    echo "[ERROR] Folder must stay inside ${FIT4MED_LOCAL_PATH}: $folder"
    fit4med_robot_sync_usage "$SCRIPT_NAME"
    exit 1
  fi

  SELECTED_FOLDER="$folder"
}

fit4med_robot_sync_set_base_rsync_options() {
  RSYNC_OPTS=(
    -av
    --update
    --exclude='.git/'
    --exclude='.github/'
  )

  if [[ -n "$SELECTED_FOLDER" ]]; then
    RSYNC_OPTS+=(--relative)
  fi
}

fit4med_robot_sync_apply_dry_run_options() {
  if [[ "$DRY_RUN" == true ]]; then
    echo "[INFO] Running in DRY-RUN mode (no changes will be applied)"
    RSYNC_OPTS+=(--dry-run "$@")
  fi
}

fit4med_robot_sync_preflight() {
  echo "[INFO] Using remote host: $REMOTE_HOST"

  echo "[INFO] Checking connectivity (ping)..."
  if ping -c 1 -W 2 "$REMOTE_HOST" >/dev/null 2>&1; then
    echo "[OK] Host reachable"
  else
    echo "[ERROR] Host not reachable: $REMOTE_HOST"
    exit 1
  fi

  if [[ "$CHECK_SSH" != true ]]; then
    echo "[INFO] Skipping SSH access check"
    return
  fi

  echo "[INFO] Checking SSH access..."
  if ssh -o BatchMode=yes -o ConnectTimeout=5 "${FIT4MED_REMOTE_USER}@${REMOTE_HOST}" "exit" >/dev/null 2>&1; then
    echo "[OK] SSH key authentication works"
  else
    echo "[WARN] Key auth failed, trying password login..."
    if ssh -o ConnectTimeout=5 "${FIT4MED_REMOTE_USER}@${REMOTE_HOST}" "exit"; then
      echo "[OK] SSH access via password verified"
    else
      echo "[ERROR] SSH connection failed"
      exit 1
    fi
  fi
}

fit4med_robot_sync_run_rsync() {
  local src="$1"
  local dest="$2"
  local filter_dry_run_output="${3:-false}"

  echo "[INFO] Starting synchronization..."
  if [[ -n "$SELECTED_FOLDER" ]]; then
    echo "[INFO] Selected folder: $SELECTED_FOLDER"
  fi
  echo "[INFO] Source:      $src"
  echo "[INFO] Destination: $dest"
  echo "[INFO] rsync options: ${RSYNC_OPTS[*]}"

  if [[ "$DRY_RUN" == true && "$filter_dry_run_output" == true ]]; then
    rsync "${RSYNC_OPTS[@]}" "$src" "$dest" | awk '$1 !~ /^cd/ && $0 !~ /\/$/'
  else
    rsync "${RSYNC_OPTS[@]}" "$src" "$dest"
  fi

  echo "[INFO] Sync completed successfully"
}

fit4med_robot_sync_source_path() {
  local base_path="$1"

  if [[ -n "$SELECTED_FOLDER" ]]; then
    printf '%s./%s' "$base_path" "$SELECTED_FOLDER"
  else
    printf '%s' "$base_path"
  fi
}

fit4med_robot_sync_check_local_selected_folder() {
  if [[ -n "$SELECTED_FOLDER" && ! -d "${FIT4MED_LOCAL_PATH}${SELECTED_FOLDER}" ]]; then
    echo "[ERROR] Local folder does not exist: ${FIT4MED_LOCAL_PATH}${SELECTED_FOLDER}"
    exit 1
  fi
}

fit4med_sync_from_robot() {
  fit4med_robot_sync_parse_args "$@"

  local src="${FIT4MED_REMOTE_USER}@${REMOTE_HOST}:$(fit4med_robot_sync_source_path "$FIT4MED_REMOTE_PATH")"
  local dest="$FIT4MED_LOCAL_PATH"

  fit4med_robot_sync_set_base_rsync_options
  RSYNC_OPTS+=(
    --exclude='__pycache__/'
    --exclude='*.pyc'
    --exclude='*.zip'
    --exclude='COLCON_IGNORE'
  )
  fit4med_robot_sync_apply_dry_run_options --itemize-changes
  fit4med_robot_sync_preflight
  fit4med_robot_sync_run_rsync "$src" "$dest" true
}

fit4med_sync_to_robot() {
  fit4med_robot_sync_parse_args "$@"

  local src
  src="$(fit4med_robot_sync_source_path "$FIT4MED_LOCAL_PATH")"
  local dest="${FIT4MED_REMOTE_USER}@${REMOTE_HOST}:${FIT4MED_REMOTE_PATH}"

  fit4med_robot_sync_check_local_selected_folder
  fit4med_robot_sync_set_base_rsync_options
  fit4med_robot_sync_apply_dry_run_options
  fit4med_robot_sync_preflight
  fit4med_robot_sync_run_rsync "$src" "$dest"
}
