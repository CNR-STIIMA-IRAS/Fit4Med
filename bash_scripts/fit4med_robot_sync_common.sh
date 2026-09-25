#!/usr/bin/env bash

# shellcheck source=fit4med_backup_common.sh
source "$(dirname -- "${BASH_SOURCE[0]}")/fit4med_backup_common.sh"

FIT4MED_DEFAULT_HOST="$FIT4MED_ROBOT_HOST"
FIT4MED_REMOTE_USER="$FIT4MED_ROBOT_USER"
# Local workspace sources: by default the "src" folder that contains this
# repository (bash_scripts/../..), wherever it is. --local-path overrides it.
FIT4MED_LOCAL_PATH="${FIT4MED_LOCAL_PATH:-$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)/}"
FIT4MED_REMOTE_PATH="${FIT4MED_ROBOT_SRC}/"

fit4med_robot_sync_usage() {
  local script_name="$1"

  cat <<USAGE
Usage: ${script_name} [--dry-run] [--host <ip>] [--folder <relative/path>] [--skip-ssh-check]
       [--backup | --no-backup] [--yes] [--local-path <dir>] [--delete-extra-folders]

Options:
  --dry-run            Show what would change without applying it.
  --host <ip>          Remote host IP address (default: ${FIT4MED_DEFAULT_HOST}).
  --folder <path>      Sync only one folder, relative to ${FIT4MED_LOCAL_PATH}.
  --path <path>        Alias for --folder.
  --skip-ssh-check     Skip the SSH preflight check.
  --no-ssh-check       Alias for --skip-ssh-check.
  --ssh-check          Run the SSH preflight check (default).
  --backup             (to robot) Back up the robot sources first, without asking.
  --no-backup          (to robot) Do not back up and do not ask.
  --yes                (to robot) Do not ask before deleting robot files missing locally.
  --local-path <dir>   Local workspace "src" folder (default: ${FIT4MED_LOCAL_PATH}).
  --delete-extra-folders
                       (to robot) Also delete the robot folders that are not in the
                       local src. By default only the local folders are synced and
                       the others on the robot are left untouched.
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
  BACKUP_MODE=ask
  ASSUME_YES=false
  DELETE_EXTRA_FOLDERS=false

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
      --backup)
        BACKUP_MODE=yes
        shift
        ;;
      --no-backup)
        BACKUP_MODE=no
        shift
        ;;
      --yes|-y)
        ASSUME_YES=true
        shift
        ;;
      --delete-extra-folders)
        DELETE_EXTRA_FOLDERS=true
        shift
        ;;
      --local-path)
        if [[ $# -lt 2 || -z "$2" ]]; then
          echo "[ERROR] --local-path requires a folder"
          exit 1
        fi
        FIT4MED_LOCAL_PATH="${2%/}/"
        shift 2
        ;;
      --local-path=*)
        FIT4MED_LOCAL_PATH="${1#*=}"
        FIT4MED_LOCAL_PATH="${FIT4MED_LOCAL_PATH%/}/"
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
    # rsync splits -e on spaces and honours quotes, not backslashes.
    -e "ssh $(printf "'%s' " "${FIT4MED_SSH_OPTS[@]}")"
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
  if ssh "${FIT4MED_SSH_OPTS[@]}" -o BatchMode=yes -o ConnectTimeout=5 "${FIT4MED_REMOTE_USER}@${REMOTE_HOST}" "exit" >/dev/null 2>&1; then
    echo "[OK] SSH key authentication works"
  else
    echo "[WARN] Key auth failed, trying password login..."
    if ssh "${FIT4MED_SSH_OPTS[@]}" -o ConnectTimeout=5 "${FIT4MED_REMOTE_USER}@${REMOTE_HOST}" "exit"; then
      echo "[OK] SSH access via password verified"
    else
      echo "[ERROR] SSH connection failed"
      exit 1
    fi
  fi
}

# fit4med_robot_sync_run_rsync <filter_dry_run_output> <dest> <src>...
fit4med_robot_sync_run_rsync() {
  local filter_dry_run_output="$1"
  local dest="$2"
  shift 2

  echo "[INFO] Starting synchronization..."
  if [[ -n "$SELECTED_FOLDER" ]]; then
    echo "[INFO] Selected folder: $SELECTED_FOLDER"
  fi
  echo "[INFO] Source:      $*"
  echo "[INFO] Destination: $dest"
  echo "[INFO] rsync options: ${RSYNC_OPTS[*]}"

  if [[ "$DRY_RUN" == true && "$filter_dry_run_output" == true ]]; then
    rsync "${RSYNC_OPTS[@]}" "$@" "$dest" | awk '$1 !~ /^cd/ && $0 !~ /\/$/'
  else
    rsync "${RSYNC_OPTS[@]}" "$@" "$dest"
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

fit4med_robot_sync_check_local_path() {
  if [[ ! -d "$FIT4MED_LOCAL_PATH" ]]; then
    echo "[ERROR] Local folder not found: $FIT4MED_LOCAL_PATH"
    exit 1
  fi
  # Mirroring a folder that is not a workspace "src" (e.g. a plain clone in
  # ~/code) would copy its unrelated siblings and delete the robot packages.
  if [[ "$(basename -- "$FIT4MED_LOCAL_PATH")" != src ]]; then
    echo "[ERROR] $FIT4MED_LOCAL_PATH is not a workspace 'src' folder."
    echo "        Pass it explicitly with --local-path <.../src>."
    exit 1
  fi
  echo "[INFO] Local workspace sources: $FIT4MED_LOCAL_PATH"
}

fit4med_robot_sync_check_local_selected_folder() {
  if [[ -n "$SELECTED_FOLDER" && ! -d "${FIT4MED_LOCAL_PATH}${SELECTED_FOLDER}" ]]; then
    echo "[ERROR] Local folder does not exist: ${FIT4MED_LOCAL_PATH}${SELECTED_FOLDER}"
    exit 1
  fi
}

fit4med_sync_from_robot() {
  fit4med_robot_sync_parse_args "$@"

  fit4med_robot_sync_check_local_path
  local src="${FIT4MED_REMOTE_USER}@${REMOTE_HOST}:$(fit4med_robot_sync_source_path "$FIT4MED_REMOTE_PATH")"
  local dest="$FIT4MED_LOCAL_PATH"

  fit4med_robot_sync_set_base_rsync_options
  # Pulling keeps local files that are newer than the robot's copy.
  RSYNC_OPTS+=(
    --update
    --exclude='__pycache__/'
    --exclude='*.pyc'
    --exclude='*.zip'
    --exclude='COLCON_IGNORE'
  )
  fit4med_robot_sync_apply_dry_run_options --itemize-changes
  fit4med_robot_sync_preflight
  fit4med_robot_sync_run_rsync true "$dest" "$src"
}

fit4med_sync_to_robot() {
  fit4med_robot_sync_parse_args "$@"

  fit4med_robot_sync_check_local_path
  local dest="${FIT4MED_REMOTE_USER}@${REMOTE_HOST}:${FIT4MED_REMOTE_PATH}"

  fit4med_robot_sync_check_local_selected_folder
  fit4med_robot_sync_set_base_rsync_options
  fit4med_robot_sync_to_robot_sources
  # The robot copy must become identical to the local one: no --update (the
  # offline robot clock makes timestamps unreliable, and edits made on the
  # robot would win), and --delete for files that exist only on the robot.
  # Excluded paths (.git, caches) are neither sent nor deleted. --checksum:
  # compare contents, not size+time (an edit can keep both unchanged).
  RSYNC_OPTS+=(
    --delete
    --checksum
    --exclude='__pycache__/'
    --exclude='*.pyc'
  )
  fit4med_robot_sync_apply_dry_run_options --itemize-changes
  fit4med_robot_sync_preflight

  if [[ "$DRY_RUN" != true ]]; then
    fit4med_robot_sync_maybe_backup
    fit4med_robot_sync_confirm_deletions "$dest" "${SYNC_SOURCES[@]}"
  fi
  fit4med_robot_sync_run_rsync false "$dest" "${SYNC_SOURCES[@]}"
}

# SYNC_SOURCES: what is mirrored onto the robot. By default each top-level
# entry of the local src separately (--relative), so --delete acts only inside
# them: robot folders the user does not have locally (other repositories of
# the workspace) are never touched.
fit4med_robot_sync_to_robot_sources() {
  SYNC_SOURCES=()
  if [[ -n "$SELECTED_FOLDER" ]]; then
    SYNC_SOURCES=("$(fit4med_robot_sync_source_path "$FIT4MED_LOCAL_PATH")")
    return
  fi
  if [[ "$DELETE_EXTRA_FOLDERS" == true ]]; then
    echo "[WARN] --delete-extra-folders: robot folders missing in the local src will be deleted"
    SYNC_SOURCES=("$FIT4MED_LOCAL_PATH")
    return
  fi

  local names=() name
  mapfile -t names < <(find "$FIT4MED_LOCAL_PATH" -mindepth 1 -maxdepth 1 \
                         ! -name '.git' ! -name '.github' -printf '%f\n' | sort)
  if [[ ${#names[@]} -eq 0 ]]; then
    echo "[ERROR] Nothing to sync: $FIT4MED_LOCAL_PATH is empty"
    exit 1
  fi
  for name in "${names[@]}"; do
    SYNC_SOURCES+=("${FIT4MED_LOCAL_PATH}./${name}")
  done
  RSYNC_OPTS+=(--relative)
  echo "[INFO] Synced from the local src: ${names[*]}"
  echo "[INFO] Other folders on the robot are left untouched (--delete-extra-folders to delete them)"
}

fit4med_robot_sync_maybe_backup() {
  local do_backup=false
  case "$BACKUP_MODE" in
    yes) do_backup=true ;;
    ask) fit4med_ask_yes_no "Create a backup of the robot sources (${FIT4MED_REMOTE_PATH}) before the sync?" y \
           && do_backup=true ;;
  esac
  if [[ "$do_backup" != true ]]; then
    echo "[INFO] No backup of the robot sources"
    return
  fi
  if ! fit4med_backup_run_on_robot "$REMOTE_HOST" fit4med_backup_create "$(date +%Y%m%d/%H%M)"; then
    echo "[ERROR] Backup failed: sync aborted"
    exit 1
  fi
}

# fit4med_robot_sync_confirm_deletions <dest> <src>...
fit4med_robot_sync_confirm_deletions() {
  local dest="$1" deletions=()
  shift
  mapfile -t deletions < <(rsync "${RSYNC_OPTS[@]}" --dry-run --itemize-changes "$@" "$dest" \
                           | sed -n 's/^\*deleting  *//p')
  if [[ ${#deletions[@]} -eq 0 ]]; then
    return
  fi
  echo "[WARN] ${#deletions[@]} file(s)/folder(s) exist only on the robot and will be DELETED:"
  printf '         %s\n' "${deletions[@]:0:50}"
  if [[ ${#deletions[@]} -gt 50 ]]; then
    echo "         ... and $(( ${#deletions[@]} - 50 )) more"
  fi
  if [[ "$ASSUME_YES" == true ]]; then
    return
  fi
  if ! fit4med_ask_yes_no "Delete them and continue with the sync?" n; then
    echo "[INFO] Sync aborted, nothing changed on the robot"
    exit 1
  fi
}
