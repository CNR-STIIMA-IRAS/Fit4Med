#!/usr/bin/env bash
# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0
#
# Backups of the robot workspace sources:
#   ${FIT4MED_ROBOT_SRC}  ->  ${FIT4MED_BKP_ROOT}/YYYYMMDD/HHMM/
#
# The fit4med_backup_* functions act on the machine they run on. On the user
# PC they are shipped to the robot over ssh (fit4med_backup_run_on_robot), so
# the robot needs neither these scripts nor internet access.

FIT4MED_ROBOT_HOST="${FIT4MED_ROBOT_HOST:-192.168.1.1}"
FIT4MED_ROBOT_USER="${FIT4MED_ROBOT_USER:-fit4med}"
FIT4MED_ROBOT_SRC="${FIT4MED_ROBOT_SRC:-/home/fit4med/fit4med_ws/src}"
FIT4MED_BKP_ROOT="${FIT4MED_BKP_ROOT:-/home/fit4med/bkp}"

# One ssh connection reused by every ssh/rsync call of a script run, so a
# password (when no key is installed) is asked only once.
mkdir -p "$HOME/.ssh" 2>/dev/null && chmod 700 "$HOME/.ssh" 2>/dev/null
FIT4MED_SSH_OPTS=(
  -o ControlMaster=auto
  -o "ControlPath=$HOME/.ssh/fit4med-%r@%h:%p"
  -o ControlPersist=120
)

# ---------------------------------------------------------------------------
# Functions executed on the robot (locally or through ssh)
# ---------------------------------------------------------------------------

# fit4med_ask_yes_no <question> <default y|n>: 0 = yes. EOF/no tty = default.
fit4med_ask_yes_no() {
  local question="$1" default="$2" hint answer
  [[ "$default" == y ]] && hint="[Y/n]" || hint="[y/N]"
  if ! read -r -p "$question $hint " answer; then
    echo
    answer=""
  fi
  answer="${answer:-$default}"
  [[ "$answer" =~ ^[Yy]([Ee][Ss])?$ ]]
}

# fit4med_backup_create [YYYYMMDD/HHMM]: copy the sources into a new backup.
# The timestamp is passed from the user PC when run remotely: the robot is
# offline and its clock may be off.
fit4med_backup_create() {
  local stamp="${1:-$(date +%Y%m%d/%H%M)}"
  local base="${FIT4MED_BKP_ROOT}/${stamp}"
  local dest="$base" n=2

  if [[ ! -d "$FIT4MED_ROBOT_SRC" ]]; then
    echo "[ERROR] Source folder not found: $FIT4MED_ROBOT_SRC"
    return 1
  fi
  while [[ -e "$dest" ]]; do  # two backups in the same minute
    dest="${base}_${n}"
    n=$((n + 1))
  done

  echo "[INFO] Backing up $FIT4MED_ROBOT_SRC -> $dest"
  mkdir -p "$dest" || return 1
  if ! cp -a "${FIT4MED_ROBOT_SRC}/." "$dest/"; then
    echo "[ERROR] Backup failed, removing the incomplete copy $dest"
    rm -rf -- "$dest"
    return 1
  fi
  echo "[OK] Backup created: $dest ($(du -sh "$dest" | cut -f1))"
}

# fit4med_backup_entries: existing backups, newest first, as YYYYMMDD/HHMM[_n].
fit4med_backup_entries() {
  [[ -d "$FIT4MED_BKP_ROOT" ]] || return 0
  (cd "$FIT4MED_BKP_ROOT" && find . -mindepth 2 -maxdepth 2 -type d \
     -regextype posix-extended -regex '\./[0-9]{8}/[0-9]{4}(_[0-9]+)?' \
     | sed 's|^\./||' | sort -r)
}

# fit4med_backup_list: numbered list of the backups (1 = newest).
fit4med_backup_list() {
  local entries=() i entry
  mapfile -t entries < <(fit4med_backup_entries)
  if [[ ${#entries[@]} -eq 0 ]]; then
    echo "[INFO] No backups in $FIT4MED_BKP_ROOT"
    return 1
  fi
  echo "Backups in $FIT4MED_BKP_ROOT (newest first):"
  for i in "${!entries[@]}"; do
    entry="${entries[$i]}"
    printf '  %3d)  %s-%s-%s %s:%s%s   %s\n' "$((i + 1))" \
      "${entry:0:4}" "${entry:4:2}" "${entry:6:2}" "${entry:9:2}" "${entry:11:2}" \
      "${entry:13}" "$(du -sh "${FIT4MED_BKP_ROOT}/${entry}" | cut -f1)"
  done
}

# fit4med_backup_restore_interactive [YYYYMMDD/HHMM]: choose a backup and make
# the sources identical to it. The optional stamp names the safety backup.
fit4med_backup_restore_interactive() {
  local stamp="${1:-}" entries=() choice selected
  mapfile -t entries < <(fit4med_backup_entries)
  fit4med_backup_list || return 1

  echo
  if ! read -r -p "Backup to restore [1-${#entries[@]}, q to quit]: " choice; then
    echo
    return 1
  fi
  if [[ "$choice" == q || -z "$choice" ]]; then
    echo "[INFO] Nothing restored"
    return 0
  fi
  if ! [[ "$choice" =~ ^[0-9]+$ ]] || (( choice < 1 || choice > ${#entries[@]} )); then
    echo "[ERROR] Invalid choice: $choice"
    return 1
  fi
  selected="${FIT4MED_BKP_ROOT}/${entries[$((choice - 1))]}"

  echo
  echo "[WARN] $FIT4MED_ROBOT_SRC will become identical to $selected:"
  echo "       files not in that backup are deleted, modified files are overwritten."
  if ! fit4med_ask_yes_no "Continue?" n; then
    echo "[INFO] Nothing restored"
    return 0
  fi
  if fit4med_ask_yes_no "Back up the current sources first?" y; then
    fit4med_backup_create ${stamp:+"$stamp"} || return 1
  fi

  echo "[INFO] Restoring $selected -> $FIT4MED_ROBOT_SRC"
  mkdir -p "$FIT4MED_ROBOT_SRC" || return 1
  if command -v rsync >/dev/null 2>&1; then
    rsync -a --delete "${selected}/" "${FIT4MED_ROBOT_SRC}/" || return 1
  else
    find "$FIT4MED_ROBOT_SRC" -mindepth 1 -maxdepth 1 -exec rm -rf -- {} + || return 1
    cp -a "${selected}/." "${FIT4MED_ROBOT_SRC}/" || return 1
  fi
  echo "[OK] Restored $selected"
  echo "[INFO] Rebuild the workspace (colcon build) before the next bring-up."
}

# fit4med_sync_apply_archive <archive.tar> <folder|.> <dry_run> <assume_yes>
# Make the sources (or one folder of them) identical to a tar archive. Used by
# ps_scripts/fmrr_to_robot_sync.ps1: Windows has no rsync, so the PC uploads a
# tar and the mirroring (with deletions) is done here. Archive is removed.
fit4med_sync_apply_archive() {
  local archive="$1" staging rc
  staging="$(mktemp -d "${TMPDIR:-/tmp}/fit4med_sync.XXXXXX")" || return 1
  _fit4med_sync_apply_staged "$staging" "$@"
  rc=$?
  rm -rf -- "$staging" "$archive"
  return $rc
}

_fit4med_sync_apply_staged() {
  local staging="$1" archive="$2" folder="$3" dry_run="$4" assume_yes="$5"
  local src_dir="$FIT4MED_ROBOT_SRC" stage_dir="$staging" crlf=() deletions=()
  [[ "$folder" == . ]] && folder=""  # "." = the whole workspace

  if ! tar -x -f "$archive" -C "$staging"; then
    echo "[ERROR] Cannot extract $archive"
    return 1
  fi
  if [[ -n "$folder" ]]; then
    src_dir="${FIT4MED_ROBOT_SRC}/${folder}"
    stage_dir="${staging}/${folder}"
    if [[ ! -d "$stage_dir" ]]; then
      echo "[ERROR] Folder not in the archive: $folder"
      return 1
    fi
  fi

  # A Windows checkout with core.autocrlf converts line endings: such shell
  # scripts fail on the robot with "$'\r': command not found".
  mapfile -t crlf < <(cd "$staging" && grep -rlI --include='*.sh' $'\r$' . 2>/dev/null)
  if [[ ${#crlf[@]} -gt 0 ]]; then
    echo "[ERROR] ${#crlf[@]} shell script(s) have Windows line endings (CRLF) and would not run:"
    printf '         %s\n' "${crlf[@]:0:20}"
    echo "        Fix the checkout on the PC (see bash_scripts/README.md), nothing was changed."
    return 1
  fi

  # Files coming from Windows have no executable bit: give it back to scripts.
  find "$stage_dir" -type f -exec sh -c \
    'for f; do [ "$(head -c 2 "$f")" = "#!" ] && chmod +x "$f"; done' _ {} +

  # No -p: files already on the robot keep their permissions. --checksum:
  # timestamps from a fresh Windows checkout say nothing about the content.
  local opts=(-rlt --checksum --delete --itemize-changes
              --exclude='.git/' --exclude='.github/' --exclude='__pycache__/' --exclude='*.pyc')
  echo "[INFO] Source (from PC): ${folder:-whole workspace}   Destination: $src_dir"

  if [[ "$dry_run" == true ]]; then
    echo "[INFO] DRY-RUN: nothing is changed on the robot"
    rsync "${opts[@]}" --dry-run "${stage_dir}/" "${src_dir}/" | grep -v '^\.[fd]\.\.t\.'
    return 0
  fi

  mapfile -t deletions < <(rsync "${opts[@]}" --dry-run "${stage_dir}/" "${src_dir}/" \
                           | sed -n 's/^\*deleting  *//p')
  if [[ ${#deletions[@]} -gt 0 ]]; then
    echo "[WARN] ${#deletions[@]} file(s)/folder(s) exist only on the robot and will be DELETED:"
    printf '         %s\n' "${deletions[@]:0:50}"
    if [[ ${#deletions[@]} -gt 50 ]]; then
      echo "         ... and $(( ${#deletions[@]} - 50 )) more"
    fi
    if [[ "$assume_yes" != true ]] && ! fit4med_ask_yes_no "Delete them and continue with the sync?" n; then
      echo "[INFO] Sync aborted, nothing changed on the robot"
      return 1
    fi
  fi

  mkdir -p "$src_dir" || return 1
  local output
  if ! output="$(rsync "${opts[@]}" "${stage_dir}/" "${src_dir}/")"; then
    printf '%s\n' "$output"
    echo "[ERROR] rsync failed"
    return 1
  fi
  printf '%s\n' "$output" | grep -v '^\.[fd]\.\.t\.'
  echo "[OK] Robot sources updated: $src_dir"
  echo "[INFO] Rebuild the workspace (colcon build) before the next bring-up."
}

# ---------------------------------------------------------------------------
# Running them on the robot from the user PC
# ---------------------------------------------------------------------------

# fit4med_backup_run_on_robot <host> <function> [args...]
fit4med_backup_run_on_robot() {
  local host="$1"
  shift
  local code tty_opt=()
  code="$(
    printf 'FIT4MED_ROBOT_SRC=%q\nFIT4MED_BKP_ROOT=%q\n' "$FIT4MED_ROBOT_SRC" "$FIT4MED_BKP_ROOT"
    declare -f fit4med_ask_yes_no fit4med_backup_create fit4med_backup_entries \
               fit4med_backup_list fit4med_backup_restore_interactive
    printf '%q ' "$@"
  )"
  [[ -t 0 ]] && tty_opt=(-t)  # prompts need a terminal on the robot side
  ssh "${tty_opt[@]}" "${FIT4MED_SSH_OPTS[@]}" "${FIT4MED_ROBOT_USER}@${host}" \
      "bash -c $(printf '%q' "$code")"
}

# fit4med_backup_is_robot: true when running on the robot itself.
fit4med_backup_is_robot() {
  [[ -d "$FIT4MED_ROBOT_SRC" && "$(id -un)" == "$FIT4MED_ROBOT_USER" ]]
}

# fit4med_backup_main <script> <function> [--local | --host <ip>] : shared CLI
# of fit4med_backup.sh / fit4med_restore.sh. The function gets the timestamp.
fit4med_backup_main() {
  local script_name="$1" action="$2"
  shift 2
  local mode="" host="$FIT4MED_ROBOT_HOST"

  while [[ $# -gt 0 ]]; do
    case "$1" in
      --local) mode=local; shift ;;
      --host)
        [[ $# -ge 2 && -n "$2" ]] || { echo "[ERROR] --host requires an IP address"; return 1; }
        mode=remote; host="$2"; shift 2 ;;
      --host=*) mode=remote; host="${1#*=}"; shift ;;
      -h|--help)
        cat <<USAGE
Usage: ${script_name} [--local | --host <ip>]

  --local       act on this machine (default when run as ${FIT4MED_ROBOT_USER} on the robot)
  --host <ip>   act on the robot through ssh (default elsewhere, host ${FIT4MED_ROBOT_HOST})

Sources: ${FIT4MED_ROBOT_SRC}   Backups: ${FIT4MED_BKP_ROOT}/YYYYMMDD/HHMM/
USAGE
        return 0 ;;
      *) echo "[ERROR] Unknown option: $1 (see --help)"; return 1 ;;
    esac
  done

  if [[ -z "$mode" ]]; then
    fit4med_backup_is_robot && mode=local || mode=remote
  fi
  local stamp
  stamp="$(date +%Y%m%d/%H%M)"
  if [[ "$mode" == local ]]; then
    "$action" "$stamp"
  else
    echo "[INFO] Running on ${FIT4MED_ROBOT_USER}@${host}"
    fit4med_backup_run_on_robot "$host" "$action" "$stamp"
  fi
}
