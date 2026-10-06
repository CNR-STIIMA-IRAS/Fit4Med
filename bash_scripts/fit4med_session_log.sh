#!/usr/bin/env bash
# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0
#
# One log archive per bring-up (run_sickPLC.launch.py), numbered progressively:
#   ${FIT4MED_LOG_ROOT}/run_NNNN_YYYYMMDD-HHMMSS/      while the bring-up runs
#   ${FIT4MED_LOG_ROOT}/run_NNNN_YYYYMMDD-HHMMSS.zip   once it is closed
# run_sickPLC.launch.py opens the session ("start") and closes it when it
# exits ("finalize"), so the logs are archived however the launch was started
# (systemd unit, fmrr_bringup.ps1, by hand). The archive contains:
#   session.txt                   start/end time, unit, GUI IP, shutdown reason
#   sickPLC/                      ROS logs of run_sickPLC.launch.py and its nodes
#   starts/NNN_<label>_HHMMSS/    one folder per start of launch_ros2_env.sh,
#                                 launch_ros2_env_z_recovery.sh and
#                                 launch_ros2_bridge.sh (e.g. after each
#                                 e-stop): console.log + ROS logs of that start
#   journal/                      bring-up unit, ethercat.service and their
#                                 merged timeline, limited to this session
#   ethercat_start.txt, ethercat_end.txt   EtherCAT master and slaves
#   ros_home_log/                 what was left in ~/.ros/log (what log.sh archived)
# A session whose launch died without closing it (SIGKILL, power loss) is
# archived at the next start, with the journal up to its last log write.
#
# Usage:
#   fit4med_session_log.sh start <launch_pid> [gui_ip]   prints the session folder
#   fit4med_session_log.sh finalize <session_dir> [launch_log_dir] [reason]
#   fit4med_session_log.sh recover                       archive sessions left open
#   fit4med_session_log.sh snapshot    zip each session still running (up to now) in /tmp
#   fit4med_session_log.sh list [N]    the last N archives (all: 0), oldest first
# snapshot and list print "path<TAB>start time<TAB>end reason" per archive
# (used by ps_scripts/fmrr_retrieve_logs.ps1).
# The launch_ros2_*.sh scripts source this file for fit4med_log_begin_run.

FIT4MED_LOG_ROOT="${FIT4MED_LOG_ROOT:-$HOME/.ros/fit4med_log}"
_FIT4MED_LOG_LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ---------------------------------------------------------------------------
# Used by the launch_ros2_*.sh scripts
# ---------------------------------------------------------------------------

# fit4med_log_begin_run <label>: inside a session (FIT4MED_SESSION_DIR, set by
# run_sickPLC.launch.py and inherited through plc_manager), give this start its
# own numbered folder: the ROS logs go there (ROS_LOG_DIR) and the output of
# the calling script is copied to console.log. Outside a session: nothing.
fit4med_log_begin_run() {
  local session="${FIT4MED_SESSION_DIR:-}" starts n dir
  [[ -n "$session" && -d "$session" ]] || return 0
  starts="$session/starts"
  mkdir -p "$starts" || return 0
  n="$(_fit4med_log_next_number "$starts/.counter")" || return 0
  dir="$(printf '%s/%03d_%s_%s' "$starts" "$n" "$1" "$(date +%H%M%S)")"
  mkdir -p "$dir" || return 0
  printf '%s\n' "$$" > "$dir/.running"
  {
    echo "LABEL=$1"
    echo "SCRIPT=$0"
    echo "START_TIME=$(date --iso-8601=ns)"
  } > "$dir/run.txt"
  export ROS_LOG_DIR="$dir"
  FIT4MED_RUN_DIR="$dir"
  # tee -i: a SIGINT to the whole group (Ctrl-C, systemd stop) must not cut the
  # copy before the launch has finished logging its shutdown.
  exec > >(tee -i -a "$dir/console.log") 2>&1
  # With a handler (not with the default action) bash waits for the running
  # ros2 launch to finish its shutdown, then exits through the EXIT trap.
  trap 'exit 130' INT
  trap 'exit 143' TERM
  trap '_fit4med_log_end_run $?' EXIT
  echo "[fit4med] Logs of this start: $dir"
}

_fit4med_log_end_run() {
  {
    echo "END_TIME=$(date --iso-8601=ns)"
    echo "EXIT_CODE=$1"
  } >> "$FIT4MED_RUN_DIR/run.txt"
  rm -f "$FIT4MED_RUN_DIR/.running"
}

# ---------------------------------------------------------------------------
# Sessions
# ---------------------------------------------------------------------------

# fit4med_log_start <launch_pid> [gui_ip]: create the next session folder and
# print it. The journal of the session starts when <launch_pid> was started:
# under systemd that is the start of the unit (the service script exec's the
# launch, so the PID is the same).
fit4med_log_start() {
  local pid="${1:?launch pid}" gui_ip="${2:-}" floor n dir starttime since unit
  mkdir -p "$FIT4MED_LOG_ROOT" || return 1
  floor="$(find "$FIT4MED_LOG_ROOT" -mindepth 1 -maxdepth 1 -name 'run_*' -printf '%f\n' \
             | sed -n 's/^run_0*\([0-9][0-9]*\)_.*/\1/p' | sort -n | tail -n 1)"
  n="$(_fit4med_log_next_number "$FIT4MED_LOG_ROOT/.session_counter" "${floor:-0}")" || return 1
  dir="$(printf '%s/run_%04d_%s' "$FIT4MED_LOG_ROOT" "$n" "$(date +%Y%m%d-%H%M%S)")"
  mkdir -p "$dir/sickPLC" "$dir/starts" || return 1

  starttime="$(_fit4med_proc_starttime "$pid")"
  since="$(_fit4med_proc_start_epoch "$starttime")" || since="$(date +%s)"
  unit="$(_fit4med_user_unit_of_pid "$pid")" || unit=""
  {
    echo "SESSION=$n"
    echo "HOST=$(hostname)"
    echo "BOOT_ID=$(cat /proc/sys/kernel/random/boot_id 2>/dev/null)"
    echo "START_TIME=$(date -d "@$since" --iso-8601=seconds)"
    echo "SINCE_EPOCH=$since"
    echo "UNIT=$unit"
    echo "GUI_IP=$gui_ip"
    echo "OWNER_PID=$pid"
    echo "OWNER_STARTTIME=$starttime"
    echo "LAUNCH_CMD=$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null)"
  } > "$dir/session.txt"

  # In the background, with their output away from the caller's pipe, so the
  # bring-up is not delayed.
  fit4med_log_ethercat_snapshot > "$dir/ethercat_start.txt" 2>&1 < /dev/null &
  fit4med_log_recover >> "$FIT4MED_LOG_ROOT/recover.log" 2>&1 < /dev/null &
  echo "$dir"
}

# fit4med_log_finalize <session_dir> [launch_log_dir] [reason] [orphan]:
# add the journals, zip the folder and remove it. "orphan": the session was
# left open by a launch that no longer exists (see fit4med_log_recover).
fit4med_log_finalize() {
  local dir="${1%/}" launch_log_dir="${2:-}" reason="${3:-}" orphan="${4:-}"
  local root name since until unit newest ros_log lock
  if [[ ! -f "$dir/session.txt" ]]; then
    echo "[ERROR] Not a session folder: $dir" >&2
    return 1
  fi
  root="$(dirname "$dir")"
  name="$(basename "$dir")"

  # The owner and the recovery of a later start must not archive it together.
  lock="$root/.${name}.lock"
  exec 8>> "$lock"
  if ! flock -w 30 8; then
    echo "[ERROR] $dir is being archived by another process" >&2
    return 1
  fi
  if [[ ! -d "$dir" ]]; then  # archived while waiting for the lock
    exec 8>&-
    return 0
  fi
  [[ -n "$orphan" ]] && echo "[fit4med] $(date --iso-8601=seconds) archiving session left open: $dir"

  since="$(_fit4med_kv_get "$dir/session.txt" SINCE_EPOCH)"
  unit="$(_fit4med_kv_get "$dir/session.txt" UNIT)"
  if [[ -n "$orphan" ]]; then
    newest="$(find "$dir" -type f -printf '%T@\n' | sort -n | tail -n 1)"
    until=$(( ${newest%.*} + 60 ))
  else
    _fit4med_log_wait_runs "$dir" "${FIT4MED_LOG_WAIT_RUNS:-10}"
    until=$(( $(date +%s) + 1 ))
  fi
  [[ "$since" =~ ^[0-9]+$ ]] || since=$(( until - 86400 ))
  {
    echo "END_TIME=$(date --iso-8601=seconds)"
    echo "END_REASON=$reason"
    echo "JOURNAL_UNTIL_EPOCH=$until"
  } >> "$dir/session.txt"

  if [[ -z "$orphan" ]]; then
    fit4med_log_ethercat_snapshot > "$dir/ethercat_end.txt" 2>&1
    ros_log="${ROS_HOME:-$HOME/.ros}/log"
    # launch.log of run_sickPLC.launch.py: created before the launch file is
    # read, so not in sickPLC/ yet.
    if [[ -n "$launch_log_dir" && -d "$launch_log_dir" && "$launch_log_dir" != "$dir"/* ]]; then
      if [[ "$launch_log_dir" == "$ros_log"/* ]]; then
        mv -f "$launch_log_dir" "$dir/sickPLC/" 2>/dev/null
      else
        cp -a "$launch_log_dir" "$dir/sickPLC/" 2>/dev/null
      fi
    fi
    # What log.sh does at the next bring-up: archive ~/.ros/log and empty it
    # (what ran outside a session, e.g. ros2 commands typed by hand).
    if [[ -d "$ros_log" && "$ros_log" != "$FIT4MED_LOG_ROOT"* ]] \
         && compgen -G "$ros_log/*" > /dev/null; then
      mkdir -p "$dir/ros_home_log"
      mv -f "$ros_log"/* "$dir/ros_home_log/" 2>/dev/null
    fi
  fi

  fit4med_log_journals "$dir" "$unit" "$since" "$until" "$orphan"

  rm -f "$root/.${name}.tmp.zip"
  if (cd "$root" && zip -q -r -y ".${name}.tmp.zip" "$name" -x '*/.running' '*/.counter*') \
       && mv -f "$root/.${name}.tmp.zip" "$root/${name}.zip"; then
    rm -rf -- "$dir"
    echo "[fit4med] Session logs archived: $root/${name}.zip ($(du -h "$root/${name}.zip" | cut -f1))"
  else
    rm -f "$root/.${name}.tmp.zip"
    echo "[ERROR] Cannot zip $dir: the folder is kept" >&2
  fi
  exec 8>&-
  rm -f "$lock"
}

# fit4med_log_recover: archive the sessions whose launch is gone.
fit4med_log_recover() {
  local dir
  for dir in "$FIT4MED_LOG_ROOT"/run_*/; do
    dir="${dir%/}"
    [[ -f "$dir/session.txt" ]] || continue
    _fit4med_log_owner_alive "$dir" && continue
    fit4med_log_finalize "$dir" "" "launch killed (SIGKILL, crash, power loss): archived at a later start" orphan
  done
}

# fit4med_log_snapshot: for each session still running, a zip with what it
# has so far and the journals up to now, in a temporary folder (to copy and
# delete); the session is left untouched. Sessions left open by a dead launch
# are archived first, so they show up in fit4med_log_list.
fit4med_log_snapshot() {
  local dir name tmp unit since now ros_log
  fit4med_log_recover > /dev/null 2>&1
  for dir in "$FIT4MED_LOG_ROOT"/run_*/; do
    dir="${dir%/}"
    [[ -f "$dir/session.txt" ]] && _fit4med_log_owner_alive "$dir" || continue
    name="$(basename "$dir")_partial"
    tmp="$(mktemp -d "${TMPDIR:-/tmp}/fit4med_snapshot.XXXXXX")" || continue
    cp -a "$dir" "$tmp/$name" 2> /dev/null
    rm -rf "$tmp/$name/journal"
    # launch.log of run_sickPLC.launch.py and the rest of ~/.ros/log, copied
    ros_log="${ROS_HOME:-$HOME/.ros}/log"
    if [[ -d "$ros_log" ]] && compgen -G "$ros_log/*" > /dev/null; then
      mkdir -p "$tmp/$name/ros_home_log"
      cp -a "$ros_log"/* "$tmp/$name/ros_home_log/" 2> /dev/null
    fi
    unit="$(_fit4med_kv_get "$dir/session.txt" UNIT)"
    since="$(_fit4med_kv_get "$dir/session.txt" SINCE_EPOCH)"
    now=$(( $(date +%s) + 1 ))
    {
      echo "END_TIME=$(date --iso-8601=seconds)"
      echo "END_REASON=still running (partial copy)"
      echo "JOURNAL_UNTIL_EPOCH=$now"
    } >> "$tmp/$name/session.txt"
    fit4med_log_ethercat_snapshot > "$tmp/$name/ethercat_end.txt" 2>&1
    fit4med_log_journals "$tmp/$name" "$unit" "${since:-$((now - 86400))}" "$now"
    if (cd "$tmp" && zip -q -r -y "$name.zip" "$name" -x '*/.running' '*/.counter*'); then
      rm -rf -- "${tmp:?}/$name"
      _fit4med_log_describe "$tmp/$name.zip"
    else
      rm -rf -- "$tmp"
    fi
  done
}

# fit4med_log_list [N]: the last N run archives (all with 0), oldest first.
fit4med_log_list() {
  local n="${1:-0}" f
  [[ "$n" =~ ^[0-9]+$ ]] || n=0
  find "$FIT4MED_LOG_ROOT" -mindepth 1 -maxdepth 1 -name 'run_*.zip' -printf '%f\n' 2> /dev/null \
    | sort -t _ -k 2,2n \
    | if (( n > 0 )); then tail -n "$n"; else cat; fi \
    | while IFS= read -r f; do
        _fit4med_log_describe "$FIT4MED_LOG_ROOT/$f"
      done
}

# _fit4med_log_describe <zip>: "path<TAB>start time<TAB>end reason"
_fit4med_log_describe() {
  local info
  info="$(unzip -p "$1" '*/session.txt' 2> /dev/null)"
  printf '%s\t%s\t%s\n' "$1" \
    "$(sed -n 's/^START_TIME=//p' <<< "$info" | tail -n 1)" \
    "$(sed -n 's/^END_REASON=//p' <<< "$info" | tail -n 1)"
}

# fit4med_log_journals <dir> <unit> <since_epoch> <until_epoch> [orphan]
fit4med_log_journals() {
  local dir="$1" unit="$2" range=(--since "@$3" --until "@$4") orphan="${5:-}"
  mkdir -p "$dir/journal"
  {
    if [[ -n "$unit" ]]; then
      journalctl --user -u "$unit" "${range[@]}" -o short-precise --no-pager
    else
      echo "# Not started by a systemd user unit: the console output is in the ROS logs (sickPLC/, ros_home_log/)."
    fi
  } > "$dir/journal/fit4med_bringup.log" 2>&1

  # ethercat.service is a system unit: reading it needs the systemd-journal
  # group (see README, one-time setup). The IgH master logs to the kernel log.
  {
    if [[ -z "$orphan" ]]; then
      echo "===== systemctl status ethercat.service ====="
      systemctl status ethercat.service --no-pager -l
      echo
    fi
    echo "===== journalctl -u ethercat.service (this session) ====="
    journalctl -u ethercat.service "${range[@]}" -o short-precise --no-pager
    echo; echo "===== kernel log, EtherCAT lines (this session) ====="
    journalctl -k "${range[@]}" -o short-precise --no-pager | grep -i ethercat
  } > "$dir/journal/ethercat_service.log" 2>&1

  fit4med_merged_journal "$unit" "${range[@]}" > "$dir/journal/fit4med_ethercat_timeline.log" 2>&1
}

fit4med_log_ethercat_snapshot() {
  echo "===== $(date --iso-8601=ns) ====="
  if ! command -v ethercat > /dev/null 2>&1; then
    echo "ethercat command not available"
    return 0
  fi
  echo "===== ethercat master ====="
  timeout 5 ethercat master
  echo; echo "===== ethercat slaves -v ====="
  timeout 10 ethercat slaves -v
}

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

# _fit4med_log_next_number <counter_file> [floor]: increment and print the
# counter (at least floor + 1). Safe when two scripts start together.
_fit4med_log_next_number() {
  local file="$1" floor="${2:-0}"
  (
    flock -w 5 9 || exit 1
    n="$(cat "$file" 2> /dev/null)"
    [[ "$n" =~ ^[0-9]+$ ]] || n=0
    n=$((10#$n))
    (( n < 10#$floor )) && n=$((10#$floor))
    n=$((n + 1))
    echo "$n" > "$file"
    echo "$n"
  ) 9>> "$file.lock"
}

# _fit4med_log_wait_runs <session_dir> <max_seconds>: wait for the starts
# still shutting down (their script is alive), so their logs are complete.
_fit4med_log_wait_runs() {
  local deadline=$((SECONDS + $2)) f pid busy
  while :; do
    busy=0
    for f in "$1"/starts/*/.running; do
      [[ -f "$f" ]] || continue
      pid="$(cat "$f" 2> /dev/null)"
      [[ -n "$pid" ]] && kill -0 "$pid" 2> /dev/null && busy=1
    done
    (( busy == 0 || SECONDS >= deadline )) && return 0
    sleep 0.5
  done
}

_fit4med_log_owner_alive() {
  local pid starttime boot
  pid="$(_fit4med_kv_get "$1/session.txt" OWNER_PID)"
  starttime="$(_fit4med_kv_get "$1/session.txt" OWNER_STARTTIME)"
  boot="$(_fit4med_kv_get "$1/session.txt" BOOT_ID)"
  [[ -n "$pid" && "$boot" == "$(cat /proc/sys/kernel/random/boot_id 2> /dev/null)" ]] || return 1
  [[ "$(_fit4med_proc_starttime "$pid")" == "$starttime" ]]  # same PID, not a reused one
}

# _fit4med_kv_get <file> <key>: last value of KEY=value.
_fit4med_kv_get() {
  sed -n "s/^$2=//p" "$1" 2> /dev/null | tail -n 1
}

# _fit4med_proc_starttime <pid>: start time in clock ticks since boot (field 22).
_fit4med_proc_starttime() {
  local stat fields
  stat="$(cat "/proc/$1/stat" 2> /dev/null)" || return 1
  stat="${stat##*) }"  # the command name may contain spaces
  read -ra fields <<< "$stat"
  echo "${fields[19]}"
}

_fit4med_proc_start_epoch() {
  local btime
  [[ "$1" =~ ^[0-9]+$ ]] || return 1
  btime="$(awk '/^btime/ { print $2 }' /proc/stat)"
  echo $(( btime + $1 / $(getconf CLK_TCK) ))
}

# _fit4med_user_unit_of_pid <pid>: the systemd user service running <pid>.
_fit4med_user_unit_of_pid() {
  local cgroup unit
  cgroup="$(grep -m 1 '^0::' "/proc/$1/cgroup" 2> /dev/null)" || return 1
  unit="${cgroup##*/}"
  [[ "$cgroup" == */user@*.service/* && "$unit" == *.service ]] || return 1
  echo "$unit"
}

# ---------------------------------------------------------------------------

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  # shellcheck source=fit4med_backup_common.sh
  source "$_FIT4MED_LOG_LIB_DIR/fit4med_backup_common.sh"  # fit4med_merged_journal
  case "${1:-}" in
    start)    shift; fit4med_log_start "$@" ;;
    finalize) shift; fit4med_log_finalize "$@" ;;
    recover)  fit4med_log_recover ;;
    snapshot) fit4med_log_snapshot ;;
    list)     shift; fit4med_log_list "$@" ;;
    *)
      sed -n 's/^#   fit4med_session_log.sh/  fit4med_session_log.sh/p' "$0"
      exit 1 ;;
  esac
fi
