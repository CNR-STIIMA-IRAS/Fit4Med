#!/usr/bin/env bash
# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0

##########################################
RED='\033[0;31m'
YELLOW='\033[1;33m'
GREEN='\033[0;32m'
MAGENTA='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color
##########################################
PIDS=""
HDR=""
##########################################

unspaw_controllers()
{
  controller_manager_name="$1"
  all_controllers=$(ros2 control list_controllers -c $controller_manager_name)
  while [ -z "$all_controllers" ]; do
    active_controllers=$(ros2 control list_controllers -c $controller_manager_name | grep "active" | awk '{print $1}')

    if [ -z "$active_controllers" ]; then
        echo "No active controllers found."
        return
    fi

    # Unspawn each active controller
    for controller in $active_controllers; do
        echo "Active Controller: $controller"
        clean_controller=$(echo "$controller" | sed 's/\x1B\[[0-9;]*m//g')
        ros2 control set_controller_state -c $controller_manager_name $clean_controller inactive
    done

    inactive_controllers=$(ros2 control list_controllers -c $controller_manager_name | grep "inactive" | awk '{print $1}')

    # Unspawn each active controller
    for controller in $inactive_controllers; do
        echo "Inactive controller: $controller"
        clean_controller=$(echo "$controller" | sed 's/\x1B\[[0-9;]*m//g')
        ros2 control unload_controller -c $controller_manager_name $clean_controller
    done
    all_controllers = $(ros2 control list_controllers -c $controller_manager_name)
  done
  echo "All active controllers have been unspawed."
}

remove_pids_from_list()
{
  read -a input_array <<< "$PIDS"
  output_array=()
  # Iterate over the array
  for PID in "${input_array[@]}"; do
    PID_INFO=$(ps -aux | grep -w "$PID" | grep -v grep)
    if [ -z "$PID_INFO" ]; then 
      : 
    else
      output_array+=("$PID")
    fi
  done
  string=$(printf "%s " "${output_array[@]}")  # Use printf to join elements with a space

  # Optional: Remove the trailing space
  PIDS=${string% }  # Remove the trailing space
}

report_zombies_with_match()
{
  local match="$1"
  local zombie_info=""

  zombie_info=$(ps -eo pid=,ppid=,stat=,cmd= | awk -v pat="$match" '
    BEGIN { IGNORECASE = 1 }
    $3 ~ /^Z/ && $0 ~ pat { print }
  ')

  if [ -n "$zombie_info" ]; then
    echo -e "${MAGENTA}!! Zombie processes still present for match \"$match\"${NC}"
    while IFS= read -r zombie_line; do
      [ -z "$zombie_line" ] && continue
      zombie_pid=$(echo "$zombie_line" | awk '{print $1}')
      zombie_ppid=$(echo "$zombie_line" | awk '{print $2}')
      parent_info=$(ps -fp "$zombie_ppid" | tail -n 1)
      echo -e "${MAGENTA}   zombie: ${zombie_line}${NC}"
      if [ -n "$parent_info" ]; then
        echo -e "${MAGENTA}   parent: ${parent_info}${NC}"
      else
        echo -e "${MAGENTA}   parent: not found, it should be reaped soon by init/systemd${NC}"
      fi
    done <<< "$zombie_info"
  fi
}

reap_zombie_parents_with_match()
{
  local match="$1"
  local zombie_ppids=""
  local parent_pid=""
  local parent_info=""

  zombie_ppids=$(ps -eo ppid=,stat=,cmd= | awk -v pat="$match" '
    BEGIN { IGNORECASE = 1 }
    $2 ~ /^Z/ && $0 ~ pat { print $1 }
  ' | sort -u)

  if [ -z "$zombie_ppids" ]; then
    return
  fi

  echo -e "${YELLOW}>> Trying to reap zombie parents for match ${GREEN}$match${NC}"
  while IFS= read -r parent_pid; do
    [ -z "$parent_pid" ] && continue

    if [ "$parent_pid" -le 1 ] || [ "$parent_pid" -eq "$$" ]; then
      echo -e "${MAGENTA}   skipping parent PID=$parent_pid for safety${NC}"
      continue
    fi

    parent_info=$(ps -fp "$parent_pid" | tail -n 1)
    if [ -z "$parent_info" ]; then
      echo -e "${MAGENTA}   parent PID=$parent_pid already exited${NC}"
      continue
    fi

    echo -e "${MAGENTA}   parent before reap attempt: ${parent_info}${NC}"
    kill -15 "$parent_pid" 2>/dev/null
    sleep 2

    if ps -p "$parent_pid" >/dev/null 2>&1; then
      echo -e "${MAGENTA}   parent PID=$parent_pid still alive, sending SIGKILL${NC}"
      kill -9 "$parent_pid" 2>/dev/null
      sleep 1
    fi
  done <<< "$zombie_ppids"
}

kill_process_with_match() {
  
  HDR="[${GREEN}$1${NC}]"
  # Get the PIDs of processes that match the first name
  echo -e "${YELLOW}>> Processing the termination of processes matching the string ${GREEN}$1${NC}"
  PIDS=$(pgrep -f -i "$1" | sort -u | tr '\n' ' ')
  timeout="$2"

  # Convert the PIDs string to an array
  # Check if any PIDs were found
  if [ -z "$PIDS" ]; then
    echo -e "$HDR No processes matching the specified patterns were found."
  else
    ps -fp $PIDS
    # Define signals to try
    SIGNALS=(2 15 9)  # INT, TERM, KILL
    
    remove_pids_from_list

    HDR="$HDR [PIDs:${CYAN}$PIDS${NC}]"
    # Try to kill the processes using the defined signals
    for SIGNAL in "${SIGNALS[@]}"; do
      if [ -z "$PIDS" ]; then
        break
      fi

      echo -e "$HDR Killing with signal -$SIGNAL (timeout $2)"

      start_time=$(date +%s)
      kill -$SIGNAL $PIDS 2>/dev/null

      duration=$(($2))
      bar_length=50
      kill_done=0
      while [ $kill_done -eq 0 ]; do

        current_time=$(date +%s)
        elapsed_time=$(( current_time - start_time ))
        percentage_time=$(echo "scale=2; $elapsed_time / $duration" | bc)

        remove_pids_from_list

        if [ -z "$PIDS" ]; then
          i=$bar_length
          progress=$((i * 100 / bar_length))
          filled_bar=$(printf "%-${i}s" "#" | tr ' ' '#')
          printf "\r$HDR [${filled_bar}] ${progress}%%\n"
          echo -e "$HDR Successfully killed processes with signal: $SIGNAL"
          kill_done=1
          break
        else
          percentage_bar=$(echo "scale=2; $percentage_time * $bar_length" | bc)
          i=$(echo "(.5 + $percentage_bar)/1" | bc)
          progress=$((i * 100 / bar_length))
          filled_bar=$(printf "%-${i}s" "#" | tr ' ' '#')
          remaining_character=$((bar_length - i))
          if [ "$remaining_character" -eq "0" ]; then
            printf "\r$HDR [${filled_bar}]${progress}%%"
          else 
            if [ "$remaining_character" -eq "$bar_length" ]; then
              empty_bar=$(printf "%-$((remaining_character))s" " " | tr ' ' ' ')
              printf "\r$HDR [${empty_bar}]${progress}%%"
            else 
              empty_bar=$(printf "%-$((remaining_character))s" " " | tr ' ' ' ')
              printf "\r$HDR [${filled_bar}${empty_bar}] ${progress}%%"
            fi
          fi
          
          sleep_time=$(echo "scale=2; $duration / $bar_length" | bc)
          sleep "$sleep_time"
        fi

        if [ "$progress" -eq 100 ]; then
          printf "\n"
          echo -e "$HDR Timeout of $duration seconds exceeded. We try with a stronger signal ..."
          break
        fi
      done
    done
    report_zombies_with_match "$1"
    reap_zombie_parents_with_match "$1"
    report_zombies_with_match "$1"
    echo -e "$HDR done"
  fi
  echo -e "${GREEN}<< Done $1${NC}"
}


#unspaw_controllers plc_controller_manager
kill_process_with_match run_sickPLC.launch.py 10
kill_process_with_match fit4med_ws 5
kill_process_with_match ros 5
kill_process_with_match jazzy 5

#sudo rm -fr /tmp/*
