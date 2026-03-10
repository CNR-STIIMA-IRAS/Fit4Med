#!/usr/bin/env bash
# Copyright 2026 CNR-STIIMA-IRAS
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

kill_process_with_match() {
  
  HDR="[${GREEN}$1${NC}]"
  # Get the PIDs of processes that match the first name
  echo -e "${YELLOW}>> Processing the termination of processes matching the string ${GREEN}$1${NC}"
  PIDS=$(ps -aux | grep -i "$1" | grep -v grep | awk '{print $2}')
  PIDS=$(echo "$PIDS" | tr ' ' '\n' | sort -u | tr '\n' ' ')
  timeout="$2"

  # Convert the PIDs string to an array
  # Check if any PIDs were found
  if [ -z "$PIDS" ]; then
    echo -e "$HDR No processes matching the specified patterns were found."
  else
    ps -aux | grep -i "$1" | grep -v grep
    # Define signals to try
    SIGNALS=(2 9)  # INT, TERM

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
          echo -e "$HDR Timeout of $duration seconds exceeded. We try with an harder signal ..."
          break
        fi
      done
    done
    echo -e "$HDR done"
  fi
  echo -e "${GREEN}<< Done $1${NC}"
}


#unspaw_controllers plc_controller_manager
kill_process_with_match run_sickPLC.launch.py 10
kill_process_with_match fit4med_ws 5
kill_process_with_match jazzy 5

#sudo rm -fr /tmp/*