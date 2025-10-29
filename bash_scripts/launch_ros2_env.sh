#!/bin/bash
source ~/fit4med_ws/install/setup.bash

perform_homing=false
while :; do
    case $1 in
        -h|--perform-homing)
            perform_homing=true
            ;;
        -h|--help)
            echo "Usage: $0 [--perform-homing]"
            exit 0
            ;;
        --) # End of all options
            shift
            break
            ;;
        -?*)
            echo "Unknown option: $1" >&2
            exit 1
            ;;
        *)  # No more options
            break
    esac
    shift
done

ros2 launch tecnobody_workbench run_platform_control.launch.py perform_homing:=$perform_homing 
