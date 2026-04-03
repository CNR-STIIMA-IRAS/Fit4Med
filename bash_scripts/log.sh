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

echo -e "${YELLOW}>> Store the last log files to the fit4med_log folder...${NC}"
now=$(date +"%Y%m%d_%H%M%S")
echo -e "${CYAN}..${NC} Creating the file log_${now}.zip ..."
zip -r  log_${now}.zip /home/fit4med/.ros/log/*
echo -e "${CYAN}...${NC} Check the log_${now}.zip file status ..."
ls -l log_${now}.zip
echo -e "${CYAN}..${NC} Moving the file under the /home/fit4med/.ros/fit4med_log/ folder ...${NC}"
mv log_${now}.zip /home/fit4med/.ros/fit4med_log/
echo -e "${CYAN}..${NC} Cleanup the .ros/log folder ...${NC}"
rm -fr /home/fit4med/.ros/log/*
echo -e "${YELLOW}<< Last log files have been stored to the fit4med_log folder...${NC}"