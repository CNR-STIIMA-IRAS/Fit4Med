#!/usr/bin/env bash
# Copyright 2026 CNR-STIIMA-IRAS
# SPDX-License-Identifier: Apache-2.0

now=$(date +"%Y%m%d_%H%M%S")
zip -r  log_${now}.zip /home/fit4med/.ros/log/*
mv log_${now}.zip /home/fit4med/.ros/fit4med_log/