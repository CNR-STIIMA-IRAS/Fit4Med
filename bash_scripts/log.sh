#!/bin/bash
now=$(date +"%Y%m%d_%H%M%S")
zip -r  log_${now}.zip /home/fit4med/.ros/log/*
mv log_${now}.zip /home/fit4med/.ros/fit4med_log/