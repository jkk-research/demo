#!/bin/bash

## ROS 2 rosbag recorder (MCAP format)

cd /mnt/bag
mkdir -p $(date -I)
cd $(date -I)
TEXT1="$1"
TIME1="$(date +"%Y-%m-%d_%H-%M")"
FILE1="$TEXT1$TIME1"
FILE2="x_rosparam_dump_$TEXT1$TIME1.txt"
PWD1="$(pwd)"
echo "Writing to file: $PWD1/$FILE1"
ros2 bag record /lexus3/gps/duro/current_pose /lexus3/gps/duro/imu /lexus3/gps/duro/mag /lexus3/gps/duro/status_string /lexus3/vehicle_speed /lexus3/vehicle_steering /tf /tf_static -s mcap -o $FILE1


