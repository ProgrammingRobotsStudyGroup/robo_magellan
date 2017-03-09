#!/bin/bash
#
# Check out and build code for Victoria
#
# Assumes log files are found in ~/.ros/log
# Assumes ROS Kinetic and MAVROS installed
#
source ~/catkin_ws/devel/setup.bash
#set -x #echo on

############################################################################
dir="$(ls -l ~/.ros/log/latest| awk '{ print $11 }')"
#echo $dir

subdir="$(basename "$dir")"
echo
echo Copying $subdir of $dir [latest]
echo
rsync -r ~/.ros/log/$subdir/* upload@40.83.250.109:/home/upload/logs/$subdir/