#!/bin/bash

#=========
# script for Victoria ROS startup
#========

echo "=======================================" >> nohup.out
date >> nohup.out
env | fgrep ROS >> nohup.out
echo "nohup launching ROS"
nohup roslaunch robo_magellan exec.launch &
echo "****** main ROS PID to kill:"
echo $!
sleep 45
echo "rebooting Pixhawk"
rostopic pub /exec_command robo_magellan/to_exec "{cmd: 'REBOOT_MAV'}" -1s
sleep 45
rostopic echo /vicky/diagnostic &
echo "****** second ROS PID to kill:"
echo $!
echo "***"
echo "push red safety button now"
echo "solid red safety button --> push kill switch to start"
echo "The start toggle sw must be forward to drive"
echo "***"

