#!/bin/bash

# ==========
# Simple script to start Victoria using pr_launch1.launch.
# Assuming in $HOME/ROS_ws/src/launch:
#   Launch with no additional diagnostics (use if headless):
#     ./pr_startup.sh 
#   Launch with additional diagnostic nodes specified in launch file:
#     ./pr_startup.sh diagnostics
#
# Output is redirected to startup.log
#
# Launch file can be expanded to include more nodes over time.
#
# ** IMPORTANT: this script will leave at least a 'roslaunch' process running, even
#    if you exit the terminal. Use 'ps -A | grep ros' to find PID for any ros processes
#    and kill them ('kill 2532' or whatever the PID is).
# **
# ===========

cd $HOME/ROS_ws/src/launch

# not sure the following are completely correct
source /opt/ros/kinetic/setup.bash
source ~/ROS_ws/devel/setup.bash
export ROS_HOSTNAME=RoboMagellan.local

echo "========== Starting startup script ==========" | tee startup.log
date > startup.log 
printenv | grep ROS >> startup.log

if [ "$1" == diagnostics ]; then
  echo "Launching with diagnostics"
  export LAUNCH_FN="pr_launch1.launch launch_diagnostics:=true"
else 
  echo "Launching without diagnostics"
  export LAUNCH_FN="pr_launch1.launch"
fi

# main ROS launch using launch file
# runs in background and nohup --> not killed by terminal exit
# stdbuf used for cleaner redirected output in startup.log
stdbuf -oL nohup roslaunch $LAUNCH_FN >> startup.log 2>&1 &
#( roslaunch pr_launch1.launch >> startup.log 2>&1 & )

# time not optimized
sleep 20

# set the Pixhawk SYSID_MYGCS param so mavros messaging works
# doesn't work to include this in the launch file (roscore not running soon enough)
echo "========== Setting Pixhawk params ==========" | tee -a startup.log
stdbuf -oL nohup rosrun mavros mavparam set SYSID_MYGCS 1 >> startup.log 2>&1 &
#( rosrun mavros mavparam set SYSID_MYGCS 1 >> startup.log 2>&1 & )

echo "========== End startup script ==========" | tee -a startup.log
