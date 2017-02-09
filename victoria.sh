#!/bin/bash
#
# Check out and build code for Victoria
#
# Assumes catkin workspace ~/catkin_ws
# Assumes ROS Kinetic and MAVROS installed
#
source ~/catkin_ws/devel/setup.bash
#set -x #echo on

############################################################################
echo
echo Fetch/update robo_magellan

if rospack list | grep "robo_magellan" >/dev/null
then
echo Update robo_magellan
roscd robo_magellan
git pull

else
echo Fetch robo_magellan
cd ~/catkin_ws/src
git clone https://github.com/ProgrammingRobotsStudyGroup/robo_magellan.git
fi 


############################################################################
echo
echo Fetch/update cone_finder

if rospack list | grep "cone_finder" >/dev/null
then
echo Update cone_finder
roscd cone_finder
git pull

else
echo Fetch cone_finder
cd ~/catkin_ws/src
git clone https://github.com/ProgrammingRobotsStudyGroup/TrafficConeFinderCode.git
fi 


############################################################################
echo Build
cd ~/catkin_ws
catkin_make

