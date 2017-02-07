#!/bin/bash
#
# Check out and build code for Victoria
#
# Assumes catkin workspace ~/catkin_ws
# Assumes ROS Kinetic
# Any assumptions for cone finder?
#
set -x #echo on

# Fetch and mvoe to the right place the cone  finder
mkdir -p ~/tmp
cd ~/tmp
git clone https://github.com/ProgrammingRobotsStudyGroup/TrafficConeFinderCode.git
cd TrafficConeFinderCode 
./installROSNode.sh

# Fetch MAVROS
# if MavROS exists, skip install

# 
rospack list|grep

# Build
cd ~/catkin_ws
catkin_make



