#!/usr/bin/env python
#
# Copyright 2017 Robot Garden, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

"""Follow_waypoint(s) until 1) transition occurs 2) exec interrupts"""
#
# Follow_waypoint(s) until 1) transition occurs 2) exec interrupts
#

# ROS
import rospy
#from std_msgs.msg import String
#from ._WaypointPull import *
#
from uav_state import MODE as MAVMODE

import exec_comm
from exec_comm import MSG_TO_STATE
from exec_comm import MSG_TO_EXEC
from state_and_transition import STATE
from state_and_transition import TRANSITION

# Globals
this_node = None

#TODO: Rework code to use to_exec and to_state message formats
#TODO: state_start. state_reset, state_pause




def iscurrent():
    """Prints list of WPs"""
    this_node.uav_control.pull_waypoints()
    print this_node.uav_control.waypoint_list
#    iscur = -1
#    for seq, waypoint in enumerate( this_node.uav_control.waypoint_list):
#        if waypoint.is_current:
#            iscur = seq
#            print (''+str(seq))
#            break
#    if iscur == -1:
#        print('no current')


def mission_item_reached_cb(data):
    rospy.loginfo("Mission Item reached"+str(data.seq))
    #notify the loop

#
#
#
def state_start():
    """Start the state"""
    rospy.loginfo('state_start %s', this_node.state_name)

    # TODO Setting mode to HOLD is precautionary.
    # Set UAV mode to hold while we get this state started
    this_node.uav_state.set_mode(MAVMODE.HOLD.name)
    this_node.uav_state.set_arm(False)
    ## TEST
    #this_node.uav_control.pull_waypoints()
    #wpl = this_node.uav_control.waypoint_list
    #print wpl
    #wpl.pop()
    #print wpl
    #this_node.uav_control.clear_waypoints()
    #this_node.uav_control.push_waypoints(wpl)
#    print this_node.uav_control.waypoint_list
    # TODO Test push
    # Pick wp to start index
    #iscurrent()
    this_node.uav_control.set_current_waypoint(0)

    this_node.uav_state.set_mode(MAVMODE.AUTO.name)
    this_node.uav_state.set_arm(True)

    rate = rospy.Rate(2) # 2 hz
    #iscurrent()

    # WP Driving loop
    segment_duration_sec = rospy.get_param("/SEGMENT_DURATION_SEC")
    timeout = rospy.Time.now() + rospy.Duration(segment_duration_sec)
    old_timeout_secs = 0

    # Add callback for wp# mavlink message
    this_node.uav_state.add_mavlink_observer(mission_item_reached_cb, 46)

    while not rospy.is_shutdown():
        timeout_secs = int(timeout.__sub__(rospy.Time.now()).to_sec())
        if timeout_secs <> old_timeout_secs:
            rospy.loginfo(
                'In %s state node. Timeout in: %d',
                this_node.state_name,
                timeout_secs)
        old_timeout_secs = timeout_secs
        #iscurrent()
        if this_node.exec_comm.cmd != MSG_TO_STATE.START.name:
            # TODO What if any transition?
            rospy.loginfo('State aborted: %s with command %s', 
                          this_node.state_name, this_node.exec_comm.cmd)
            break
        if rospy.Time.now() > timeout:
            segment_timeout = True
            # TODO What's the transition?
            rospy.loginfo('State timed out: %s', this_node.state_name)
            break
        # TODO Are we near a cone?
        rate.sleep()

    this_node.uav_state.set_mode(MAVMODE.HOLD.name)
    #
    # Transition
    #
    near_cone = True
    obstacle_seen = False
    # Publish transition
    if obstacle_seen:
        this_node.exec_comm.send_message_to_exec(
            MSG_TO_EXEC.DONE.name,
            TRANSITION.obstacle_seen.name)
    elif near_cone:
        this_node.exec_comm.send_message_to_exec(
            MSG_TO_EXEC.DONE.name,
            TRANSITION.near_cone.name)




if __name__ == '__main__':
    try:
        this_node = exec_comm.StateNode(STATE.Following_waypoint.name)
        this_node.start = state_start
        this_node.run_state_node()
    except rospy.ROSInterruptException:
        pass

