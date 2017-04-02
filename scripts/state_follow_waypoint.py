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

import time

# ROS
import rospy

from std_msgs.msg import Int16
from mavros_msgs.msg import Mavlink
from mavros import mavlink

from uav_state import MODE as MAVMODE

import exec_comm
from exec_comm import MSG_TO_STATE, MSG_TO_EXEC
from state_and_transition import STATE,  TRANSITION

# Globals
this_node = None
do_once = False

# TODO: Test follow WP
# TODO: move from wp to wp
# Read WP list
# list_cone_item = Create list of cone item #'s
# start_at_wp = last_item
# 
# if wp in list_cone_item, we are at cone location, break.




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
    """Reached mission item"""
    rospy.loginfo("Mission Item reached"+str(data))
    rospy.set_param("/LAST_ITEM", data.payload64[0] & 0xFFFF)




#
#
#
def state_start():
    """Start the state"""

    cone_list = []
    cone_idx_list = []

    rospy.loginfo('state_start %s', this_node.state_name)
    global do_once
    if not do_once:
        # Add callback for wp# mavlink message
        this_node.uav_state.add_mavlink_observer(mission_item_reached_cb, 46)
        do_once = True

    #
    # Transition
    #
    near_cone = False
    obstacle_seen = False
    segment_timeout = False
    last_cone = False
    last_cone_no_bkup = False

    # TODO Setting mode to HOLD is precautionary.
    # Set UAV mode to hold while we get this state started
#    this_node.uav_state.set_mode(MAVMODE.HOLD.name)
#    time.sleep(0.5)
#    this_node.uav_state.set_arm(False)
#    time.sleep(0.5)
    #iscurrent()
    last_item = rospy.get_param("/LAST_ITEM")
    next_item = rospy.get_param("/NEXT_ITEM")

    this_node.uav_control.set_current_waypoint(next_item)

    # Force waypoint list refresh
    this_node.uav_control.pull_waypoints()
    waypoint_list = this_node.uav_control.waypoint_list
    if next_item >= len(this_node.uav_control.waypoint_list):
        this_node.exec_comm.send_message_to_exec(
            MSG_TO_EXEC.DONE.name,
            TRANSITION.exit_out.name)
        return

    idx = 0
    for waypoint in waypoint_list:
        # Altitude >= 1000 indicates cone node
        if waypoint.z_alt >= 1000:
            cone_list.append(waypoint)
            cone_idx_list.append(idx)
        idx += 1

    rospy.loginfo("cone_list:\n%s", str(cone_list))
    rospy.loginfo("cone_wp item # list: %s", str(cone_idx_list))

    this_node.uav_state.set_mode(MAVMODE.AUTO.name)
    time.sleep(0.5)
    if not this_node.uav_state.arm:
        this_node.uav_state.set_arm(True)
        time.sleep(0.5)

    rate = rospy.Rate(4) # 4 hz
    #iscurrent()

    # WP Driving loop
    segment_duration_sec = rospy.get_param("/SEGMENT_DURATION_SEC")
    timeout = rospy.Time.now() + rospy.Duration(segment_duration_sec)
    old_timeout_secs = 0

    while not rospy.is_shutdown():
        the_last = rospy.get_param("/LAST_ITEM")
        next_item = rospy.get_param("/NEXT_ITEM")
        if last_item != the_last:
            rospy.loginfo(
                "WP # Changed. /LAST_ITEM: %s, previously: %s; /NEXT_ITEM: %s",
                str(the_last),
                str(last_item),
                str(next_item)
                )
            last_item = the_last
            rospy.set_param("/LAST_ITEM", last_item)
            rospy.set_param("/NEXT_ITEM", last_item+1)

            if the_last+1==len(this_node.uav_control.waypoint_list):
                # Reached last WP
                # TODO: Deal with 2000 stuff
                if waypoint_list[the_last].z_alt >= 2000:
                    last_cone_no_bkup = True
                else:
                    near_cone = True
                break
            if the_last>=len(this_node.uav_control.waypoint_list):
                # WP # too big
                rospy.loginfo(
                    "len(this_node.uav_control.waypoint_list): %s , BAD LAST_ITEM",
                    len(this_node.uav_control.waypoint_list)
                    )
                bad_wp = True
                break
            else:
                if the_last in cone_idx_list:
                    near_cone = True
                    break

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
            rospy.loginfo(
                'State aborted: %s with command %s',
                this_node.state_name,
                this_node.exec_comm.cmd)
            break
        if rospy.Time.now() > timeout:
            segment_timeout = True
            rospy.loginfo('State timed out: %s', this_node.state_name)
            break
        # TODO Are we near a cone?
        rate.sleep()

    this_node.uav_state.set_mode(MAVMODE.HOLD.name)
    time.sleep(0.5)

    # Publish transition
    if obstacle_seen:
        this_node.exec_comm.send_message_to_exec(
            MSG_TO_EXEC.DONE.name,
            TRANSITION.obstacle_seen.name)
    elif near_cone:
        this_node.exec_comm.send_message_to_exec(
            MSG_TO_EXEC.DONE.name,
            TRANSITION.near_cone.name)
    elif segment_timeout:
        this_node.exec_comm.send_message_to_exec(
            MSG_TO_EXEC.DONE.name,
            TRANSITION.segment_timeout.name)
    else:
        this_node.exec_comm.send_message_to_exec(
            MSG_TO_EXEC.DONE.name,
            'unknown')

def simulate_reached_cb(topic):
    """Simulate reached wp callback"""
    # Simulate a Mavlink message
    ml = Mavlink()
    ml.len = 2
    ml.msgid = 46
    ml.payload64 = mavlink.convert_to_payload64([topic.data])
    mission_item_reached_cb(ml)
    pass

def startup_code():
    """Custom startup code"""
    rospy.Subscriber('simulate_reached_wp', Int16, simulate_reached_cb)


if __name__ == '__main__':
    try:
        this_node = exec_comm.StateNode(STATE.Following_waypoint.name)
        this_node.start = state_start
        this_node.custom_startup = startup_code

        this_node.run_state_node()
    except rospy.ROSInterruptException:
        pass

