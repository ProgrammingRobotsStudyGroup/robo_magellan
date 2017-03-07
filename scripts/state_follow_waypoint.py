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
import uav_state
from uav_state import MODE as MAVMODE
import uav_control

import exec_comm
from exec_comm import MSG_TO_STATE
from exec_comm import MSG_TO_EXEC
from state_and_transition import STATE
from state_and_transition import TRANSITION

# Globals

'''
TODO
1) Rework code to use msg formats, state_start
2) Find common approach to: state_start, state_reset, state_pause
3) .
'''
#
# Exec command listener callback
#
def cmd_callback(data):
    # Parses the message
    # State is returned. If message state is our state, cmd is updated.
    theState = __ExecComm.parse_msg_to_state(data.data)

    if theState == __ExecComm.state:
        rospy.loginfo(rospy.get_caller_id() + ' cmd_callback: %s', data.data)
        # Handle start, reset, pause, etc.
        if __ExecComm.cmd == MSG_TO_STATE.START.name:
            state_start()
        elif __ExecComm.cmd == MSG_TO_STATE.RESET.name:
            state_reset()
        elif __ExecComm.cmd == MSG_TO_STATE.PAUSE.name:
            state_pause()
        else:
            rospy.logwarn('Invalid cmd: '+data.data)




#
# Reset the state
# For safety, for now set to HOLD
#
def state_reset():
    """Reset the state"""
    # Set UAV mode to hold while we get this state started
    __UAV_State.set_mode(MAVMODE.HOLD.name)
    __UAV_State.set_arm(False)




#
# Pause the state
#
def state_pause():
    """Pause the state"""
    # Set UAV mode to hold while we get this state started
    __UAV_State.set_mode(MAVMODE.HOLD.name)
    __UAV_State.set_arm(False)




#
#
#
def state_start():
    rospy.loginfo('state_start')

    # TODO Setting mode to HOLD is precautionary.
    # Set UAV mode to hold while we get this state started
    resp1 = __UAV_State.set_mode(MAVMODE.HOLD.name)
    resp1 = __UAV_State.set_arm(False)

    # Update waypoint topic
    __UAV_Control.set_current_waypoint(1)
    resp = __UAV_Control.pull_waypoints()
    ## TEST
    wpl = __UAV_Control.waypoint_list
    print wpl
    #wpl.pop()
    #print wpl
    #__UAV_Control.clear_waypoints()
    #__UAV_Control.push_waypoints(wpl)
#    print __UAV_Control.waypoint_list
    # TODO Test push
#    __UAV_Control.set_current_waypoint(0)

    resp1 = __UAV_State.set_mode(MAVMODE.AUTO.name)
    resp1 = __UAV_State.set_arm(True)

    flag = True
    rate = rospy.Rate(0.5) # some hz
    test_count = 2 # test code

    # WP Driving loop
    while not rospy.is_shutdown() and flag:
        rospy.loginfo('Driving to waypoint. Status: '+str(test_count))
        if __ExecComm.cmd != MSG_TO_STATE.START.name:
            flag = False
        if test_count < 1:
            flag = False
        # TODO Are we near a cone?
        test_count = test_count -1
        rate.sleep()

    resp1 = __UAV_State.set_mode(MAVMODE.HOLD.name)
    #
    # Transition
    #
    near_cone = True
    obstacle_seen = False
    # Publish transition
    if obstacle_seen:
#        newState = STATE.Avoiding_obstacle.name
        __ExecComm.send_message_to_exec(
            MSG_TO_EXEC.DONE.name,
            TRANSITION.obstacle_seen.name)
    elif near_cone:
#        newState = STATE.Driving_toward_cone.name
        __ExecComm.send_message_to_exec(
            MSG_TO_EXEC.DONE.name,
            TRANSITION.near_cone.name)


#
#
#
def state_node():
    state_name = STATE.Following_waypoint.name
	# Start our node
    rospy.loginfo('State node starting: '+state_name)
    rospy.init_node(state_name, anonymous=False)

    # Initialize UAV models
    global __UAV_State
    __UAV_State = uav_state.UAV_State()
    global __UAV_Control
    __UAV_Control = uav_control.UAV_Control()

    # Exec/state comm
    global __ExecComm
    __ExecComm = exec_comm.ExecComm(state_name, cmd_callback)

    rate = rospy.Rate(10) # 10 hz
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    try:
        state_node()
    except rospy.ROSInterruptException:
        pass

