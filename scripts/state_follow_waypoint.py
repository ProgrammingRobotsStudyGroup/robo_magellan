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

#TODO: Rework code to use to_ezec and to_state message formats
#TODO: state_start. state_reset, state_pause


#
# Exec command listener callback
#
def cmd_callback(data):
    """Exec command listener callback"""
    # Parses the message
    # State is returned. If message state is our state, cmd is updated.
    the_state = __ExecComm.parse_msg_to_state(data.data)

    if the_state == __ExecComm.state:
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



def iscurrent():
    """Prints list of WPs"""
    __UAV_Control.pull_waypoints()
    print __UAV_Control.waypoint_list
#    iscur = -1
#    for seq, waypoint in enumerate( __UAV_Control.waypoint_list):
#        if waypoint.is_current:
#            iscur = seq
#            print (''+str(seq))
#            break
#    if iscur == -1:
#        print('no current')

#
#
#
def state_start():
    """Start the state"""
    state_name = STATE.Following_waypoint.name
    rospy.loginfo('state_start %s', state_name)

    # TODO Setting mode to HOLD is precautionary.
    # Set UAV mode to hold while we get this state started
    __UAV_State.set_mode(MAVMODE.HOLD.name)
    __UAV_State.set_arm(False)
    print "HERE 1"
    ## TEST
    #__UAV_Control.pull_waypoints()
    #wpl = __UAV_Control.waypoint_list
    #print wpl
    #wpl.pop()
    #print wpl
    #__UAV_Control.clear_waypoints()
    #__UAV_Control.push_waypoints(wpl)
#    print __UAV_Control.waypoint_list
    # TODO Test push
    # Pick wp to start index
    #iscurrent()
    __UAV_Control.set_current_waypoint(0)
    print "HERE 2"

    __UAV_State.set_mode(MAVMODE.AUTO.name)
    __UAV_State.set_arm(True)

    rate = rospy.Rate(0.5) # some hz
    #iscurrent()

    # WP Driving loop
    segment_duration_sec = rospy.get_param("/SEGMENT_DURATION_SEC")
    timeout = rospy.Time.now() + rospy.Duration(segment_duration_sec)
    old_timeout_secs = 0

    while not rospy.is_shutdown():
        timeout_secs = int(timeout.__sub__(rospy.Time.now()).to_sec())
        if timeout_secs <> old_timeout_secs:
            rospy.loginfo(
                'In %s state NODE. Timeout in: %d',
                state_name,
                timeout_secs)
        old_timeout_secs = timeout_secs
        #iscurrent()
        if __ExecComm.cmd != MSG_TO_STATE.START.name:
            # TODO What if any transition?
            break
        if rospy.Time.now() > timeout:
            # TODO What's the transition?
            rospy.loginfo('State timed out: %s', state_name)
            break
        # TODO Are we near a cone?
        rate.sleep()

    __UAV_State.set_mode(MAVMODE.HOLD.name)
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
# Start our node
#
def state_node(state_name):
    """Start node"""

    rospy.loginfo('State node starting: %s', state_name)
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
        state_node(STATE.Following_waypoint.name)
    except rospy.ROSInterruptException:
        pass

