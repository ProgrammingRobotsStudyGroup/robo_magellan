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

#
# Exec uses a state machine to control macro behavior
#

"""Exec uses a state machine to control macro behavior"""

# ROS
import rospy
from std_msgs.msg import String

## Our Code

from statemachine import StateMachine

from uav_state import MODE as MAVMODE
import uav_state
import uav_control

import exec_comm
from exec_comm import MSG_TO_STATE
from exec_comm import MSG_TO_EXEC
from state_and_transition import STATE
from state_and_transition import TRANSITION
#
# Globals
#
__UAV_State = None
__UAV_Control = None

pub_sound_token = None
state_complete = False


def mod_state(the_state):
    """Modify state"""
    # To run normally, we default to start state
    # Exit (SUCCESS) if parameter START_STATE != 'Start'
    start_state = rospy.get_param("/START_STATE")
    if start_state != 'Start':
        the_state = STATE.Success.name
    return the_state




#
# Wait for the state node to complete
#
def wait_on_state_node(state_name):
    """Wait for the state node to complete"""
    rate = rospy.Rate(1) # 2 hz

    # Calculate time out
    segment_sec = rospy.get_param("/SEGMENT_DURATION_SEC") + rospy.get_param("/SEGMENT_EXTRA_SEC")
    timeout = rospy.Time.now() + rospy.Duration(segment_sec)
    old_timeout_secs = 0

    while not rospy.is_shutdown():
        timeout_secs = int(timeout.__sub__(rospy.Time.now()).to_sec())
        if timeout_secs <> old_timeout_secs:
            rospy.loginfo(
                'In %s state. Timeout in: %d',
                state_name,
                timeout_secs)
        old_timeout_secs = timeout_secs

        # If timeout, reset state as a fail safe
        if rospy.Time.now() > timeout:
            __ExecComm.send_message_to_state(state_name, MSG_TO_STATE.RESET.name)
            rospy.loginfo('State timed out: %s', state_name)
            break

        if state_complete is True:
            rospy.loginfo('State complete: %s', state_name)
            break
        rate.sleep()


#
#  Start
#
def start_transitions(txt):
    state_name = STATE.Start.name
    rospy.loginfo('Entered state: ' + state_name)

    # Set us to hold upon start
    __UAV_State.set_arm(False)
    __UAV_State.set_mode(MAVMODE.HOLD.name)
    # Set next state
    return (STATE.Following_waypoint.name, txt)




#
# Following waypoint
# transitions
# - obstacle_seen => Avoiding_obstacle
# - near_cone => Driving_toward_cone
#
def following_waypoint_transitions(txt):
    """Follow waypoint State node & transitions"""
    state_name = STATE.Following_waypoint.name
    rospy.loginfo('Entered state: ' + state_name)

    __ExecComm.send_message_to_state(state_name, MSG_TO_STATE.RESET.name)
    __ExecComm.send_message_to_state(state_name, MSG_TO_STATE.START.name)

    wait_on_state_node(state_name)

    # Possibly unnecessary
    __ExecComm.send_message_to_state(state_name, MSG_TO_STATE.RESET.name)

    # Select next state based upon transition
    if __ExecComm.transition == TRANSITION.obstacle_seen.name:
        new_state = STATE.Avoiding_obstacle.name
    elif __ExecComm.transition == TRANSITION.near_cone.name:
        new_state = STATE.Driving_toward_cone.name
    else:
        rospy.logwarn('Unknown transition:'+__ExecComm.transition)
        new_state = STATE.Failure.name
        rospy.logwarn('Default to:'+new_state)
    return (mod_state(new_state), txt)




#
# Avoiding obstacle
# transitions
# - obstacle_cleared => Following_waypoint
#
def avoiding_obstacle_transitions(txt):
    """Avoid obstacle State node & transitions"""
    state_name = STATE.Avoiding_obstacle.name
    rospy.loginfo('Entered state: ' + state_name)

    __ExecComm.send_message_to_state(state_name, MSG_TO_STATE.RESET.name)
    __ExecComm.send_message_to_state(state_name, MSG_TO_STATE.START.name)

    wait_on_state_node(state_name)

    # Possibly unnecessary
    __ExecComm.send_message_to_state(state_name, MSG_TO_STATE.RESET.name)

    if __ExecComm.transition == TRANSITION.obstacle_cleared.name:
        new_state = STATE.Following_waypoint.name
    else:
        rospy.logwarn('Unknown transition:'+__ExecComm.transition)
        new_state = STATE.Failure.name
        rospy.logwarn('Default to:'+new_state)
    return (mod_state(new_state), txt)




#
# Driving toward cone
# transitions
# - touched_cone => Driving_away_from_cone
# - passed_cone => Following_waypoint
# - segment_timeout => Following_waypoint
# - touched_last_cone => Success
# - passed_last_cone => Failure
# - course_timeout => Failure
#
def driving_toward_cone_transitions(txt):
    """Driving toward cone State node & transitions"""
    state_name = STATE.Driving_toward_cone.name
    rospy.loginfo('Entered state: ' + state_name)

    global state_complete
    state_complete = False

    # Set UAV mode to hold while we get this state started
    __UAV_State.set_mode(MAVMODE.HOLD.name)

    __ExecComm.send_message_to_state(state_name, MSG_TO_STATE.RESET.name)
    __ExecComm.send_message_to_state(state_name, MSG_TO_STATE.START.name)

    wait_on_state_node(state_name)

    # Possibly unnecessary
    __ExecComm.send_message_to_state(state_name, MSG_TO_STATE.RESET.name)

    # Select next state based upon transition
    if __ExecComm.transition == TRANSITION.passed_last_cone.name:
        new_state = STATE.Failure.name
    elif __ExecComm.transition == TRANSITION.course_timeout.name:
        new_state = STATE.Failure.name
    elif __ExecComm.transition == TRANSITION.touched_last_cone.name:
        new_state = STATE.Success.name
    elif __ExecComm.transition == TRANSITION.passed_cone.name:
        new_state = STATE.Following_waypoint.name
    elif __ExecComm.transition == TRANSITION.segment_timeout.name:
        new_state = STATE.Following_waypoint.name
    elif __ExecComm.transition == TRANSITION.touched_cone.name:
        new_state = STATE.Driving_away_from_cone.name
    else:
        rospy.logwarn('Unknown transition:'+__ExecComm.transition)
        new_state = STATE.Failure.name
        rospy.logwarn('Default to:'+new_state)
    return (mod_state(new_state), txt)




#
# Driving away from cone
# transitions
# - cleared_cone => Following_waypoint
#
def driving_away_from_cone_transitions(txt):
    """Driving away from cone State node & transitions"""
    state_name = STATE.Driving_away_from_cone.name
    rospy.loginfo('Entered state: ' + state_name)


    __ExecComm.send_message_to_state(state_name, MSG_TO_STATE.RESET.name)
    __ExecComm.send_message_to_state(state_name, MSG_TO_STATE.START.name)

    wait_on_state_node(state_name)

    # Possibly unnecessary
    __ExecComm.send_message_to_state(state_name, MSG_TO_STATE.RESET.name)

    # Select next state based upon transition
    if __ExecComm.transition == TRANSITION.cleared_cone.name:
        new_state = STATE.Following_waypoint.name
    else:
        rospy.logwarn('Unknown transition:'+__ExecComm.transition)
        new_state = STATE.Failure.name
        rospy.logwarn('Default to:'+new_state)
    return (mod_state(new_state), txt)






#
# Success!
# transitions
# - End
#
def success_transitions(txt):
    """Success State node & transitions"""
    state_name = STATE.Success.name
    rospy.loginfo('Entered state: ' + state_name)

    # Set UAV to hold
    __UAV_State.set_mode(MAVMODE.HOLD.name)
    # Disarm
    __UAV_State.set_arm(False)

    new_state = STATE.End.name
    return (new_state, txt)




#
# Failure :(
# transitions
# - End
#
def failure_transitions(txt):
    """Failure State node & transitions"""
    state_name = STATE.Failure.name
    rospy.loginfo('Entered state: ' + state_name)

    # Set UAV to hold
    __UAV_State.set_mode(MAVMODE.HOLD.name)
    # Disarm
    __UAV_State.set_arm(False)

    new_state = STATE.End.name
    return (new_state, txt)


#
#
#
def __state_resp_cb(data):
    """Handle messages from state nodes"""
    #rospy.loginfo('__state_resp_cb: '+data.data)
    __ExecComm.parse_msg_to_exec(data.data)
    global state_complete
    #rospy.loginfo('__ExecComm.cmd: '+__ExecComm.cmd)
    if __ExecComm.cmd == MSG_TO_EXEC.DONE.name:
        state_complete = True
    elif __ExecComm.cmd == MSG_TO_EXEC.START_EXEC.name:
        # Set state to start with
        start_state = rospy.get_param("/START_STATE")
        machine.set_start(start_state)
        # TODO What is our cargo? "TEST"? A series of waypoints?
        cargo = "RUN"
        # Start state machine
        machine.run(cargo)




#
# State machine setup
#
def executive():
    """"State machine setup"""
    global machine
    machine = StateMachine()

    machine.add_state(STATE.Start.name, start_transitions)
    machine.add_state(STATE.Following_waypoint.name,
                      following_waypoint_transitions)
    machine.add_state(STATE.Avoiding_obstacle.name,
                      avoiding_obstacle_transitions)
    machine.add_state(STATE.Driving_toward_cone.name,
                      driving_toward_cone_transitions)
    machine.add_state(STATE.Driving_away_from_cone.name,
                      driving_away_from_cone_transitions)
    machine.add_state(STATE.Success.name, success_transitions)
    machine.add_state(STATE.Failure.name, failure_transitions)
    machine.add_state(STATE.End.name, None, end_state=1)
    #
    machine.set_start(STATE.Start.name)


    global pub_sound_token
    pub_sound_token = rospy.Publisher('play', String, queue_size=10)

	# Start our node
    rospy.init_node('executive', anonymous=True)
    # Initialize UAV models
    global __UAV_State
    __UAV_State = uav_state.UAV_State()
    global __UAV_Control
    __UAV_Control = uav_control.UAV_Control()

    # Exec/state comm
    global __ExecComm
    __ExecComm = exec_comm.ExecComm(
        STATE.Following_waypoint.name,
        exec_msg_cb=__state_resp_cb)

    rate = rospy.Rate(2) # 1 hz
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        executive()
    except rospy.ROSInterruptException:
        pass
