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

# ROS
import rospy
from std_msgs.msg import String

#
from statemachine import StateMachine

## Our Code

# Enum maps to sound name
import soundmap

from auto_number import AutoNumber
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

pubSoundToken = None
state_complete = False


#
#  Start
#
def start_transitions(txt):
    stateName = STATE.Start.name
    rospy.loginfo('Entered state: ' + stateName)

    # Set us to hold upon start
    resp1 = __UAV_State.set_arm(False)
    resp1 = __UAV_State.set_mode(MAVMODE.HOLD.name)
    # Set next state 
    return (STATE.Following_waypoint.name, txt)




#
# Following waypoint
# transitions
# - obstacle_seen => Avoiding_obstacle
# - near_cone => Driving_toward_cone
#
def following_waypoint_transitions(txt):
    stateName = STATE.Following_waypoint.name
    rospy.loginfo('Entered state: ' + stateName)

    global state_complete
    state_complete = False

    # Set UAV mode to hold while we get this state started
    resp1 = __UAV_State.set_mode(MAVMODE.HOLD.name)

    __ExecComm.send_message_to_state(stateName,MSG_TO_STATE.RESET.name)
    __ExecComm.send_message_to_state(stateName,MSG_TO_STATE.START.name)

    flag = True
    rate = rospy.Rate(1) # some hz
    test_count = 20 # test code

    while not rospy.is_shutdown() and flag:
        # TODO if timeout kill state? Do we do this here as a failsafe or state only?
        rospy.loginfo('In '+stateName+' state. Countdown: '+str(test_count))

        # TODO Are we near a cone?

#        rospy.loginfo('state_complete: '+str(state_complete))
        if state_complete is True:
            flag = False
        if test_count < 1:
            flag = False
            __ExecComm.send_message_to_state(stateName,MSG_TO_STATE.RESET.name)
            rospy.loginfo('Timed out')
        test_count = test_count -1
        rate.sleep()

    # Possibly unnecessary
    __ExecComm.send_message_to_state(stateName,MSG_TO_STATE.RESET.name)
    # Select next state based upon transition
    if __ExecComm.transition == TRANSITION.obstacle_seen.name:
        newState = STATE.Avoiding_obstacle.name
    elif __ExecComm.transition == TRANSITION.near_cone.name:
        newState = STATE.Driving_toward_cone.name
    else:
        rospy.logwarn('Unknown transition: '+str(__ExecComm.transition))
        newState = STATE.Failurecone.name

    return (newState, txt)




#
# Avoiding obstacle
# transitions
# - obstacle_cleared => Following_waypoint
#
def avoiding_obstacle_transitions(txt):
    rospy.loginfo('Entered state: '+STATE.Avoiding_obstacle.name)

    obstacle_cleared = True
    if obstacle_cleared:
        newState = STATE.Following_waypoint.name
    return (newState, txt)




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
    stateName = STATE.Driving_toward_cone.name
    rospy.loginfo('Entered state: ' + stateName)

    global state_complete
    state_complete = False

    # Set UAV mode to hold while we get this state started
    resp1 = __UAV_State.set_mode(MAVMODE.HOLD.name)

    __ExecComm.send_message_to_state(stateName,MSG_TO_STATE.RESET.name)
    __ExecComm.send_message_to_state(stateName,MSG_TO_STATE.START.name)

    flag = True
    rate = rospy.Rate(1) # some hz
    test_count = 30 # test code

    while not rospy.is_shutdown() and flag:
        # TODO if timeout kill state? Do we do this here as a failsafe or state only?
        rospy.loginfo('Driving to a CONE! Within '+str(test_count)+' meters')

        # TODO Are we near a cone?

#        rospy.loginfo('state_complete: '+str(state_complete))
        if state_complete is True:
            flag = False
        if test_count < 1:
            flag = False
            __ExecComm.send_message_to_state(stateName,MSG_TO_STATE.RESET.name)
            rospy.loginfo('Timed out')
            __ExecComm.transition = TRANSITION.segment_timeout.name
        test_count = test_count -1
        rate.sleep()

    # Possibly unnecessary
    __ExecComm.send_message_to_state(stateName,MSG_TO_STATE.RESET.name)

    # Select next state based upon transition
    if __ExecComm.transition == TRANSITION.passed_last_cone.name:
        newState = STATE.Failure.name
    elif __ExecComm.transition == TRANSITION.course_timeout.name:
        newState = STATE.Failure.name
    elif __ExecComm.transition == TRANSITION.touched_last_cone.name:
        newState = STATE.Success.name
    elif __ExecComm.transition == TRANSITION.passed_cone.name:
        newState = STATE.Following_waypoint.name
    elif __ExecComm.transition == TRANSITION.segment_timeout.name:
        newState = STATE.Following_waypoint.name
    elif __ExecComm.transition == TRANSITION.touched_cone.name:
        newState = STATE.Driving_away_from_cone.name
    else:
        rospy.logwarn('Unknown transition:'+__ExecComm.transition)
        newState = STATE.Failure.name
        rospy.logwarn('Default to:'+newState)
    return (newState, txt)




#
# Driving away from cone
# transitions
# - cleared_cone => Following_waypoint
#
def driving_away_from_cone_transitions(txt):
    stateName = STATE.Driving_away_from_cone.name
    rospy.loginfo('Entered state: ' + stateName)

    global state_complete
    # Set UAV mode
    resp1 = __UAV_State.set_mode(MAVMODE.HOLD.name)

    __ExecComm.send_message_to_state(stateName,MSG_TO_STATE.RESET.name)
    __ExecComm.send_message_to_state(stateName,MSG_TO_STATE.START.name)

    flag = True
    rate = rospy.Rate(1) # some hz
    test_count = 20 # test code

    while not rospy.is_shutdown() and flag:
        # TODO if timeout kill state? Do we do this here as a failsafe or state only?
        rospy.loginfo('In '+stateName+' state. Countdown: '+str(test_count))

        # TODO Are we near a cone?

        #rospy.loginfo('state_complete: '+str(state_complete))
        if state_complete is True:
            flag = False
        if test_count < 1:
            flag = False
            __ExecComm.send_message_to_state(STATE.Following_waypoint.name,MSG_TO_STATE.RESET.name)
            rospy.loginfo('Timed out')
        test_count = test_count -1
        rate.sleep()
    ###
    #__ExecComm.send_message_to_state(STATE.Following_waypoint.name,MSG_TO_STATE.RESET.name)
    near_cone = True
    obstacle_seen = False

    # Unnecessary, but a bit of code.
    __ExecComm.send_message_to_state(STATE.Following_waypoint.name,MSG_TO_STATE.RESET.name)
    #
    if obstacle_seen:
        newState = STATE.Avoiding_obstacle.name
    elif near_cone:
        newState = STATE.Driving_toward_cone.name
    return (newState, txt)

    cleared_cone = True
    if cleared_cone:
        newState = STATE.Following_waypoint.name
    return (newState, txt)






#
# Success!
# transitions
# - End
#
def success_transitions(txt):
    rospy.loginfo('Entered state: '+STATE.Success.name)

    # Set UAV to hold
    resp1 = __UAV_State.set_mode(MAVMODE.HOLD.name)
    # Disarm
    resp1 = __UAV_State.set_arm(False)

    newState = STATE.End.name
    return (newState, txt)




#
# Failure :(
# transitions
# - End
#
def failure_transitions(txt):
    rospy.loginfo('Entered state: '+STATE.Failure.name)

    # Set UAV to hold
    resp1 = __UAV_State.set_mode(MAVMODE.HOLD.name)
    # Disarm
    resp1 = __UAV_State.set_arm(False)

    newState = STATE.End.name
    return (newState, txt)


#
#
#
def __state_resp_cb(data):
    #rospy.loginfo('__state_resp_cb: '+data.data)
    __ExecComm.parse_msg_to_exec(data.data)
    global state_complete
    #rospy.loginfo('__ExecComm.cmd: '+__ExecComm.cmd)
    if __ExecComm.cmd==MSG_TO_EXEC.DONE.name: 
        state_complete = True
    pass

def executive():

    # State machine setup
    machine = StateMachine()
    # 
    machine.add_state(STATE.Start.name, start_transitions)
    machine.add_state(STATE.Following_waypoint.name, following_waypoint_transitions)
    machine.add_state(STATE.Avoiding_obstacle.name, avoiding_obstacle_transitions)
    machine.add_state(STATE.Driving_toward_cone.name, driving_toward_cone_transitions)
    machine.add_state(STATE.Driving_away_from_cone.name, driving_away_from_cone_transitions)
    machine.add_state(STATE.Success.name, success_transitions)
    machine.add_state(STATE.Failure.name, failure_transitions)
    machine.add_state(STATE.End.name, None, end_state=1)
    #
    machine.set_start(STATE.Start.name)


    global pubSoundToken
    pubSoundToken = rospy.Publisher('play', String, queue_size=10)

	# Start our node
    rospy.init_node('executive', anonymous=True)
    # Initialize UAV models 
    global __UAV_State
    __UAV_State = uav_state.UAV_State()
    global __UAV_Control
    __UAV_Control = uav_control.UAV_Control()

    # Exec/state comm
    global __ExecComm
    __ExecComm = exec_comm.ExecComm(STATE.Following_waypoint.name, exec_msg_cb=__state_resp_cb)

    # Start state machine
    # TODO What is our cargo?
    machine.run("RUN")
    #machine.run("TEST")
    #machine.run("Series of waypoints?")
    pass

if __name__ == '__main__':
    try:
        executive()
    except rospy.ROSInterruptException:
        pass
