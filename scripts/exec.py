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
# Exec uses a state machine to control macro behavior
# See:
# http://python-3-patterns-idioms-test.readthedocs.io/en/latest/StateMachine.html
#
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
import uav_rc_control

#
# Globals
#
__UAV_State = None
__UAV_RcControl = None

class STATE(AutoNumber):
    Start = ()
    Following_waypoint = ()
    Avoiding_obstacle = ()
    Driving_toward_cone = ()
    Driving_away_from_cone = ()
    Success = ()
    Failure = ()
    End = ()

#
#  Start
#
def start_transitions(txt):
    rospy.loginfo('Entered state: '+STATE.Start.name)

    # Set us to hold upon start
    resp1 = __UAV_State.set_arm(False)
    resp1 = __UAV_State.set_mode(MAVMODE.HOLD.name)
    # Set next state 
    newState = STATE.Following_waypoint.name
    return (newState, txt)




#
# Following waypoint
# transitions
# - obstacle_seen => Avoiding_obstacle
# - near_cone => Driving_toward_cone
#
def following_waypoint_transitions(txt):
    rospy.loginfo('Entered state: '+STATE.Following_waypoint.name)

    # Set UAV mode to hold while we get this state started
    resp1 = __UAV_State.set_mode(MAVMODE.HOLD.name)

    # Push 1 waypoint - it's a test
    waypoints = None
    #__UAV_RcControl.push_waypoints(waypoints)

    resp1 = __UAV_State.set_arm(True)
    #resp1 = __UAV_State.set_mode(MAVMODE.AUTO.name)
    resp1 = __UAV_State.set_mode(MAVMODE.LEARNING.name)
    
    flag = True
    rate = rospy.Rate(0.5) # some hz
    test_count = 10 # test code

    # WP Driving loop
    while not rospy.is_shutdown() and flag:
        rospy.loginfo('Driving to waypoint. Status: '+str(test_count))
        # TODO Are we near a cone?
        if test_count < 1:
            flag = False
        test_count = test_count -1
        rate.sleep()
    near_cone = True
    obstacle_seen = False

    #
    if obstacle_seen:
        newState = STATE.Avoiding_obstacle.name
    elif near_cone:
        newState = STATE.Driving_toward_cone.name
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
    rospy.loginfo('Entered state: '+STATE.Driving_toward_cone.name)

    # Set UAV mode
    resp1 = __UAV_State.set_mode(MAVMODE.MANUAL.name)

    #
    touched_cone = True
    #
    passed_cone = True
    segment_timeout = True
    #
    touched_last_cone = True
    #
    passed_last_cone = False
    course_timeout = False

    rate = rospy.Rate(0.5) # some hz
    flag = True
    test_count = 6 # test code

    # WP Driving loop
    while not rospy.is_shutdown() and flag:
        rospy.loginfo('Driving to a CONE! Within '+str(test_count)+' meters')
        # TODO Are we near a cone?
        if test_count < 1:
            flag = False

        test_count = test_count -1
        rate.sleep()

    if passed_last_cone or course_timeout:
        newState = STATE.Failure.name
    elif touched_last_cone:
        newState = STATE.Success.name
    elif passed_cone or segment_timeout:
        newState = STATE.Following_waypoint.name
    elif touched_cone:
        newState = STATE.Driving_away_from_cone.name
    return (newState, txt)




#
# Driving away from cone
# transitions
# - cleared_cone => Following_waypoint
#
def driving_away_from_cone_transitions(txt):
    rospy.loginfo('Entered state: '+STATE.Driving_away_from_cone.name)

    resp1 = __UAV_State.set_mode(MAVMODE.HOLD.name)
    # Set UAV mode
    #resp1 = __UAV_State.set_mode(MAVMODE.MANUAL.name)
    throttle = None
    servo = None
    #__UAV_RcControl.set_throttle_servo(throttle,servo)

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

    # ROS
    global __UAV_State
    __UAV_State = uav_state.UAV_State()
    global __UAV_RcControl
    __UAV_RcControl = uav_rc_control.UAV_RcControl()

    pubSoundToken = rospy.Publisher('play', String, queue_size=10)

	# Start our node
    rospy.init_node('executive', anonymous=True)

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
