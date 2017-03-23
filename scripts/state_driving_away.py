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

"""Drive away from cone state node"""
#
# Drive away from cone state node
#
import time

# ROS
import rospy
#

from uav_state import MODE as MAVMODE

import exec_comm
from exec_comm import MSG_TO_STATE
from exec_comm import MSG_TO_EXEC
from state_and_transition import STATE
from state_and_transition import TRANSITION

# Globals
this_node = None
steering_limits = []
throttle_limits = []
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

    this_node.uav_state.set_mode(MAVMODE.MANUAL.name)
    this_node.uav_state.set_arm(True)

    # Get radio calibration values
    # [1] is neutral
    global steering_limits
    steering_limits = [this_node.uav_control.get_param_int('RC1_MIN'),
                       this_node.uav_control.get_param_int('RC1_TRIM'),
                       this_node.uav_control.get_param_int('RC1_MAX')]
    global throttle_limits
    throttle_limits = [this_node.uav_control.get_param_int('RC3_MIN'),
                       this_node.uav_control.get_param_int('RC3_TRIM'),
                       this_node.uav_control.get_param_int('RC3_MAX')]

    rate = rospy.Rate(10) # 10 hz

    # Driving away loop
    segment_duration_sec = rospy.get_param("/SEGMENT_DURATION_SEC")
    timeout = rospy.Time.now() + rospy.Duration(segment_duration_sec)
    old_timeout_secs = 0
    drive_pattern()
    timeout = rospy.Time.now()
    while not rospy.is_shutdown():
        timeout_secs = int(timeout.__sub__(rospy.Time.now()).to_sec())
        if timeout_secs <> old_timeout_secs:
            rospy.loginfo(
                'In %s state node. Timeout in: %d',
                this_node.state_name,
                timeout_secs)
        old_timeout_secs = timeout_secs
        if this_node.exec_comm.cmd != MSG_TO_STATE.START.name:
            # TODO What if any transition?
            rospy.loginfo(
                'State aborted: %s with command %s',
                this_node.state_name,
                this_node.exec_comm.cmd)
            break
        if rospy.Time.now() > timeout:
            segment_timeout = True
            # TODO What's the transition?
            rospy.loginfo('State timed out: %s', this_node.state_name)
            break
        # TODO Are we clear?
        rate.sleep()

    # Publish transition.
    cleared_cone = True
    if cleared_cone:
        this_node.exec_comm.send_message_to_exec(
            MSG_TO_EXEC.DONE.name,
            TRANSITION.cleared_cone.name)


#
#  1) Back up for 2 seconds
#  2) Stop for 1 second
#  3) Adjust servo by pct (can be + or -)
#
def drive_pattern():
    """The pattern """
    print "steering_limits"
    print steering_limits
    print "throttle_limits"
    print throttle_limits
    pct_servo = rospy.get_param("/AWAY_PCT_SERVO") / 100.0
    if pct_servo > 0:
        steering = steering_limits[1] + (steering_limits[2]-steering_limits[1]) * pct_servo
    else:
        steering = steering_limits[1] - (steering_limits[1]-steering_limits[0]) * pct_servo

    pct_throttle = rospy.get_param("/AWAY_PCT_THROTTLE")
    pct_throttle /= 100.0
    pos_throttle = (throttle_limits[1] + 
                    (throttle_limits[2] - throttle_limits[1]) * pct_throttle)
    neg_throttle = (throttle_limits[1] - 
                    (throttle_limits[1] - throttle_limits[0]) * pct_throttle)

    rate = rospy.Rate(0.1) # 1 hz

    rospy.loginfo("Second 00: Pause");
    this_node.uav_control.set_throttle_servo(throttle_limits[1], steering_limits[1])
    time.sleep(0.1)
    
    rospy.loginfo("Second 01: Backup");
    start = time.clock()
    this_node.uav_control.set_throttle_servo(neg_throttle, steering_limits[1])
    elapsed = time.clock()
    elapsed = elapsed - start
    rospy.loginfo("Time spent in set_throttle_servo() is: %s ",
                  str(elapsed))
    time.sleep(0.1) 

    rospy.loginfo("Second 02: Backup");
    start = time.clock()
    this_node.uav_control.set_throttle_servo(neg_throttle, steering_limits[1])
    elapsed = time.clock()
    elapsed = elapsed - start
    rospy.loginfo("Time spent in set_throttle_servo() is: %s ",
                  str(elapsed))
    time.sleep(0.1)

    rospy.loginfo("Second 03: Backup");
    start = time.clock()
    this_node.uav_control.set_throttle_servo(neg_throttle, steering_limits[1])
    elapsed = time.clock()
    elapsed = elapsed - start
    rospy.loginfo("Time spent in set_throttle_servo() is: %s ",
                  str(elapsed))
    time.sleep(0.1)

    rospy.loginfo("Second 04: Pause");
    this_node.uav_control.set_throttle_servo(throttle_limits[1], steering_limits[1])
    time.sleep(0.1)

    rospy.loginfo("Second 05: Forward");
    this_node.uav_control.set_throttle_servo(pos_throttle, steering)
    time.sleep(0.1)

    rospy.loginfo("Second 06: Forward");
    this_node.uav_control.set_throttle_servo(pos_throttle, steering)
    time.sleep(0.1)

    rospy.loginfo("Second 07: Pause/Done");
    this_node.uav_control.set_throttle_servo(throttle_limits[1], steering_limits[1])
    time.sleep(0.1) 
    this_node.uav_state.set_mode(MAVMODE.HOLD.name)




if __name__ == '__main__':
    try:
        this_node = exec_comm.StateNode(STATE.Driving_away_from_cone.name)
        this_node.start = state_start
        this_node.run_state_node()
    except rospy.ROSInterruptException:
        pass

