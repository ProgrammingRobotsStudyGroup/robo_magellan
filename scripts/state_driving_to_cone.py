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

"""read cone_finder location messages and publish OverrideRCIn messages"""
#
# Node publishes /mavros_msgs/OverrideRCIn messages
# using cone_finder/location messages
#

import sys, argparse, math

# ROS
import rospy
from std_msgs.msg import Bool
#
from uav_state import MODE as MAVMODE

import exec_comm
from exec_comm import MSG_TO_STATE
from exec_comm import MSG_TO_EXEC
from state_and_transition import STATE
from state_and_transition import TRANSITION

from mavros_msgs.msg import OverrideRCIn
from robo_magellan.msg import location_msgs as Locations

from cone_code import ConeSeeker

# Globals
this_node = None


# We will get angle between +pi/2 to -pi/2 for steering
# We will get 480 pixels range for throttle but should limit this
class Args(object):
    """Class to contain program arguments"""
    # Ranges are between 0. to 1.0
    throttle_range = 1.0
    steering_range = 1.0
    min_throttle = 0.3


# Globals

args = Args()

#settings = termios.tcgetattr(sys.stdin)
hard_limits = [1000, 1500, 2000]  # microseconds for servo signal
steering_limits = [1135, 1435, 1735]  # middle is neutral
# throttle_limits = [1200, 1500, 1800]  # middle is neutral
throttle_limits = [1650, 1650, 1800]  # fwd range only; for testing; middle is NOT neutral




#
#
## Transitions
# - touched_cone => Driving_away_from_cone
# - passed_cone => Following_waypoint
# - segment_timeout => Following_waypoint
# - touched_last_cone => Success
# - passed_last_cone => Failure
# - course_timeout => Failure
#
def state_start():
    """Start the state"""
    rospy.loginfo('state_start %s', this_node.state_name)

    # TODO Setting mode to HOLD is precautionary.
    # Set UAV mode to hold while we get this state started
    this_node.uav_state.set_mode(MAVMODE.HOLD.name)
    this_node.uav_state.set_arm(False)

    #
    touched_cone = False
    #
    passed_cone = True
    segment_timeout = False
    #
    touched_last_cone = True
    #
    passed_last_cone = False
    course_timeout = False

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
    print "steering_limits"
    print steering_limits
    print "throttle_limits"
    print throttle_limits

    rate = rospy.Rate(10) # 2 hz
    global touched
    touched = False
    sub_touch = rospy.Subscriber('/touch', Bool, touched_cb)

    sub_location = rospy.Subscriber('/cone_finder/locations', Locations, drive_to, queue_size=1)
    this_node.uav_state.set_mode(MAVMODE.MANUAL.name)
    this_node.uav_state.set_arm(True)

    # Driving To cone loop
    segment_duration_sec = rospy.get_param("/SEGMENT_DURATION_SEC")
    timeout = rospy.Time.now() + rospy.Duration(segment_duration_sec)
    old_timeout_secs = 0

    while not rospy.is_shutdown():
        timeout_secs = int(timeout.__sub__(rospy.Time.now()).to_sec())
        if timeout_secs <> old_timeout_secs:
            rospy.loginfo(
                'In %s state node. Timeout in: %d',
                this_node.state_name,
                timeout_secs)
        old_timeout_secs = timeout_secs
        if touched:
            touched_cone = True # Signal we touched a cone
            break
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
        rate.sleep()

    # Stop subscribing
    sub_location.unregister()
    sub_touch.unregister()
    this_node.uav_control.set_throttle_servo(throttle_limits[1], steering_limits[1])

    # Put in safe mode
    this_node.uav_state.set_mode(MAVMODE.HOLD.name)
    this_node.uav_state.set_arm(False)

    # Publish transition
    if passed_last_cone:
        this_node.exec_comm.send_message_to_exec(MSG_TO_EXEC.DONE.name, TRANSITION.passed_last_cone.name)
    elif course_timeout:
        this_node.exec_comm.send_message_to_exec(MSG_TO_EXEC.DONE.name, TRANSITION.course_timeout.name)
    elif touched_last_cone:
        this_node.exec_comm.send_message_to_exec(MSG_TO_EXEC.DONE.name, TRANSITION.touched_last_cone.name)
    elif passed_cone:
        this_node.exec_comm.send_message_to_exec(MSG_TO_EXEC.DONE.name, TRANSITION.passed_cone.name)
    elif segment_timeout:
        this_node.exec_comm.send_message_to_exec(MSG_TO_EXEC.DONE.name, TRANSITION.segment_timeout.name)
    elif touched_cone:
        this_node.exec_comm.send_message_to_exec(MSG_TO_EXEC.DONE.name, TRANSITION.touched_cone.name)



#
# Touch listener
#
def touched_cb(data):
    """Touch listener"""
    global touched
    rospy.loginfo("touched_cb: "+str(data.data))
    if data.data == True:
        touched = True




#
#
#
def drive_to(loc):
    args.min_throttle = rospy.get_param("/CONE_MIN_THROTTLE")
    cs = ConeSeeker(args.min_throttle)
    # sadj = [-1. to 1.], tadj = [0 to 1.]
    (cl, conf, sadj, tadj) = cs.seek_cone(loc.poses)

    # Assuming equal range spread for steering
    steering_range = (steering_limits[2] - steering_limits[0])/2
    #Throttle doesn't have -ve range
    throttle_range = throttle_limits[2] - throttle_limits[1]

    steering = steering_limits[1] + args.steering_range*sadj*steering_range
    throttle = throttle_limits[1] + args.throttle_range*tadj*throttle_range

    #-- test with fixed throttle to start
    #throttle = 1675
    #throttle = throttle + tadj
    #pct_throttle = rospy.get_param("/CONE_PCT_THROTTLE")
    #if pct_throttle > 1:
    #    pct_throttle /= 100.0
    #    throttle = throttle_limits[1] + (throttle_limits[2]-throttle_limits[1])*pct_throttle

    # Everything must be bounded
    if steering > steering_limits[2]:
        steering = steering_limits[2]
    if steering < steering_limits[0]:
        steering = steering_limits[0]
    if throttle > throttle_limits[2]:
        throttle = throttle_limits[2]
    if throttle < throttle_limits[0]:
        throttle = throttle_limits[0]

    this_node.uav_control.set_throttle_servo(throttle, steering)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Drive to cone found')
    parser.add_argument('--throttle_range', '-t', default=1.0, type=float,
                        help='Throttle range on a scale of 0. (min) to 1.0 (max)')
    parser.add_argument('--steering_range', '-s', default=1.0, type=float,
                        help='Steering range on a scale of 0. (min) to 1.0 (max)')
    parser.add_argument('--min_throttle', '-m', default=0.3, type=float,
                        help='Minimum throttle factor to use')
    parser.parse_args(rospy.myargv(sys.argv[1:]), args)
    try:
        this_node = exec_comm.StateNode(STATE.Driving_toward_cone.name)
        this_node.start = state_start
        this_node.run_state_node()
    except rospy.ROSInterruptException:
        pass

