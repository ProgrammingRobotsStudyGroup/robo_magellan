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
import rospy
import rospkg

import std_msgs.msg as std_msgs
from std_msgs.msg import Bool
from mavros_msgs.msg import Mavlink

from mavros_msgs.msg import State, OverrideRCIn, ParamValue, WaypointList
from mavros_msgs.srv import ParamGet, ParamSet, SetMode, CommandBool
from mavros_msgs.srv import WaypointSetCurrent

from geometry_msgs.msg import Pose, PoseStamped, Twist, TwistStamped

import uav_state

class RCChannels:
	SPEED = 2
	TURNING = 0

class Topics:
	EXEC_CMD = '~exec_cmd'
	RC_OVERRIDE = '/mavros/rc/override'
	KILL_SW_ENABLE = '/kill_sw_enabled'
	SETPOINT = '/setpoint'
	CONTROL = '/control'
	SET_MODE = '/mavros/set_mode'

class Modes:
	MANUAL = 'MANUAL'
	HOLD = 'HOLD'
	AUTO = 'AUTO'
	GUIDED = 'GUIDED'
	RTL = 'RTL'


# Sets the R/C override speeds. Turning is positive
# to the left, while the R/C PWM value for turning
# decreases to the left.
def set_manual_speed(speed, turning):
	channels = [OverrideRCIn.CHAN_NOCHANGE, # 0
				OverrideRCIn.CHAN_NOCHANGE, # 1
				OverrideRCIn.CHAN_NOCHANGE, # 2
				OverrideRCIn.CHAN_NOCHANGE, # 3
				OverrideRCIn.CHAN_NOCHANGE, # 4
				OverrideRCIn.CHAN_NOCHANGE, # 5
				OverrideRCIn.CHAN_NOCHANGE, # 6
				OverrideRCIn.CHAN_NOCHANGE] # 7

	speed = max(-500, min(500, speed))
	turning = max(-500, min(500, turning))
	channels[RCChannels.SPEED] = 1500 + speed
	channels[RCChannels.TURNING] = 1435 + turning

	msg = OverrideRCIn()
	msg.channels = channels
	rc_pub.publish(msg)

def on_kill_switch_enable(msg):
	rospy.loginfo('pwm_test - on_exec_cmd')
	cmd = msg.data
	if cmd is True:
		linear_speed = -0.8
		twist = TwistStamped()
		twist.twist.linear.x = linear_speed
		twist.twist.angular.z = 0
		vel_pub.publish(twist)
		rospy.loginfo('[GUIDED] speed=%f turning=%f', twist.twist.linear.x, twist.twist.angular.z)
	else:
		# set_manual_speed(0, 0)
		linear_speed = 0
		twist = TwistStamped()
		twist.twist.linear.x = linear_speed
		twist.twist.angular.z = 0
		vel_pub.publish(twist)

def pwm_test():
	rospy.init_node('pwm_test')
	global rc_pub
	rc_pub = rospy.Publisher(Topics.RC_OVERRIDE, OverrideRCIn, queue_size=1)
	# setpoint_pub = rospy.Publisher(, Bool, queue_size=10)
	rospy.Subscriber(Topics.KILL_SW_ENABLE, Bool, on_kill_switch_enable)
	# rospy.Subscriber()
	global vel_pub
	vel_pub = rospy.Publisher(Topics.SETPOINT_VELOCITY, TwistStamped, queue_size=1)

	global _mavros_set_mode
	_mavros_set_mode = get_proxy(Topics.SET_MODE, SetMode)

	set_mode(Modes.GUIDED)

	rate = rospy.Rate(rospy.get_param('~rate', 10))

	while not rospy.is_shutdown():
		rate.sleep()

def get_proxy(topic, type):
	rospy.wait_for_service(topic)
	return rospy.ServiceProxy(topic, type)

def set_mode(mode):
	_mavros_set_mode(0, mode)

if __name__ == '__main__':
	# Start the node
	try:
		pwm_test()
	except rospy.ROSInterruptException:
		pass
