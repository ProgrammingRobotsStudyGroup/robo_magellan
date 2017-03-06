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
# UAV State Model:
# Encapsulates UAV state and abstracts communication
# States:
# - Setpoint pose
# - local_position
# - MAV mode
# - arm

#
from datetime import datetime

# import ROS libraries
import rospy
import mavros
#from mavros.utils import *
from mavros import setpoint as SP
import mavros_msgs.msg
import mavros_msgs.srv
from sensor_msgs.msg import BatteryState

from scripts.auto_number import AutoNumber


class MODE(AutoNumber):
    """MAV MODE"""
    MANUAL = ()
    LEARNING = ()
    STEERING = ()
    HOLD = ()
    AUTO = ()
    RTL = ()
    GUIDED = ()
    INITIALISING = ()

class ARM(AutoNumber):
    """MAV ARM STATE"""
    ARMED = ()
    DISARMED = ()

class _coord:
    """Pose coordinate"""
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0

#
class UAV_State:
    """State of UAV"""
    def __init__(self):
        self.current_pose = _coord()
        self.setpoint_pose = _coord()
        self.mode = "None"
        self.arm = "None"
        self.guided = "None"
        self.timestamp = float(datetime.utcnow().strftime('%S.%f'))
        self.connection_delay = 0.0
        self.voltage = 0
        self.current = 0
        mavros.set_namespace("/mavros")

        # Subscribers
        self.local_position_sub = rospy.Subscriber(
            mavros.get_topic('local_position', 'pose'),
            SP.PoseStamped, self.__local_position_cb)
        self.setpoint_local_sub = rospy.Subscriber(
            mavros.get_topic('setpoint_raw', 'target_local'),
            mavros_msgs.msg.PositionTarget, self.__setpoint_position_cb)
        self.state_sub = rospy.Subscriber(
            mavros.get_topic('state'),
            mavros_msgs.msg.State, self.__state_cb)
        self.battery_sub = rospy.Subscriber("/mavros/battery", BatteryState, self.__battery_cb)

    def __local_position_cb(self, topic):
#        rospy.loginfo('__local_position_cb')
        self.current_pose.x = topic.pose.position.x
        self.current_pose.y = topic.pose.position.y
        self.current_pose.z = topic.pose.position.z

    def __setpoint_position_cb(self, topic):
#        rospy.loginfo('__setpoint_position_cb')
        self.setpoint_pose.x = topic.position.x
        self.setpoint_pose.y = topic.position.y
        self.setpoint_pose.z = topic.position.z

    def __state_cb(self, topic):
#        rospy.loginfo('__state_cb')
        self.__calculate_delay()
        self.mode = topic.mode
        self.guided = topic.guided
        self.arm = topic.armed

    def __battery_cb(self, data):
#        rospy.loginfo('__battery_cb')
        self.current = round(data.current, 2)
        self.voltage = round(data.voltage, 2)

    def __calculate_delay(self):
        tmp = float(datetime.utcnow().strftime('%S.%f'))
        if tmp < self.timestamp:
            # over a minute
            self.connection_delay = 60.0 - self.timestamp + tmp
        else:
            self.connection_delay = tmp - self.timestamp
        self.timestamp = tmp


    def get_mode(self):
        """Get pixhawk MAV state"""
        return self.mode

    def set_mode(self, new_mode):
        """Set pixhawk MAV state"""
        rospy.loginfo('/mavros/set_mode: '+new_mode)
        rospy.wait_for_service('/mavros/set_mode')
        if self.mode == new_mode:
            pass
        try:
            flight_mode_service = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
            is_mode_changed = flight_mode_service(custom_mode=new_mode)
        except rospy.ServiceException, e:
            rospy.loginfo(
                "Service set_mode call failed: %s. Mode %s could not be set. "
                "Check that GPS is enabled.",
                e, new_mode)
        return is_mode_changed


    def get_arm(self):
        """Get pixhawk arm state"""
        return self.arm

    def set_arm(self, new_arm):
        """Arm pixhawk"""
        rospy.loginfo('/mavros/cmd/arming: '+str(new_arm))
        rospy.wait_for_service('/mavros/cmd/arming')
        if self.arm == new_arm:
            pass
        try:
            arm_service = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            resp = arm_service(new_arm)
            rospy.loginfo(resp)
            rospy.loginfo('/mavros/cmd/arming: '+str(new_arm))
            return resp
        except rospy.ServiceException, e:
            rospy.loginfo("Service arm call failed: %s. "
                          "Attempted to set %s",
                          e, new_arm)


    def get_current_pose(self):
        """Get setpoint pose"""
        return self.current_pose

    def get_setpoint_pose(self):
        """Get setpoint pose"""
        return self.setpoint_pose


    def get_guided(self):
        """Get guided"""
        return self.guided


    def get_delay(self):
        """Get delay"""
        return self.connection_delay


    def get_current(self):
        """Get battery current"""
        return self.current

    def get_voltage(self):
        """Get battery voltage"""
        return self.voltage
