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
# UAV Drive Model:
# Encapsulates UAV RC Control and abstracts communication
#

#
# UAV_Control client example:
#
# import uav_state
#
#     __UAV_State = uav_state.UAV_State()
#     __UAV_Control = uav_control.UAV_Control()
#     resp1 = __UAV_State.set_mode(uav_state.MODE.MANUAL)
#     if "True" in str(resp1):
#         __UAV_Control.set_throttle_servo(1200,1400) #throttle and servo desired PWM value
#

import time

# ROS
import rospy

# MAVROS
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import CommandBool 
from mavros_msgs.srv import WaypointPush, WaypointPull, WaypointClear, WaypointSetCurrent #, WaypointGOTO
from mavros_msgs.srv import ParamGet, ParamSet, SetMode

# Globals
throttle_channel=2
steer_channel=0

exec_time=1 #exc time in secs

class UAV_Control:
    def __init__(self):
        #mavros.set_namespace("/mavros")

        # Proxies
        rospy.wait_for_service('/mavros/param/get')
        self.get_param = rospy.ServiceProxy('/mavros/param/get', ParamGet)

        rospy.wait_for_service('/mavros/mission/push')
        self.push_waypoints = rospy.ServiceProxy('/mavros/mission/push', WaypointPush)

        rospy.wait_for_service('/mavros/mission/pull')
        self.pull_waypoints = rospy.ServiceProxy('/mavros/mission/pull', WaypointPull)

        rospy.wait_for_service('/mavros/mission/clear')
        self.clear_waypoints = rospy.ServiceProxy('mavros/mission/clear', WaypointClear)

        rospy.wait_for_service('/mavros/mission/set_current')
        self.set_current_waypoint = rospy.ServiceProxy('mavros/mission/set_current', WaypointSetCurrent)

        # Publishers
        self.pubOverride = rospy.Publisher('mavros/rc/override', OverrideRCIn, queue_size=10)
        # Subscribers

        pass

    #
    # throttle: Desired PWM value
    #
    def set_throttle(self, throttle):
        rospy.loginfo('mavros/rc/override, throttle')
        r = rospy.Rate(10) #10hz
        msg = OverrideRCIn()
        start = time.time()
        flag=True #time flag
        msg.channels[steer_channel]=servo        # Desired PWM value
        while not rospy.is_shutdown() and flag:
            sample_time=time.time()
            if ((sample_time - start) > exec_time):
                flag=False
                rospy.loginfo(msg)
                self.pubOverride.publish(msg)
                r.sleep()


    #
    # servo: Desired PWM value
    #
    def set_servo(self, servo):
        rospy.loginfo('mavros/rc/override, servo')
        r = rospy.Rate(10) #10hz
        msg = OverrideRCIn()
        start = time.time()
        flag=True #time flag
        msg.channels[steer_channel]=servo        # Desired PWM value
        while not rospy.is_shutdown() and flag:
            sample_time=time.time()
            if ((sample_time - start) > exec_time):
                flag=False
                rospy.loginfo(msg)
                self.pubOverride.publish(msg)
                time.sleep(0.05)
                #r.sleep()


    #
    # throttle: Desired PWM value
    # servo: Desired PWM value
    #
    def set_throttle_servo(self, throttle,servo):
        rospy.loginfo('mavros/rc/override, throttle and servo')
        r = rospy.Rate(10) #10hz
        msg = OverrideRCIn()
        start = time.time()
        flag=True #time flag
        msg.channels[throttle_channel]=throttle  # Desired PWM value
        msg.channels[steer_channel]=servo        # Desired PWM value
        while not rospy.is_shutdown() and flag:
            sample_time=time.time()
            if ((sample_time - start) > exec_time):
                flag=False
                rospy.loginfo(msg)
                self.pubOverride.publish(msg)
                r.sleep()


    #
    # Push waypoints
    #
    def push_waypoints(self, waypoints):
        rospy.loginfo('/mavros/mission/push')
        try:
            resp = self.push_waypoints(waypoints)
            rospy.loginfo(resp)
            return resp
        except rospy.ServiceException, e:
            rospy.loginfo('Service call failed: {0}'.format(e))
            return None

    #
    # Pull waypoints
    #
    def pull_waypoints(self):
        rospy.loginfo('/mavros/mission/pull')
        try:
            resp = self.pull_waypoints()
            rospy.loginfo(resp)
            return resp
        except rospy.ServiceException, e:
            rospy.loginfo('Service call failed: {0}'.format(e))
            return None


    #
    # Clear waypoints
    #
    def clear_waypoints(self):
        rospy.loginfo('/mavros/mission/clear')
        try:
            resp = self.clear_waypoints()
            rospy.loginfo(resp)
            return resp
        except rospy.ServiceException, e:
            rospy.loginfo('Service call failed: {0}'.format(e))
            return None


    #
    # Set current wp
    #
    def set_current_waypoint(self, idx):
        rospy.loginfo('/mavros/mission/set_current: '+str(idx))
        try:
            resp = self.set_current_waypoint(idx)
            rospy.loginfo(resp)
            return resp
        except rospy.ServiceException, e:
            rospy.loginfo('Service call failed: {0}'.format(e))
            return None


    #
    # Goto wp
    #
#    def goto_waypoint(self, wp):
#        try:
#            service = rospy.ServiceProxy('mavros/mission/goto', WaypointSetCurrent)
#            resp = service(idx)
#            rospy.loginfo(resp)
#            return resp
#        except rospy.ServiceException, e:
#            rospy.loginfo('Service call failed: {0}'.format(e))
#            return None

    def get_param_int(self,name):
        ret = None
        try:
            ret = self.get_param(param_id = name)
            return ret.value.integer
        except rospy.ServiceException as ex:
            rospy.logerr(ex)
            return None

