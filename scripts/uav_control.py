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
"""This module encapsulates UAV RC Control and abstracts communication."""

import threading
import time

# ROS
import rospy

from std_msgs.msg import Int16
# MAVROS
from mavros_msgs.msg import OverrideRCIn
#from mavros_msgs.srv import CommandBool
from mavros_msgs.msg import WaypointList, WaypointReached

from mavros_msgs.srv import WaypointPush, WaypointPull, WaypointClear, WaypointSetCurrent
#from mavros_msgs.srv import *
from mavros_msgs.srv import ParamGet

from mavros_msgs.srv import CommandLong
#from mavros_msgs.srv import ParamSet, SetMode
#TODO Missing import from mavros_msgs.srv import WaypointGOTO
#from mavros.mission import *

# Globals
THROTTLE_CHANNEL = 2
STEER_CHANNEL = 0

EXEC_TIME = 1 #exc time in secs

class UAV_Control:
    """UAV WP and Manual Control"""
    def __init__(self):
        self.lock = threading.Lock()
        #mavros.set_namespace("/mavros")
        self.waypoint_list = None
        self.current_waypoint = None

        # Proxies
        rospy.wait_for_service('/mavros/param/get')
        self.svc_get_param = rospy.ServiceProxy('/mavros/param/get', ParamGet)

        rospy.wait_for_service('/mavros/mission/push')
        self.svc_push_waypoints = rospy.ServiceProxy('/mavros/mission/push', WaypointPush)

        rospy.wait_for_service('/mavros/mission/pull')
        self.svc_pull_waypoints = rospy.ServiceProxy('/mavros/mission/pull', WaypointPull)

        rospy.wait_for_service('/mavros/mission/clear')
        self.svc_clear_waypoints = rospy.ServiceProxy('mavros/mission/clear', WaypointClear)

        rospy.wait_for_service('/mavros/mission/set_current')
        self.svc_set_current_waypoint = rospy.ServiceProxy(
            'mavros/mission/set_current',
            WaypointSetCurrent)

        rospy.wait_for_service('/mavros/cmd/command')
        self._srv_cmd_long = rospy.ServiceProxy(
            '/mavros/cmd/command', CommandLong, persistent=True)


        # Publishers
        self.pub_rc_override = rospy.Publisher(
            'mavros/rc/override', OverrideRCIn, queue_size=10)

        # Subscribers
        self.sub_waypoints = rospy.Subscriber(
            "/mavros/mission/waypoints",
            WaypointList, self.__waypoints_cb)

        self.sub_current = rospy.Subscriber(
            "/mavros/mission/reached", WaypointReached,
            self.__current_cb)


    def __waypoints_cb(self, topic):
        self.lock.acquire()
        try:
            self.waypoint_list = topic.waypoints
        finally:
            self.lock.release()


    def __current_cb(self, waypoint_reached):
        rospy.loginfo('__current_cb: ')
        rospy.loginfo('__current_cb: %d', waypoint_reached.wp_seq)
        self.lock.acquire()
        try:
            self.current_waypoint = waypoint_reached.wp_seq
            wp = self.waypoint_list[self.current_waypoint]
            cone_alt = wp.z_alt
            (q, r) = divmod(cone_alt, 2)
            if r == 1:
                rospy.set_param("/CONE_ON_GRASS", True)
                rospy.loginfo('Cone is on grass')
            else:
                rospy.set_param("/CONE_ON_GRASS", False)
                rospy.loginfo('Cone is not on grass')
        except:
            rospy.loginfo("Failed to get current waypoint details")
            # Make a safe bet
            rospy.set_param("/CONE_ON_GRASS", True)
        finally:
            self.lock.release()


    def print_waypoints(self):
        """Prints Pixhawk waypoints to stdout"""
        for seq, waypoint in enumerate(self.waypoint_list):
            print (' seq: '+str(seq)+
                   ' waypoint.is_current: '+str(waypoint.is_current)+
                   ' waypoint.autocontinue: '+str(waypoint.autocontinue)+
                   ' waypoint.frame: '+str(waypoint.frame)+
                   ' waypoint.command: '+str(waypoint.command)+
                   ' waypoint.param1: '+str(waypoint.param1)+
                   ' waypoint.param2: '+str(waypoint.param2)+
                   ' waypoint.param3: '+str(waypoint.param3)+
                   ' waypoint.param4: '+str(waypoint.param4)+
                   ' waypoint.x_lat: '+str(waypoint.x_lat)+
                   ' waypoint.y_long: '+str(waypoint.y_long)+
                   ' waypoint.z_alt: '+str(waypoint.z_alt)+
                   '')

    #
    # throttle: Desired PWM value
    #
    def set_throttle(self, throttle):
        """Set throttle"""
        rospy.loginfo('mavros/rc/override, throttle')
        msg = OverrideRCIn()
        msg.channels[THROTTLE_CHANNEL] = throttle        # Desired PWM value
        rospy.loginfo(msg)
        self.pub_rc_override.publish(msg)


    #
    # servo: Desired PWM value
    #
    def set_servo(self, servo):
        """Set servo"""
        rospy.loginfo('mavros/rc/override, servo')
        msg = OverrideRCIn()
        msg.channels[STEER_CHANNEL] = servo        # Desired PWM value
        rospy.loginfo(msg)
        self.pub_rc_override.publish(msg)


    #
    # throttle: Desired PWM value
    # servo: Desired PWM value
    #
    def set_throttle_servo(self, throttle, servo):
        """Set throttle AND servo"""
        rospy.loginfo('mavros/rc/override, throttle and servo')
        msg = OverrideRCIn()
        msg.channels[THROTTLE_CHANNEL] = throttle  # Desired PWM value
        msg.channels[STEER_CHANNEL] = servo        # Desired PWM value
        rospy.loginfo(msg)
        self.pub_rc_override.publish(msg)


    #
    # Push waypoints
    #
    def push_waypoints(self, waypoints):
        """Push waypoints to Pixhawk"""
        rospy.loginfo('/mavros/mission/push')
        try:
            resp = self.svc_push_waypoints(waypoints)
            rospy.loginfo(resp)
            return resp
        except rospy.ServiceException, err:
            rospy.loginfo(
                "Service push_waypoints call failed: %s.",
                err)
            return None

    #
    # Pull waypoints
    # Request update waypoint list.
    #
    def pull_waypoints(self):
        """Request update waypoint list"""
        rospy.loginfo('/mavros/mission/pull')
        try:
            resp = self.svc_pull_waypoints()
            rospy.loginfo('success: '+str(resp.success)+' wp_received: '+str(resp.wp_received))
            return resp
        except rospy.ServiceException, err:
            rospy.loginfo(
                "Service pull_waypoints call failed: %s.",
                err)
            return None


    #
    # Clear waypoints
    #
    def clear_waypoints(self):
        """Clear waypoints"""
        rospy.loginfo('/mavros/mission/clear')
        try:
            resp = self.svc_clear_waypoints()
            rospy.loginfo(resp)
            return resp
        except rospy.ServiceException, err:
            rospy.loginfo(
                "Service clear_waypoints call failed: %s.",
                err)
            return None


    #
    # Set current waypoint
    #
    def set_current_waypoint(self, idx):
        """Set current wp"""
        rospy.loginfo('/mavros/mission/set_current: '+str(idx))
        try:
            resp = self.svc_set_current_waypoint(idx)
            rospy.loginfo(resp)
            return resp
        except rospy.ServiceException, err:
            rospy.loginfo(
                "Service set_current_waypoint call failed: %s. Index %d could not be set. "
                "Check that GPS is enabled.",
                err, idx)
            return None


    #
    # Goto wp
    #
#    def goto_waypoint(self, args):
#        """Go to WP"""
#        wp = Waypoint(
#            frame=args.frame,
#            command=args.command,
#            param1=args.param1,
#            param2=args.param2,
#            param3=args.param3,
#            param4=args.param4,
#            x_lat=args.x_lat,
#            y_long=args.y_long,
#            z_alt=args.z_alt
#        )
#        try:
#            service = rospy.ServiceProxy('mavros/mission/goto', WaypointGOTO)
#            resp = service(waypoint=wp)
#            rospy.loginfo(resp)
#            return resp
#        except rospy.ServiceException, e:
#            rospy.loginfo('Service call failed: {0}'.format(e))
#            return None

    def get_param_int(self, name):
        """Get parameter value from UAV"""
        ret = None
        try:
            ret = self.svc_get_param(param_id=name)
            return ret.value.integer
        except rospy.ServiceException as ex:
            rospy.logerr(ex)
            return None

    def send_mavros_cmd(self, bool1, msgid, bool2, p0, p1, p2, p3, p4, p5, p6):
        """Send a mavros command"""
        rospy.loginfo("/mavros/cmd/command/ %s %s %s %s %s %s %s %s %s %s",
                      str(bool1), str(msgid), 
                      str(bool2), str(p0), 
                      str(p1), str(p2), str(p3), str(p4), str(p5), str(p6))
        self._srv_cmd_long(bool1, msgid, bool2, p0, p1, p2, p3, p4, p5, p6)

