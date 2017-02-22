#!/usr/bin/env python
#
# UAV Drive Model:
# Encapsulates UAV RC Control and abstracts communication
#

#
# UAV_RcControl client example:
#
# import uav_state
#
#     __UAV_State = uav_state.UAV_State()
#     __UAV_RcControl = uav_rc_control.UAV_RcControl()
#     resp1 = __UAV_State.set_mode(uav_state.MODE.MANUAL)
#     if "True" in str(resp1):
#         __UAV_RcControl.set_throttle_servo(1200,1400) #throttle and servo desired PWM value
#

import time

# ROS
import rospy

from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import CommandBool, WaypointPush, WaypointClear, WaypointSetCurrent

#
throttle_channel=2
steer_channel=0

exec_time=1 #exc time in secs

class UAV_RcControl:
    def __init__(self):
        #mavros.set_namespace("/mavros")

        # Publishers
        pubOverride = rospy.Publisher('mavros/rc/override', OverrideRCIn, queue_size=10)
        # Subscribers
        pass

    #
    # throttle: Desired PWM value
    #
    def set_throttle(throttle):
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
                pubOverride.publish(msg)
                r.sleep()


    #
    # servo: Desired PWM value
    #
    def set_servo(servo):
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
                pubOverride.publish(msg)
                r.sleep()


    #
    # throttle: Desired PWM value
    # servo: Desired PWM value
    #
    def set_throttle_servo(throttle,servo):
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
                pubOverride.publish(msg)
                r.sleep()


    #
    # Push waypoints
    #
    def push_waypoints(self, waypoints):
        try:
            service = rospy.ServiceProxy('mavros/mission/push', WaypointPush)
            resp = service(waypoints)
            rospy.loginfo(resp)
            return resp
        except rospy.ServiceException, e:
            rospy.loginfo('Service call failed: {0}'.format(e))
            return None


    #
    # Clear waypoints
    #
    def clear_waypoints(self):
        try:
            service = rospy.ServiceProxy('mavros/mission/clear', WaypointClear)
            resp = service()
            rospy.loginfo(resp)
            return resp
        except rospy.ServiceException, e:
            rospy.loginfo('Service call failed: {0}'.format(e))
            return None


    #
    # Set current
    #
#    def set_current_waypoints(self):
#        try:
#            service = rospy.ServiceProxy('mavros/mission/set_current', WaypointSetCurrent)
#            resp = service()
#            rospy.loginfo(resp)
#            return resp
#        except rospy.ServiceException, e:
#            rospy.loginfo('Service call failed: {0}'.format(e))
#            return None
