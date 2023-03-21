#!/usr/bin/env python3
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

from std_msgs.msg import Bool
from mavros_msgs.msg import Mavlink

import uav_state

def callback(msg):
    if msg.msgid == 180:
        rospy.loginfo(rospy.get_caller_id() + ' msgid %s detected', msg.msgid)
        pubTouch.publish(True)
        if not UAV.set_mode("HOLD"):
            rospy.logwarn(rospy.get_caller_id() + ' set_mode(HOLD) FAILED')
        

def touch_detector():
    rospy.init_node('touch_detector', anonymous=True)
    rospy.Subscriber('/mavlink/from', Mavlink, callback)
    global pubTouch
    pubTouch  = rospy.Publisher('touch', Bool, queue_size=10)
    rate = rospy.Rate(20)  # hz

    global UAV  # must be a better way
    UAV = uav_state.UAV_State()  

    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    # Start the node
    try:
        touch_detector()
    except rospy.ROSInterruptException:
        pass

