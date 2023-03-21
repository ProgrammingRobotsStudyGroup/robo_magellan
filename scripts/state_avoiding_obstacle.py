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

"""Avoiding obstacle state node"""

#
#
#

# ROS
import rospy

from uav_state import MODE as MAVMODE

import exec_comm
from exec_comm import MSG_TO_STATE
from exec_comm import MSG_TO_EXEC
from state_and_transition import STATE
from state_and_transition import TRANSITION

# Globals
this_node = None


#
#
#
def state_start():
    """Start the state"""
    rospy.loginfo('state_start %s', this_node.state_name)

    # TODO Setting mode to HOLD is precautionary.
    # Set UAV mode to hold while we get this state started
#    this_node.uav_state.set_mode(MAVMODE.HOLD.name)
#    this_node.uav_state.set_arm(True)

#    this_node.uav_state.set_mode(MAVMODE.AUTO.name)
    this_node.uav_state.set_mode(MAVMODE.MANUAL.name)
    time.sleep(0.5)
    if not this_node.uav_state.arm:
        this_node.uav_state.set_arm(True)
        time.sleep(0.5)

    rate = rospy.Rate(2) # 2 hz

    # Avoiding obstacles loop
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
        if this_node.exec_comm.cmd != MSG_TO_STATE.START.name:
            # TODO What if any transition?
            rospy.loginfo('State aborted: %s with command %s',
                          this_node.state_name, this_node.exec_comm.cmd)
            break
        if rospy.Time.now() > timeout:
            segment_timeout = True
            # TODO What's the transition?
            rospy.loginfo('State timed out: %s', this_node.state_name)
            break
        rate.sleep()

    # Put in safe mode
    this_node.uav_state.set_mode(MAVMODE.HOLD.name)
#    this_node.uav_state.set_arm(False)

    # Publish transition
    obstacle_cleared = True # TODO: Hard coded
    if obstacle_cleared:
        this_node.exec_comm.send_message_to_exec(
            MSG_TO_EXEC.DONE.name, TRANSITION.obstacle_cleared.name)




if __name__ == '__main__':
    try:
        this_node = exec_comm.StateNode(STATE.Avoiding_obstacle.name)
        this_node.start = state_start
        this_node.run_state_node()
    except rospy.ROSInterruptException:
        pass

