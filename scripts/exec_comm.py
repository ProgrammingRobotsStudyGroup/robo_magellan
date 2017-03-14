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

"""Encapsulates communication between state node and exec"""
#
# Encapsulates communication between state node and exec
# TODO Define our own messages

from auto_number import AutoNumber

# ROS
import rospy
from std_msgs.msg import String

import uav_control
import uav_state
from uav_state import MODE as MAVMODE


class MSG_TO_STATE(AutoNumber):
    """The permitted messages"""
    START = ()
    RESET = ()
    PAUSE = ()

class MSG_TO_EXEC(AutoNumber):
    """The permitted messages"""
    START_EXEC = ()
    DONE = ()

class TOPICS(AutoNumber):
    """exec and state topic names"""
    state_command = ()
    exec_command = ()




class ExecComm():
    #
    #
    #
    def __init__(self, state, state_msg_cb=None, exec_msg_cb=None):
        self.cmd = None
        self.state = state
        self.transition = None

        # Publishers
        self.pub_state_response = rospy.Publisher(
            TOPICS.exec_command.name, String, queue_size=10)
        self.pub_state_cmd = rospy.Publisher(
            TOPICS.state_command.name, String, queue_size=10)

        # Subscribers
        if state_msg_cb != None:
            rospy.Subscriber(TOPICS.state_command.name, String, state_msg_cb)
        if exec_msg_cb != None:
            rospy.Subscriber(TOPICS.exec_command.name, String, exec_msg_cb)




    #
    #
    #
    def send_message_to_exec(self, msg, transition):
        self.pub_state_response.publish(self.state+","+msg+","+transition)




    #
    #
    #
    def send_message_to_state(self, state, cmd):
        self.pub_state_cmd.publish(state+","+cmd)




    #
    #
    #
    def parse_msg_to_state(self, msg):
        """Parse a message sent to a state"""
        rospy.loginfo(rospy.get_caller_id() + ' parse_msg_to_state: %s', msg)
        # Handle start, reset, pause, etc.
        msg_token_list = msg.split(",")
        the_state = msg_token_list[0]
        cmd = msg_token_list[1]
        #rospy.loginfo('the_state: '+the_state+'  cmd: '+cmd)
        if the_state == self.state:
            # TODO Command should be in MSG_TO_STATE
            self.cmd = cmd
        return the_state




    #
    # Parses a state's message to exec
    # msg contains:
    #  msg from state
    #  msg text
    #  next transition
    #
    def parse_msg_to_exec(self, msg):
        """Parse a message sent to a exec"""
        rospy.loginfo(rospy.get_caller_id() + ' parse_msg_from_state: %s', msg)
        msg_token_list = msg.split(",")
        self.state = msg_token_list[0]
        self.cmd = msg_token_list[1]
        self.transition = msg_token_list[2]



class StateNode():
    #
    #
    #
    def __init__(self, state_name):
        """Constructor"""
        self.state_name = state_name
#         self.exec_comm = ExecComm(self.state_name, self.cmd_callback)
        self.exec_comm = None
        self.uav_state = None
        self.uav_control = None
        self.start = self._state_start
        self.pause = self._state_pause
        self.reset = self._state_reset


    #
    # Exec command listener callback
    #
    def cmd_callback(self, data):
        """Exec command listener callback"""
        # Parses the message
        # State is returned. If message state is our state, cmd is updated.
        the_state = self.exec_comm.parse_msg_to_state(data.data)
    
        if the_state == self.exec_comm.state:
            rospy.loginfo(rospy.get_caller_id() + ' cmd_callback: %s', data.data)
            # Handle start, reset, pause, etc.
            if self.exec_comm.cmd == MSG_TO_STATE.START.name:
                self.start()
            elif self.exec_comm.cmd == MSG_TO_STATE.RESET.name:
                self.reset()
            elif self.exec_comm.cmd == MSG_TO_STATE.PAUSE.name:
                self.pause()
            else:
                rospy.logwarn('Invalid cmd: '+data.data)


    #
    # Start the state
    # This is a do nothing state expected to be
    # overridden by an actual implementation
    #
    def _state_start(self):
        """Reset the state"""
        # Set UAV mode to hold while we get this state started
        self.uav_state.set_mode(MAVMODE.HOLD.name)
        self.uav_state.set_arm(False)


    #
    # Reset the state
    # For safety, for now set to HOLD
    #
    def _state_reset(self):
        """Reset the state"""
        # Set UAV mode to hold while we get this state started
        self.uav_state.set_mode(MAVMODE.HOLD.name)
        self.uav_state.set_arm(False)


    #
    # Pause the state
    #
    def _state_pause(self):
        """Pause the state"""
        # Set UAV mode to hold while we get this state started
        self.uav_state.set_mode(MAVMODE.HOLD.name)
        self.uav_state.set_arm(False)


    def run_state_node(self):
        """Start State node"""

        rospy.loginfo('State node starting: %s', self.state_name)
        rospy.init_node(self.state_name, anonymous=False)

        self.exec_comm = ExecComm(self.state_name, self.cmd_callback)
        self.uav_state = uav_state.UAV_State()
        self.uav_control = uav_control.UAV_Control()

        rate = rospy.Rate(10) # 10 hz
        while not rospy.is_shutdown():
            rate.sleep()
    
