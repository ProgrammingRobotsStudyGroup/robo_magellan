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
        self.pubStateResponse = rospy.Publisher(
            TOPICS.exec_command.name, String, queue_size=10)
        self.pubStateCmd = rospy.Publisher(
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
        self.pubStateResponse.publish(self.state+","+msg+","+transition)




    #
    #
    #
    def send_message_to_state(self, state, cmd):
        self.pubStateCmd.publish(state+","+cmd)




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
        pass




