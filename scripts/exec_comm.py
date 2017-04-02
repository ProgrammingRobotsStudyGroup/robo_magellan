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

from robo_magellan.msg import to_exec as to_exec
from robo_magellan.msg import to_state as to_state

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
    """Communication between exec and states"""
    #
    #
    #
    def __init__(self, state, state_msg_cb=None, exec_msg_cb=None):
        self.cmd = None
        self.state = state
        self.transition = None

        # Publishers
        self.pub_state_response = rospy.Publisher(
            TOPICS.exec_command.name, to_exec, queue_size=10)
        self.pub_state_cmd = rospy.Publisher(
            TOPICS.state_command.name, to_state, queue_size=10)

        # Subscribers
        if state_msg_cb != None:
            rospy.Subscriber(TOPICS.state_command.name, to_state, state_msg_cb)
        if exec_msg_cb != None:
            rospy.Subscriber(TOPICS.exec_command.name, to_exec, exec_msg_cb)




    #
    #
    #
    def send_message_to_exec(self, inp_cmd, inp_transition):
        """Send message to exec"""
        toexec = to_exec()
        toexec.state = self.state
        toexec.cmd = inp_cmd
        toexec.transition = inp_transition
        rospy.loginfo("State: %s; Cmd: %s; Transition: %s",
            toexec.state,
            toexec.cmd,
            toexec.transition
            )
        self.pub_state_response.publish(toexec)




    #
    #
    #
    def send_message_to_state(self, inp_state, inp_cmd):
        """Send message to state"""
        tostate = to_state()
        tostate.state = inp_state
        tostate.cmd = inp_cmd
        self.pub_state_cmd.publish(tostate)




class StateNode():
    """State node common code"""
    #
    #
    #
    def __init__(self, state_name):
        """Constructor"""
        self.state_name = state_name
        self.exec_comm = None
        self.uav_state = None
        self.uav_control = None
        self.start = self._state_start
        self.pause = self._state_pause
        self.reset = self._state_reset
        self.custom_startup = None


    #
    # State subscriber of exec commands
    #
    def cmd_callback(self, data):
        """State subscriber of exec commands"""

        # State is returned. If message state is our state, cmd is updated.
        if data.state == self.exec_comm.state:
            rospy.loginfo(
                '%s cmd_callback: %s; Cmd: %s',
                rospy.get_caller_id(),
                data.state,
                data.cmd)
            self.exec_comm.cmd = data.cmd
            # Handle start, reset, pause, etc.
            if self.exec_comm.cmd == MSG_TO_STATE.START.name:
                self.start()
            elif self.exec_comm.cmd == MSG_TO_STATE.RESET.name:
                self.reset()
            elif self.exec_comm.cmd == MSG_TO_STATE.PAUSE.name:
                self.pause()
            else:
                rospy.logwarn(
                    'Invalid cmd: %s', data.data)


    #
    # Start the state
    # This is a do nothing state expected to be
    # overridden by an actual implementation
    #
    def _state_start(self):
        """Reset the state"""
        # Set UAV mode to hold while we get this state started
#        self.uav_state.set_mode(MAVMODE.HOLD.name)
#        self.uav_state.set_arm(False)
        pass


    #
    # Reset the state
    # For safety, for now set to HOLD
    #
    def _state_reset(self):
        """Reset the state"""
        # Set UAV mode to hold while we get this state started
#        self.uav_state.set_mode(MAVMODE.HOLD.name)
#        self.uav_state.set_arm(False)
        pass


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
        if None != self.custom_startup:
            self.custom_startup()
        rate = rospy.Rate(10) # 10 hz
        while not rospy.is_shutdown():
            rate.sleep()

