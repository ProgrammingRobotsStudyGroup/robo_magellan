#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2017, Robot Garden
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
import rospkg
from std_msgs.msg import String
import os
import commands
import soundmap

def callback_file(data):
    rospy.loginfo(rospy.get_caller_id() + ' Play file: %s', data.data)
    os.system('paplay '+data.data)
    #x = commands.getoutput(cmd)
    #rospy.loginfo(rospy.get_caller_id() + 'Result: ', x)

def callback_token(data):
    rospy.loginfo(rospy.get_caller_id() + ' Play token: %s', data.data)
    # Convert token to file name
    try:
        #   Get an instance of RosPack with the default search paths
        rospack = rospkg.RosPack()
        # get the file path for rospy_tutorials
        basepath = rospack.get_path('robo_magellan')
        # Find the enum by token, create path, play
        fn = soundmap.SoundMapEnum.__getattr__(data.data).file_name
        path = basepath + '/scripts/sounds/' + fn
        print(path)
        os.system('paplay '+path)
        #obj = soundmap.SoundMapEnum._member_map_[data.data]
        #rospy.loginfo(obj)
        #rospy.loginfo(obj.file_name)
        #os.system('paplay '+obj.file_name)
    except:
        rospy.loginfo('No enum named: '+data.data)


def chatty():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('chatty', anonymous=True)

    rospy.Subscriber('playfile', String, callback_file)
    rospy.Subscriber('play', String, callback_token)

    rospy.spin()

if __name__ == '__main__':
    chatty()
