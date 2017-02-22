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
# This file map a file names to an enum
# from pkg/scripts/soundmapfiles.yaml
#
# Chatty client example:
#
# import rospy
# import soundmap
# ...
#   # Play 1) a sound ref'd by token name or 2) file path.
#
#   # Play sound by token
#   pubToken = rospy.Publisher('play', String, queue_size=10)
#   pubToken.publish(soundmap.SoundMapEnum.DISARMED.name)
#
#   # Play sound by file path.
#   pubFile  = rospy.Publisher('playfile', String, queue_size=10)
#   pubFile.publish(soundfilepath)
#

from std_msgs.msg import String
import yaml
import rospkg

from auto_number import AutoNumber


class SoundMapEnum(AutoNumber):
    ARMED = ()
    AUTO = ()
    AUTONOMOUS_MODE = ()
    BATTERY_LOW = ()
    CHANGING_MODE = ()
    DISARMED = ()
    GUIDED = ()
    HOLD = ()
    INITIALIZING = ()
    LEARN = ()
    LEARNING = ()
    LINUX_SHUTDOWN = ()
    LOITER = ()
    MANUAL_MODE = ()
    MANUAL = ()
    PAUSE_MODE = ()
    RETURN_TO_LAUNCH = ()
    ROS_IS_STARTING = ()
    ROS_IS_STOPPING = ()
    SD_DRIVE = ()
    SD_NAVIGATION = ()
    SD_POWER = ()
    SEE_CONE = ()
    SELF_DIAGNOSTIC = ()
    STABILIZE = ()
    STEERING = ()
    SYSTEM_ERROR = ()
    SYSTEM_OK = ()
    TAZER = ()
    TOUCH_CONE = ()

    def __init__(self):
        self.file_name = None

    @classmethod
    def tostring(cls, val):
      for k,v in vars(cls).iteritems():
          if v==val:
              return k

    @classmethod
    def fromstring(cls, str):
        return getattr(cls, str.upper(), None)

    @classmethod
    def set_file_name(cls, str):
        file_name = str

    @classmethod
    def get_file_name(cls):
        return file_name


# Read sound name/file association file
  # Get an instance of RosPack with the default search paths
rospack = rospkg.RosPack()
  # Get the file path for rospy_tutorials
path = rospack.get_path('robo_magellan') + "/scripts/soundmapfiles.yaml"
print(path)
stream = open(path, "r")
sound_name_file_assoc = yaml.load_all(stream)
  # Associate enumerated key (token) with value (file name)
for doc in sound_name_file_assoc:
    if doc is None:
        break
    for k,v in doc.items():
        try:
            # Find the enum corresponding to the key
            SoundMapEnum.__getattr__(k).file_name = v
        except:
            print 'No enum named: '+k
        #print k, "->", v
        #pass
    #print "\n"

#print list(SoundMapEnum)
#print(SoundMapEnum.ARMED.file_name)
#print(SoundMapEnum.ARMED)
#print(SoundMapEnum.AUTO.file_name)
#print(SoundMapEnum.AUTO)
#print SoundMapEnum

