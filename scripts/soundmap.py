#!/usr/bin/env python
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
import enum
import yaml
import rospkg

class AutoNumber(enum.Enum):
    def __new__(cls):
        value = len(cls.__members__) + 1
        obj = object.__new__(cls)
        obj._value_ = value
        return obj


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

