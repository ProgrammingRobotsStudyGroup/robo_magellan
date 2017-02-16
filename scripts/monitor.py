#!/usr/bin/env python
#
import rospy
import rospkg

from std_msgs.msg import String

# Enum maps to sound name
import soundmap

def monitor():
    pubFile  = rospy.Publisher('playfile', String, queue_size=10)
    pubToken = rospy.Publisher('play', String, queue_size=10)
    rospy.init_node('monitor', anonymous=True)
    rate = rospy.Rate(0.3) # some hz
    # Get an instance of RosPack with the default search paths
    rospack = rospkg.RosPack()
    # Get the file path for rospy_tutorials
    basepath = rospack.get_path('robo_magellan') + "/scripts/sounds/"
    #basepath = '~/catkin_ws/src/robo_magellan/scripts/sounds/'

    play_file = False
    while not rospy.is_shutdown():
        # TODO Are we connected to px? Is MAVROS up?
        # TODO Get MAV mode
        # TODO Test code. Play 1) a sound ref'd by token name or 2) file path.
        if play_file:
            soundfile = basepath + soundmap.SoundMapEnum.ARMED.file_name
            rospy.loginfo(soundfile)
            pubFile.publish(soundfile)
        else:
            soundToken =  soundmap.SoundMapEnum.DISARMED.name
            rospy.loginfo(soundToken)
            pubToken.publish(soundToken)
        play_file = not play_file
        rate.sleep()

if __name__ == '__main__':
    # Start the node
    try:
        monitor()
    except rospy.ROSInterruptException:
        pass

