#!/usr/bin/env python
#
# Node using cone_finder/location messages and
# Publishes /mavros_msgs/OverrideRCIn messages
#
import sys, argparse, rospy
from mavros_msgs.msg import OverrideRCIn
from robo_magellan.msg import location_msgs as Locations
from cone_code import ConeSeeker

msg = """
cone_seeker: reading cone_finder location messages and 
             publishing OverrideRCIn messages
"""

#settings = termios.tcgetattr(sys.stdin)
hard_limits = [1000, 1500, 2000]  # microseconds for servo signal
steering_limits = [1135, 1435, 1735]  # middle is neutral
# throttle_limits = [1200, 1500, 1800]  # middle is neutral
throttle_limits = [1650, 1650, 2000]  # fwd range only; for testing; middle is NOT neutral

# We will get angle between +pi/2 to -pi/2 for steering
# We will get 480 pixels range for throttle but should limit this
class Args(object):
    # Typically less than 1 unless the range isn't responsive
    throttle_factor = 3.0
    steering_factor = 1.0

args = Args()

def drive_to_cone(loc):
    cs = ConeSeeker()
    # sadj = [-1. to 1.], tadj = [0 to 1.]
    (cl, conf, sadj, tadj) = cs.seek_cone(loc.poses)

    # Assuming equal range spread
    steering_range = (steering_limits[2] - steering_limits[0])/2
    throttle_range = (throttle_limits[2] - throttle_limits[0])/2
    
    steering = steering_limits[1] + args.steering_factor*sadj*steering_range
    throttle = throttle_limits[1] + args.throttle_factor*tadj*throttle_range

    rc = OverrideRCIn()
    rc.channels = [int(steering), 0, int(throttle), 0, 0, 0, 0, 0]
    pub.publish(rc)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Drive to cone found')
    parser.add_argument('--throttle_factor', '-t', default=1.0, type=float,
                        help='Throttle step size factor')
    parser.add_argument('--steering_factor', '-s', default=1.0, type=float,
                        help='Steering step size factor')
    parser.parse_args(rospy.myargv(sys.argv[1:]), args)
    try:
        pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)
        rospy.init_node('cone_seeker', anonymous=True)
        rospy.Subscriber('/cone_finder/locations', Locations, drive_to_cone)
        rospy.loginfo('cone_seeker init.')
        rc = OverrideRCIn()
        rc.channels = [1435, 0, 1500, 0, 0, 0, 0, 0]
        pub.publish(rc)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

