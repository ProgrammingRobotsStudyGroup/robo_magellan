#!/usr/bin/env python
#
# Node using cone_finder/location messages and
# Publishes /mavros_msgs/OverrideRCIn messages
#
import sys, argparse, rospy, math
from mavros_msgs.msg import OverrideRCIn
from cone_finder.msg import location_msgs as Locations

msg = """
cone_seeker: reading cone_finder location messages and 
             publishing OverrideRCIn messages
"""

#settings = termios.tcgetattr(sys.stdin)
hard_limits = [1000, 1500, 2000]  # microseconds for servo signal
steering_limits = [1135, 1435, 1735]  # middle is neutral
# throttle_limits = [1200, 1500, 1800]  # middle is neutral
throttle_limits = [1650, 1650, 1800]  # fwd range only; for testing; middle is NOT neutral
 
# We will get angle between +pi/2 to -pi/2 for steering
# We will get 480 pixels range for throttle but should limit this
class Args(object):
    # Typically less than 1 unless the range isn't responsive
    throttle_factor = 1.0
    steering_factor = 1.0

args = Args()

def seek_cone(loc):
    # Sort the poses by y distance to get the nearest cone
    poses = sorted(loc.poses, key=lambda loc: loc.y)
    cone_loc = poses[0]

    steering = steering_limits[1]
    # Steer if not in front
    if(cone_loc.x < -20 or cone_loc.x > 20):
        # Sin(theta)
        z = math.sqrt(cone_loc.x*cone_loc.x + cone_loc.y*cone_loc.y)
        #steering = steering + args.steering_factor*500*cone_loc.x/z
        steering = steering + args.steering_factor*2*cone_loc.x
        
    # Slowest approach to cone
    throttle = throttle_limits[1] + 20
    # Use real depth when available for throttle
    if(cone_loc.z > 0):
        # Real depth is in mm and maximum would probably be less than 6m
        if(cone_loc.z > 300):
            throttle = throttle + args.throttle_factor*(cone_loc.z - 300)/20
    else:
        y = cone_loc.y - 40
        if(y > 0):
            throttle = throttle + args.throttle_factor*(y)

    #-- test with fixed throttle to start
    throttle = 1675

    # Everything must be bounded    
    if(steering > steering_limits[2]):
        steering = steering_limits[2]
    if(steering < steering_limits[0]):
        steering = steering_limits[0]
    if(throttle > throttle_limits[2]):
        throttle = throttle_limits[2]
    if(throttle < throttle_limits[0]):
        throttle = throttle_limits[0]

    rc = OverrideRCIn()
    rc.channels = [int(steering), 0, int(throttle), 0, 0, 0, 0, 0]
    pub.publish(rc)

if __name__=="__main__":
    parser = argparse.ArgumentParser(description='Drive to cone found')
    parser.add_argument('--throttle_factor', '-t', default=1.0, type=float, 
                         help='Throttle step size factor')    
    parser.add_argument('--steering_factor', '-s', default=1.0, type=float,
                         help='Steering step size factor')
    parser.parse_args(rospy.myargv(sys.argv[1:]), args)
    try:
        pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)
        rospy.init_node('cone_seeker', anonymous=True)
        rospy.Subscriber('/cone_finder/locations', Locations, seek_cone)
        rospy.loginfo('cone_seeker init.')
        rc = OverrideRCIn()
        rc.channels = [1435,0,1500,0,0,0,0,0]
        pub.publish(rc)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
        
