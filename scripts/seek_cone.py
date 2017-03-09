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
class Args():
    # Typically less than 1 unless the range isn't responsive
    throttle_factor = 1.0
    steering_factor = 1.0
    conf_decay_factor = 0.90
    prev_pos_confs = []

args = Args()

def update_prev_poses():
#    if(len(args.prev_pos_confs) == 0):
#      return

    new_pos_confs = []
    for (prev_pose, confidence) in args.prev_pos_confs:
        confidence *= args.conf_decay_factor
        new_pos_confs.append((prev_pose, confidence))

    new_pos_confs = sorted(new_pos_confs, key=lambda pose: pose[1], reverse=True)
    args.prev_pos_confs = new_pos_confs[0:16]
    
def getConfFromOldFrames(pose):
    matching_poses = ()
    x1 = pose.x - pose.w/2
    x2 = pose.x + pose.w/2
    confs = []
    for (prev_pose, prev_conf) in args.prev_pos_confs:
      old_x1 = prev_pose.x - prev_pose.w/2
      old_x2 = prev_pose.x + prev_pose.w/2
      # Find overlap
      if(old_x2 < x1 or old_x1 > x2):
        continue
      # We assume we cannot get closer very fast. Also if we are moving away
      # from cone, we don't use the confidence
      if(prev_pose.y > pose.y + pose.h or pose.y > prev_pose.y):
        continue
      # We only use real depth if it is available
      if(prev_pose.z > 0 and pose.z > 0):
          if(prev_pose.z > pose.z + pose.d or prev_pose.z + prev_pose.d < pose.z):
            continue
      # We have overlap so we must get the confidence
      # Best is to scale the confidence by overlap area but for now, let's use
      # just the confidence
      confs.append(prev_conf)
      
    if(len(confs)):
      return sum(confs)/len(confs)
      
    return 0.0 
      
def seek_cone(loc):
    # Compute confidence for each hull by area and h distance
    maxArea = max(pose.area for pose in loc.poses)
    new_pos_confs = []
    update_prev_poses()
    for pose in loc.poses:
      # Need to figure out appropriate weightage for area and distance
      # Scale distance as farther objects will use less pixels
      pd = 1 + (pose.x/80.0)**2 + (pose.y/120.0)**2
      # Find this cone among cones from previous frames and use the confidence
      confidence = 1/pd + pose.area/(4.0*maxArea) + getConfFromOldFrames(pose)
      new_pos_confs.append((pose, confidence))
    
    # Sort the new list by confidence and descending
    new_pos_confs = sorted(new_pos_confs, key=lambda pose: pose[1], reverse=True)
    args.prev_pos_confs.extend(new_pos_confs)
    (cone_loc, confidence) = new_pos_confs[0]

    rospy.loginfo('Confidence (%d, %d) = %f' % (cone_loc.x, cone_loc.y, confidence))
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
        
