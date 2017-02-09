#!/usr/bin/env python
#
# Simple node using keyboard to adjust 'joysticks' for RC steering and throttle
# Publishes /mavros_msgs/OverrideRCIn messages
#
import sys, select, termios, tty
from numpy import interp
import rospy
from mavros_msgs.msg import OverrideRCIn
#import roslib; roslib.load_manifest('teleop_twist_keyboard')

msg = """
Keysticks: reading keys and publishing OverrideRCIn messages
------------------------------------------------------------
Throttle:
    e    increase 5%
    d    set to zero
    c    decrease 5%
    w    increase max throttle 5%
    x    decrease max throttle 5%
    q    increase min throttle 5%
    z    decrease min throttle 5%

Steering:
    j    increase servo L 5%
    k    set steering to neutral
    l    increase servo R 5%
    u    increase max L servo 5%
    m    decrease max L servo 5%
    o    increase max R servo 5%
    .    decrease max R servo 5%

any other key sets throttle and steering neutral

CTRL-C to quit
------------------------------------------------------------
"""

#settings = termios.tcgetattr(sys.stdin)
hard_limits = [1000, 1500, 2000]  # microseconds for servo signal
steering_limits = [1000, 1500, 2000]  # middle is neutral
throttle_limits = [1200, 1500, 1800]  # middle is neutral
 
steering = steering_limits[1]  # start neutral
throttle = throttle_limits[1]  # start neutral

STEP_SIZE = 0.05  # normalized step size for adjustments

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def print_vals(which):
    if which == "t":
        print "Throttle:\t %s;  Limits:\t %s" % (throttle, throttle_limits)
    elif which == "s":
        print "Steering:\t %s;  Limits:\t %s" % (steering, steering_limits)
    else:
        print "Throttle:\t %s;  Limits:\t %s" % (throttle, throttle_limits)
        print "Steering:\t %s;  Limits:\t %s" % (steering, steering_limits)

# makes a normalized adjustment to the current value and returns new value on servo scale
def adjust(current, adj_norm, limits, norm_range=[-1, 0, 1]):
    current_norm = interp(current, limits, norm_range)
    new_norm = current_norm + adj_norm
    new = interp(new_norm, norm_range, limits)
    if new < limits[0]:
        new = limits[0]
    if new > limits[2]:
        new = limits[2]
    return int(new)

#def run_sticks():


if __name__=="__main__":
    try:
        settings = termios.tcgetattr(sys.stdin)        
        print msg
        print_vals("b")

        pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)
        rospy.init_node('keysticks', anonymous=True)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # do stuff, including publish msg
            key = getKey()
            if key == 'd':
                throttle = throttle_limits[1]  # neutral    
                print_vals("t")
            elif key == 'e':
                throttle = adjust(throttle, STEP_SIZE, throttle_limits)
                print_vals("t")
            elif key == 'c':
                throttle = adjust(throttle, -STEP_SIZE, throttle_limits)
                print_vals("t")
            elif key == 'w':
                throttle_limits[2] = adjust(throttle_limits[2], STEP_SIZE, hard_limits)
                print_vals("t")
            elif key == 'x':
                throttle_limits[2] = adjust(throttle_limits[2], -STEP_SIZE, hard_limits)
                print_vals("t")
            elif key == 'q':
                throttle_limits[0] = adjust(throttle_limits[0], STEP_SIZE, hard_limits)
                print_vals("t")
            elif key == 'z':
                throttle_limits[0] = adjust(throttle_limits[0], -STEP_SIZE, hard_limits)
                print_vals("t")
            elif key == 'k':
                 steering = steering_limits[1]  # neutral
                 print_vals("s")
            elif key == 'j':
                steering = adjust(steering, -STEP_SIZE, steering_limits)
                print_vals("s")
            elif key == 'l':
                steering = adjust(steering, STEP_SIZE, steering_limits)
                print_vals("s")
            elif key == 'u':
                steering_limits[0] = adjust(steering_limits[0], -STEP_SIZE, hard_limits)
                print_vals("s")
            elif key == 'm':
                steering_limits[0] = adjust(steering_limits[0], STEP_SIZE, hard_limits)
                print_vals("s")
            elif key == 'o':
                steering_limits[2] = adjust(steering_limits[2], STEP_SIZE, hard_limits)
                print_vals("s")
            elif key == '.':
                steering_limits[2] = adjust(steering_limits[2], -STEP_SIZE, hard_limits)
                print_vals("s")
            else:
                throttle = throttle_limits[1]
                steering = steering_limits[1]
                print "Exiting keysticks"
                break
                if (key == '\x03'):  # not really needed?
                    break
            
            rc = OverrideRCIn()
            #rospy.loginfo("OverrideRCIn: (%d,%d,%d, ...)", rc.channels[0], rc.channels[1], rc.channels[2])
            rc.channels = [int(steering), 0, int(throttle), 0, 0, 0, 0, 0]
            pub.publish(rc)
            rate.sleep()

#        run_sticks()

    except rospy.ROSInterruptException:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)        
        print_vals("b")        
        #twist = Twist()
    #twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
    #twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
    #pub.publish(twist)
        
        
