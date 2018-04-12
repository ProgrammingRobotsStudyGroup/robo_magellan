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
#------------------------------------------------------------------------------ 
# Monitors state of the kill switch by monitoring an UP GPIO input, which
#   is connected to a contact closure from the kill switch module.
#
# Topics subscribed: None
# Topics published:
#   kill_sw_enabled: std_msgs/Bool
#------------------------------------------------------------------------------
#
import RPi.GPIO as GPIO
import time
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import String

def kill_sw_mon():
    rospy.init_node('kill_sw_mon', anonymous=True)
    pub = rospy.Publisher('kill_sw_enabled', Bool, queue_size=10)

    # first push of kill switch is going to start the state machine
    global pub_exec_simple
    pub_exec_simple = rospy.Publisher("exec_cmd_simple", String, queue_size = 10)
    global once
    once = False

    rate = rospy.Rate(20) 

    gpio_pin = 7
    GPIO.setmode(GPIO.BOARD)

    # assumes the pin is pulled up be external resistor
    GPIO.setup(gpio_pin, GPIO.IN)

    # find the starting state of the input pin
    n_qual = 5  # number of successive readings that must match to qualify
    count = 0
    last_val = False
    start = time.time()
    while count < n_qual:
        if (time.time() - start) > 10:
            break
        val = GPIO.input(gpio_pin) == 0
        if val == last_val:
            count += 1
            last_val = val
        else:
            count = 0  # reset
            last_val = val
        time.sleep(0.1)  # pause between reads

    if count >= n_qual:
        kill_sw_ok = val  # the value of n_qual consecutive reads
        #   print "Initial value is: %s" % val
    else:
        kill_sw_ok = False;
        print "Initial value not found; count = %s" % count
        # TODO need better error handling?

    if not rospy.is_shutdown():
        time.sleep(0.2)        
        pub.publish(kill_sw_ok)     # publish initial state
        rospy.loginfo(kill_sw_ok)

    while not rospy.is_shutdown():
        try:        
            if not kill_sw_ok:
                # Use falling edge detection to see if pin is pulled 
                #   low to avoid repeated polling
                GPIO.wait_for_edge(gpio_pin, GPIO.FALLING)
                time.sleep(0.1)  # wait for sw bounce
                if not GPIO.input(gpio_pin):  # re-read to confirm
                    kill_sw_ok = True
                    pub.publish(kill_sw_ok)
                    rospy.loginfo(kill_sw_ok)
                    pub_exec_simple.publish("START_EXEC")
                    rospy.loginfo("kill_sw_mon: Pubishing START_EXEC")
            else:
                GPIO.wait_for_edge(gpio_pin, GPIO.RISING)
                time.sleep(0.1)  # wait for sw bounce
                if GPIO.input(gpio_pin):
                    kill_sw_ok = False
                    pub.publish(kill_sw_ok)
                    rospy.loginfo(kill_sw_ok)
            rate.sleep()
        except Exception, e:
            #Revert all GPIO pins to their normal states (i.e. input = safe)
            GPIO.cleanup()    
    
if __name__ == '__main__':
    try:
        kill_sw_mon()
    except rospy.ROSInterruptException:
        #Revert all GPIO pins to their normal states (i.e. input = safe)
        GPIO.cleanup()
        pass
