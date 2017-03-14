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
############################################################
#  Node to monitor magnetic wheel sensor
#  Sensor provides 2 digital outputs to UP GPIO:
#    1) moving: true if the wheel is rotating
#    2) direction: true if wheel direction is forward
#  Note: the direction signal from the sensor is not valid for the first 
#    approx 1/2 wheel rotation when starting from a stopped condition
#
#  This node publishes on topic 'wheel' a String message with values:
#    * 'Not moving'
#    * 'Moving forward'
#    * 'Moving reverse'
#    * 'Moving direction unknown'
############################################################

import rospy
import rospkg
import RPi.GPIO as GPIO
import time

from std_msgs.msg import String
       
def wheel_sensor():
    NODE_FREQ = 20  # Hz
    rospy.init_node('wheel_sensor', anonymous=True)
    pubWheel  = rospy.Publisher('wheel', String, queue_size=10)
    rate = rospy.Rate(NODE_FREQ)  

    moving_gpio_pin = 29
    direction_gpio_pin = 31
    GPIO.setmode(GPIO.BOARD)

    GPIO.setup(moving_gpio_pin, GPIO.IN)
    GPIO.setup(direction_gpio_pin, GPIO.IN)

    DIRECTION_BLACKOUT = 0.5  # seconds after standing start when direction signal is invalid

    while not rospy.is_shutdown():
        try:
            if not GPIO.input(moving_gpio_pin):
                pubWheel.publish("Not moving")
                start_time = time.time()  # gets reset every 1/NODE_FREQ sec
                elapsed_time = 0
            else:
                if elapsed_time > DIRECTION_BLACKOUT:
                    if GPIO.input(direction_gpio_pin):
                        pubWheel.publish("Moving forward")
                    else:
                        pubWheel.publish("Moving reverse")
                else:
                    pubWheel.publish("Moving direction unknown")
                    elapsed_time = time.time() - start_time  # how long have we been moving
            rate.sleep()

        except Exception, e:
            # print e
            #Revert all GPIO pins to their normal states (i.e. input = safe)
            GPIO.cleanup()
            # break 

if __name__ == '__main__':
    # Start the node
    try:
        wheel_sensor()
    except rospy.ROSInterruptException:
        #Revert all GPIO pins to their normal states (i.e. input = safe)
        GPIO.cleanup()
        pass

