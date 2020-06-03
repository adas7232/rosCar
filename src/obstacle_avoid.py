#!/usr/bin/env python
import os, sys
import rospy
from geometry_msgs.msg import Pose2D
import time
from time import sleep
import RPi.GPIO as gpio
import pdb
#
dist_front = 0
dist_left = 0
dist_right = 0
## board and PIN allocation
# Setup pi board as BCM
gpio.setmode(gpio.BCM)
gpio.setwarnings(False)
## Ultrasonic sensor 
trigger = 4 # blue
echoR    = 17 # white
echoC    = 18 # white
echoL    = 15 # white
#
gpio.setup(trigger,gpio.OUT)  # trigger
gpio.setup(echoR,gpio.IN)      # Echo Right
gpio.setup(echoC,gpio.IN)      # Echo Center
gpio.setup(echoL,gpio.IN)      # Echo Left
## Front  DC motor control
# Motor A, Left Side gpio CONSTANTS
pwm_drive_front_left = 23        # ENA - H-Bridge enable pin (white)
drive_fwd_front_left = 25    # IN1 - Forward Drive (yellow)
drive_rvs_front_left = 24    # IN2 - Reverse Drive (green)
# Motor B, Right Side gpio CONSTANTS
pwm_drive_front_right = 27        # ENB - H-Bridge enable pin (blue)
drive_fwd_front_right = 22    # IN1 - Forward Drive (brown)
drive_rvs_front_right = 10   # IN2 - Reverse Drive (red)
# Setup the gpio pins as Output
gpio.setup(pwm_drive_front_left,gpio.OUT)
gpio.setup(drive_fwd_front_left,gpio.OUT)
gpio.setup(drive_rvs_front_left,gpio.OUT)
gpio.setup(pwm_drive_front_right,gpio.OUT)
gpio.setup(drive_fwd_front_right,gpio.OUT)
gpio.setup(drive_rvs_front_right,gpio.OUT)
# Setup two gpio driver pins as PWM
pwm_front_left = gpio.PWM(pwm_drive_front_left, 100)
pwm_front_right = gpio.PWM(pwm_drive_front_right, 100)
#
## Rear  DC motor control
# Motor A, Left Side gpio CONSTANTS
pwm_drive_rear_left = 26       # ENA - H-Bridge enable pin (yellow)
drive_fwd_rear_left = 19    # IN1 - Forward Drive (yellow)
drive_rvs_rear_left = 13    # IN2 - Reverse Drive (green)
# Motor B, Right Side gpio CONSTANTS
pwm_drive_rear_right = 21        # ENB - H-Bridge enable pin (purple)
drive_fwd_rear_right = 20    # IN1 - Forward Drive (orange)
drive_rvs_rear_right = 16   # IN2 - Reverse Drive (grey)
# Setup the gpio pins as Output
gpio.setup(pwm_drive_rear_left,gpio.OUT)
gpio.setup(drive_fwd_rear_left,gpio.OUT)
gpio.setup(drive_rvs_rear_left,gpio.OUT)
gpio.setup(pwm_drive_rear_right,gpio.OUT)
gpio.setup(drive_fwd_rear_right,gpio.OUT)
gpio.setup(drive_rvs_rear_right,gpio.OUT)
# Setup two gpio driver pins as PWM
pwm_rear_left = gpio.PWM(pwm_drive_rear_left, 100)
pwm_rear_right = gpio.PWM(pwm_drive_rear_right, 100)

def drive(front_left, front_right, rear_left, rear_right):
    if front_left <0: 
        gpio.output(drive_fwd_front_left, False)
        gpio.output(drive_rvs_front_left, True)  
    else:
        gpio.output(drive_fwd_front_left, True) 
        gpio.output(drive_rvs_front_left, False) 
    if front_right <0: 
        gpio.output(drive_fwd_front_right, False)
        gpio.output(drive_rvs_front_right, True) 
    else:
        gpio.output(drive_fwd_front_right, True)
        gpio.output(drive_rvs_front_right, False) 
       
    pwm_front_left.start(abs(front_left))
    pwm_front_right.start(abs(front_right))
    # 
    if rear_left <0: 
        gpio.output(drive_fwd_rear_left, False)
        gpio.output(drive_rvs_rear_left, True)  
    else:
        gpio.output(drive_fwd_rear_left, True) 
        gpio.output(drive_rvs_rear_left, False) 
    if rear_right <0: 
        gpio.output(drive_fwd_rear_right, False)
        gpio.output(drive_rvs_rear_right, True) 
    else:
        gpio.output(drive_fwd_rear_right, True)
        gpio.output(drive_rvs_rear_right, False) 
       
    pwm_rear_left.start(abs(rear_left))
    pwm_rear_right.start(abs(rear_right))

def all_stop():
    gpio.output(drive_fwd_front_left, False) 
    gpio.output(drive_rvs_front_left, False) 
    gpio.output(drive_fwd_front_right, False) 
    gpio.output(drive_rvs_front_right, False) 
    gpio.output(drive_fwd_rear_left, False) 
    gpio.output(drive_rvs_rear_left, False) 
    gpio.output(drive_fwd_rear_right, False) 
    gpio.output(drive_rvs_rear_right, False) 
    pwm_rear_left.stop()
    pwm_rear_right.stop()
    pwm_front_left.stop()
    pwm_front_right.stop()   
    
def callback(msg):
    global dist_front, dist_left, dist_right
    # rospy.loginfo("Received a POSE2D message!")
    rospy.loginfo_throttle(2, "distance in cm: [%f,%f,%f]"%(msg.x, msg.y, msg.theta))
    dist_front = round(msg.x, 2)
    dist_right = round(msg.y, 2)
    dist_left = round(msg.theta, 2)    
    
    #movebot()

def movebot():
    global dist_front, dist_left, dist_right
    flc = 33 # front left command
    frc = 30 # front right command
    rlc = 33 # rear left command
    rrc = 30 # rear right command
    dist_lim_front = 40
    dist_lim_right = 10
    dist_lim_left = 10
    if dist_front > dist_lim_front and dist_left > dist_lim_left and dist_right > dist_lim_right:        
        print("Just drive forward ...")
        drive(flc, frc, rlc, rrc)                       
    elif dist_front > dist_lim_front or dist_left > dist_lim_left or dist_right > dist_lim_right:
        print("Wait, there is something on my way ...")
        all_stop()
        sleep(2)  
        print("Backing up a little ...")
        drive(-flc, -frc, -rlc, -rrc)
        sleep(0.5)
        all_stop()
        sleep(1.5)      
        print("Thinking about turning: Right or Left? ...")
        if dist_right > dist_left:        
            print("Decided to turn right ...")
            drive(flc+10, -frc-10, rlc+10, -rrc-10)
            sleep(1.5)
            all_stop()
            sleep(2)
        elif dist_left > dist_right:        
            print("Decided to turn left ...")
            drive(-flc-10, frc+10, -rlc-10, rrc+10)
            sleep(1.5)
            all_stop()
            sleep(2)
        else:
            print("Backing up more ...")
            drive(-flc, -frc, -rlc, -rrc)
            sleep(1.5)
            all_stop()
            sleep(1.5)

if __name__ == '__main__':
    try:
        rospy.init_node('dist_listener', anonymous=True)
        rospy.Subscriber('distance_msg', Pose2D, callback)
        rate = rospy.Rate(5)
        # spin() simply keeps python from exiting until this node is stopped        
        while not rospy.is_shutdown():
            movebot() 
            rate.sleep()             
        #movebot()
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
    except KeyboardInterrupt:
        gpio.cleanup()
    finally:  
        gpio.cleanup() 
    
        