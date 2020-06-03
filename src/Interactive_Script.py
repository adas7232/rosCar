# -*- coding: utf-8 -*-
"""
Created on Sun Apr 12 18:02:36 2020

@author: adas
"""
start_time = 0
# dc motor drive 
from gpiozero import PWMOutputDevice
import time
from time import sleep
import RPi.GPIO as gpio
import pdb

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
gpio.setup(trigger,gpio.OUT)   # trigger
gpio.setup(echoR,gpio.IN)      # Echo Right
gpio.setup(echoC,gpio.IN)      # Echo Center
gpio.setup(echoL,gpio.IN)      # Echo Left
## Front  DC motor control
# Motor A, Left Side gpio CONSTANTS
pwm_drive_front_left = 23      # ENA - H-Bridge enable pin (white)
drive_fwd_front_left = 25      # IN1 - Forward Drive (yellow)
drive_rvs_front_left = 24      # IN2 - Reverse Drive (green)
# Motor B, Right Side gpio CONSTANTS
pwm_drive_front_right = 27     # ENB - H-Bridge enable pin (blue)
drive_fwd_front_right = 22     # IN1 - Forward Drive (brown)
drive_rvs_front_right = 10     # IN2 - Reverse Drive (red)
# Setup the gpio pins as Output
gpio.setup(pwm_drive_front_left,gpio.OUT)
gpio.setup(drive_fwd_front_left,gpio.OUT)
gpio.setup(drive_rvs_front_left,gpio.OUT)
gpio.setup(pwm_drive_front_right,gpio.OUT)
gpio.setup(drive_fwd_front_right,gpio.OUT)
gpio.setup(drive_rvs_front_right,gpio.OUT)
# Setup two gpio driver pins as PWM
pwm_front_left = gpio.PWM(pwm_drive_front_left, 1000)
pwm_front_right = gpio.PWM(pwm_drive_front_right, 1000)
#
# def fdrive(front_left, front_right):
#     if front_left <0: 
#         gpio.output(drive_fwd_front_left, False)
#         gpio.output(drive_rvs_front_left, True)  
#     else:
#         gpio.output(drive_fwd_front_left, True) 
#         gpio.output(drive_rvs_front_left, False) 
#     if front_right <0: 
#         gpio.output(drive_fwd_front_right, False)
#         gpio.output(drive_rvs_front_right, True) 
#     else:
#         gpio.output(drive_fwd_front_right, True)
#         gpio.output(drive_rvs_front_right, False) 
       
#     pwm_front_left.start(abs(front_left))
#     pwm_front_right.start(abs(front_right))


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
pwm_rear_left = gpio.PWM(pwm_drive_rear_left, 1000)
pwm_rear_right = gpio.PWM(pwm_drive_rear_right, 1000)

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

indx = 0

def measuredDistance(echo):      
    distance_sum = 0    
    maxLoop = 2           
    for indx in range(maxLoop):
        # Allow module to settle and makeit false if not already
        gpio.output(trigger, False)
        time.sleep(0.001)
        # Send 10us pulse to trigger
        gpio.output(trigger, True)
        time.sleep(0.00001)
        gpio.output(trigger, False)   
        # Center                      
        while gpio.input(echo)==0:                    
            start_time = time.time() 
        else:
            start_time = time.time()             
        while gpio.input(echo)==1:              
            stop_time = time.time()    
        else:
            stop_time = time.time()             
        # Calculate pulse length
        distance_ini = (stop_time-start_time) * 17150 # elapsed_time * 34300 / 2 in cm             
        distance_sum = distance_sum + distance_ini            
    distance = distance_sum/maxLoop
    return distance
    
def main():
    while True:
        dist_c = measuredDistance(echoC)
        print("Ultrasonic Measurement")
        print("Distance from the center is :", dist_c, " cm")        
        if dist_c > 40:        
            print("Just drive forward")
            drive(40, 40, 40, 40)                       
        else:
            print("Wait, there is something in the front!")
            all_stop()
            sleep(2)
            print("Backing up a little")
            drive(-50, -50, -50, -50)
            sleep(1.5)
            all_stop()
            print("Thinking about turning: Right or Left?")
            sleep(3)
            print("Decided to turn right")
            drive(70, -70, 70, -70)
            sleep(2)
            all_stop()
            sleep(3)
        
       
if __name__ == "__main__":
    """ This is executed when run from the command line """    
    try:
       main() 
    except KeyboardInterrupt:
        gpio.cleanup()
    finally:  
        gpio.cleanup() 
