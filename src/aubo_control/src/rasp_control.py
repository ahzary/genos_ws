#!/usr/bin/env python
from numpy import angle
import rospy
import RPi.GPIO as GPIO
from aubo_control.msg import armCmd
from time import sleep
from numpy import pi
GPIO.setmode(GPIO.BCM)

# Notes area:



LED = 24
stepper = [21,5,19]
direction = [20,6,26]
GPIO.setup(stepper, GPIO.OUT)
GPIO.setup(direction, GPIO.OUT)
prev_angle = [0,0,0]
error = [0.0,0.0,0.0]

#stepper settings
angle_deg = 1.8
angle_rad = angle_deg * 0.01745329251
step_mode = 1 # 1 for fullstep, 2 for half , etc 
step_count = [0,0,0]

step_position = [0,0,0]
tolerance = angle_rad * 3
for i in range (3):
    step_position[i] = step_count[i] * step_mode * angle_rad

def step(step_count):
    steps_done = [0,0,0]
    while steps_done != step_count :
        for h in range(3):
            if step_count[h] != steps_done[h]:
                if step_count[h] > steps_done[h] :
                        GPIO.output(direction[h],True)
                        steps_done[h] += 1
                else:
                        GPIO.output(direction[h],False)
                        steps_done[h] += -1
                GPIO.output(stepper[h],True)
                GPIO.output(stepper[h],False)
        sleep(0.0006)
    

def armCmd_callback(data):
    global prev_angle

    for h in range(3):
        error[h] = data.angle[h] - step_position[h]
        step_count[h] =int( (error[h]/2*pi) * 360 * step_mode / 1.8 )

    step(step_count)
    #prev_angle = data.angle
    print(step_count)
 

def listner():
    rospy.Subscriber("arduino/armCmd",armCmd,armCmd_callback)
    rospy.spin()


if __name__=='__main__':

    rospy.init_node('pi_4',anonymous=False)
    listner()
