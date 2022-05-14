#!/usr/bin/env python
from numpy import angle
import rospy
import RPi.GPIO as GPIO
from aubo_control.msg import armCmd
from time import sleep
GPIO.setmode(GPIO.BCM)

# Notes area:



LED = 24
stepper = [21,5,19]
direction = [20,6,26]
GPIO.setup(stepper, GPIO.OUT)
GPIO.setup(direction, GPIO.OUT)
prev_angle = [0,0,0]

#stepper settings
angle_deg = 1.8
angle_rad = angle_deg * 0.01745329251
step_mode = 1 # 1 for fullstep, 2 for half , etc 
step_count = [0,0,0]
step_position = [0,0,0]
tolerance = angle_rad * 3
for i in range (3):
    step_position[i] = step_count[i] * step_mode * angle_rad

def step(i,j,k):

    if i == 1:
        GPIO.output(direction[0],True)
    elif i == -1:
        GPIO.output(direction[0],False)

    if j == 1:
        GPIO.output(direction[1],True)
    elif j == -1:
        GPIO.output(direction[1],False)

    if k == 1:
        GPIO.output(direction[2],True)
    elif k == -1:
        GPIO.output(direction[2],False)
    
    for h in range(3):
        GPIO.output(stepper[h],True)
        GPIO.output(stepper[h],False)

    step_count[0] += i
    step_count[1] += j
    step_count[2] += k

   # sleep(0.0006)
    

def armCmd_callback(data):
    global prev_angle

    if  data.angle[0] > step_position[0] and (data.angle[0] - step_position[0]) > tolerance  : # for angle 1
        i=1
    else:
        i=-1

    if data.angle[1] < step_position[1] and (data.angle[1] - step_position[1]) > tolerance :     # for angle 2
        j=1
    else:
        j=-1

    if prev_angle[2] < data.angle[2] and (data.angle[2] - step_position[2]) > tolerance :     # for angle 3
        k=1
    else:
        k=-1

    step(i,j,k)
    prev_angle = data.angle
    print(step_count)
 

def listner():
    rospy.Subscriber("arduino/armCmd",armCmd,armCmd_callback)
    rospy.spin()


if __name__=='__main__':

    rospy.init_node('pi_4',anonymous=False)
    listner()
