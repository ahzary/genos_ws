#!/usr/bin/env python
import rospy
import RPi.GPIO as GPIO
from aubo_control.msg import armCmd
from time import sleep
GPIO.setmode(GPIO.BCM)
LED = 24
stepper = [21,5,19]
direction = [20,6,26]
GPIO.setup(stepper, GPIO.OUT)
GPIO.setup(direction, GPIO.OUT)
prev_angle = [0,0,0]


def step(i,j,k):

    for h in range(3):
        GPIO.output(stepper[h],True)
        GPIO.output(stepper[h],False)
        GPIO.output(direction[h],i)
    sleep(0.0006)
    

def armCmd_callback(data):
    global prev_angle

    if prev_angle[0] < data.angle[0] : # for angle 1
        i=True
    else:
        i=False
    if prev_angle[1] < data.angle[1] : # for angle 2
        j=True
    else:
        j=False
    if prev_angle[2] < data.angle[2] : # for angle 3
        k=True
    else:
        k=False
    step(i,j,k)
    prev_angle = data.angle
    print(prev_angle)
 

def listner():
    rospy.Subscriber("arduino/armCmd",armCmd,armCmd_callback)
    rospy.spin()


if __name__=='__main__':

    rospy.init_node('pi_4',anonymous=False)
    listner()
