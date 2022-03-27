#!/usr/bin/env python
import rospy
import RPi.GPIO as GPIO
from aubo_control.msg import armCmd
from time import sleep
GPIO.setmode(GPIO.BCM)
LED = 24
stepper = 21
direction = 20
GPIO.setup(stepper, GPIO.OUT) 
GPIO.setup(direction, GPIO.OUT) 
prev_angle = 0
    
def step(data):
    GPIO.output(stepper,True)
    GPIO.output(stepper,False)
    GPIO.output(direction,data)
    sleep(0.0006)

def armCmd_callback(data):
    global prev_angle
    if prev_angle < data.angle[0]:
        step(True)
    if prev_angle > data.angle[0]:
        step(False)
    prev_angle = data.angle[0]
    print(prev_angle)
   
def listner():
    rospy.init_node('pi_4',anonymous=False)
    rospy.Subscriber("arduino/armCmd",armCmd,armCmd_callback)
    rospy.spin()
if __name__=='__main__':
    listner()
