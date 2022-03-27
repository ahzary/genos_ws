#!/usr/bin/env python
import rospy
import RPi.GPIO as GPIO
from aubo_control.msg import armCmd

GPIO.setmode(GPIO.BCM)
LED = 24
GPIO.setup(LED, GPIO.OUT)

def armCmd_callback(data):
    if data.angle[0] > 1.14 :
        GPIO.output(LED,True)
    else:
        GPIO.output(LED,False)


def listner():
    rospy.init_node('pi_4',anonymous=False)
    rospy.Subscriber("arduino/armCmd",armCmd,armCmd_callback)
    rospy.spin()
if __name__=='__main__':
    listner()
