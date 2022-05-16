#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import time
from aubo_control.msg import emgmsg
#-----------------


group_name = "arm_group"

class move_group(object):
  group = moveit_commander.MoveGroupCommander(group_name)
  
  def __init__(self):
    #super(move_group,self).__init__()
    group = self.group
    moveit_commander.roscpp_initialize(sys.argv)
    
    #ros node setup
    self.sub = rospy.Subscriber("arduino/emg", emgmsg, self.emg_cb)

    robot = moveit_commander.RobotCommander()
    #scene = moveit_commander.PlanningSceneInterface()
    
    #group = moveit_commander.MoveGroupCommander(group_name)
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                  moveit_msgs.msg.DisplayTrajectory,
                                                  queue_size=3)
    #planning_frame = group.get_planning_frame()
    #print ("============ Reference frame: %s" % planning_frame)
    group_names = robot.get_group_names()
    #print ("============ Robot Groups:", robot.get_group_names())
    #print ("============ Printing robot state")
    #print (robot.get_current_state())
    #print ("")

  def plan_joint(self,i,j,k):
    group = self.group
    joint_goal = group.get_current_joint_values()
    joint_goal = [i,j,k]
    #joint_goal[0] = i
    #joint_goal[1] = j
    #joint_goal[2] = k
    group.go(joint_goal, wait=False)
    print(joint_goal)
    time.sleep(0.06)

  def emg_cb(self,data):
    i = data.data[0]
    j = data.data[1]
    k = data.data[2]
    self.plan_joint(i,j,k)



def main():
  genos = move_group()
  #print("go to zero position")
  #genos.plan_joint(0,0,0)
  #time.sleep(3)
  print("class configured__________________________")
  rospy.spin()
# for h in range(6000):
#    start = rospy.get_time()
#    seconds = rospy.get_time()
#    genos.plan_joint(0,0,0.01 * h)
#    rospy.sleep(0.1)
#    print(seconds - start)

#  while not rospy.is_shutdown():
#    genos.plan_joint(i,j,k)
#    time.sleep(2.175)
#    print("done")


if __name__ == '__main__':
  try:
    rospy.init_node('emg_listener',anonymous=False)
    main()
    rospy.spin()
  except rospy.ROSInterruptException:
    pass 

