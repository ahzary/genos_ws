#!/usr/bin/env python
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import time

group_name = "cyborg_sys_101"
h=0

class move_group(object):
  group = moveit_commander.MoveGroupCommander(group_name)

  def __init__(self):
    super(move_group,self).__init__()
    group = self.group
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial',
                    anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    
    #group = moveit_commander.MoveGroupCommander(group_name)
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                  moveit_msgs.msg.DisplayTrajectory,
                                                  queue_size=20)
    planning_frame = group.get_planning_frame()
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
    group.go(joint_goal, wait=False)
    print(joint_goal)


def main():
  genos = move_group()
  print("planning now__________________________")
  h=0
  while True:
    genos.plan_joint(0.01 * h,1.5 * pi, 2)
#    print(h)
    h = h + 1
    time.sleep(0.2)
#    genos.plan_joint(0,pi,0)
#    time.sleep(0.2)
  print("done")


if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass


