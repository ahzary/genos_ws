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


class move_group(object):

  group_name = "arm_group"
  group = moveit_commander.MoveGroupCommander(group_name)

  def __init__(self):
    super(move_group,self).__init__()
    group = self.group
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_planner',
                    anonymous=False)
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
  def speed_set(self,speed):

    #speed_control = rospy.Publisher('/move_group/motion_plan_request',moveit_msgs.msg.MotionPlanRequest,queue_size=10)
    ##motionPlanMsg = moveit_msgs.msg.MotionPlanRequest
    #motionPlanMsg.max_velocity_scaling_factor = 0.01
    #speed_control.publish(motionPlanMsg)
    group = self.group
    group.set_max_velocity_scaling_factor(speed)

  def plan_joint(self,i,j,k):
    #limit check
    if i > 1.557 or i < 0  or j < -1.57 or j > 0 or k < -1.57 or k > 0 :
      print("plan is not within limits")
    else:
      #planning
      group = self.group
      group.clear_pose_targets()
      joint_goal = group.get_current_joint_values()
      joint_goal = [i,j,k]
      group.go(joint_goal, wait=True)
      print(joint_goal)

  def pose_plan(self):
    group = self.group
    group.clear_pose_targets()
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.w = 1.0
    pose_target.position.x = 2
    pose_target.position.y = 1.5
    pose_target.position.z = 1.5
    group.set_pose_target(pose_target)
    group.go(wait = True)

  def named_pose(self,pose):
    group = self.group
    group.clear_pose_targets()
    group.set_named_target(pose)
    group.go(wait = True)



def main():
  print("creating class")
  genos = move_group()
  print("class creation done__________________________")

  print("T_pose")
  genos.speed_set(1)
  genos.named_pose("T_pose")
  print("flex")
  genos.named_pose("flex")

  print("T_pose")
  genos.speed_set(0.5)
  genos.named_pose("T_pose")
  print("flex")
  genos.named_pose("flex")
  
  
  genos.speed_set(0.3)
  print("going to zero postion")
  genos.named_pose("down")
  print("pose planning done")

  print("done")


if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass


