#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header

#import for gripper
import baxter_interface
import baxter_external_devices
from baxter_interface import CHECK_VERSION




class gomatlab:
    
    
    def __init__(self):
        self.final_pose = PoseStamped()
        
    def __init__(self):
        self.stop_pose = PoseStamped()
        
    def callback(self,data):
        self.stop_pose = data;
        rospy.loginfo(rospy.get_caller_id() + " \n %s", self.stop_pose)
        
    def listen(self):
        rospy.Subscriber("/mypose1", PoseStamped, self.callback)
        rospy.sleep(0.2)
        
    def callback2(self,data):
        self.final_pose = data;
        rospy.loginfo(rospy.get_caller_id() + " \n %s", self.final_pose)
        
    def listen2(self):
        rospy.Subscriber("/mypose2", PoseStamped, self.callback2)
        rospy.sleep(0.2)
        
        
        
        

def move_group_python_interface_tutorial():
  print "============ Starting test code"
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('move_group_python_interface_tutorial',
                  anonymous=True)


  robot = moveit_commander.RobotCommander()
  group = moveit_commander.MoveGroupCommander("right_arm")


  display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory)


  
  print "============ Generating plan 1"
  
  pose_target1 = geometry_msgs.msg.Pose()
    
  pose_target1.position.x = 0.7
  pose_target1.position.y = -0.55
  pose_target1.position.z = 0.17
  
  pose_target1.orientation.x = 1
  pose_target1.orientation.y = 0
  pose_target1.orientation.z = 0
  pose_target1.orientation.w = 0

  group.set_pose_target(pose_target1)
  plan1 = group.plan()
  
  
  
  
  print "============ Visualizing plan1"
  display_trajectory = moveit_msgs.msg.DisplayTrajectory()
  display_trajectory.trajectory_start = robot.get_current_state()
  display_trajectory.trajectory.append(plan1)
  display_trajectory_publisher.publish(display_trajectory);
  rospy.sleep(4)



  group.execute(plan1)


  ## When finished shut down moveit_commander.
  moveit_commander.roscpp_shutdown()

  ## END_TUTORIAL

  print "============ STOPPING"


if __name__=='__main__':
  try:
    move_group_python_interface_tutorial()
  except rospy.ROSInterruptException:
    pass

