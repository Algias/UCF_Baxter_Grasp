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
        #rospy.loginfo(rospy.get_caller_id() + " \n %s", self.stop_pose)
        
    def listen(self):
        rospy.Subscriber("/mypose1", PoseStamped, self.callback)
        rospy.sleep(0.2)
        
    def callback2(self,data):
        self.final_pose = data;
        #rospy.loginfo(rospy.get_caller_id() + " \n %s", self.final_pose)
        
    def listen2(self):
        rospy.Subscriber("/mypose2", PoseStamped, self.callback2)
        rospy.sleep(0.2)
        
        
        
        

def move_group_python_interface_tutorial():
  print "============ Starting test code"
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('move_group_python_interface_tutorial',
                  anonymous=True)


  robot = moveit_commander.RobotCommander()
  scene = moveit_commander.PlanningSceneInterface()
  
  group = moveit_commander.MoveGroupCommander("right_arm")
  #group.set_planner_id("RRTStarkConfigDeault")
  group.set_goal_orientation_tolerance(0.01)
  group.set_goal_position_tolerance(0.01)

  

  grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
  Grasp = gomatlab()
  Grasp.listen()  ## listen to stop pose
  Grasp.listen2() ## listen to final pose
  grip_right.calibrate()
  grip_right.open()
    


  ## We create this DisplayTrajectory publisher which is used below to publish
  ## trajectories for RVIZ to visualize.
  display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory)



  
  print "============ Generating plan - initial pose ===================="
  
  pose_init1 = geometry_msgs.msg.Pose()
    
  pose_init1.position.x = 0.7
  pose_init1.position.y = -0.55
  pose_init1.position.z = 0.17
  pose_init1.orientation.x = 1
  pose_init1.orientation.y = 0
  pose_init1.orientation.z = 0
  pose_init1.orientation.w = 0
  
  group.set_pose_target(pose_init1)
  plani = group.plan()
  group.execute(plani)
  rospy.sleep(8)
  group.clear_pose_targets()
  
  print "============ Generating plan - stop pose======================="
  pose_target1 = geometry_msgs.msg.Pose()
    
  pose_target1.position.x = Grasp.stop_pose.pose.position.x
  pose_target1.position.y = Grasp.stop_pose.pose.position.y
  pose_target1.position.z = Grasp.stop_pose.pose.position.z
  pose_target1.orientation.x = Grasp.stop_pose.pose.orientation.x
  pose_target1.orientation.y = Grasp.stop_pose.pose.orientation.y
  pose_target1.orientation.z = Grasp.stop_pose.pose.orientation.z
  pose_target1.orientation.w = Grasp.stop_pose.pose.orientation.w

  group.set_pose_target(pose_target1)
  plan1 = group.plan()
  group.execute(plan1)
  rospy.sleep(8)
  group.clear_pose_targets()
  
  print "============ Generating plan - final pose======================="
  pose_target2 = geometry_msgs.msg.Pose()
  pose_target2.position.x = Grasp.final_pose.pose.position.x
  pose_target2.position.y = Grasp.final_pose.pose.position.y
  pose_target2.position.z = Grasp.final_pose.pose.position.z 
  pose_target2.orientation.x = Grasp.final_pose.pose.orientation.x
  pose_target2.orientation.y = Grasp.final_pose.pose.orientation.y
  pose_target2.orientation.z = Grasp.final_pose.pose.orientation.z
  pose_target2.orientation.w = Grasp.final_pose.pose.orientation.w

  group.set_pose_target(pose_target2)
  plan2 = group.plan()
  group.execute(plan2)
  rospy.sleep(8)
  grip_right.close()
  rospy.sleep(5)
  group.clear_pose_targets()
  
    
  print "============ Generating plan - back to stop pose================="
  group.set_pose_target(pose_target1)
  plan3 = group.plan()
  group.execute(plan3)
  rospy.sleep(8)
  group.clear_pose_targets()
  
  print "============ Generating plan - back to initial pose==============="
  group.set_pose_target(pose_init1)
  plani2 = group.plan()
  group.execute(plani2)
  rospy.sleep(5)
  group.clear_pose_targets()
  
  


  ## When finished shut down moveit_commander.
  #moveit_commander.roscpp_shutdown()

  ## END_TUTORIAL

  print "============ STOPPING"


if __name__=='__main__':
  try:
      while not rospy.is_shutdown():
          
        n = raw_input("==== enter to continue :  q to quit...==== \n")
        if n == "q":  # Quit the program
            break         
        else: move_group_python_interface_tutorial()
  except rospy.ROSInterruptException:
    pass

