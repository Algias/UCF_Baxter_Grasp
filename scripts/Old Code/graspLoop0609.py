#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import moveit_msgs.srv
from copy import deepcopy
from moveit_msgs.srv import (
    GetPlanningScene                             
)
from moveit_msgs.msg import (
    PlanningScene,
    PlanningSceneComponents,                                                      
)
import geometry_msgs.msg
from std_msgs.msg import String
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header
import baxter_interface
import baxter_external_devices
from baxter_interface import CHECK_VERSION

class gomatlab:
    
    
    def __init__(self):
        self.final_pose = PoseStamped()
        
    def __init__(self):
        self.stop_pose = PoseStamped()
        
    def callback(self, data):
        self.stop_pose = data;
        #rospy.loginfo(rospy.get_caller_id() + " \n %s", self.stop_pose)
        
    def listen(self):
        rospy.Subscriber("/mypose1", PoseStamped, self.callback) #subs to topic
        rospy.sleep(0.2)
        
    def callback2(self, data):
        self.final_pose = data;
        #rospy.loginfo(rospy.get_caller_id() + " \n %s", self.final_pose)
        
    def listen2(self):
        rospy.Subscriber("/mypose2", PoseStamped, self.callback2)
        rospy.sleep(0.2)
class autonomousGrasper
    def __init__(self):
      moveit_commander.roscpp_initialize(sys.argv)
      rospy.init_node('graspLoop',
                    anonymous=True)
      self.robot = moveit_commander.RobotCommander()
      self.scene = moveit_commander.PlanningSceneInterface()
      self.group = moveit_commander.MoveGroupCommander("right_arm")
      self.display_trajectory_publisher = rospy.Publisher(
                                          '/move_group/display_planned_path',
                                          moveit_msgs.msg.DisplayTrajectory
                                          ,queue_size=10)
       
    def generate_Plan(self,poseInput):
            
      # # Create poses from stop pose stamped (eliminates header)
      pose_target_temp = geometry_msgs.msg.Pose()
        
      pose_target_temp.position.x = poseInput.position.x
      pose_target_temp.position.y = poseInput.position.y
      pose_target_temp.position.z = poseInput.position.z
      
      pose_target_temp.orientation.x = poseInput.orientation.x
      pose_target_temp.orientation.y = poseInput.orientation.y
      pose_target_temp.orientation.z = poseInput.orientation.z
      pose_target_temp.orientation.w = poseInput.orientation.w
    
      group.set_pose_target(pose_target_temp)
      return group.plan()
                
    def go_Init(self,grip):
    
        
      print "==== Resetting Pose to Initial Position ===="
      pose_target1 = geometry_msgs.msg.Pose()
        
      pose_target1.position.x = 0.7
      pose_target1.position.y = -0.55
      pose_target1.position.z = 0.17
      
      pose_target1.orientation.x = 1
      pose_target1.orientation.y = 0
      pose_target1.orientation.z = 0
      pose_target1.orientation.w = 0
      if not move_to_pose(robot, scene, group, display_trajectory_publisher
                 ,pose_target1):
        print "==== Movement failed, shutting down ===="
        moveit_commander.roscpp_shutdown()
    
      grip.open()
    
      print "==== Reset Complete ===="              
    
    def move_to_pose(self,poseInput):
        planTemp = generate_Plan(robot, scene, group, poseInput)
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(planTemp)
        display_trajectory_publisher.publish(display_trajectory);
        rospy.sleep(4)
        if not group.execute(planTemp):
            return False
        else:
            rospy.sleep(6)
            return True
    
    def pick_object(self, poseInput,grip):
        
            waypoints = []
            waypoints.append(poseInput)        
            (plan,fraction) = group.compute_cartesian_path(waypoints,.01,0.0,False)
            print "==== moving to object ===="
            print "Percentage of Path followed: ", fraction/1.00, " %"
            if not group.execute(plan):
                return False
            else:
                print "==== Closing gripper ===="
                rospy.sleep(6)
                grip.close()
                return True
        
    def graspLoop(self):
        

      
      ## Resets arm to initial position
      
      print "==== Gripper Initialization and Pose Subscribing ===="
      
      grip_left = baxter_interface.Gripper('left', CHECK_VERSION) ## create gripper
      grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
      Grasp = gomatlab() ## Instantiate class
      Grasp.listen()  # # listen to stop pose
      Grasp.listen2()  # # listen to final pose
      grip_right.calibrate()  # # Initial Gripper Calibration
      go_Init(robot,scene,group,display_trajectory_publisher,grip_right) 
    
      rospy.sleep(1)
    
      print "==== Ready to move to point ====" 
      
      
      while not rospy.is_shutdown():
          
        n = raw_input("==== Waiting for input: s for side and t for top q to quit...==== \n")
        
        if n == "q": # Quit the program
         break
     
        elif  n == "t": # Top grasping motion
            
            print "==== Going to top ===="
            
            if not move_to_pose(robot, scene, group, display_trajectory_publisher
                         ,Grasp.stop_pose.pose):
                print "==== Movement failed, returning to initial pose ===="
                go_Init(robot, scene, group, display_trajectory_publisher, grip_right)
                continue
            
            if not pick_object(robot, scene, group, display_trajectory_publisher
                        ,Grasp.final_pose.pose,grip_right):
                print "==== Movement failed, returning to initial pose ===="
                go_Init(robot, scene, group, display_trajectory_publisher, grip_right)
                continue            
    
            print "==== Returning to top ===="
            
            if not move_to_pose(robot, scene, group, display_trajectory_publisher
                         ,Grasp.stop_pose.pose):
                print "==== Movement failed, returning to initial pose ===="
                go_Init(robot, scene, group, display_trajectory_publisher, grip_right)
                continue
            
            print "==== Going init ===="
            
            go_Init(robot, scene, group, display_trajectory_publisher,grip_right)
            
        elif n == "s": # Side grasping motion
            
            print "==== Going to side ===="
           
            if not move_to_pose(robot, scene, group, display_trajectory_publisher
                         ,Grasp.stop_pose.pose):
                print "==== Movement failed, returning to initial pose ===="
                go_Init(robot, scene, group, display_trajectory_publisher, grip_right)
                continue
            if not pick_object(robot, scene, group, display_trajectory_publisher
                        ,Grasp.final_pose.pose,grip_right):
                print "==== Movement failed, returning to initial pose ===="
                go_Init(robot, scene, group, display_trajectory_publisher, grip_right)
                continue  
           
            print "==== Returning to top ===="
            
            if not move_to_pose(robot, scene, group, display_trajectory_publisher
                         ,Grasp.stop_pose.pose):
                print "==== Movement failed, returning to initial pose ===="
                go_Init(robot, scene, group, display_trajectory_publisher, grip_right)
                continue
            print "==== Going init ===="
            go_Init(robot, scene, group, display_trajectory_publisher,grip_right)
        
        else: 
            print "==== Invalid command ===="
            continue    
        
        rate = rospy.Rate(1)  # 1hz
        rate.sleep()
    
     
      # # When finished shut down moveit_commander.
      moveit_commander.roscpp_shutdown()


  print "==== STOPPING ===="

if __name__ == '__main__':
  try:
    graspLoop()
  except rospy.ROSInterruptException:
    pass
