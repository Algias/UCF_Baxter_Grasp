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
        
class autonomousGrasper:
    def __init__(self):
      moveit_commander.roscpp_initialize(sys.argv)
      rospy.init_node('move_group_python_interface_tutorial',
                    anonymous=True)
      self.display_trajectory = moveit_msgs.msg.DisplayTrajectory()

      self.robot = moveit_commander.RobotCommander()
      self.scene = moveit_commander.PlanningSceneInterface()
      self.group = moveit_commander.MoveGroupCommander("right_arm")
      self.display_trajectory_publisher = rospy.Publisher(
                                          '/move_group/display_planned_path',
                                          moveit_msgs.msg.DisplayTrajectory
                                          ,queue_size=10)
      self.grip_left = baxter_interface.Gripper('left', CHECK_VERSION) ## create gripper
      self.grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
      self.group.set_planner_id("RRTConnectkConfigDefault")
      self.group.set_goal_orientation_tolerance(0.01)
      self.group.set_goal_position_tolerance(0.01)
      self.Grasp = gomatlab() ## Instantiate class
      self.group.allow_replanning(1)
      self.group.set_num_planning_attempts(5)


       
    def generate_Plan(self,poseInput):
      self.group.clear_pose_targets()

      # # Create poses from stop pose stamped (eliminates header)
      pose_target_temp = geometry_msgs.msg.Pose()
        
      pose_target_temp.position.x = poseInput.position.x
      pose_target_temp.position.y = poseInput.position.y
      pose_target_temp.position.z = poseInput.position.z
      
      pose_target_temp.orientation.x = poseInput.orientation.x
      pose_target_temp.orientation.y = poseInput.orientation.y
      pose_target_temp.orientation.z = poseInput.orientation.z
      pose_target_temp.orientation.w = poseInput.orientation.w
    
      self.group.set_pose_target(pose_target_temp)
      return self.group.plan()
                
    def go_Init(self):
      self.group.clear_pose_targets()
        
      print "==== Resetting Pose to Initial Position ===="
      pose_target1 = geometry_msgs.msg.Pose()
        
      pose_target1.position.x = 0.7
      pose_target1.position.y = -0.55
      pose_target1.position.z = 0.17
      
      pose_target1.orientation.x = 1
      pose_target1.orientation.y = 0
      pose_target1.orientation.z = 0
      pose_target1.orientation.w = 0
      if not self.move_to_pose(pose_target1):
        print "==== Movement failed, shutting down ===="
        moveit_commander.roscpp_shutdown()
    
      self.grip_right.open()
    
      print "==== Reset Complete ===="              
    
    def move_to_pose(self,poseInput):
        planTemp = self.generate_Plan(poseInput)
        self.display_trajectory.trajectory_start = self.robot.get_current_state()
        self.display_trajectory.trajectory.append(planTemp)
        self.display_trajectory_publisher.publish(self.display_trajectory);
        rospy.sleep(2)
        if not self.group.execute(planTemp):
            return False
        else:
            #rospy.sleep(6)
            return True
    
    def pick_object(self, poseInput):
        
            waypoints = []
            waypoints.append(poseInput)        
            (plan,fraction) = self.group.compute_cartesian_path(waypoints,.01,0.0,False)
            print "==== moving to object ===="
            print "Percentage of Path followed: ", fraction/1.00* 100, " %"
            if not self.group.execute(plan):
                return False
            else:
                print "==== Closing gripper ===="
                rospy.sleep(6)
                self.grip_right.close()
                return True
        
    def graspLoop(self):
        

      
      ## Resets arm to initial position
      
      print "==== Gripper Initialization and Pose Subscribing ===="
      

      self.Grasp.listen()  # # listen to stop pose
      self.Grasp.listen2()  # # listen to final pose
      self.grip_right.calibrate()  # # Initial Gripper Calibration
      self.go_Init() 
    
      rospy.sleep(1)
    
      print "==== Ready to move to point ====" 
      
      
      while not rospy.is_shutdown():
          
        n = raw_input("==== Waiting for input: s for side and t for top q to quit...==== \n")
        
        if n == "q": # Quit the program
         break
     
        elif  n == "t": # Top grasping motion
            
            print "==== Going to top ===="
            
            if not self.move_to_pose(self.Grasp.stop_pose.pose):
                print "==== Movement failed, returning to initial pose ===="
                continue
            
            if not self.pick_object(self.Grasp.final_pose):
                print "==== Movement failed, returning to initial pose ===="
                self.go_Init()
                continue            
    
            print "==== Returning to top ===="
            
            if not self.move_to_pose(self.Grasp.stop_pose.pose):
                print "==== Movement failed, returning to initial pose ===="
                self.go_Init()
                continue
            
            print "==== Going init ===="
            
            self.go_Init()
            
        elif n == "s": # Side grasping motion
            
            print "==== Going to side ===="
           
            if not self.move_to_pose(self.Grasp.stop_pose.pose):
                print "==== Movement failed, returning to initial pose ===="
                continue
            if not self.pick_object(self.Grasp.final_pose.pose):
                print "==== Movement failed, returning to initial pose ===="
                self.go_Init()
                continue  
           
            print "==== Returning to top ===="
            
            if not self.move_to_pose(self.Grasp.stop_pose.pose):
                print "==== Movement failed, returning to initial pose ===="
                self.go_Init()
                continue
            print "==== Going init ===="
            self.go_Init()
        
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
      run = autonomousGrasper()
      run.graspLoop()
  except rospy.ROSInterruptException:
    pass
