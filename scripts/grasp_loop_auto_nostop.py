#!/usr/bin/env python

import baxter_external_devices
from baxter_interface import CHECK_VERSION
import baxter_interface
import copy
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
import geometry_msgs.msg
import moveit_commander
import moveit_msgs.msg
import std_srvs.srv
import rospy
from std_msgs.msg import Header
from std_msgs.msg import String
from std_srvs.srv import Empty
import sys


class ReceivePoses:
    
    
    def __init__(self):
        self.final_pose = PoseStamped()
        self.stop_pose = PoseStamped()

    def callback(self, data):
        self.stop_pose = data;
        # rospy.loginfo(rospy.get_caller_id() + " \n %s", self.stop_pose)
        
    def listen(self):
        rospy.Subscriber("/mypose1", PoseStamped, self.callback)
        rospy.Subscriber("/mypose2", PoseStamped, self.callback2)
        rospy.sleep(0.2)
        
    def callback2(self, data):
        self.final_pose = data;
        # rospy.loginfo(rospy.get_caller_id() + " \n %s", self.final_pose)
        

              
class GraspLoop():
    
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        
        rospy.init_node('grasper_loop',
                      anonymous=True)
        
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("right_arm")
        
        self.group.allow_replanning(True)
        
        self.group.set_goal_orientation_tolerance(0.01)
        self.group.set_goal_position_tolerance(0.01)
        
        self.grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
        
        self.display_trajectory_publisher = rospy.Publisher(
                                        '/move_group/display_planned_path',
                                        moveit_msgs.msg.DisplayTrajectory,
                                         queue_size=10)
        
        #Creates instance of class for subscribing to and listening to poses
        self.Grasp = ReceivePoses()
        
        self.group.set_num_planning_attempts(3)
        
        #Defines initial starting pose to return to (joint angles)
        self.initpose = [1.093728,
                        - 0.76929136,
                        - 0.14764565,
                        0.98174770,
                        0.08091748,
                        1.34338367,
                        3.0]
        # 1.7 works as well for last number
        
        #Clears the scene in case of leftover objects
        self.scene.remove_world_object("box1")
        self.scene.remove_world_object("table")
        self.scene.remove_world_object("kinect")
        
        #Runs the main loop function
        self.grasper_loop()
        
    def update_pose(self, pose_input):
        
        n_pose = geometry_msgs.msg.Pose()
        
        if isinstance(pose_input, list):
            n_pose.position.x = pose_input[0]
            n_pose.position.y = pose_input[1]
            n_pose.position.z = pose_input[2]
            n_pose.orientation.x = pose_input[3]
            n_pose.orientation.y = pose_input[4]
            n_pose.orientation.z = pose_input[5]
            n_pose.orientation.w = pose_input[6]
            return n_pose
        
        if isinstance(pose_input, geometry_msgs.msg.Pose):
            n_pose.position.x = pose_input.position.x
            n_pose.position.y = pose_input.position.y
            n_pose.position.z = pose_input.position.z
            n_pose.orientation.x = pose_input.orientation.x
            n_pose.orientation.y = pose_input.orientation.y
            n_pose.orientation.z = pose_input.orientation.z
            n_pose.orientation.w = pose_input.orientation.w
            return n_pose
        
        if isinstance(pose_input, geometry_msgs.msg.PoseStamped):
            n_pose.position.x = pose_input.pose.position.x
            n_pose.position.y = pose_input.pose.position.y
            n_pose.position.z = pose_input.pose.position.z
            n_pose.orientation.x = pose_input.pose.orientation.x
            n_pose.orientation.y = pose_input.pose.orientation.y
            n_pose.orientation.z = pose_input.pose.orientation.z
            n_pose.orientation.w = pose_input.pose.orientation.w
            return n_pose
        
        else: 
            return Empty() 
            print "error"
    
    def go_cartesian(self, pose):
            n_waypoint = []
            n_waypoint.append(pose)        
            (n_plan, n_fraction) = self.group.compute_cartesian_path(n_waypoint,
                                                              .01, 0.0, False)
            print "===== Moving in Cartesian Path ====="
            print "===== Percentage of Path followed: ", (n_fraction / 1.00 * 100), " % ====="
            if n_fraction > 0:
                return self.group.execute(n_plan)
            return False
        
    def clear_map(self):
        rospy.wait_for_service("/clear_octomap")
        try:
            h = rospy.ServiceProxy("/clear_octomap", std_srvs.srv.Empty())
            h()
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
            
    def print_results(self,result,motion):
        if result:
            print "===== Successfully went to ", motion," ====="
        if not result:
            print "====== Attempt to go to ", motion, "failed ====="
            
    def grasper_loop(self):
        
        print "===== Starting test code ====="
        ready_for_pose = bool
        #set to true for grippers to be used
        use_gripper = True
        #Set to true for clearing octomap
        octomap = True
        # set to true for attached object
        object = True

        pose_table = geometry_msgs.msg.PoseStamped()
        pose_table.header.frame_id = "base"
        pose_table.pose = self.update_pose([.5, -.92, -.71, 0, 0, 0, 1])
        
        pose_kinect = geometry_msgs.msg.PoseStamped()
        pose_kinect.header.frame_id = "base"
        pose_kinect.pose = self.update_pose([-.1325, -.45, -.4, 0, 0, 1, 0])
        
        pose_generic = geometry_msgs.msg.PoseStamped()
        pose_generic.header.frame_id = "right_gripper"
        pose_generic.pose = self.update_pose([0, 0, 0, 0, 0, 0, 1])
        
        rospy.sleep(.5)
    
        self.scene.add_box("table", pose_table, size=(1.1, 1.1, 1.1))
        self.scene.add_box("kinect", pose_kinect, size=(.25, .25, 1.25))
        
        if octomap:
            self.clear_map()
        
        # Wait for scene to be updated with table
        print"=====Waiting for scene to be updated ====="
        rospy.sleep(3.0)
        
        # Wait until first point is selected, or quit out if you need to
        n = raw_input("\n!!!!! Enter to continue or q to quit(Ensure first point is selected)!!!!!\n")
        
        if n == "^C" or n == "q":
            moveit_commander.roscpp_shutdown()
        if n == "":
            pass
        else:
            moveit_commander.roscpp_shutdown()
            
        if use_gripper:
            self.grip_right.calibrate()
            
        if octomap:
            self.clear_map()    
            
        self.group.set_joint_value_target(self.initpose)
        
        print "===== Going to intial pose ====="
        first_go_initial = self.group.go(wait=True)
        
        if octomap:
            self.clear_map()    
        
        # For automatic picking
        pose_previous = geometry_msgs.msg.Pose()
        
        first_motion = True
    
        while not rospy.is_shutdown():
            
            first_stop_move = bool
            go_final = bool
            second_stop_move = bool
            
            self.Grasp.listen()
            
            if self.Grasp.final_pose.pose != pose_previous:
                
                pose_previous = self.Grasp.final_pose.pose
                rospy.sleep(.5)
                
                if use_gripper:
                    self.grip_right.open()
                    
                self.scene.remove_world_object("box1")  
                    
                #If this is not the first motion, and returning initial failed,
                #Try to go to initial again
                if first_motion == False and return_initial == False:
                    print"===== Returning to initial position after previous failure ====="
                    if octomap:
                        self.clear_map()  
                    self.group.set_joint_value_target(self.initpose)
                    first_go_initial = self.group.go(wait=True)
                    self.print_results(first_go_initial, "return initial")
                    
                return_initial = bool

                # This is the go to stop pose function
                # If the arm is in the initial position, go to stop pose
                if first_go_initial is True:
                    
                    if octomap:
                        self.clear_map()
                        
                    first_motion = False
                    
                    print "===== Generating plan - stop pose ====="
                    pose_target1 = geometry_msgs.msg.Pose()
                    pose_target1 = self.update_pose(self.Grasp.stop_pose.pose)
                    self.group.set_pose_target(pose_target1)
                    first_stop_move = self.group.go(wait=True)
                    self.print_results(first_stop_move,"stop pose")
                    
                    rospy.sleep(.5)
                    
                    self.group.clear_pose_targets()
                
                # This is the move to final pose function
                # If the arm is at the stop pose position, move to final pose
                if first_stop_move is True:
                    
                    print "===== Generating plan - final pose ====="
                    
                    if octomap:
                        self.clear_map()
                        
                    pose_target2 = geometry_msgs.msg.Pose()
                    pose_target2 = self.update_pose(self.Grasp.final_pose.pose)
                    go_final = self.go_cartesian(pose_target2)
                    self.print_results(go_final, "final pose")
                    
                    rospy.sleep(.5)
                    
                # This is the return to stop pose function
                if go_final is True:
                    
                    print "===== Closing Gripper ====="
                    if use_gripper:
                        self.grip_right.close()
                        
                    if octomap:
                        self.clear_map()  
                        
                    rospy.sleep(2)
                    self.group.clear_pose_targets()
                    
                    if object:
                        print"===== Attaching collision object ====="
                        self.scene.attach_box("right_gripper", "box1",
                                     pose=pose_generic,
                                     size=(.2, .2, .2),
                                     touch_links=["right_gripper",
                                                    "r_gripper_r_finger_tip",
                                                    "r_gripper_r_finger",
                                                    "r_gripper_l_finger",
                                                    "r_gripper_l_finger_tip",
                                                    "right_gripper_base",
                                                    "right_hand_range",
                                                    "right_hand_camera",
                                                    "right_hand",
                                                    "right_wrist",
                                                    "right_lower_forearm",
                                                    "right_hand_accelerometer"])
                    rospy.sleep(1)
     
                    print "===== Generating plan - back to stop pose ====="
                    second_stop_move = self.go_cartesian(pose_target1)
                    self.print_results(second_stop_move,"return stop pose")
                    
                    rospy.sleep(1.5)
                    
                    self.group.clear_pose_targets()
                    
                        
                # if the arm returned to stop pose, return to initial
                if second_stop_move is True:
                    if octomap:
                        self.clear_map()  
                    print "===== Generating plan - back to initial pose ====="
                    self.group.set_joint_value_target(self.initpose)
                    
                    return_initial = self.group.go(wait=True)
                    self.print_results(return_initial, "return initial")
                    
                    rospy.sleep(.5)
                    
                    if use_gripper:
                        self.grip_right.open()
                        
                    self.group.clear_pose_targets()
                    rospy.sleep(1)
                
                # If the arm returned to initial successfully
                if return_initial is True:
                    print"===== Pickup Succeeded ====="
                else:
                    print"===== Pickup failed ====="
                    
                self.scene.remove_attached_object("right_gripper")
                self.scene.remove_world_object("box1")
                
                rospy.sleep(2.0)
                ready_for_pose = False
            #Ensures that waiting for new pose is printed only once
            #Also only prints if it had not received a new pose
            if go_final == False or second_stop_move == False:
                return_initial = False
                
            if first_motion == False and (first_stop_move == False or second_stop_move == False):
                print"===== Returning to initial position after previous failure ====="
                if octomap:
                    self.clear_map()  
                self.group.set_joint_value_target(self.initpose)
                return_initial = self.group.go(wait=True)
                self.print_results(first_go_initial, "return initial")
                
                
            if self.Grasp.final_pose.pose == pose_previous and not ready_for_pose:
                print "===== Waiting for new pose... =====\n"
                ready_for_pose = True
                
            # Rate at which the loop executes, when waiting for new poses
            rate = rospy.Rate(1)  # 1hz
            rate.sleep()
            
        moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        run = GraspLoop()
    except rospy.ROSInterruptException:
        pass
