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
        
        self.Grasp = ReceivePoses()
        
        self.group.set_num_planning_attempts(3)
        
        self.initpose = [1.093728,
                        - 0.76929136,
                        - 0.14764565,
                        0.98174770,
                        0.08091748,
                        1.34338367,
                        - 0.02914563]
        
        self.scene.remove_world_object("box1")
        self.scene.remove_world_object("table")
        self.scene.remove_world_object("kinect")
        
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
            print "===== Moving to object ====="
            print "===== Percentage of Path followed: ", (n_fraction / 1.00 * 100), " % ====="
            return n_plan, n_fraction
        
    def clear_map(self):
        rospy.wait_for_service("/clear_octomap")
        try:
            h = rospy.ServiceProxy("/clear_octomap", std_srvs.srv.Empty())
            h()
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
            
    def grasper_loop(self):
        
        print "===== Starting test code ====="
        ready = bool
        use_gripper = True
        octomap = False

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
        self.scene.add_box("kinect", pose_kinect, size=(.25, .25, 1))
        
        if octomap:
            self.clear_map()
        
        # set to true for attached object
        object = False
        
        #Wait for scene to be updated with table
        rospy.sleep(3.0)
        
        # Wait until first point is selected
        n = raw_input("\n!!!!! Enter to continue or q to quit(Ensure first point is selected)!!!!!\n")
        
        if n is "^C" or n is "q":
            moveit_commander.roscpp_shutdown()
    
        if n is "":
            pass
        
        else:
            moveit_commander.roscpp_shutdown()
            
        if use_gripper:
            self.grip_right.calibrate()
    
        self.group.set_joint_value_target(self.initpose)
        
        initMove1 = self.group.go(wait=True)
        # For automatic picking
        pose_previous = geometry_msgs.msg.Pose()
        
        first_motion = True
    
        while not rospy.is_shutdown():
            
            self.Grasp.listen()
            
            if self.Grasp.final_pose.pose != pose_previous:
                
                pose_previous = self.Grasp.final_pose.pose
                rospy.sleep(.5)
                
                if use_gripper:
                    self.grip_right.open()
                    
                self.scene.remove_world_object("box1")
                
                if octomap:
                    self.clear_map()    
                          
                # This is the go to stop pose function
                # If the arm is in the initial position, go to stop pose
                if initMove1:
                    
                    first_motion = False
                    
                    print "===== Generating plan - stop pose ====="
                    pose_target1 = geometry_msgs.msg.Pose()
                    pose_target1 = self.update_pose(self.Grasp.stop_pose.pose)
                    self.group.set_pose_target(pose_target1)
                    
                    stopMove1 = self.group.go(wait=True)
                    print "===== Motion Succeed? =====", stopMove1
                    
                    rospy.sleep(.5)
                    
                    self.group.clear_pose_targets()
                    
                    if octomap:
                        self.clear_map()
    
                    
                # If the arm did not reach initial pose, stay.
                else:
                    print "***** ERROR: Could not reach initial Pose *****"
                    stopMove1 = False
                    finalMove = False
                    stopMove2 = False
                    initMove2 = False
                
                # This is the move to final pose function
                # If the arm is at the stop pose position, move to final pose
                if stopMove1:
                    
                    print "===== Generating plan - final pose ====="
                    
                    if octomap:
                        self.clear_map()
                        
                    pose_target2 = geometry_msgs.msg.Pose()
                    pose_target2 = self.update_pose(self.Grasp.final_pose.pose)

                    (plan2, fraction1) = self.go_cartesian(pose_target2)
                    
                    if fraction1 > 0:
                        finalMove = self.group.execute(plan2)
                        print "===== Motion Succeed? =====", finalMove
                        
                    else:
                        finalMove = False
                        initMove2 = False
                        
                    rospy.sleep(.5)
                    
                # If the arm does not go to stop pose, stay at initial pose        
                else:
                    
                    print"***** ERROR: Could not go to stop pose! *****"
                    finalMove = False
                    stopMove2 = False
                    initMove2 = False
                    
                # This is the return to stop pose function
                # if the arm is at the final pose, return to stop pose
                if finalMove:
                    
                    print "===== Closing Gripper and Attaching Object ====="
                    if use_gripper:
                        self.grip_right.close()
                        
                    rospy.sleep(2)
                    self.group.clear_pose_targets()
                    
                    if object:
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
                    
                    if octomap:
                        self.clear_map()
     
                    print "===== Generating plan - back to stop pose ====="
                    (plan3, fraction2) = self.go_cartesian(pose_target1)

                    if fraction2 > 0:
                        stopMove2 = self.group.execute(plan3)
                        print "===== Motion Succeed? ======", stopMove2
                        
                    else: stopMove2 = False
                    
                    rospy.sleep(1.5)
                    
                    self.group.clear_pose_targets()
                    
                    if octomap:
                        self.clear_map()
    
        
                # if the arm returned to stop pose, return to initial
                if stopMove2:
                    
                    print "===== Generating plan - back to initial pose ====="
                    self.group.set_joint_value_target(self.initpose)
                    
                    initMove2 = self.group.go(wait=True)
                    print "===== Motion Succeed? =====", initMove2
                    
                    rospy.sleep(.5)
                    
                    if use_gripper:
                        self.grip_right.open()
                        
                    self.group.clear_pose_targets()
                    rospy.sleep(1)
                
                # If the arm returned to initial successfully
                if initMove2:
                    print"===== Pickup Succeeded ====="
                    
                self.scene.remove_attached_object("right_gripper")
                self.scene.remove_world_object("box1")
                
                if octomap:
                    self.clear_map()
                
                rospy.sleep(2.0)
                ready = False
                
            # Rate at which the loop executes, when waiting for new poses
            rate = rospy.Rate(1)  # 1hz
            rate.sleep()
            
            if self.Grasp.final_pose.pose == pose_previous and not ready:
                print "===== Waiting for new pose... ====="
                ready = True
            
        moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        run = GraspLoop()
    except rospy.ROSInterruptException:
        pass
