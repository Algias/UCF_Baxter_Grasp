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
        
    def clear_map(self):
        rospy.wait_for_service("/clear_octomap")
        try:
            h = rospy.ServiceProxy("/clear_octomap", std_srvs.srv.Empty())
            h()
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
              
class grapser_loop():
    print "===== Starting test code ====="

    moveit_commander.roscpp_initialize(sys.argv)
    
    rospy.init_node('grasper_loop',
                  anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("right_arm")
    group.allow_replanning(True)
    group.set_goal_orientation_tolerance(0.01)
    group.set_goal_position_tolerance(0.01)
    grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
    display_trajectory_publisher = rospy.Publisher(
                                    '/move_group/display_planned_path',
                                    moveit_msgs.msg.DisplayTrajectory,
                                     queue_size=10)
    
    Grasp = ReceivePoses()
    group.set_num_planning_attempts(3)
    pose_table = geometry_msgs.msg.PoseStamped()
    pose_table.header.frame_id = "base"
    pose_table.pose.position.x = .5
    pose_table.pose.position.y = -.92
    pose_table.pose.position.z = -.71
    pose_table.pose.orientation.x = 0
    pose_table.pose.orientation.y = 0
    pose_table.pose.orientation.z = 0
    pose_table.pose.orientation.w = 1
    
    pose_kinect = geometry_msgs.msg.PoseStamped()
    pose_kinect.header.frame_id = "base"
    pose_kinect.pose.position.x = -.1325
    pose_kinect.pose.position.y = -0.45
    pose_kinect.pose.position.z = -.4
    pose_kinect.pose.orientation.x = 0
    pose_kinect.pose.orientation.y = 0
    pose_kinect.pose.orientation.z = 1
    pose_kinect.pose.orientation.w = 0
    
    pose_generic = geometry_msgs.msg.PoseStamped()
    pose_generic.header.frame_id = "right_gripper"
    pose_generic.pose.position.x = 0
    pose_generic.pose.position.y = 0
    pose_generic.pose.position.z = 0
    pose_generic.pose.orientation.x = 0
    pose_generic.pose.orientation.y = 0
    pose_generic.pose.orientation.z = 0
    pose_generic.pose.orientation.w = 1
    
    scene.remove_world_object("box1")
    scene.remove_world_object("table")
    scene.remove_world_object("kinect")



    rospy.sleep(.5)

    scene.add_box("table", pose_table, size=(1.1, 1.1, 1.1))
    scene.add_box("kinect", pose_kinect, size=(.25, .25, 1))
    
    Grasp.clear_map()
    
    # set to true for attached object
    object = False
    
    
    
    rospy.sleep(2.5)
    
    # Wait until first point is selected
    n = raw_input("\n!!!!! Enter to continue or q to quit(Ensure first point is selected)!!!!!\n")
    
    if n is "^C" or n is "q":
        moveit_commander.roscpp_shutdown()

    if n is "":
        pass
    
    grip_right.calibrate()
    
    initpose = [1.093728,
                    - 0.76929136,
                    - 0.14764565,
                    0.98174770,
                    0.08091748,
                    1.34338367,
                    - 0.02914563]


    group.set_joint_value_target(initpose)
    
    initMove1 = group.go(wait=True)
    # For automatic picking
    pose_previous = geometry_msgs.msg.Pose()
    
    first_motion = True

    while not rospy.is_shutdown():
        
        Grasp.listen()    

        if Grasp.final_pose.pose != pose_previous:
            
            pose_previous = Grasp.final_pose.pose
            rospy.sleep(.5)
            # # listen to poses
            Grasp.listen()    
            grip_right.open()
            scene.remove_world_object("box1")
            Grasp.clear_map()
            if not first_motion:
                # This is the go to initial pose function
                print "===== Generating plan - initial pose =====" 
                 
                # Joint angles for initial pose
                initpose = [1.093728,
                                - 0.76929136,
                                - 0.14764565,
                                0.98174770,
                                0.08091748,
                                1.34338367,
                                - 0.02914563]
             
                group.set_joint_value_target(initpose)
                 
                initMove1 = group.go(wait=True)
                print "===== Motion Succeed? =====", initMove1
                 
                rospy.sleep(.5)
                group.clear_pose_targets()
                Grasp.clear_map()
 
             
            # This is the go to stop pose function
            # If the arm is in the initial position, go to stop pose
            if initMove1:
                first_motion = False
                print "===== Generating plan - stop pose ====="
                pose_target1 = geometry_msgs.msg.Pose()
                pose_target1.position.x = Grasp.stop_pose.pose.position.x
                pose_target1.position.y = Grasp.stop_pose.pose.position.y
                pose_target1.position.z = Grasp.stop_pose.pose.position.z
                pose_target1.orientation.x = Grasp.stop_pose.pose.orientation.x
                pose_target1.orientation.y = Grasp.stop_pose.pose.orientation.y
                pose_target1.orientation.z = Grasp.stop_pose.pose.orientation.z
                pose_target1.orientation.w = Grasp.stop_pose.pose.orientation.w
                group.set_pose_target(pose_target1)
                
                stopMove1 = group.go(wait=True)
                print "===== Motion Succeed? =====", stopMove1
                
                rospy.sleep(.5)
                group.clear_pose_targets()
                Grasp.clear_map()

                
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
                Grasp.clear_map()
                pose_target2 = geometry_msgs.msg.Pose()
                pose_target2.position.x = Grasp.final_pose.pose.position.x
                pose_target2.position.y = Grasp.final_pose.pose.position.y
                pose_target2.position.z = Grasp.final_pose.pose.position.z 
                pose_target2.orientation.x = Grasp.final_pose.pose.orientation.x
                pose_target2.orientation.y = Grasp.final_pose.pose.orientation.y
                pose_target2.orientation.z = Grasp.final_pose.pose.orientation.z
                pose_target2.orientation.w = Grasp.final_pose.pose.orientation.w
                waypoint1 = []
                waypoint1.append(pose_target2)        
                (plan2, fraction1) = group.compute_cartesian_path(waypoint1,
                                                                   .01, 0.0, False)
                print "===== Moving to object ====="
                print "===== Percentage of Path followed: ", (fraction1 / 1.00 * 100), " % ====="
                if fraction1 > 0:
                    finalMove = group.execute(plan2)
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
                grip_right.close()
                rospy.sleep(2)
                group.clear_pose_targets()
                if object:
                    scene.attach_box("right_gripper", "box1",
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
                Grasp.clear_map()
 
                print "===== Generating plan - back to stop pose ====="
                waypoint2 = []
                waypoint2.append(pose_target1)        
                (plan3, fraction2) = group.compute_cartesian_path(waypoint2
                                     , .01, 0.0, False)
                print "===== backing from object =========="
                print "===== Percentage of Path followed: ", (fraction2 / 1.00 * 100), " % ======"  
                if fraction2 > 0:
                    stopMove2 = group.execute(plan3)
                    print "===== Motion Succeed? ======", stopMove2
                else: stopMove2 = False
                
                rospy.sleep(1.5)
                group.clear_pose_targets()
                Grasp.clear_map()

    
            # if the arm returned to stop pose, return to initial
            if stopMove2:
                print "===== Generating plan - back to initial pose ====="
                # group.set_pose_target(pose_init1)
                group.set_joint_value_target(initpose)
                
                initMove2 = group.go(wait=True)
                print "===== Motion Succeed? =====", initMove2
                
                rospy.sleep(.5)
                grip_right.open()
                group.clear_pose_targets()
                rospy.sleep(1)
            
            # If the arm returned to initial successfully
            if initMove2:
                print"===== Pickup Succeeded ====="
            scene.remove_attached_object("right_gripper")
            scene.remove_world_object("box1")
            Grasp.clear_map()
            
            rospy.sleep(2.0)
            print "===== STOPPING ====="
            ready = False
        # Rate at which the loop executes, when waiting for new poses
        rate = rospy.Rate(1)  # 1hz
        rate.sleep()
        
        if Grasp.final_pose.pose == pose_previous and not ready:
            print "===== Waiting for new pose... ====="
            ready = True
        
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        grapser_loop()
    except rospy.ROSInterruptException:
        pass
