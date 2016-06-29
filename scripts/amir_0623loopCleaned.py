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
        self.stop_pose = PoseStamped()

    def callback(self,data):
        self.stop_pose = data;
        #rospy.loginfo(rospy.get_caller_id() + " \n %s", self.stop_pose)
        
    def listen(self):
        rospy.Subscriber("/mypose1", PoseStamped, self.callback)
        rospy.Subscriber("/mypose2", PoseStamped, self.callback2)
        rospy.sleep(0.2)
        
    def callback2(self,data):
        self.final_pose = data;
        #rospy.loginfo(rospy.get_caller_id() + " \n %s", self.final_pose)
              
def move_group_python_interface_tutorial():
    
    n = raw_input("======== enter to continue ======== \n") 
     
    print "============ Starting test code ============"
    
    moveit_commander.roscpp_initialize(sys.argv)
    
    rospy.init_node('move_group_python_interface_tutorial',
                  anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("right_arm")
    
    group.set_goal_orientation_tolerance(0.01)
    group.set_goal_position_tolerance(0.01)
    
    grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
    
    Grasp = gomatlab()
    ## listen to poses
    Grasp.listen()
    
    grip_right.calibrate()
    grip_right.open()
    
    pose_generic = geometry_msgs.msg.PoseStamped()
    pose_generic.header.frame_id = "right_gripper"
    pose_generic.pose.position.x = 0
    pose_generic.pose.position.y = 0
    pose_generic.pose.position.z = 0
    pose_generic.pose.orientation.x = 0
    pose_generic.pose.orientation.y = 0
    pose_generic.pose.orientation.z = 0
    pose_generic.pose.orientation.w = 1
    
    print "============ Generating plan - initial pose ====================" 
    pose_init1 = geometry_msgs.msg.Pose()
    pose_init1.position.x = 0.7
    pose_init1.position.y = -0.25
    pose_init1.position.z = 0.17
    pose_init1.orientation.x = 1
    pose_init1.orientation.y = 0
    pose_init1.orientation.z = 0
    pose_init1.orientation.w = 0
    group.set_pose_target(pose_init1)
    
    initMove1 = group.go(wait=True)
    print "==================== Succeed? =================", initMove1
    
    rospy.sleep(.5)
    group.clear_pose_targets()
    
    if initMove1:
        print "============= Generating plan - stop pose =================="
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
        print "============== Succeed? ==================", stopMove1
        
        rospy.sleep(.5)
        group.clear_pose_targets()
        
    else: print "********** ERROR: Could not reach initial Pose **********"
    
    if stopMove1:
        print "============ Generating plan - final pose======================="
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
        (plan2, fraction1) = group.compute_cartesian_path(waypoint1
                             , .01, 0.0, False)
        print "================= moving to object ======================"
        print "Percentage of Path followed: ", fraction1 / 1.00 * 100, " %"
        if fraction1 > 0:
            finalMove = group.execute(plan2)
            print "================ Succeed? ==================",finalMove
        
        rospy.sleep(.5)
        
    else: print"********* ERROR: Could not go to final pose! **********"
    
    if finalMove:
        print"=========== Closing Gripper and Attaching Object ==============="
        grip_right.close()
        rospy.sleep(2)
        group.clear_pose_targets()
        scene.attach_box("right_gripper","box1",
                         pose = pose_generic,
                         size = (.2,.2,.2),
                         touch_links = ["right_gripper",
                                        "r_gripper_r_finger_tip",
                                        "r_gripper_r_finger",
                                        "r_gripper_l_finger",
                                        "r_gripper_l_finger_tip",
                                        "right_gripper_base"])
            
        print "============ Generating plan - back to stop pose =============="
        waypoint2 = []
        waypoint2.append(pose_target1)        
        (plan3, fraction2) = group.compute_cartesian_path(waypoint2
                             , .01, 0.0, False)
        print "========== backing from object =========="
        print "=====Percentage of Path followed: ",
            fraction2 / 1.00 * 100, " %======"  
        if fraction2 > 0:
            stopMove2 = group.execute(plan3)
            print "====== Succeed? ======",stopMove2
        
        rospy.sleep(.5)
        group.clear_pose_targets()
        
    if stopMove2:
        print "============ Generating plan - back to initial pose==============="
        group.set_pose_target(pose_init1)
        
        initMove2 = group.go(wait=True)
        print "====== Succeed? ======",initMove2
        
        rospy.sleep(.5)
        grip_right.open()
        group.clear_pose_targets()
        
        scene.remove_attached_object("right_gripper_base","box1")
        scene.remove_world_object("box1")
        
    else:print"********* ERROR: Could not reach stop pose **********"
    
    if initMove2:
        print"=========== Pickup Succeeded ==========="
        
    print "============ STOPPING ================"
if __name__=='__main__':
  try:
      while not rospy.is_shutdown():
          move_group_python_interface_tutorial()
  except rospy.ROSInterruptException:
    pass

