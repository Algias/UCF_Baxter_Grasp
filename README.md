# UCF_Baxter_Grasp

This repo deals with automating grasping objects with the Baxter Research robot.

## Requirements:

[Full baxter install](http://sdk.rethinkrobotics.com/wiki/Workstation_Setup)

[Baxter MoveIt install](http://sdk.rethinkrobotics.com/wiki/MoveIt_Tutorial)

[IAI Kinect2 bridge install](https://github.com/code-iai/iai_kinect2)

##Grasp loop code

Ensure connectivity with Baxter
Enable baxter and launch moveit, and the kinect with:
 > roslaunch baxter_grasp baxter_grasp.launch
Run the grasp loop code:
>
  rosrun baxter_grasp grasp_loop_auto_nostop.py
  
This code takes in poses from the /mypose1 and /mypose2 topics. The arm will then attempt to plan a path and execute motions in order to autonomously grasp objects and place them in a container.

##Camera calibrator

This file is intended to take clicked points and a calibrated frame and write their transforms to a file for use in matlab.

This allows for quick calibration of the kinect frame with respect to Baxter's base.

To run:
  
  >roslaunch baxter_Grasp baxter_camera.launch
  
  >rosrun baxter_grasp camera_calibrator.py

##Camera streamer

This file places a camera stream on Baxter's head display.
 
> roslaunch baxter_Grasp baxter_camera.launch
> rosrun baxter_grasp camera_streamer.py

##click_to_point code: 

Allows the user to click three points on a pointcloud, and create a pose that that fits the position. Points 1 and 2 are for the gripper closing direction, and point 3 creates the orientation
  >roslaunch baxter_grasp baxter_grasp.launch
  >rosrun baxter_grasp click_to_point.py

##camera.rviz

Saved RVIZ layout for camera calibrations.
