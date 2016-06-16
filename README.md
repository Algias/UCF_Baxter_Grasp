# UCF_Baxter_Grasp

This repo deals with automating grasping objects with the Baxter Research robot.

## Requirements:

[Full baxter install](http://sdk.rethinkrobotics.com/wiki/Workstation_Setup)

[Baxter MoveIt install](http://sdk.rethinkrobotics.com/wiki/MoveIt_Tutorial)

[IAI Kinect2 bridge install](https://github.com/code-iai/iai_kinect2)

###Modified Kinect2 Bridge code and sensor Manager code (To allow kinect2 to be used)

In Baxter_moveit_sensor_manager: 

Replace

> include file="$(find freenect..."" 

with 

> include file="$(find kinect2_bridge)/launch/kinect2_bridge.launch"

And set "Camera_link" to "kinect2_link" for transform and octomap

Also, in /baxter/baxter_moveit_config/config, modify kinect_sensor.yaml

> point_cloud_topic: /kinect2/sd/points

##click_to_point: 

Allows the user to click three points on a pointcloud, and create a pose that that fits the position. Points 1 and 2 are for the gripper closing direction, and point 3 creates the orientation
