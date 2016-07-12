#!/usr/bin/env python
import baxter_interface
import rospy
from sensor_msgs.msg import Image
import sys
import struct
import argparse


rospy.init_node("my_cam")
display_pub= rospy.Publisher('/robot/xdisplay',Image)
def republish(msg):
        """
            Sends the camera image to baxter's display
        """             
        display_pub.publish(msg)
def streamer(camera):
    if camera == "head":
#         left_camera = baxter_interface.CameraController("left_hand_camera")
#         left_camera.close()
#         right_camera = baxter_interface.CameraController("right_hand_camera")
#         right_camera.close()
        head_camera = baxter_interface.CameraController("head_camera")
        head_camera.open()
        camera_name = "head_camera"
        head_camera.resolution =(960, 600)
        sub = rospy.Subscriber('/cameras/' + camera_name + "/image", Image,republish,None,1)
        
    if camera == "right_hand_camera":
#         left_camera = baxter_interface.CameraController("left_hand_camera")
#         left_camera.close()
        right_camera = baxter_interface.CameraController("right_hand_camera")
        right_camera.open()
#         head_camera = baxter_interface.CameraController("head_camera")
#         head_camera.close()
        camera_name = "right_hand_camera"
        right_camera.resolution =(960, 600)

        sub = rospy.Subscriber('/cameras/' + camera_name + "/image", Image,republish,None,1)
        
    if camera == "left_hand_camera":
        left_camera = baxter_interface.CameraController("left_hand_camera")
        left_camera.open()
#         right_camera = baxter_interface.CameraController("right_hand_camera")
#         right_camera.close()
#         head_camera = baxter_interface.CameraController("head_camera")
#         head_camera.close()
        camera_name = "left_hand_camera"
        left_camera.resolution =(960, 600)
        sub = rospy.Subscriber('/cameras/' + camera_name + "/image", Image,republish,None,1)
        
    if camera == "kinect":
#         left_camera = baxter_interface.CameraController("left_hand_camera")
#         left_camera.close()
#         right_camera = baxter_interface.CameraController("right_hand_camera")
#         right_camera.close()
#         head_camera = baxter_interface.CameraController("head_camera")
#         head_camera.close()
        sub = rospy.Subscriber('/kinect2/sd/image_color_rect', Image,republish,None,1)
    
    rospy.spin()
def main():
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    parser.add_argument(
        '-c', '--camera', choices=['head', 'left_hand_camera', 'right_hand_camera', 'kinect'], required=False,
        type = str,
        default="kinect",
        help="the camera to display on baxter"
    )
    args = parser.parse_args(rospy.myargv()[1:])
    streamer(args.camera)
if __name__ == '__main__':
    sys.exit(main())
