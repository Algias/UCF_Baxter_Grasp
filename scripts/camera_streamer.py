#!/usr/bin/env python
import baxter_interface
import rospy
from sensor_msgs.msg import Image


rospy.init_node("my_cam")
display_pub= rospy.Publisher('/robot/xdisplay',Image)
def republish(msg):
        """
            Sends the camera image to baxter's display
        """             
        display_pub.publish(msg)
sub = rospy.Subscriber('/kinect2/sd/image_color_rect', Image,republish,None,1)
rospy.spin()
