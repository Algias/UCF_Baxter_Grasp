#!/usr/bin/env python
import copy
from geometry_msgs.msg import (
    PointStamped,
    Point,
    Quaternion,
)
import geometry_msgs.msg
import numpy
import rospy
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Header
from std_msgs.msg import String
import sys
import tf
import argparse
import io
import re

#===============================================================================
# DESCRIPTION
# This file is intended to take clicked points and a calibrated frame and write
# their transforms to a file for use in matlab
#===============================================================================
class CameraCalibrator:
        
    def __init__(self):

        # Initialize global variables
        self.clickedPoint = PointStamped()

        
    def callback(self, data):
        self.clickedPoint = data;
        # Uncomment to print out subscriber 
        # rospy.loginfo(rospy.get_caller_id() + " \n %s", self.clickedPoint)
        
    def listen(self):
        # Subscribe to /clicked_point topic from RVIZ
        rospy.Subscriber("/clicked_point", PointStamped, self.callback)
        rospy.sleep(0.2)
      
    def print_points(self):
        # Print out point values
        print "point 1: ", self.point1
        
    def update_point(self, ClickedPoint):
            nPoint = [0, 0, 0]
            nPoint[0] = ClickedPoint.x
            nPoint[1] = ClickedPoint.y
            nPoint[2] = ClickedPoint.z
            # return [ClickedPoint.x,ClickedPoint.y,ClickedPoint.z]
            return nPoint
        
    def calibrate(self):
        print "*****Starting camera calibrator Node"
        rospy.init_node('calibrate_camera',
                      anonymous=True)
        
        clicked_array = []
        calib_frame_listener = tf.TransformListener()
        calib_frame_position = []
        calib_array = []
        calib_list = []
        clicked_list = []
        
        with io.FileIO("camera_transforms", "w") as outFile:  # creates a file to write out to
            # w opens a file for writing
            rospy.sleep(.5)
            self.listen()
            for x in range(0, 20):
                print "\n!!!!! Enter for point ",x+1,"!!!!!"
                n = raw_input()
                rospy.sleep(.1)
                self.listen()
                if n == "":
                    pass
                else:
                    break
                # Move the arm?
                # Click the point / auto transform
                # transform from clicked pt. to kinect2
                clicked_array = self.update_point(self.clickedPoint.point)
                # transform from calib_frame to base
                try:
                    (calib_array, rot) = calib_frame_listener.lookupTransform(
                                                          'base',
                                                          'calib_frame',
                                                          rospy.Time(0))
                except (tf.LookupException,tf.ConnectivityException,tf.ExtrapolationException):
                    print "transform error"
                    break
                # write to a list
                calib_array = list(calib_array)
                print "calib array: ",calib_array
                print "clicked array: ", clicked_array
                calib_list.append(calib_array)
                clicked_list.append(clicked_array)
            print "calib list: ", calib_list
            print "clicked list: ", clicked_list
            
            count = 0
            for member in calib_list:
                if count == 0:
                    outFile.write("calib list: \n[")
                outFile.write(str(calib_list[count][0]) + " "+ str(calib_list[count][1]) +" "+ str(calib_list[count][2])+"; ")  
                
                count += 1  
            outFile.write("] \n")
            
            count = 0
            for member in clicked_list:
                if count == 0:
                    outFile.write("clicked list: \n[")
                outFile.write(str(clicked_list[count][0]) + " "+ str(clicked_list[count][1]) +" "+ str(clicked_list[count][2])+"; ")  
                
                count += 1  
            outFile.write("]")
            
        outFile.close()  # close io
        
        
        

      
def main():
    run = CameraCalibrator()
    run.calibrate()

if __name__ == '__main__':
    sys.exit(main())
