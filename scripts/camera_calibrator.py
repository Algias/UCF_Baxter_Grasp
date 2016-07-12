#!/usr/bin/env python
import argparse
import copy
from geometry_msgs.msg import (
    PointStamped,
    Point,
    Quaternion,
)
import geometry_msgs.msg
import io
import numpy
import re
import rospy
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Header
from std_msgs.msg import String
import sys
import tf


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
        
    def listen(self):
        # Subscribe to /clicked_point topic from RVIZ
        rospy.Subscriber("/clicked_point", PointStamped, self.callback)
        rospy.sleep(0.2)
      
    def print_points(self):
        # Print out point values
        print "point 1: ", self.point1
        
    def update_point(self, ClickedPoint):
        #Returns a point list from a ROS point to use for calculations
            nPoint = [0, 0, 0]
            nPoint[0] = ClickedPoint.x
            nPoint[1] = ClickedPoint.y
            nPoint[2] = ClickedPoint.z
            # return [ClickedPoint.x,ClickedPoint.y,ClickedPoint.z]
            return nPoint
        
    def calibrate(self):
        print "*****Starting Camera Calibrator Node"
        rospy.init_node('calibrate_camera',
                      anonymous=True)
        
        clicked_array = []
        calib_frame_listener = tf.TransformListener()
        calib_frame_position = []
        calib_array = []
        calib_list = []
        clicked_list = []
        
         # creates a file to write out to
        with io.FileIO("camera_transforms", "w") as outFile: 
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
                #Perhaps not for safety, use joint keyboard?
                
                # Click the point / auto transform from clicked pt. to kinect2
                clicked_array = self.update_point(self.clickedPoint.point)
                
                # transform from calib_frame to base
                try:
                    (calib_array, rot) = calib_frame_listener.lookupTransform(
                                                          '/base',
                                                          '/calib_frame',
                                                          rospy.Time(0))
                except (tf.LookupException,
                        tf.ConnectivityException,
                        tf.ExtrapolationException):
                    print "transform error"
                    break
                
                # write to a list / convert tuple to list
                calib_array = list(calib_array)
                
                print "calib array: ",calib_array
                print "clicked array: ", clicked_array
                
                #Add the current values to the 2D array
                calib_list.append(calib_array)
                clicked_list.append(clicked_array)
                
            print "\n calib list: ", calib_list
            print "\n clicked list: ", clicked_list
            
            #Foreach member in the list, write to individual numbers to the file
            count = 0
            for member in calib_list:
                if count == 0:
                    outFile.write("calib list: \n[")
                outFile.write(str(calib_list[count][0]) +
                               " "+ str(calib_list[count][1]) +
                               " "+ str(calib_list[count][2])+
                               "; ")  
                
                count += 1  
            outFile.write("] \n")
            
            #Same process as earlier, but with the clicked list
            count = 0
            for member in clicked_list:
                if count == 0:
                    outFile.write("clicked list: \n[")
                outFile.write(str(clicked_list[count][0]) +
                               " "+ str(clicked_list[count][1]) +
                               " "+ str(clicked_list[count][2])+
                               "; ")  
                
                count += 1  
            outFile.write("]")
            
        outFile.close()  # close file
        
        
        

      
def main():
    run = CameraCalibrator()
    run.calibrate()

if __name__ == '__main__':
    sys.exit(main())
