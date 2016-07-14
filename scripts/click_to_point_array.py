#!/usr/bin/env python
import copy
from geometry_msgs.msg import (
    PoseStamped,
    PointStamped,
    Pose,
    Point,
    Quaternion,
    PoseArray,
)
import geometry_msgs.msg
import numpy
import rospy
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Header
from std_msgs.msg import String
from std_msgs.msg import Empty
import sys

#===============================================================================
# DESCRIPTION
# This script takes 3 points from the RVIZ /clicked_point publisher and finds a
# pose calculation to fit those three points. It then publishes two poses, a
# stop pose and a final pose for the gripper.
# USAGE
# The first two points create the axis of rotation for the closing of grippers,
# and the third point finds the final orientation. The centroid of the three
# points is used as the position
# ROS INFORMATION
# Subscribes to rviz's /clicked_point publisher.
# Publishes to mypose1 and mypose2
#===============================================================================
class PointClick:
        
    def __init__(self):
        # Initialize global variables
        self.clickedPoint = PointStamped()
        self.point1 = []
        self.point2 = []
        self.point3 = []
        
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
        print "point 2: ", self.point2
        print "point 3: ", self.point3
        
    def update_array(self,obj1,pose_s,pose_n):
        _n_pose = geometry_msgs.msg.Pose()
        _n_pose.position.x = pose_n.position.x
        _n_pose.position.y = pose_n.position.y
        _n_pose.position.z = pose_n.position.z
        _n_pose.orientation.x = pose_n.orientation.x
        _n_pose.orientation.y = pose_n.orientation.y
        _n_pose.orientation.z = pose_n.orientation.z
        _n_pose.orientation.w = pose_n.orientation.w
        
        _s_pose = geometry_msgs.msg.Pose()
        _s_pose.position.x = pose_s.position.x
        _s_pose.position.y = pose_s.position.y
        _s_pose.position.z = pose_s.position.z
        _s_pose.orientation.x = pose_s.orientation.x
        _s_pose.orientation.y = pose_s.orientation.y
        _s_pose.orientation.z = pose_s.orientation.z
        _s_pose.orientation.w = pose_s.orientation.w



        obj1.posearray_s.poses.append(_s_pose)
        obj1.posearray_n.poses.append(_n_pose)
        
    def point_clicker(self):
        print "*****Starting point_clicker Node"
        rospy.init_node('point_clicker',
                      anonymous=True)
            
        # Subscribe to /clicked_point
        self.listen()
        
        num_object = int(raw_input("Enter Number of objects: "))
        
        # Create publishers
        pubpose1 = rospy.Publisher('myposearray1', PoseArray, queue_size=5)
        pubpose2 = rospy.Publisher('myposearray2', PoseArray, queue_size=5)
        
        # Initialize variables
        previousPoint = geometry_msgs.msg.Point()
        n = 0
        w = 0
        obj1 = PoseCalc()
        
        print "Enter point number: 1 \n"
        pose_previous = geometry_msgs.msg.PoseStamped()
        for x in xrange(0,num_object):

            while not rospy.is_shutdown():
    
            # if we have a full set of points, and 
            # the clicked point is new, reset points
            # and use the new point as the first point
                    #print w," ",num_object
                if n == 4 and self.clickedPoint.point != previousPoint:
                    n = 2
                    self.point1 = obj1.update_point(self.clickedPoint.point)
                    self.point2 = []
                    self.point3 = []
                    
                    previousPoint = self.clickedPoint.point
                    
                    self.print_points()
                    print "\n Enter point number", n
        
                # if the point counter is less than 4, and the 
                # clicked point is new, update the point
                
                if n < 4 and self.clickedPoint.point != previousPoint:
                        
                    if n == 1:
                        self.point1 = obj1.update_point(self.clickedPoint.point)
                            
                    if n == 2:
                        self.point2 = obj1.update_point(self.clickedPoint.point)
                            
                    if n == 3:
                        self.point3 = obj1.update_point(self.clickedPoint.point)
                            
                    if n is not 0:
                        self.print_points()
                            
                    if n != 0 and n < 3: print "\n Enter point number: ", n + 1
                        
                    if n == 3: print "\n Enter point 1 for new pose"    
                    n += 1
                        
                    previousPoint = self.clickedPoint.point
                        
                # if we have all points in the set, 
                # calculate two poses based on that set
                
                if self.point1 and self.point2 and self.point3:
                    obj1.calculate_pose(self.point1, self.point2, self.point3)
                    self.update_array(obj1, obj1.pose_s.pose,obj1.pose_n.pose)
                    self.point1 = []
                    self.point2 = []
                    self.point3 = []
                    break
                    w += 1
                
            rate = rospy.Rate(1)  # 1hz
            rate.sleep()
                
        pubpose1.publish(obj1.posearray_n)
        pubpose2.publish(obj1.posearray_s) 
        #rospy.spin()

class PoseCalc:
    
    def __init__(self):
        
        self.pose_n = geometry_msgs.msg.PoseStamped()
        self.pose_s = geometry_msgs.msg.PoseStamped()

        self.posearray_n = geometry_msgs.msg.PoseArray()
        self.posearray_s = geometry_msgs.msg.PoseArray()

        self.pose_n.header.frame_id = "base"
        self.pose_s.header.frame_id = "base"
        
        self.posearray_n.header.frame_id = "base"
        self.posearray_s.header.frame_id = "base"
        
    def get_quaternion(self, lst1, lst2, matchlist=None):
        # Python implementation of this method:
        # Paul J. Besl and Neil D. McKay "Method for registration of 3-D shapes", 
        # Sensor Fusion IV: Control Paradigms and Data Structures, 
        # 586 (April 30, 1992); http://dx.doi.org/10.1117/12.57955
        if not matchlist:
            matchlist = range(len(lst1))
        M = numpy.matrix([[0, 0, 0], [0, 0, 0], [0, 0, 0]])
    
        for i, coord1 in enumerate(lst1):
            x = numpy.matrix(numpy.outer(coord1, lst2[matchlist[i]]))
            M = M + x
    
        N11 = float(M[0][:, 0] + M[1][:, 1] + M[2][:, 2])
        N22 = float(M[0][:, 0] - M[1][:, 1] - M[2][:, 2])
        N33 = float(-M[0][:, 0] + M[1][:, 1] - M[2][:, 2])
        N44 = float(-M[0][:, 0] - M[1][:, 1] + M[2][:, 2])
        N12 = float(M[1][:, 2] - M[2][:, 1])
        N13 = float(M[2][:, 0] - M[0][:, 2])
        N14 = float(M[0][:, 1] - M[1][:, 0])
        N21 = float(N12)
        N23 = float(M[0][:, 1] + M[1][:, 0])
        N24 = float(M[2][:, 0] + M[0][:, 2])
        N31 = float(N13)
        N32 = float(N23)
        N34 = float(M[1][:, 2] + M[2][:, 1])
        N41 = float(N14)
        N42 = float(N24)
        N43 = float(N34)
    
        N = numpy.matrix([[N11, N12, N13, N14], \
                  [N21, N22, N23, N24], \
                  [N31, N32, N33, N34], \
                  [N41, N42, N43, N44]])
    
    
        values, vectors = numpy.linalg.eig(N)
        w = list(values)
        mw = max(w)
        quat = vectors[:, w.index(mw)]
        quat = numpy.array(quat).reshape(-1,).tolist()
        return quat
 
    def update_point(self, ClickedPoint):
            nPoint = [0, 0, 0]
            nPoint[0] = ClickedPoint.x
            nPoint[1] = ClickedPoint.y
            nPoint[2] = ClickedPoint.z
            # return [ClickedPoint.x,ClickedPoint.y,ClickedPoint.z]
            return nPoint
 
    def calculate_pose(self, point1, point2, point3):

            # Create Y axis and Z Axis
            VY = self.subtractPoints(point1, point2)
            VZ = numpy.cross(self.subtractPoints(point1, point2)
                                                  , self.subtractPoints
                                                  (point1, point3))
            
            # normalize Y and Z vectors
            VYn = VY / numpy.linalg.norm(VY)
            VZn = VZ / numpy.linalg.norm(VZ)
            
            # ensure that Z is correct orientation
            if numpy.abs(VZn[1]) > numpy.abs(VZn[2]):
                # Vy must be negative.
                VZn = -numpy.sign(VZn[1]) * VZn
            if numpy.abs(VZn[1]) < numpy.abs(VZn[2]):
                # Vz must be negative.
                VZn = -numpy.sign(VZn[2]) * VZn
            # create X vector
            VX = numpy.cross(VYn, VZn)
            
            # normalize the X vector
            VXn = VX / numpy.linalg.norm(VX)
            myFrame = [VXn, VYn, VZn]
            
            # Create Identity matrix
            nX = [1, 0, 0]
            nY = [0, 1, 0]
            nZ = [0, 0, 1]
            normalFrame = [nX, nY, nZ]
            
            # return transformation from one coordinate frame to another
            quat = self.get_quaternion(normalFrame, myFrame, matchlist=None)
            
            
            # find centroid of the three points to use as coordinates
            self.pose_n.pose.position.x = (point1[0] + point2[0] + point3[0]) / 3 + 0.04 * VZn[0]
            self.pose_n.pose.position.y = (point1[1] + point2[1] + point3[1]) / 3 + 0.04 * VZn[1]
            self.pose_n.pose.position.z = (point1[2] + point2[2] + point3[2]) / 3 + 0.04 * VZn[2]
            self.pose_n.pose.orientation.w = quat[0]
            self.pose_n.pose.orientation.x = quat[1]
            self.pose_n.pose.orientation.y = quat[2]
            self.pose_n.pose.orientation.z = quat[3]
           
            # stop pose
            
            # find centroid of the three points to use as coordinates and move
            # along the Z axis back .1 meters
            self.pose_s.pose.position.x = (point1[0] + point2[0] 
                                           + point3[0]) / 3
            self.pose_s.pose.position.y = (point1[1] + point2[1] 
                                           + point3[1]) / 3
            self.pose_s.pose.position.z = (point1[2] + point2[2] 
                                           + point3[2]) / 3 + 0.20
            self.pose_s.pose.orientation.w = quat[0]
            self.pose_s.pose.orientation.x = quat[1]
            self.pose_s.pose.orientation.y = quat[2]
            self.pose_s.pose.orientation.z = quat[3]    
    
    def subtractPoints(self, pf, pi):
            return numpy.subtract(pf, pi)  
                 
def main():
    run = PointClick()
    run.point_clicker()

if __name__ == '__main__':
    sys.exit(main())
