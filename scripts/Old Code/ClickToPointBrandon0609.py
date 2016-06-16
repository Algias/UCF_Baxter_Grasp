#!/usr/bin/env python
import sys
import copy
import rospy
import geometry_msgs.msg
from std_msgs.msg import String
from geometry_msgs.msg import (
    PoseStamped,
    PointStamped,
    Pose,
    Point,
    Quaternion,
)
from rospy.numpy_msg import numpy_msg
import numpy
from std_msgs.msg import Header
class PointClick:

        
    def __init__(self):
        self.clickedPoint = PointStamped()
        
    def callback(self, data):
        self.clickedPoint = data;
        rospy.loginfo(rospy.get_caller_id() + " \n %s", self.clickedPoint)
        
    def listen(self):
        rospy.Subscriber("/clicked_point", PointStamped, self.callback)
        rospy.sleep(0.2)
        
def updatePoint(ClickedPoint):
        return [ClickedPoint.x,ClickedPoint.y,ClickedPoint.z]

def subtractPoints(pf,pi):
        return numpy.subtract(pf,pi)
def get_quaternion(lst1,lst2,matchlist=None):
 if not matchlist:
     matchlist=range(len(lst1))
 M=numpy.matrix([[0,0,0],[0,0,0],[0,0,0]])

 for i,coord1 in enumerate(lst1):
     x=numpy.matrix(numpy.outer(coord1,lst2[matchlist[i]]))
     M=M+x

 N11=float(M[0][:,0]+M[1][:,1]+M[2][:,2])
 N22=float(M[0][:,0]-M[1][:,1]-M[2][:,2])
 N33=float(-M[0][:,0]+M[1][:,1]-M[2][:,2])
 N44=float(-M[0][:,0]-M[1][:,1]+M[2][:,2])
 N12=float(M[1][:,2]-M[2][:,1])
 N13=float(M[2][:,0]-M[0][:,2])
 N14=float(M[0][:,1]-M[1][:,0])
 N21=float(N12)
 N23=float(M[0][:,1]+M[1][:,0])
 N24=float(M[2][:,0]+M[0][:,2])
 N31=float(N13)
 N32=float(N23)
 N34=float(M[1][:,2]+M[2][:,1])
 N41=float(N14)
 N42=float(N24)
 N43=float(N34)

 N=numpy.matrix([[N11,N12,N13,N14],\
              [N21,N22,N23,N24],\
              [N31,N32,N33,N34],\
              [N41,N42,N43,N44]])


 values,vectors=numpy.linalg.eig(N)
 w=list(values)
 mw=max(w)
 quat= vectors[:,w.index(mw)]
 quat=numpy.array(quat).reshape(-1,).tolist()
 return quat

def calculatePose(point1,point2,point3):
        print "p1 - p2: ", subtractPoints(point1,point2)
        print "(p1-p2)x(p1-p3): ", numpy.cross(subtractPoints(point1,point2)
                                              ,subtractPoints(point1,point3))
        VY = subtractPoints(point1,point2)
        VZ = numpy.cross(subtractPoints(point1,point2)
                                              ,subtractPoints(point1,point3))
        VX = numpy.cross(VY,VZ)
        #normalize the vectors
        VYn = VY/numpy.linalg.norm(VY)
        VZn = VZ/numpy.linalg.norm(VZ)
        VXn = VX/numpy.linalg.norm(VX)
        myFrame = [VXn,VYn,VZn]
        #Create Identity matrix
        nX = [1,0,0]
        nY = [0,1,0]
        nZ = [0,0,1]
        normalFrame = [nX,nY,nZ]
        #return transformation from one coordinate frame to another
        quat = get_quaternion(normalFrame, myFrame, matchlist = None)
        
        pose_n = geometry_msgs.msg.PoseStamped()
        pose_n.header.frame_id = "base"
        pose_n.pose.position.x = point1[0]
        pose_n.pose.position.y = point1[1]
        pose_n.pose.position.z = point1[2]
        pose_n.pose.orientation.w = quat[0]
        pose_n.pose.orientation.x = quat[1]
        pose_n.pose.orientation.y = quat[2]
        pose_n.pose.orientation.z = quat[3]
        print quat
        print pose_n
        return pose_n 
               
def pointClicker():
    print "*****Starting pointClicker Node"
    rospy.init_node('pointClicker',
                  anonymous=True)
    
    #Create Point objects
#     point1 = geometry_msgs.msg.Point()
#     point2 = geometry_msgs.msg.Point()
#     point3 = geometry_msgs.msg.Point()
    point1 = []
    point2 = []
    point3 = []
    previousPoint = geometry_msgs.msg.Point()
    
    #Create Pose objects
    pose_target1 = geometry_msgs.msg.PoseStamped()
    pose_target2 = geometry_msgs.msg.PoseStamped()
    pose_targetPrevious = geometry_msgs.msg.PoseStamped()
    
    #Instantiate PointClick class
    clickPoint = PointClick()
    
    #Subscribe to /clicked_point
    clickPoint.listen()
    
    #Create publishers
    pubpose1 = rospy.Publisher('mypose1', PoseStamped, queue_size=10)
    pubpose2 = rospy.Publisher('mypose2', PoseStamped, queue_size=10)
    
    #Point counter
    n = 0

    while not rospy.is_shutdown() and n < 4:
        if clickPoint.clickedPoint.point != previousPoint:
            if n != 0: print "enter point number: ", n    
            if n == 1:
                point1 = updatePoint(clickPoint.clickedPoint.point)
                
            if n == 2:
                point2 = updatePoint(clickPoint.clickedPoint.point)
                
            if n == 3:
                point3 = updatePoint(clickPoint.clickedPoint.point)
            print "point1:",point1
            print "point2:",point2
            print "point3:",point3                
            n+=1

        previousPoint = clickPoint.clickedPoint.point
            
   

        #pubpose2.publish(pose_target1)
        
        rate = rospy.Rate(1)  # 1hz
        rate.sleep()
    pubpose1.publish(calculatePose(point1, point2, point3))

    rospy.spin()
    
def main():

    pointClicker()


if __name__ == '__main__':
    sys.exit(main())
