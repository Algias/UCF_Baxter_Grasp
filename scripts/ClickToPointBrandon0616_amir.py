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
        #rospy.loginfo(rospy.get_caller_id() + " \n %s", self.clickedPoint)
        
    def listen(self):
        rospy.Subscriber("/clicked_point", PointStamped, self.callback)
        rospy.sleep(0.2)
      
def updatePoint(ClickedPoint):
        nPoint = [0,0,0]
        nPoint[0] = ClickedPoint.x
        nPoint[1] = ClickedPoint.y
        nPoint[2] = ClickedPoint.z
        #return [ClickedPoint.x,ClickedPoint.y,ClickedPoint.z]
        return nPoint

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

class clcpose:
     def __init__(self):
         self.pose_n = geometry_msgs.msg.PoseStamped()
         self.pose_s = geometry_msgs.msg.PoseStamped()

     def calculatePose(self,point1,point2,point3):
            #Create Y axis and Z Axis
            VY = subtractPoints(point1,point2)
            VZ = numpy.cross(subtractPoints(point1,point2)
                                                  ,subtractPoints(point1,point3))
            
            #normalize Y and Z vectors
            VYn = VY/numpy.linalg.norm(VY)
            VZn = VZ/numpy.linalg.norm(VZ)
            
            #ensure that Z is correct orientation
            if numpy.abs(VZn[1]) > numpy.abs(VZn[2]):
                #Vy must be negative.
                VZn = -numpy.sign(VZn[1]) * VZn
            if numpy.abs(VZn[1]) < numpy.abs(VZn[2]):
                #Vz must be negative.
                VZn = -numpy.sign(VZn[2]) * VZn
            #create X vector
            VX = numpy.cross(VYn,VZn)
            
            #normalize the X vector
            VXn = VX/numpy.linalg.norm(VX)
            myFrame = [VXn,VYn,VZn]
            
            #Create Identity matrix
            nX = [1,0,0]
            nY = [0,1,0]
            nZ = [0,0,1]
            normalFrame = [nX,nY,nZ]
            
            #return transformation from one coordinate frame to another
            quat = get_quaternion(normalFrame, myFrame, matchlist = None)
            
            
            self.pose_n.header.frame_id = "base"
            #find centroid of the three points to use as coordinates
            self.pose_n.pose.position.x = (point1[0] + point2[0] + point3[0])/3 +0.04*VZn[0]
            self.pose_n.pose.position.y = (point1[1] + point2[1] + point3[1])/3 +0.04*VZn[1]
            self.pose_n.pose.position.z = (point1[2] + point2[2] + point3[2])/3 +0.04*VZn[2]
            self.pose_n.pose.orientation.w = quat[0]
            self.pose_n.pose.orientation.x = quat[1]
            self.pose_n.pose.orientation.y = quat[2]
            self.pose_n.pose.orientation.z = quat[3]
           
            # stop pose
            
            self.pose_s.header.frame_id = "base"
            #find centroid of the three points to use as coordinates
            self.pose_s.pose.position.x = (point1[0] + point2[0] + point3[0])/3 -0.10*VZn[0]
            self.pose_s.pose.position.y = (point1[1] + point2[1] + point3[1])/3 -0.10*VZn[1]
            self.pose_s.pose.position.z = (point1[2] + point2[2] + point3[2])/3 -0.10*VZn[2]
            self.pose_s.pose.orientation.w = quat[0]
            self.pose_s.pose.orientation.x = quat[1]
            self.pose_s.pose.orientation.y = quat[2]
            self.pose_s.pose.orientation.z = quat[3]
     
               
def pointClicker():
    print "*****Starting pointClicker Node"
    rospy.init_node('pointClicker',
                  anonymous=True)

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
    
    point1 = []
    point2 = []
    point3 = []
    previousPoint = geometry_msgs.msg.Point()
    n = 0
    while not rospy.is_shutdown():
        if n == 4 and clickPoint.clickedPoint.point != previousPoint:
            n = 2
            point1 = updatePoint(clickPoint.clickedPoint.point)
            point2 = []
            point3 = []
            previousPoint = clickPoint.clickedPoint.point
            print "point1:",point1
            print "point2:",point2
            print "point3:",point3


        if n < 4:
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
            
        rate = rospy.Rate(5)  # 1hz
        rate.sleep()
        obj1 = clcpose()
        if point1 and point2 and point3:
            obj1.calculatePose(point1, point2, point3)
            
        pubpose1.publish(obj1.pose_s)
        pubpose2.publish(obj1.pose_n)
        rate = rospy.Rate(1)  # 1hz
        rate.sleep()    
    rospy.spin()
    
def main():

    pointClicker()


if __name__ == '__main__':
    sys.exit(main())
