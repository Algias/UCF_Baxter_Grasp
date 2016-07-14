#!/usr/bin/env python

import rospy
import numpy as np
import scipy.stats
from pylab import *
import random

class ICPClass:

     
    def T(self,x, T0, T1, k=1.0):
        # apply an affine transformation to `x`
        y = x * T0.T
        y[:, 0] += T1[0, 0]
        y[:, 1] += T1[1, 0]
        return y * k
     

    
    def shuffle(self,X):
        N = X.shape[0]
        idx = range(N)
        np.random.shuffle(idx)
        return X[idx]
     
    
    def translate(self,X, Y, errorfct):
        # translate to align the centers of mass
        mx = np.mean(X, axis=0).T
        my = np.mean(Y, axis=0).T
        translation = mx - my
        I = np.matrix(np.eye(2))
        Yp = self.T(Y, I, translation)
        return errorfct(X, Yp), translation
     
    def randrot(self,X, Y, errorfct):
        # perform a random rotation
        theta = scipy.stats.uniform(0.0, 2.0 * np.pi).rvs()
        c = np.cos(theta)
        s = np.sin(theta)
        rotation = np.matrix([[c, -s], [s, c]])
        Z = np.matrix(np.zeros((2, 1)))
        Yp = T(Y, rotation, Z)
        return errorfct(X, Yp), rotation
     
    def randscale(self,X, Y, errorfct):
        # perform a random scaling
        k = scipy.stats.uniform(0.5, 1.0).rvs()
        scaling = k * np.matrix(np.eye(2))
        Z = np.matrix(np.zeros((2, 1)))
        Yp = T(Y, scaling, Z)
        return errorfct(X, Yp), scaling
    
    def SSE(self,X, Y): 
        return np.sum(np.array(X - Y) ** 2.0)
     
    def ptSSE(self,pt, X):
        '''
        Point-wise smallest squared error.
        This is the distance from the point `pt`
        to the closest point in `X`
        '''
        difference = pt - X
        # x and y columns
        xcol = np.ravel(difference[:, 0])
        ycol = np.ravel(difference[:, 1])
        # sum of the squared differences btwn `pt` and `X`
        sqr_difference = xcol ** 2.0 + ycol ** 2.0
        # nearest squared distance
        distance = np.min(sqr_difference)
        # index of the nearest point to `pt` in `X`
        nearest_pt = np.argmin(sqr_difference)
        return distance
     
    def NSSE(self,X, Y):
        '''
        Nearest sum squared error.
        This is the sum of the squares of the
        nearest differences between the points
        of `X` and the points of `Y`
        '''
        err = 0.0
        for x in X:
            err += self.ptSSE(x, Y)
    
    def fit(self,X, Y, M, N, errorfct, threshold=1e-5):
        T0 = list()
        T1 = list()
        errors = list()
        errors.append(errorfct(X, Y))
        print errors[-1]
        Yp = Y.copy()
        for iter in range(M):
     
            err, translation = self.translate(X, Yp, errorfct)
            if err < threshold:
                break
            elif err < errors[-1]:
                errors.append(err)
                print errors[-1]
                T1.append(translation)
                I = np.matrix(np.eye(2))
                Yp = T(Yp, I, T1[-1])
                 
            rot = [ randrot(X, Yp, errorfct) for i in range(N) ] 
            rot.sort()
            err, rotation = rot[0]
            if err < threshold:
                break
            elif err < errors[-1]:
                errors.append(err)
                print errors[-1]
                T0.append(rotation)
                Z = np.matrix(np.zeros((2, 1)))
                Yp = T(Yp, T0[-1], Z)
                 
            scale = [ randscale(X, Yp, errorfct) for i in range(N) ]
            scale.sort()
            err, scaling = scale[0]
            if err < threshold:
                break
            elif err < errors[-1]:
                errors.append(err)
                print errors[-1]
                T0.append(scaling)
                Z = np.matrix(np.zeros((2, 1)))
                Yp = T(Yp, T0[-1], Z)
                 
        return Yp
    
icpclass = ICPClass()

# X = np.matrix(scipy.stats.norm(0, 100).rvs((500, 2)))
#  
# # rotate by some angle theta
# theta = scipy.stats.uniform(0.0, 2.0 * np.pi).rvs()
# c, s = np.cos(theta), np.sin(theta)
# # rotation matrix
# T0 = np.matrix([[c, -s], [s, c]])
# # translation vector
# T1 = np.matrix(scipy.stats.norm((3, 3), 1).rvs((2, 1)))
# # scaling factor
# k = scipy.stats.uniform(1, 5).rvs()
# 
# Y = icpclass.T(X, T0, T1, k)
# 
# sY = icpclass.shuffle(Y)

X = [[0,1,2],[3,4,5],[6,7,8]]

sY = [[6,7,8],[0,1,2],[3,4,5]]

Yp = icpclass.fit(X, sY, M=30, N=100, errorfct= icpclass.NSSE, threshold=1e-3)
scatter(np.ravel(X[:, 1]), np.ravel(X[:, 0]), marker='x', facecolor='b', s=100)
scatter(np.ravel(Yp[:, 1]), np.ravel(Yp[:, 0]), marker='o', edgecolor='r', facecolor='none', s=100)
grid() ; title('ICP with 100 Points')
savefig('icp_100pts.png', fmt='png', dpi=200)
