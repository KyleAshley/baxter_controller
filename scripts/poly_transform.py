#!/usr/bin/env python
import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import tf


def T_Matrix(x, y, z):
	return np.array([[1, 0, 0, x],
					 [0, 1, 0, y], 
					 [0, 0, 1, z], 
					 [0, 0,	0, 1]])

def Rx_Matrix(theta):
	return np.array([[math.cos(theta), 	-math.sin(theta), 	0, 0],
					 [math.sin(theta), 	math.cos(theta), 	0, 0], 
					 [0, 				0, 					1, 0], 
					 [0, 				0, 					0, 1]])

def Ry_Matrix(theta):
	return np.array([[math.cos(theta), 	0, math.sin(theta), 0], 
					 [0, 				1, 0, 				0], 
					 [-math.sin(theta), 0, math.cos(theta), 0], 
					 [0, 				0, 0, 				1]])

def Rz_Matrix(theta):
	return np.array([[1, 0, 				0, 					0], 
					 [0, math.cos(theta), 	-math.sin(theta), 	0], 
					 [0, math.sin(theta), 	math.cos(theta), 	0], 
					 [0, 0, 				0, 					1]])

# by hand
#rosrun tf static_transform_publisher 0.537395 0.1491 1.152432 -0.00477489, 1.250298, 0.134348, -0.0351965 /camera_link /base

# /camera_link /base
# second time? rosrun tf static_transform_publisher 0.01290 -0.0411 -0.50 0.00 -20.0 0.00 180.0 /camera_link /base 30

# From Kinext Calibration-> xyz 0.011797, 0.153913, 0.955444, xyzw: -0.00477489, 0.990298, 0.134348, -0.0351965
T = T_Matrix(0.01290, -0.0411, -0.50)
print "T", T

quaternion = (0.00, -20.0, 0.00, 180.0)
euler = tf.transformations.euler_from_quaternion(quaternion)
roll = euler[0]
pitch = euler[1]
yaw = euler[2]

Rz = Rz_Matrix(pitch)
Ry = Ry_Matrix(yaw)
Rx = Rx_Matrix(roll)

R = np.dot(np.dot(Rz, Ry), Rx)
RT = np.dot(R, T)

f = open('kinectToWorld.mat', 'w')
f.write((str(RT).strip('[')).strip(']').strip('[').strip(']'))
print "RT", RT



'''
# example
f = 3
xa0 = np.array([0, 0, 0])
#c = np.array(np.dot(xa0, T_Matrix()))
xac = np.array([10, 10, 10])

vertices = np.array([[0, 0, 0, 1], [0, 0, 1, 1], [0, 1, 0, 1], [1, 0, 0, 1], [1, 1, 1, 1], [1, 1, 0, 1], [1, 0, 1, 1], [0, 1, 1, 1]])
edges = np.array([[1, 2], [1, 3], [1, 4], [5, 6], [5, 7], [5, 8], [2, 7], [2, 8], [3, 6], [3, 8], [4, 6], [4,7]])
edges = edges.transpose()

print "Input"
#print c
print f
print vertices
print edges


# test with origin
yaw = getYaw(xa0, xac, f)
print "YAW:", yaw

pitch = getPitch(xa0, xac, f)
print "PITCH:", pitch


T = T_Matrix(xa0[0], xa0[1], xa0[2])
Rx = Rx_Matrix(pitch)
Ry = Ry_Matrix(yaw)
C = np.dot(np.dot(Rx, Ry), T)
print C

for v in range(len(vertices)):
	vertices[v] = np.dot(C, np.array([vertices[v]]).transpose())
'''


'''
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

plt.show()
'''

'''
# Translation is defined by camera coordinates
T_Matrix(c[0], c[1], c[2])
Rx_Matrix()
Ry_Matrix()
Rz_Matrix()

print Rz(1.0)
#print getProjectionMatrix(f)
#print thetaP(c, x)
'''
