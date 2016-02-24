#!/usr/bin/env python
import rospy
import numpy as np
import tf
import tf_conversions
import math
import tf2_ros

rospy.init_node('matmaker', anonymous=True)

rate = rospy.Rate(60) # 10hz


tfl = tf.TransformListener()
t = tfl.getLatestCommonTime("/camera_depth_optical_frame", "/base" )

trans1 = None
rot1 = None
while trans1 == None:
	try:
		#(trans1, rot1) = tfl.lookupTransform('/camera_depth_optical_frame', '/base', t)
		(trans1, rot1) = tfl.lookupTransform('/base', '/camera_depth_optical_frame', t)
	except:
		pass

print "TRANS/ROT"
print trans1
print rot1

rot1 = (rot1[2], rot1[1], rot1[0], rot1[3])
rpy = tf.transformations.euler_from_quaternion(rot1)
print "RPY", rpy
print "RPY DEG", rpy[0] * 180.0/math.pi, rpy[1] * 180.0/math.pi, rpy[2] * 180.0/math.pi
rpyd = [0, 0, 0]
rpyd[0] = rpy[0] * 180.0/math.pi
rpyd[1] = rpy[1] * 180.0/math.pi
rpyd[2] = rpy[2] * 180.0/math.pi
print "QUAT1", tf.transformations.quaternion_from_euler(rpyd[2], rpyd[1], rpyd[0], 'rzyx')
#print "QUAT1", tf.transformations.quaternion_from_euler(-0.350506689052, -20.1555331021, 1.97197712758, 'rzyx')
print "QUAT2", tf.transformations.quaternion_from_euler(rpy[2], rpy[1], rpy[0], 'rzyx')

#rot2 = rot1
rot2 = [0, 0, 0, 0]
rot2[0] = rot1[3]
rot2[1] = rot1[0]
rot2[2] = rot1[1]
rot2[3] = rot1[2]
# WTF uses xyzw convention -> pcl uses wxyz convention FIX?
trans1_mat = tf.transformations.translation_matrix(trans1)

print rot1
rot1_mat   = tf.transformations.quaternion_matrix(rot2)

print "TRANS"
print trans1_mat

print "ROT"
print rot1_mat
mat1 = np.dot(trans1_mat, rot1_mat)

print mat1
rospy.spin()