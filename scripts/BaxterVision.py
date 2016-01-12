#!/usr/bin/env python
import rospy
import BaxterPositions
import baxter_interface
from BaxterUtil import *
import sys
import copy

import time
import subprocess, signal, os

import rospy
import roslib
import numpy as np
import cv2
from cv2 import *
import math

import tf
from tf import transformations
from tf import TransformListener
from geometry_msgs.msg import TransformStamped

from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float64
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker
from sensor_msgs.msg import (
    Image
)

from subprocess import Popen
import os

from tf.transformations import quaternion_from_euler, euler_from_quaternion
from std_msgs.msg import (
	Header,
	UInt16,
	Int32MultiArray
)
from sensor_msgs.msg import (
	Image,
	JointState
)
from geometry_msgs.msg import (
	PoseStamped,
	Pose,
	Point,
	Quaternion,
)

DEPTH_WINDOW = "Depth Window"
RGB_WINDOW = "RGB Window"
PROCESS_WINDOW = "Process Window"
LEFT_HAND_WINDOW = "Left Hand"
RIGHT_HAND_WINDOW = "Right Hand"

class BaxterVision:
	def __init__(self):
		rospy.loginfo("Initializing Baxter Vision Interface")
		cv2.namedWindow(PROCESS_WINDOW, WINDOW_AUTOSIZE)
		cv2.namedWindow(RGB_WINDOW, WINDOW_AUTOSIZE)
		cv2.namedWindow(DEPTH_WINDOW, WINDOW_AUTOSIZE)

		self.bridge = CvBridge()

		self.br = tf.TransformBroadcaster()
		self.tf = tf.Transformer(True, rospy.Duration(10.0))
		self.tf_listener = TransformListener()

		self.l_hand_state_sub = rospy.Subscriber("/gesture/hand_state/left", Float64, self.l_hand_state_cb)
		self.r_hand_state_sub = rospy.Subscriber("/gesture/hand_state/right", Float64, self.r_hand_state_cb)
		self.l_hand_state = 50
		self.r_hand_state = 50

		self.has_rgb_data = False
		self.has_depth_data = False
		self.img_kinect_depth = None
		self.img_kinect_rgb = None
		self.img_kinect_gray = None
		self.img_calibration = None

		self.sub_kinect_depth = rospy.Subscriber("/camera/depth/image_raw", Image, self.kinect_depth_callback)
		self.sub_kinect_rgb = rospy.Subscriber("/camera/rgb/image_color", Image, self.kinect_rgb_callback)

		self.pub_viz_campose = rospy.Publisher("/viz/camera_marker", Marker)
		self.pub_kinect_tf = rospy.Publisher("/viz/camera_tf", TransformStamped)
		
		# camera pose averaging filters
		self.camera_pos_filters = [dataFilter(3), dataFilter(3), dataFilter(3)]
		self.camera_rot_filters = [dataFilter(3), dataFilter(3), dataFilter(3)]

	def kinect_depth_callback(self, data):
		try:
			self.img_kinect_depth = self.bridge.imgmsg_to_cv2(data, "16UC1")
			cv2.normalize(self.img_kinect_depth, self.img_kinect_depth, 0, 65535, NORM_MINMAX, CV_16UC1)
			#cv2.bitwise_not(self.img_kinect_depth, self.img_kinect_depth)
			if(self.img_kinect_depth is not None):
				self.has_depth_data = True
				#imshow(DEPTH_WINDOW, self.img_kinect_depth)
				#waitKey(10)

		except CvBridgeError, e:
			print e

	def kinect_rgb_callback(self, data):
		try:
			self.img_kinect_rgb = self.bridge.imgmsg_to_cv2(data, "bgr8")
			self.img_kinect_gray = cv2.cvtColor(self.img_kinect_rgb, cv2.COLOR_BGR2GRAY)

			if(not self.img_kinect_rgb is None):
				self.has_rgb_data = True
				#imshow(RGB_WINDOW, self.img_kinect_rgb)
				#waitKey(10)

				
		except CvBridgeError, e:
			print e

	def l_hand_state_cb(self, data):
		val = data.data * 180.0
		if val > 100:
			val = 100
		if val < 0:
			val = 0

		self.l_hand_state = val

	def r_hand_state_cb(self, data):
		val = data.data * 180.0
		if val > 100:
			val = 100
		if val < 0:
			val = 0

		self.r_hand_state = val
		print "right hand: ", self.r_hand_state

	def draw(self, img, corners, imgpts):
		corner = tuple(corners[0].ravel())
		cv2.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 5)
		cv2.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 5)
		cv2.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 5)
		return img

	def calibrateCamera(self):
		# termination criteria
		criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

		# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
		objp = np.zeros((6*8,3), np.float32)
		objp[:,:2] = np.mgrid[0:8,0:6].T.reshape(-1,2)

		# Arrays to store object points and image points from all the images.
		objpoints = [] # 3d point in real world space
		imgpoints = [] # 2d points in image plane.
		num_samples = 0
		axis = np.float32([[3,0,0], [0,3,0], [0,0,-3]]).reshape(-1,3)
		print axis

		has_checker = False
		while(1):
		    img = self.img_kinect_rgb
		    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
		    self.img_calibration = self.img_kinect_rgb.copy()

		    # Find the chess board corners
		    ret, corners = cv2.findChessboardCorners(gray, (8,6),None)
		    #imshow("gray", gray)
		    
		    
		    # If found, add object points, image points (after refining them)
		    if ret == True:
				#num_samples += 1
				has_checker = True

				#print "Sameple added: " + str(num_samples)
				objpoints.append(objp)

				cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
				imgpoints.append(corners)

				# Draw and display the corners
				#cv2.drawChessboardCorners(self.img_calibration, (8,6), corners,ret)
				#cv2.imshow('Checker Board',self.img_calibration)
				#waitKey(10)

				mtx = []
				dist = []
				rvecs = []
				tvecs = []

				ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
				rvecs, tvecs, inliers = cv2.solvePnPRansac(objp, corners, mtx, dist)

				
				# camera pose from object points
				RMat = cv2.Rodrigues(rvecs)[0]
				cam_pos = -np.matrix(RMat).T * np.matrix(tvecs)

				'''
				# get Euler angles form R
				P = np.float32([[RMat[0][0], RMat[0][1], RMat[0][2], cam_pos[0]], [RMat[1][0], RMat[1][1], RMat[1][2], cam_pos[1]], [RMat[2][0], RMat[2][1], RMat[2][2], cam_pos[2]]]).reshape(3, -1)
				eulerAngles = cv2.decomposeProjectionMatrix(P)[-1]
				'''

				print(chr(27) + "[2J")
				print "-------T-------"
				print cam_pos
				print "-------R-------"
				print rvecs
				#print "-------Euler Angles-------"
				#print eulerAngles
				
				
				'''
				self.cam_marker = Marker()
				self.cam_marker.header.frame_id = "camera_link"
				self.cam_marker.header.stamp = rospy.Time()
				self.cam_marker.ns = "/viz"
				self.cam_marker.id = 0
				self.cam_marker.type = Marker.CUBE
				self.cam_marker.action = Marker.ADD
				self.cam_marker.scale.x = 1.0
				self.cam_marker.scale.y = 1.0
				self.cam_marker.scale.z = 1.0
				self.cam_marker.color.a = 1.0
				self.cam_marker.color.g = 1.0
				self.cam_marker.pose.orientation.x = eulerAngles[0]
				self.cam_marker.pose.orientation.y = eulerAngles[1]
				self.cam_marker.pose.orientation.z = eulerAngles[2]
				self.cam_marker.pose.orientation.w = 1.0
				self.cam_marker.pose.position.x = cam_pos[0]
				self.cam_marker.pose.position.y = cam_pos[1]
				self.cam_marker.pose.position.z = cam_pos[2]
				
				self.pub_viz_campose.publish(self.cam_marker)
				'''
				'''
				roll = atan2(-cam_pos[2][1], cam_pos[2][2])
				pitch = asin(cam_pos[2][0])
				yaw = atan2(-cam_pos[1][0], cam_pos[0][0])
				'''

				# visualization
				'''
				imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, mtx, dist)
				img_project = self.draw(self.img_calibration,corners,imgpts)
				cv2.imshow('Checker Board',self.img_calibration)
				waitKey(10)
				'''

				# tf publishing
				m = TransformStamped()
				m.header.frame_id = "left_wrist"
				m.child_frame_id = "kinect"
				m.transform.translation.x = cam_pos[0]
				m.transform.translation.y = cam_pos[1]
				m.transform.translation.z = cam_pos[2]

				m.transform.rotation.x = (rvecs[0])[0]
				m.transform.rotation.y = (rvecs[1])[0]
				m.transform.rotation.z = (rvecs[2])[0]
				#m.transform.rotation.w = (rvecs[3])[0]
				self.tf.setTransform(m)

				
				self.camera_pos_filters[0].add(cam_pos[2]/50.0)
				self.camera_pos_filters[0].average()

				self.camera_pos_filters[1].add(-cam_pos[1]/50.0)
				self.camera_pos_filters[1].average()

				self.camera_pos_filters[2].add(cam_pos[0]/50.0)
				self.camera_pos_filters[2].average()

				self.camera_rot_filters[0].add(((rvecs[2])[0]))     # aboiut gren
				self.camera_rot_filters[0].average()
				if(self.camera_rot_filters[0].size > 0):
					print self.camera_rot_filters[0].avg

				self.camera_rot_filters[1].add(-(rvecs[1])[0]) 		# about blue
				self.camera_rot_filters[1].average()
				if(self.camera_rot_filters[1].size > 0):
					print self.camera_rot_filters[1].avg

				self.camera_rot_filters[2].add(((rvecs[0])[0]))     # about red
				self.camera_rot_filters[2].average()
				if(self.camera_rot_filters[2].size > 0):
					print self.camera_rot_filters[2].avg
				

				self.br.sendTransform((self.camera_pos_filters[0].avg, self.camera_pos_filters[1].avg, self.camera_pos_filters[2].avg), (self.camera_rot_filters[0].avg, self.camera_rot_filters[1].avg, self.camera_rot_filters[2].avg, -math.pi/10.0), rospy.Time.now(), "kinect", "left_wrist")

				imgpoints = []
				objpoints = []

				'''
			if(has_checker and num_samples > 20):

				# calibrate the camera
				ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)

				h,  w = gray.shape[:2]
				newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))
				print newcameramtx
				print "-------------"
				
				dst = cv2.undistort(gray, mtx, dist, None, newcameramtx)
				cv2.imshow('After Calibration',self.img_calibration)
				
				cv2.waitKey(-1)
				return
				'''

		cv2.destroyAllWindows()
