#!/usr/bin/env python

import rospy

import sys, os, sched, time, errno

import glob
import cv2
from cv2 import *

import multiprocessing
import numpy as np
import math

import smtplib
from cv2 import __version__

# Baxter
import BaxterAction
import BaxterVision
import BaxterPositions

import baxter_interface
from baxter_interface import CHECK_VERSION
from baxter_core_msgs.msg import EndpointState
from baxter_core_msgs.msg import EndEffectorState

from sensor_msgs.msg import Range

import sys
import copy

import time
import subprocess, signal, os


import cv2
from cv2 import *
import multiprocessing
import numpy as np

from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import (
    Image,
)

from subprocess import Popen
import os


class GripperState:
    def __init__(self, l_r):

        #-----------------------------------------------------------------------------------------#
        # State Info
        # gripper info (constantly updated through ros subscribers)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        self.range = 0.0
        self.force = 0.0

        self.rate = rospy.Rate(10) # 10hz
        self.l_r = ""
        #-----------------------------------------------------------------------------------------#
        # make ROS subscribers for corresponding arm
        if(l_r == "right"):
            self.l_r = "right"
            self.sub_gripperPos = rospy.Subscriber("robot/limb/right/endpoint_state", EndpointState, self.gripperPos_callback)
            self.sub_gripperRange = rospy.Subscriber("robot/range/right_hand_range/state", Range, self.gripperRange_callback)
            self.sub_gripperForce = rospy.Subscriber("robot/end_effector/right_gripper/state", EndEffectorState, self.gripperForce_callback)

        elif(l_r == "left"):
            self.l_r = "left"
            self.sub_gripperPos = rospy.Subscriber("robot/limb/left/endpoint_state", EndpointState, self.gripperPos_callback)
            self.sub_gripperRange = rospy.Subscriber("robot/range/left_hand_range/state", Range, self.gripperRange_callback)
            self.sub_gripperForce = rospy.Subscriber("robot/end_effector/left_gripper/state", EndEffectorState, self.gripperForce_callback)


    # callbacks
    def gripperPos_callback(self, data):
        self.x = data.pose.position.x
        self.y = data.pose.position.y
        self.z = data.pose.position.z
        #print self.x, self.y, self.z

    def gripperRange_callback(self, data):
        minVal = data.min_range
        maxVal = data.max_range

        if data.range > maxVal:
            self.range = maxVal
        elif data.range < minVal:
            self.range = minVal
        else:
            self.range = data.range

    def gripperForce_callback(self, data):
        self.force = float(data.force)


class BaxterController:
    def __init__(self):
        rospy.init_node('BaxterController', anonymous=True)
        self.rate = rospy.Rate(60) # 10hz

        self.my_env = os.environ
        self.openni_process = None

        self.gripperStateLeft = GripperState("left")
        self.gripperStateRight = GripperState("right")

        # Baxter Action Interface
        rospy.loginfo("Initializing Baxter Action Interface")
        self.action = BaxterAction.BaxterAction()
        self.action.calibrateGripper(selectedGripper = "left")
        self.action.calibrateGripper(selectedGripper = "right")

       
        #self.action.resetArms()
        '''
        # Baxter Vision Interface     
        self.vision = BaxterVision.BaxterVision()
        rospy.loginfo("Waiting for Kinect data...")
        while(not self.vision.has_depth_data and not self.vision.has_rgb_data):
            pass
        rospy.loginfo("Kinect Data Received")

        #self.action.moveGripper(selectedGripper = 1, pos = self.vision.r_hand_state, openClose = None)
        #self.action.mimic(self.vision)
        '''
        


        #rospy.sleep(2)
        #self.vision.calibrateCamera()



        '''
        # Calibrate Gripper
        rospy.loginfo("Calibrating Gripper")
        self.action.calibrateGripper("both")

        rospy.loginfo("Going to waiting position")
        self.action.goToWaiting("left")
        rospy.sleep(1)
        #self.action.goToWaiting("right")
        #rospy.sleep(1)
        '''




bx = BaxterController()

rospy.spin()


