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
from BaxterUtil import *

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

# TODO: add this to launch script along with openni.launch
# roslaunch baxter_controller baxter_controller.launch

# NEWEST DOWNWARD FACING: rosrun tf static_transform_publisher 0.24 0.018 0.555 -0.000 0.135 0.005 0.5 /base /camera_link 25
# NEWEST FOWARD FACING: rosrun tf static_transform_publisher 0.1 0.018 0.9 -0.000 0.0 0.005 0.5 /base /camera_link 25

# POWERBOT ssh guest@192.168.2.112
# Pass: mobilerobot
# cd /usr/local/Arnl
# ./arnlServer

# roslaunch openni_launch openni.launch device_id:=B00364613926048B (downward)
# roslaunch openni_launch openni.launch device_id:=A00367801249047A (upward)
# TO GET KINECT ID: lsusb -v -d 045e:02ae | grep -e "Bus\|iSerial"

class BaxterController:
    def __init__(self):
        rospy.init_node('BaxterController', anonymous=True)
        self.rate = rospy.Rate(60) # 10hz

        self.my_env = os.environ
        self.openni_process = None

        # Baxter Action Interface
        rospy.loginfo("Initializing Baxter Action Interface")
        self.action = BaxterAction.BaxterAction()
        self.action.calibrateGripper(selectedGripper = "left")
        self.action.calibrateGripper(selectedGripper = "right")

        cv2.namedWindow("KEY_TO_SHUTOFF")
        #self.action.resetArms()
        '''
        # Baxter Vision Interface     
        self.vision = BaxterVision.BaxterVision()
        rospy.loginfo("Waiting for Kinect data...")
        while(not self.vision.has_depth_data and not self.vision.has_rgb_data):
            pass
        rospy.loginfo("Kinect Data Received")

        #self.action.moveGripper(selectedGripper = 1, pos = self.vision.r_hand_state, openClose = None)
        
        #-----------------------------------------------------------#
        # MIMIC DEMO
        #-----------------------------------------------------------#
        #self.action.mimic(self.vision)
        #-----------------------------------------------------------#
        '''
        #self.action.command_retrieve_book("kyle", "computer vision")
        #self.action.command_retrieve_book("kyle", "python pocket reference")
        #self.action.command_retrieve_book("kyle", "python pocket reference")
        #self.action.command_retrieve_book("kyle", "the x files fight the future")
        #self.action.command_retrieve_book("kyle", "stiquito for beginners")
        #self.action.command_sort()

        #self.action.measureDeformation(1)

        #self.action.command_grasp_object('white')
        self.action.command_retrieve('kyle', 'white')
        '''
        #self.action.record(self.vision)

        #self.action.playback()
        '''
        #self.action.command_sort()
        #rospy.sleep(2)
        #self.vision.calibrateCamera()

        #self.action.command_grasp_object_moveit('pink')



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
