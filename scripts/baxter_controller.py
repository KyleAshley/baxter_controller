#!/usr/bin/env python
'''
#************************************************************************************
#   Baxter Command Node
#   Author: Kyle Ashley
#************************************************************************************
'''

# Baxter
import baxter_interface
from baxter_interface import CHECK_VERSION
from baxter_core_msgs.msg import EndpointState
from baxter_core_msgs.msg import EndEffectorState

from sensor_msgs.msg import Range

import threading
import sys
import copy

import time
import subprocess, signal, os, gtk.gdk

from thread import *

import rospy
import cv2

from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray
import cv_bridge
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
        print self.x, self.y, self.z

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

class BaxterActionServer:
    def __init__(self):
        # initialize interfaces
        rs = baxter_interface.RobotEnable(CHECK_VERSION)
        init_state = rs.state().enabled

        # Grippers
        self.leftGripper = baxter_interface.Gripper('left', CHECK_VERSION)
        self.rightGripper = baxter_interface.Gripper('right', CHECK_VERSION)

        # Limbs
        self.leftLimb = baxter_interface.Limb('left')
        self.rightLimb = baxter_interface.Limb('right')
        
        # JOINT SPEEDS
        self.manualJointAccuracy = baxter_interface.settings.JOINT_ANGLE_TOLERANCE
        self.manualJointTimeout = 20.0
        self.manualJointSpeed = 0.1

        # set manual control
        self.leftLimb.set_joint_position_speed(self.manualJointSpeed)
        self.rightLimb.set_joint_position_speed(self.manualJointSpeed)

    def calibrateGripper(self, selectedGripper):
        if selectedGripper.lower() == 'left':
            self.leftGripper.calibrate()
        elif selectedGripper.lower() == 'right':
            self.rightGripper.calibrate()
        elif selectedGripper.lower() == 'both':
            self.leftGripper.calibrate()
            self.rightGripper.calibrate()



class BaxterController:
    def __init__(self):
        rospy.init_node('BaxterController', anonymous=True)
        self.rate = rospy.Rate(10) # 10hz

        self.my_env = os.environ
        self.openni_process = None

        self.gripperStateLeft = GripperState("left")
        self.gripperStateRight = GripperState("right")

        # Grab Baxter Controller
        print "Initializing Baxter Action Server"
        self.action = BaxterActionServer()

        # Calibrate Gripper
        print "Calibrating Gripper...."
        self.action.calibrateGripper("both")





bx = BaxterController()


while(cv2.waitKey(10) is not 27):
    pass