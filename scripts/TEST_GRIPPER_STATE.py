#!/usr/bin/env python
import rospy

from BaxterUtil import *
import numpy as np

from baxter_interface import CHECK_VERSION
from baxter_core_msgs.msg import EndpointState
from baxter_core_msgs.msg import EndEffectorState

from sensor_msgs.msg import Range
import math
import sys
import copy

import time
import subprocess, signal, os

import rospy
import cv2
from cv2 import *

from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray
import cv_bridge

import tf
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from std_msgs.msg import (
    Header,
    UInt16
)
from sensor_msgs.msg import (
    Image,
    JointState
)

from baxter_interface import (
    RobotEnable,
    CameraController,
    Limb
)
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest
)

class GripperState:
    def __init__(self,l_r):

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
        #print self.l_r,  self.force


cv2.namedWindow('Key to exit')
rospy.init_node('BaxterController', anonymous=True)
rate = rospy.Rate(1) # 10hz

rightGripperState = GripperState(l_r='right')
leftGripperState = GripperState(l_r='left')


r = dataFilter(20)
while 1:
    r.add(leftGripperState.range)
    r.average()

    print r.avg
    if cv2.waitKey(20) != -1:
        break
