#!/usr/bin/env python
import rospy
import BaxterMoveIt
import BaxterPositions
import baxter_interface
from BaxterUtil import *
import numpy as np

import matplotlib.pyplot as plt

from baxter_interface import CHECK_VERSION
from baxter_core_msgs.msg import EndpointState
from baxter_core_msgs.msg import EndEffectorState

from ar_track_alvar.msg import AlvarMarker
from ar_track_alvar.msg import AlvarMarkers

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
from geometry_msgs.msg import (
    PoseStamped,
    PointStamped,
    Pose,
    Point,
    Quaternion
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

from subprocess import Popen
import os

import OPEAssist

OPENNI_CMD = "roslaunch openni_launch openni.launch"
COLORS_CMD = "rosrun baxter_controller ObjectColors"
OPE_DIR = "/home/baxter/ros/ws_carrt/src/baxter_controller/src/OPE-Baxter/"
KILL_XNSENSOR_CMD = "killall killXnSensorServer"

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


class BaxterArmTrajectory:
    def __init__(self):

        self.trajectory = []

    def isEmpty(self):
        return not bool(len(self.trajectory))

    def recordWaypoint(self, x, y, z, er, ep, ey):
        self.trajectory.append((x, y, z, er, ep, ey))

    def clear(self):
        self.trajectory = []

    def top(self):
        return self.trajectory[0]

    def pop(self):
        x, y, z, er, ep, ey = self.trajectory.pop(0)
        return x, y, z, er, ep, ey



class BaxterAction:
    IKSVC_LEFT_URI = "ExternalTools/left/PositionKinematicsNode/IKService"
    IKSVC_RIGHT_URI = "ExternalTools/right/PositionKinematicsNode/IKService"

    def __init__(self):

        #------------------------------------------------------------------------------------------#
        # initialize interfaces
        self.rs = baxter_interface.RobotEnable(CHECK_VERSION)
        init_state = self.rs.state().enabled

        self.tf = tf.TransformListener()

        self.moveit = BaxterMoveIt.BaxterMoveIt()

        # Grippers
        self.leftGripper = baxter_interface.Gripper('left', CHECK_VERSION)
        self.rightGripper = baxter_interface.Gripper('right', CHECK_VERSION)

        self.rightGripperState = GripperState(l_r='right')
        self.leftGripperState = GripperState(l_r='left')

        # Limbs
        self.leftLimb = baxter_interface.Limb('left')
        self.rightLimb = baxter_interface.Limb('right')
        self.leftLimb.set_joint_position_speed(0.5)
        self.rightLimb.set_joint_position_speed(0.5)

        # IK service
        self.left_iksvc = rospy.ServiceProxy(BaxterAction.IKSVC_LEFT_URI, SolvePositionIK)
        self.right_iksvc = rospy.ServiceProxy(BaxterAction.IKSVC_RIGHT_URI, SolvePositionIK)

        self.leftLimbWaitingPos = None
        self.rightLimbWaitingPos = None
        
        # JOINT SPEEDS
        self.manualJointAccuracy = baxter_interface.settings.JOINT_ANGLE_TOLERANCE
        self.manualJointTimeout = 20.0
        self.manualJointSpeed = 0.5

        self.ik_rate = 30   #100
        self.joint_update_pub = rospy.Publisher('/robot/joint_state_publish_rate', UInt16)
        self.joint_update_pub.publish(self.ik_rate)
        #------------------------------------------------------------------------------------------#

        # MIMIC
        self.mimic_timer = None
        self.ik_window_size = 10
        self.detection_window_size = 10
        self.ik_out_of_range = False

        # store position commands
        self.r_trans_prev = []
        self.l_trans_prev = []

        self.r_rot_prev = []
        self.l_rot_prev = []

        self.r_rot_curr = None
        self.l_rot_curr = None

        # store IK solution history
        self.r_ik_hist = []
        self.l_ik_hist = []

        self.isRecording = False
        self.isPlayback = False
        self.r_traj = BaxterArmTrajectory()
        self.l_traj = BaxterArmTrajectory()

        #------------------------------------------------------------------------------------------#
        # Object Retreival
        # Initialize ROS node
        self.activeArm = 0              # 0 = left
        self.command_done = False
        self.location_reached = False
        self.rotation_reached = False
        self.arTag_reached = False
        self.openni_process = None

        rospy.loginfo("Setting up ROS SUBS/PUBS....")
        self.pub_detectNav = rospy.Publisher('Navigation/faceID', String, queue_size=1)
        self.sub_faceNav = rospy.Subscriber("Navigation/personReached", String, self.locationReached_callback)
        
        self.pub_locNav = rospy.Publisher('Navigation/desiredLocation', Int32MultiArray, queue_size = 1 )
        self.sub_locNav = rospy.Subscriber("Navigation/locationReached", String, self.locationReached_callback)

        self.pub_rotNav = rospy.Publisher('Navigation/desiredRotation', Int32MultiArray, queue_size = 1 )
        self.sub_rotNav = rospy.Subscriber("Navigation/rotationReached", String, self.rotationReached_callback)

        self.pub_arNav = rospy.Publisher('Navigation/desiredARTag', String, queue_size = 1)
        self.sub_arNav = rospy.Subscriber('Navigation/arTagReached', String, self.arTagReached_callback)
        #--------------------------------------------------------------------------------------------#
        # Book Retrieval
        self.ar_markers = []
        self.preBookReached = False
        self.bookFound = False
        self.pub_bookTitle = rospy.Publisher('OCR/bookTitle', String, queue_size = 1 )
        self.sub_bookLocation = rospy.Subscriber("OCR/bookLocation", Int32MultiArray, self.bookFound_callback)
        self.sub_arMarker = rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.arMarker_callback)


        #------------------------------------------------------------------------------------------#

    '''
    # -------------------------------------------------------------------------------------------------------------------------#
    # MISC
    # -------------------------------------------------------------------------------------------------------------------------#
    '''
    def display(self, path):
        img = cv2.imread(path)
        msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
        pub = rospy.Publisher('/robot/xdisplay', Image, latch=True)
        pub.publish(msg)
        # Sleep to allow for image to be published.
        rospy.sleep(1)

    def scanObject(self):
        self.rightLimb.move_to_joint_positions(BaxterPositions.ref_pos_r1, 15.0)
        self.leftLimb.move_to_joint_positions(BaxterPositions.ref_pos_l1, 15.0)

        # take pictures here
        self.leftLimb.move_to_joint_positions(BaxterPositions.pos1_l_pivot1, 15.0)
        self.leftLimb.move_to_joint_positions(BaxterPositions.pos1_l_pivot2, 15.0)
        self.leftLimb.move_to_joint_positions(BaxterPositions.pos1_l_pivot3, 15.0)
        self.leftLimb.move_to_joint_positions(BaxterPositions.pos1_l_pivot4, 15.0)
        
        self.rightLimb.move_to_joint_positions(BaxterPositions.ref_pos_r2, 15.0)
        self.leftLimb.move_to_joint_positions(BaxterPositions.ref_pos_l1, 15.0)
            
        self.leftLimb.move_to_joint_positions(BaxterPositions.pos1_l_pivot1, 15.0)
        self.leftLimb.move_to_joint_positions(BaxterPositions.pos1_l_pivot2, 15.0)
        self.leftLimb.move_to_joint_positions(BaxterPositions.pos1_l_pivot3, 15.0)
        self.leftLimb.move_to_joint_positions(BaxterPositions.pos1_l_pivot4, 15.0)

        self.rightLimb.move_to_joint_positions(BaxterPositions.ref_pos_r3, 15.0)
        self.leftLimb.move_to_joint_positions(BaxterPositions.ref_pos_l3, 15.0)

        self.leftLimb.move_to_joint_positions(BaxterPositions.pos2_l_pivot1, 15.0)
        self.leftLimb.move_to_joint_positions(BaxterPositions.pos2_l_pivot2, 15.0)
        self.leftLimb.move_to_joint_positions(BaxterPositions.pos2_l_pivot3, 15.0)
        self.leftLimb.move_to_joint_positions(BaxterPositions.pos2_l_pivot4, 15.0)

        self.rightLimb.move_to_joint_positions(BaxterPositions.ref_pos_r1, 15.0)
        self.leftLimb.move_to_joint_positions(BaxterPositions.ref_pos_l1, 15.0)



    def calibrateGripper(self, selectedGripper):
        if selectedGripper.lower() == 'left':
            self.leftGripper.calibrate()
        elif selectedGripper.lower() == 'right':
            self.rightGripper.calibrate()
        elif selectedGripper.lower() == 'both':
            self.leftGripper.calibrate()
            self.rightGripper.calibrate()

    def moveJointPositions(self, jointPositions, selectedLimb):
        if selectedLimb.lower() == 'left':
            self.leftLimb.move_to_joint_positions(jointPositions, self.manualJointTimeout, self.manualJointAccuracy)
        elif selectedLimb.lower() == 'right':
            self.rightLimb.move_to_joint_positions(jointPositions, self.manualJointTimeout, self.manualJointAccuracy)

    def moveGripper(self, selectedGripper, pos, openClose):
        if pos is not None or openClose is None:
            if selectedGripper == 1:
                self.rightGripper.command_position(pos, block=True)
            else:
                self.leftGripper.command_position(pos, block=True)

        elif openClose == "open" or openClose == "Open":
            if selectedGripper == 1:
                self.rightGripper.command_position(100)
            else:
                self.leftGripper.command_position(100)

        elif openClose == "close" or openClose == "Close":
            if selectedGripper == 1:
                self.rightGripper.command_position(0)
            else:
                self.leftGripper.command_position(0)

        elif openClose == "half" or openClose == "Half":
            if selectedGripper == 1:
                self.rightGripper.command_position(50)
            else:
                self.leftGripper.command_position(50)

    def resetArms(self):
        #self.leftLimb.move_to_neutral()
        #self.rightLimb.move_to_neutral()
        self.goToWaiting(0)
        self.goToWaiting(1)


    def goToWaiting(self, baxterArm):
        if baxterArm == 0 or baxterArm == "left":
            rospy.loginfo("Going to waiting position (left)")
            waitingPlan = self.moveit.createPathPlan(BaxterPositions.leftWaitingPos, 0, BaxterPositions.leftWaitingRot)
            self.moveit.slowPlanVelocity(waitingPlan)
            if self.moveit.leftGroup.execute(waitingPlan):
                self.leftLimbWaitingPos = self.leftLimb.joint_angles()
                #rospy.loginfo("Saved left limb waiting position: " + str(self.leftLimbWaitingPos))
                return True
            else:
                print "Could not move to waiting position!"
                return False

        elif baxterArm == 1 or baxterArm == "right":
            rospy.loginfo("Going to waiting position (right)")
            waitingPlan = self.moveit.createPathPlan(BaxterPositions.rightWaitingPos, 1, BaxterPositions.rightWaitingRot)
            self.moveit.slowPlanVelocity(waitingPlan)
            if self.moveit.rightGroup.execute(waitingPlan):
                self.rightLimbWaitingPos = self.rightLimb.joint_angles()
                #rospy.loginfo("Saved right limb waiting position: " + str(self.rightLimbWaitingPos))
                return True
            else:
                print "Could not move to waiting position!"
                return False

    '''
    # -------------------------------------------------------------------------------------------------------------------------#
    # ARM MOVEMENT AND IK
    # -------------------------------------------------------------------------------------------------------------------------#
    '''
    def set_left_coords(self, x, y, z, er=math.pi * -1, ep=math.pi * 0.5, ey=math.pi * -1):
        self.set_arm_coords(self.left_iksvc, self.leftLimb, x, y, z, er, ep, ey, self.leftLimb.joint_angles(), self.l_ik_hist)

    def set_right_coords(self, x, y, z, er=math.pi * -1, ep=math.pi * 0.5, ey=math.pi * -1):
        self.set_arm_coords(self.right_iksvc, self.rightLimb, x, y, z, er, ep, ey, self.rightLimb.joint_angles(), self.r_ik_hist)

    def set_arm_coords(self, iksvc, limb, x, y, z, er, ep, ey, jointState, ik_hist):
        resp = self.get_ik(iksvc, x, y, z, er, ep, ey, jointState)
        positions = resp[0]
        isValid = resp[1]
        self.ik_out_of_range = not isValid

        # smooth the ik commands
        ik_hist.append(positions)
        while (len(ik_hist) > self.ik_window_size):
            ik_hist.pop(0)

        smooth_positions = {}
        for joint_name in positions:
            smooth_positions[joint_name] = sum(map(lambda x: x[joint_name], ik_hist)) / self.ik_window_size
        
        # Check if result valid, and type of seed ultimately used to get solution
        # convert rospy's string representation of uint8[]'s to int's
        smooth_positions = positions
        limb.set_joint_positions(smooth_positions)


    def get_ik(self, iksvc, x, y, z, er, ep, ey, jointAngles):
        #print er, ep, ey
        q = quaternion_from_euler(er, ep, ey)

        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        pose = PoseStamped(header=hdr, pose=Pose(position=Point(x=x,y=y,z=z), orientation=Quaternion(q[0], q[1], q[2], q[3])))

        ikreq = SolvePositionIKRequest()
        ikreq.pose_stamp.append(pose)

        try:
            if(iksvc is self.left_iksvc):
                rospy.wait_for_service(BaxterAction.IKSVC_LEFT_URI, 5.0)
            else:
                rospy.wait_for_service(BaxterAction.IKSVC_RIGHT_URI, 5.0)
            resp = iksvc(ikreq)

        except(rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))

        if (resp.isValid[0]):
            #print("SUCCESS - Valid Joint Solution Found:")
            # Format solution into Limb API-compatible dictionary
            positions = dict(zip(resp.joints[0].name, resp.joints[0].position))
            return (positions, resp.isValid[0])
        else:
            print("INVALID POSE - No Valid Joint Solution Found.")

        #positions = dict(zip(resp.joints[0].name, resp.joints[0].position))
    
    '''
    # -------------------------------------------------------------------------------------------------------------------------#
    # MIMIC DEMO
    # -------------------------------------------------------------------------------------------------------------------------#
    '''
    def mimic(self, vision):
        self.rs.enable()
        #self.resetArms()
        self.mimic_timer = rospy.Timer(rospy.Duration(1.0 / self.ik_rate), self.mimic_callback)
        #rate = rospy.Rate(self.detection_window_size)
        rate = rospy.Rate(30)               # hz
        rospy.loginfo("Starting Mimic")

        while not rospy.is_shutdown():

            try:
                # gripper
                self.moveGripper(selectedGripper = 0, pos = vision.r_hand_state, openClose = None)
                self.moveGripper(selectedGripper = 1, pos = vision.l_hand_state, openClose = None)

                # translation
                (r_trans,r_rot) = self.tf.lookupTransform(
                    '/right_hand_1',
                    '/torso_1',
                    rospy.Time(0))

                (l_trans,l_rot) = self.tf.lookupTransform(
                    '/left_hand_1',
                    '/torso_1',
                    rospy.Time(0))

                self.l_trans_prev.append(l_trans)
                if (len(self.l_trans_prev) > self.detection_window_size):
                    self.l_trans_prev.pop(0)
                
                self.r_trans_prev.append(r_trans)
                if (len(self.r_trans_prev) > self.detection_window_size):
                    self.r_trans_prev.pop(0)


                # rotation
                (r_trans,r_rot) = self.tf.lookupTransform(
                    '/right_hand_1',
                    '/right_shoulder_1',
                    rospy.Time(0))

                (l_trans,l_rot) = self.tf.lookupTransform(
                    '/left_elbow_1',
                    '/torso_1',
                    rospy.Time(0))
                

                self.l_rot_curr = euler_from_quaternion(l_rot)
                self.l_rot_prev.append(self.l_rot_curr)
                if (len(self.l_rot_prev) > self.detection_window_size):
                    self.l_rot_prev.pop(0)

                self.r_rot_curr = euler_from_quaternion(r_rot)
                self.r_rot_prev.append(self.r_rot_curr)
                if (len(self.r_rot_prev) > self.detection_window_size):
                    self.r_rot_prev.pop(0)
                

            except:
                self.ik_out_of_range = True
                rate.sleep()
                rospy.loginfo("Error: No tf lookup")
                continue

            rate.sleep()


    def mimic_callback(self, event):
        try:
            
            l_trans = (
                sum(map(lambda x: x[0], self.l_trans_prev)) / self.detection_window_size,
                sum(map(lambda x: x[1], self.l_trans_prev)) / self.detection_window_size,
                sum(map(lambda x: x[2], self.l_trans_prev)) / self.detection_window_size)

            r_trans = (
                sum(map(lambda x: x[0], self.r_trans_prev)) / self.detection_window_size,
                sum(map(lambda x: x[1], self.r_trans_prev)) / self.detection_window_size,
                sum(map(lambda x: x[2], self.r_trans_prev)) / self.detection_window_size)

            # get initial horizontal positions and clamp values of opposite end effector to this value
            lh_y = clamp(r_trans[0], -1.4, 1.4)
            rh_y = clamp(l_trans[0], -1.4, 1.4)

            rh_limit = lh_y - 0.15
            lh_limit = rh_y + 0.15

            rh_x = clamp(0.65 + (l_trans[2] / 1.5), 0.5, 1.0)
            rh_y = clamp(l_trans[0], -1.2, rh_limit)
            rh_z = clamp((-l_trans[1]+0.1)*1.2, 0.22, 1.1)
            lh_x = clamp(0.65 + (r_trans[2] / 1.5), 0.5, 1.0)
            lh_y = clamp(r_trans[0], lh_limit, 1.2)
            lh_z = clamp((-r_trans[1]+0.1)*1.2, 0.22, 1.1)

            # angles
            
            l_rot = (
                sum(map(lambda x: x[0], self.l_rot_prev)) / self.detection_window_size,
                sum(map(lambda x: x[1], self.l_rot_prev)) / self.detection_window_size,
                sum(map(lambda x: x[2], self.l_rot_prev)) / self.detection_window_size)

            r_rot = (
                sum(map(lambda x: x[0], self.r_rot_prev)) / self.detection_window_size,
                sum(map(lambda x: x[1], self.r_rot_prev)) / self.detection_window_size,
                sum(map(lambda x: x[2], self.r_rot_prev)) / self.detection_window_size)
            '''
            rh_roll = clamp(r_rot[0], math.pi * -1, -0.9 * math.pi)
            lh_roll = clamp(l_rot[0], math.pi * -1, -0.9 * math.pi)
            rh_pitch = clamp(r_rot[1], math.pi*0.4, math.pi*0.6)
            lh_pitch = clamp(l_rot[1], math.pi*0.4, math.pi*0.6)
            rh_yaw = clamp(r_rot[2], math.pi * -1, -0.9 * math.pi)
            lh_yaw = clamp(l_rot[2], math.pi * -1, -0.9 * math.pi)
            '''
            rh_yaw = r_rot[2]
            rh_pitch = r_rot[1]
            rh_roll = r_rot[0]

            lh_yaw = l_rot[2]
            lh_pitch = l_rot[1]
            lh_roll = l_rot[0]


            rh_pitch = -(rh_pitch) + math.pi/4.0
            rh_yaw = (-math.pi/2.0 - rh_yaw)

            lh_pitch = -(lh_pitch) + math.pi/4.0
            lh_yaw = (-math.pi/2.0 + lh_yaw)
            #lh_pitch = -(math.pi/4.0 - (lh_pitch - math.pi))

            #print "Orig: ", r_rot[2], "Right: ", rh_yaw
            #print -1.0*math.pi + rh_roll, 0.5*math.pi + rh_pitch, -1.0*math.pi + rh_yaw
            
            #self.set_left_coords(lh_x, lh_y, lh_z, ep=0.5*math.pi + 3.0*(lh_pitch+0.1), ey=-1.0*math.pi - (lh_roll/2.0))
            #self.set_right_coords(rh_x, rh_y, rh_z, er=((-1.0*m.pi) + (r_rot[0]/4.0)), ep=((m.pi*0.5) + (r_rot[1]/4.0)), ey=((-1.0*m.pi) + (r_rot[2]/4.0)))
            
            # gooood
            
            #self.set_right_coords(rh_x, rh_y, rh_z, ep=0.5*math.pi + 4.0*(rh_pitch+0.2))
            #self.set_left_coords(lh_x, lh_y, lh_z, ep=0.5*math.pi + 4.0*(lh_pitch-0.2))
            

            #self.set_right_coords(rh_x, rh_y, rh_z, ey=math.pi*-1.0 + 6.0*(-rh_yaw - 0.15))
            #self.set_left_coords(lh_x, lh_y, lh_z, ey=math.pi*-1.0 - 6.0*(lh_yaw))

            #self.set_left_coords(lh_x, lh_y, lh_z, lh_roll, lh_pitch, lh_yaw)
            self.set_right_coords(rh_x, rh_y, rh_z)
            self.set_left_coords(lh_x, lh_y, lh_z)
            #self.set_left_coords(lh_x, lh_y, lh_z, ep=rh_pitch, ey=rh_yaw)
            #self.set_right_coords(rh_x, rh_y, rh_z, ep=lh_pitch, ey=lh_yaw)

        except:
            return


    def record(self, vision):
        self.isRecording = True

        self.rs.enable()
        #self.resetArms()
        self.record_timer = rospy.Timer(rospy.Duration(1.0 / self.ik_rate), self.record_callback)
        #rate = rospy.Rate(self.detection_window_size)
        rate = rospy.Rate(30)              # hz
        rospy.loginfo("Starting Record")

        while not rospy.is_shutdown() and self.isRecording and len(self.r_traj.trajectory) < 200:

            try:
                # gripper
                #self.moveGripper(selectedGripper = 0, pos = vision.r_hand_state, openClose = None)
                #self.moveGripper(selectedGripper = 1, pos = vision.l_hand_state, openClose = None)

                # translation
                (r_trans,r_rot) = self.tf.lookupTransform(
                    '/right_hand_1',
                    '/torso_1',
                    rospy.Time(0))

                (l_trans,l_rot) = self.tf.lookupTransform(
                    '/left_hand_1',
                    '/torso_1',
                    rospy.Time(0))

                self.l_trans_prev.append(l_trans)
                if (len(self.l_trans_prev) > self.detection_window_size):
                    self.l_trans_prev.pop(0)
                
                self.r_trans_prev.append(r_trans)
                if (len(self.r_trans_prev) > self.detection_window_size):
                    self.r_trans_prev.pop(0)


                
                # rotation
                (r_trans,r_rot) = self.tf.lookupTransform(
                    '/right_elbow_1',
                    '/torso_1',
                    rospy.Time(0))

                (l_trans,l_rot) = self.tf.lookupTransform(
                    '/left_elbow_1',
                    '/torso_1',
                    rospy.Time(0))
                

                self.l_rot_curr = euler_from_quaternion(l_rot)
                self.l_rot_prev.append(self.l_rot_curr)
                if (len(self.l_rot_prev) > self.detection_window_size):
                    self.l_rot_prev.pop(0)

                self.r_rot_curr = euler_from_quaternion(r_rot)
                self.r_rot_prev.append(self.r_rot_curr)
                if (len(self.r_rot_prev) > self.detection_window_size):
                    self.r_rot_prev.pop(0)
                

            except:
                self.ik_out_of_range = True
                rate.sleep()
                rospy.loginfo("Error: No tf lookup")
                continue

            rate.sleep()

        self.stopRecording()

    def stopRecording(self):
        self.isRecording = False
        rospy.loginfo("Ending Record")
        

    def record_callback(self, event):

        #print len(self.r_traj.trajectory), len(self.l_traj.trajectory)
        try:
            
            l_trans = (
                sum(map(lambda x: x[0], self.l_trans_prev)) / self.detection_window_size,
                sum(map(lambda x: x[1], self.l_trans_prev)) / self.detection_window_size,
                sum(map(lambda x: x[2], self.l_trans_prev)) / self.detection_window_size)

            r_trans = (
                sum(map(lambda x: x[0], self.r_trans_prev)) / self.detection_window_size,
                sum(map(lambda x: x[1], self.r_trans_prev)) / self.detection_window_size,
                sum(map(lambda x: x[2], self.r_trans_prev)) / self.detection_window_size)

            # get initial horizontal positions and clamp values of opposite end effector to this value
            lh_y = clamp(r_trans[0], -1.4, 1.4)
            rh_y = clamp(l_trans[0], -1.4, 1.4)

            rh_limit = lh_y - 0.15
            lh_limit = rh_y + 0.15

            rh_x = clamp(0.65 + (l_trans[2] / 1.5), 0.5, 1.0)
            rh_y = clamp(l_trans[0], -1.2, rh_limit)
            rh_z = clamp((-l_trans[1]+0.1)*1.2, 0.22, 1.1)
            lh_x = clamp(0.65 + (r_trans[2] / 1.5), 0.5, 1.0)
            lh_y = clamp(r_trans[0], lh_limit, 1.2)
            lh_z = clamp((-r_trans[1]+0.1)*1.2, 0.22, 1.1)

            # angles
            
            l_rot = (
                sum(map(lambda x: x[0], self.l_rot_prev)) / self.detection_window_size,
                sum(map(lambda x: x[1], self.l_rot_prev)) / self.detection_window_size,
                sum(map(lambda x: x[2], self.l_rot_prev)) / self.detection_window_size)

            r_rot = (
                sum(map(lambda x: x[0], self.r_rot_prev)) / self.detection_window_size,
                sum(map(lambda x: x[1], self.r_rot_prev)) / self.detection_window_size,
                sum(map(lambda x: x[2], self.r_rot_prev)) / self.detection_window_size)
            '''
            rh_roll = clamp(r_rot[0], math.pi * -1, -0.9 * math.pi)
            lh_roll = clamp(l_rot[0], math.pi * -1, -0.9 * math.pi)
            rh_pitch = clamp(r_rot[1], math.pi*0.4, math.pi*0.6)
            lh_pitch = clamp(l_rot[1], math.pi*0.4, math.pi*0.6)
            rh_yaw = clamp(r_rot[2], math.pi * -1, -0.9 * math.pi)
            lh_yaw = clamp(l_rot[2], math.pi * -1, -0.9 * math.pi)
            '''
            rh_yaw = r_rot[2]
            rh_roll = r_rot[1]
            rh_pitch = r_rot[0]

            lh_yaw = l_rot[2]
            lh_roll = l_rot[1]
            lh_pitch = l_rot[0]

            # no rpy for now
            self.r_traj.recordWaypoint(rh_x, rh_y, rh_z, math.pi * -1, math.pi * 0.5, math.pi * -1)
            self.l_traj.recordWaypoint(lh_x, lh_y, lh_z, math.pi * -1, math.pi * 0.5, math.pi * -1)


        except:
            return

        if not self.isRecording:
            return 

    def endPlayback(self):
        self.isPlayback = False
        self.r_traj.clear()
        self.l_traj.clear()
        self.r_ik_hist = []
        self.l_ik_hist = []

    # playback most recently recorded trajecotry
    def playback(self):
        self.isPlayback = True
        #self.rs.enable()
        #self.resetArms()
        self.playback_timer = rospy.Timer(rospy.Duration(1.0 / self.ik_rate), self.playback_callback)
        #rate = rospy.Rate(self.detection_window_size)
        rate = rospy.Rate(30)               # hz

        while not rospy.is_shutdown() and self.isPlayback:
            #rate.sleep
            pass

        self.endPlayback()
        
    def playback_callback(self, event):

        if not self.isPlayback:
            return

        rospy.loginfo("Playback tick")
        if not self.r_traj.isEmpty():
            rx, ry, rz, rer, rep, rey = self.r_traj.top()
            try:
                self.set_right_coords(rx, ry, rz)
                self.r_traj.pop()
            except:
                rospy.loginfo("Failed right IK playbck")

        if not self.l_traj.isEmpty():
            lx, ly, lz, ler, lep, ley = self.l_traj.top()
            try:
                self.set_left_coords(lx, ly, lz)
                self.l_traj.pop()
            except:
                rospy.loginfo("Failed left IK playbck")



        if self.l_traj.isEmpty() and self.r_traj.isEmpty():
            rospy.loginfo("Playback ended")
            self.endPlayback()
            return

    # -------------------------------------------------------------------------------------------------------------------------#
    # Object Retreive
    # -------------------------------------------------------------------------------------------------------------------------#
    def locationReached_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "--Destination Status: %s\n", data.data)
        #speak("Destination Reached")
        if data.data == "1":
            self.location_reached = True
        else:
            self.location_reached = False

    def rotationReached_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "--Destination Status: %s\n", data.data)
        #speak("Rotation Reached")
        if data.data == "1":
            self.rotation_reached = True
        else:
            self.rotation_reached = False

    def arTagReached_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "--AR Destination Status: %s\n", data.data)
        #speak("Rotation Reached")
        if data.data == "1":
            self.arTag_reached = True
        else:
            self.arTag_reached = False



    def gotoObject(self, objectLoc, objectRot, objectNum):
        # PRE-OBJECT POSE
        preObjectLoc = copy.deepcopy(objectLoc)
        preObjectLoc[0] = preObjectLoc[0] - 0.2 # subtract 0.1 meters in x

        # OBJECT POSE (RAISED)
        objectRaisedLoc = copy.deepcopy(objectLoc)
        objectRaisedLoc[2] = objectRaisedLoc[2] + 0.2

        raisedPreObjectLoc = copy.deepcopy(objectLoc)
        raisedPreObjectLoc[0] = raisedPreObjectLoc[0] - 0.1
        raisedPreObjectLoc[2] = raisedPreObjectLoc[2] + 0.2

        # Extend POSE (RAISED)
        extendRaisedLoc = copy.deepcopy(objectRaisedLoc)
        extendRaisedLoc[0] = extendRaisedLoc[0] + 0.2

        if objectLoc[1] < 0:
            baxterArm = 1
        else:
            baxterArm = 0

    
        print "Going to pre-object position...."
        # speak("Going to pre-object position....")
        print preObjectLoc

        preObjectPlan = self.moveit.createPathPlan(preObjectLoc, baxterArm, BaxterPositions.normalRot)
        self.moveit.slowPlanVelocity(preObjectPlan)
        self.moveit.executePlan(baxterArm, preObjectPlan)

        self.moveit.scene.remove_world_object("OBJECT" + str(objectNum))
        rospy.sleep(1)

        print "Opening gripper...."
        # speak("Opening gripper....")
        self.moveGripper(baxterArm, pos = None, openClose = "open")
        #self.gripperAction(baxterArm, openClose = "open")
        
        print "Going to object position...."
        # speak("Going to object position....")
        objectPlan = self.moveit.createPathPlan(objectLoc, baxterArm, BaxterPositions.normalRot)
        #self.moveit.slowPlanVelocity(objectPlan)
        self.moveit.executePlan(baxterArm, objectPlan)

        return True

    # bring arms to lower side 
    def gotoSide(self, SideLocation, SideRotation, leftRight):

        if leftRight == "left" or leftRight == "Left":
            baxterArm = 1
            print "Going to Lower Left Side"
        elif leftRight == "right" or leftRight == "Right":
            print "Going to Lower Right Side"
            baxterArm = 0
        else:
            print "Not a Valid Side, must be <left> or <right>"
            return False

        lowerSideLocLocal = copy.deepcopy(SideLocation)
        lowerSideRotLocal = copy.deepcopy(SideRotation)

        print SideLocation

        lowerSidePlan = self.moveit.createPathPlan(lowerSideLocLocal, baxterArm, lowerSideRotLocal)
        self.moveit.executePlan(baxterArm, lowerSidePlan)
        rospy.sleep(1)

        return True

    def goAwayFromObject(self, objectLoc, objectRot, objectNum):
        # PRE-OBJECT POSE
        preObjectLoc = copy.deepcopy(objectLoc)
        preObjectLoc[0] = preObjectLoc[0] - 0.2 # subtract 0.1 meters in x

        # OBJECT POSE (RAISED)
        objectRaisedLoc = copy.deepcopy(objectLoc)
        objectRaisedLoc[2] = objectRaisedLoc[2] + 0.2

        raisedPreObjectLoc = copy.deepcopy(objectLoc)
        raisedPreObjectLoc[0] = raisedPreObjectLoc[0] - 0.1
        raisedPreObjectLoc[2] = raisedPreObjectLoc[2] + 0.2

        # Extend POSE (RAISED)
        extendRaisedLoc = copy.deepcopy(objectRaisedLoc)
        extendRaisedLoc[0] = extendRaisedLoc[0] + 0.2

        if objectLoc[1] < 0:
            baxterArm = 1
        else:
            baxterArm = 0

        print "Going to object (raised) position...."
        # speak("Going to object (raised) position....")
        objectRaisedPlan = self.moveit.createPathPlan(objectRaisedLoc, baxterArm, BaxterPositions.normalRot)
        self.moveit.slowPlanVelocity(objectRaisedPlan)
        self.moveit.executePlan(baxterArm, objectRaisedPlan)

        print "Going to raised pre-object position...."
        # speak("Going to pre-object position....")
        print raisedPreObjectLoc

        raisedPreObjectPlan = self.moveit.createPathPlan(raisedPreObjectLoc, baxterArm, BaxterPositions.normalRot)
        self.moveit.slowPlanVelocity(raisedPreObjectPlan)
        self.moveit.executePlan(baxterArm, raisedPreObjectPlan)
        rospy.sleep(1)

        self.goToWaiting(baxterArm)
        rospy.sleep(1)
        
        return True   

    def measureDeformation(self, baxterArm):
        print "Closing gripper...."
        
        # speak("Opening gripper....")
        # go to initial location
        pos = 100
        force = 0.0

        if baxterArm == 1:
            gripper = self.leftGripper
            gripperState = self.leftGripperState
        else:
            gripper = self.rightGripper
            gripperState = self.rightGripperState

        gripper.command_position(pos)
        rospy.sleep(1)
        '''
        print "Finding initial position"
        while(gripperState.force <= 0.001 and pos > 0):
            gripper.command_position(pos)
            pos -= 1
            rospy.sleep(0.1)
            print gripperState.force
        '''
        pos = 100 
        contact = pos
        print "encountered at:", contact
        gripper.command_position(contact-10)
        # close gripper all the way, get force and pos deltas
        dx=0
        forces = []
        overthirty = []
        max_force = 0
        max_pos = 0
        while(pos > 0):
            dx+=1
            pos = contact-dx
            gripper.command_position(pos)
            rospy.sleep(0.005)

            forces.append(gripperState.force)
            if gripperState.force > max_force:
                max_force = gripperState.force
                max_pos = pos

            if gripperState.force > 28.0:
                overthirty.append(gripperState.force)

        gripper.command_position(100)
        #print forces
        '''
        mean = np.mean(forces)
        std = np.std(forces)
        print 'mean: ', mean, 'std: ', std
        print 'max force: ', max_force, 'max pos: ', max_pos

        # non-deform max_force > 90 ish, max_pos > 10, mean force ~ 9
        # semi-deform
        # deform: max force < 1.0, max_pos < 10

        plt.plot(forces)


        plt.plot(overthirty)
        mean = np.mean(overthirty)
        std = np.std(overthirty)
        print "over 30", len(overthirty)
        print 'mean: ', mean, 'std: ', std
        print 'max force: ', max_force, 'max pos: ', max_pos

        plt.show()
        '''
        #print mean + std - 3*len(overthirty) + max_force
        mean = np.mean(overthirty)
        return mean

    def graspObject(self, objectLoc, objectRot, objectNum):
        # PRE-OBJECT POSE
        preObjectLoc = copy.deepcopy(objectLoc)
        preObjectLoc[0] = preObjectLoc[0] - 0.2 # subtract 0.2 meters in x

        # OBJECT POSE (RAISED)
        objectRaisedLoc = copy.deepcopy(objectLoc)
        objectRaisedLoc[2] = objectRaisedLoc[2] + 0.2

        raisedPreObjectLoc = copy.deepcopy(objectLoc)
        raisedPreObjectLoc[0] = raisedPreObjectLoc[0] - 0.2
        raisedPreObjectLoc[2] = raisedPreObjectLoc[2] + 0.2

        # Extend POSE (RAISED)
        extendRaisedLoc = copy.deepcopy(objectRaisedLoc)
        extendRaisedLoc[0] = extendRaisedLoc[0]

        gripperState = None
        if objectLoc[1] < 0:
            baxterArm = 1
            gripperState = self.rightGripperState
        else:
            baxterArm = 0
            gripperState = self.leftGripperState


        vertAdjustRes = 0.02
        vertAdjust = 0.0
        tryRaised = 0.0                       # try to raise the gripper to get a successful plan
        while(gripperState.force == 0):
            if tryRaised > 3:
                return baxterArm
            # modify pre-object pose to try to move higher and avoid collisions, incrementally increase vertical position
            preObjectLoc[2] = preObjectLoc[2] + vertAdjustRes*tryRaised
            tryRaised+=1.0

            print "Opening gripper...."
            # speak("Opening gripper....")
            self.moveGripper(baxterArm, pos = None, openClose = "open")
            #self.gripperAction(baxterArm, openClose = "open")
            rospy.sleep(1)

            print "Going to pre-object position...."
            # speak("Going to pre-object position....")
            print preObjectLoc

            success_plan, preObjectPlan = self.moveit.createPathPlanMulti(preObjectLoc, baxterArm, BaxterPositions.normalRot, 3)
            
            if not success_plan:
                print "Failed to find a plan"
                return 

            self.moveit.slowPlanVelocity(preObjectPlan)
            success_exec = self.moveit.executePlanMulti(preObjectPlan, baxterArm, 3)
            
            if not success_exec:
                print "Failed to execute the plan"
                return 

            self.moveit.scene.remove_world_object("OBJECT" + str(objectNum))
            rospy.sleep(1)
            
            print "Going to object position...."
            # speak("Going to object position....")
            success_plan, objectPlan = self.moveit.createPathPlanMulti(objectLoc, baxterArm, BaxterPositions.normalRot, 3)
            
            if not success_plan:
                print "Failed to find a plan"
                return 

            self.moveit.slowPlanVelocity(objectPlan)
            success_exec = self.moveit.executePlanMulti(objectPlan, baxterArm, 3)
            
            if not success_exec:
                print "Failed to execute the plan"
                return 

            print "Closing gripper..."
            # speak("Closing gripper...")
            self.moveGripper(baxterArm, pos = None, openClose = "close")
            #self.gripperAction(baxterArm, openClose = "close")
            rospy.sleep(1)

            print "Going to object (raised) position...."
            # speak("Going to object (raised) position....")
            success_plan, objectRaisedPlan = self.moveit.createPathPlanMulti(objectRaisedLoc, baxterArm, BaxterPositions.normalRot, 3)
            
            if not success_plan:
                print "Failed to find a plan"
                return 

            self.moveit.slowPlanVelocity(objectRaisedPlan)
            success_exec = self.moveit.executePlanMulti(objectRaisedPlan, baxterArm, 3)
            
            if not success_exec:
                print "Failed to execute the plan"
                return 

            print "Going to raised pre-object position...."
            # speak("Going to pre-object position....")
            print raisedPreObjectLoc

            success_plan, raisedPreObjectPlan = self.moveit.createPathPlanMulti(raisedPreObjectLoc, baxterArm, BaxterPositions.normalRot, 3)
            
            if not success_plan:
                print "Failed to find a plan"
                return 

            self.moveit.slowPlanVelocity(raisedPreObjectPlan)
            success_exec = self.moveit.executePlanMulti(raisedPreObjectPlan, baxterArm, 3)
            
            if not success_exec:
                print "Failed to execute the plan"
                return 

            '''
            self.goToWaiting(baxterArm)
            rospy.sleep(1)
            '''
            return baxterArm

        # return the arm it was grasped with
        return baxterArm 

    def command_navigate(self, personStr):
        #*********************************************************************************#
        # Face Navigation Section (Working - dependant on PowerBotNavigation ROS Node)
        # - Load Face profiles and navigate to coordinates of 'personId' 
        #personId = 2
        personId = int(personStr)
        if personId is not None and personId is not -1:
            # speak("Navigating to Person" + str(personId))
            self.command_done = False
            faceID = str(personId)

            #if not self.command_done:
            rospy.loginfo("Starting PowerBot Navigation to person: " + faceID)
            self.pub_detectNav.publish(faceID)
        else:
            rospy.loginfo("No Valid PersonID: " + faceID)

        while self.location_reached != True:
            pass
        self.location_reached = False 

    # Object Grasping
    # - Assume objects are in view of robot on a table
    # - Runs OPE and grasps  object with color of parameter 'color'
    def command_grasp_object(self, color):
        rospy.loginfo("Retreive Grasp Starting")

        # remove any remaining collision models of OPE objects
        for i in range(50):
            self.moveit.scene.remove_world_object("OBJECT" + str(i))
        self.moveit.scene.remove_world_object("TABLE")

        # reset arms
        self.goToWaiting(0)
        #rospy.sleep(1)

        self.goToWaiting(1)
        #rospy.sleep(1)

        #Remove Old PCDs
        removePCDs(OPE_DIR)
        #rospy.sleep(1)
        
        #if self.gotoWaiting():
        #*********************************************************************************#
        # Object Pose Estimation Selection (WORKING)
        # - Estimate Object Poses 
        # - Save OPE Results to txt
        # - Average Hues and save to color file

        # Run OPE
        OPEAssist.runOPE()
        OPEAssist.loadOPEResults()
        #rospy.sleep(1)

        # Get OPE Object Colors
        colors_process = Popen(COLORS_CMD, shell=True, preexec_fn=os.setsid)
        #rospy.sleep(1)

        # Colored Object Selection
        #TODO: Obtain color form command string
        desired_color = color
        #desired_color = "white"
        objectNum = 0
        baxterArm = 0

        colorFile = open(OPE_DIR + "ObjectColors.txt")
        content = colorFile.readlines()
        print content

        i = 0
        for colors in content:
            objColor = colors.strip("'").rstrip()

            print "checking: " + objColor
            if objColor == desired_color:
                objectNum = i
                break
            i = i + 1

        print "Object Number: " , objectNum
        #*********************************************************************************#
        # TODO: transform based on kinect
        #*********************************************************************************#
        # Show OPE Results, Grab the Object
        objectLoc = None
        if OPEAssist.objCount > 0:

            rospy.loginfo("Adding Table Collision Model")
            #ADD Table Collision Model
            self.moveit.addObject("TABLE",
                                  OPEAssist.tablePos,
                                  OPEAssist.tableSize)

            # ADD Object Collision Models
            for x in OPEAssist.objList:
                self.moveit.addObject("OBJECT" + str(x['objNumber']),
                                      x['objPos'],
                                      x['objSize'])

            #rospy.sleep(1)
            #OPEAssist.showOPEResults()
            
            objectLoc = OPEAssist.objList[objectNum]['objPos']
            print "OBJECT ", objectNum
            
        
            self.moveit.scene.remove_world_object("OBJECT" + str(objectNum))
            baxterArm = self.graspObject(objectLoc, [0, 0, 0, 0], objectNum)

            for i in range(OPEAssist.objCount):
                self.moveit.scene.remove_world_object("OBJECT" + str(i))
            self.moveit.scene.remove_world_object("TABLE")
            

            #self.bringToSide( BaxterPositions.lowerRightSidePos, BaxterPositions.lowerRightSideRot, leftRight = "right")
            #self.bringToSide( BaxterPositions.lowerLeftSidePos, BaxterPositions.lowerLeftSideRot, leftRight = "left")
        #*********************************************************************************#
        else:
            rospy.loginfo("No Objects Detected")
            return 

    # Object Retrieval
    # - Navigate to set Table coords
    # - Run OPE and grasp object
    # - Rotate 180 and navigate to user
    # - Rotate 180 and navigate back to table coords
    def command_retrieve(self, personStr, color):
        
        rospy.loginfo("Retreive Object Starting")
        
        t_start = time.time()
        # reset arms
        self.goToWaiting(0)
        #rospy.sleep(1)

        self.goToWaiting(1)
        #rospy.sleep(1)

        # navigate to table
        tablePos = Int32MultiArray()
        tablePos.data = BaxterPositions.default_table

        self.pub_locNav.publish(tablePos)
        while self.location_reached != True:
            pass
        rospy.loginfo("Arrived at default object table coordinates")
        self.location_reached = False 
        
        t_table = time.time()
        # Launch Openni Drivers (not needed on lab laptop?)
        #self.openni_process = Popen(OPENNI_CMD, shell=True, preexec_fn=os.setsid)

        #Remove Old PCDs
        removePCDs(OPE_DIR)
        #rospy.sleep(1)
        
        #if self.gotoWaiting():
        #*********************************************************************************#
        # Object Pose Estimation Selection (WORKING)
        # - Estimate Object Poses 
        # - Save OPE Results to txt
        # - Average Hues and save to color file

        # Run OPE
        OPEAssist.runOPE()
        OPEAssist.loadOPEResults()
        #rospy.sleep(1)

        t_ope = time.time()
        #OPEAssist.showOPEResults()
        #rospy.sleep(2)
        
        # Get OPE Object Colors
        colors_process = Popen(COLORS_CMD, shell=True, preexec_fn=os.setsid)
        #rospy.sleep(1)

        # Colored Object Selection
        #TODO: Obtain color form command string
        desired_color = color
        #desired_color = "white"
        objectNum = -1
        baxterArm = 0

        colorFile = open(OPE_DIR + "ObjectColors.txt")
        content = colorFile.readlines()
        print content

        i = 0
        for colors in content:
            objColor = colors.strip("'").rstrip()

            print "checking: " + objColor
            if objColor == desired_color:
                objectNum = i
                break
            i = i + 1

        print "Object Number: " , objectNum
        #*********************************************************************************#
        # TODO: transform based on kinect
        #*********************************************************************************#
        # Show OPE Results, Grab the Object
        objectLoc = None

        t_color = time.time()

        if OPEAssist.objCount > 0:

            rospy.loginfo("Adding Table Collision Model")
            #ADD Table Collision Model
            self.moveit.addObject("TABLE",
                                  OPEAssist.tablePos,
                                  OPEAssist.tableSize)

            # ADD Object Collision Models
            for x in OPEAssist.objList:
                self.moveit.addObject("OBJECT" + str(x['objNumber']),
                                      x['objPos'],
                                      x['objSize'])

            #rospy.sleep(1)
            #OPEAssist.showOPEResults()
            objectLoc = OPEAssist.objList[objectNum]['objPos']
            self.moveit.scene.remove_world_object("OBJECT" + str(objectNum))
            baxterArm = self.graspObject(objectLoc, [0, 0, 0, 0], objectNum)

            
            for i in range(OPEAssist.objCount):
                self.moveit.scene.remove_world_object("OBJECT" + str(i))
            self.moveit.scene.remove_world_object("TABLE")
            

            #self.bringToSide( BaxterPositions.lowerRightSidePos, BaxterPositions.lowerRightSideRot, leftRight = "right")
            #self.bringToSide( BaxterPositions.lowerLeftSidePos, BaxterPositions.lowerLeftSideRot, leftRight = "left")
        #*********************************************************************************#
        else:
            rospy.loginfo("No Objects Detected")
            return 
        
        # do we need this?
        #self.openni_process = Popen(KILL_XNSENSOR_CMD, shell=True, preexec_fn=os.setsid)
        
        '''
        self.openni_process = Popen(OPENNI_CMD, shell=True, preexec_fn=os.setsid)
        rospy.sleep(10)
        '''
        
        t_grasp = time.time()

        #*********************************************************************************#
        # Face Navigation Section (Working - dependant on PowerBotNavigation ROS Node)
        # - Load Face profiles and navigate to coordinates of 'personId' 
        personId = -1
        #personId = int(personStr)
        if personId is not None:
            # nav using face recognition
            
            # speak("Navigating to Person" + str(personId))
            self.command_done = False
            faceID = str(personId)

            # rotate away from the table
            rotation = Int32MultiArray()
            rotation.data = [-1, -1 , 180]

            self.rotation_reached = False
            self.pub_rotNav.publish(rotation)
            while self.rotation_reached != True:
                pass

            rotation.data = [-1, -1 , -90] 
            self.rotation_reached = False
            self.pub_rotNav.publish(rotation)
            while self.rotation_reached != True:
                pass
            
            rospy.loginfo("Facing away from table")
            self.location_reached = False
            
            '''
            ## COMMENT OUT
            rospy.sleep(10)

            defaultPos = Int32MultiArray()
            defaultPos.data = [1470,-850,0]
            self.pub_locNav.publish(defaultPos)
            while self.location_reached != True:
                pass
            rospy.loginfo("Arrived at default navigation coordinates")
            self.location_reached = False;
            ## END HERE
            '''

            if personId != -1:
                #if not self.command_done:
                rospy.loginfo("Starting PowerBot Navigation to person: " + faceID)
                self.pub_detectNav.publish(faceID)

            # anonymous nav using AR tags
            else:
                rospy.loginfo("Starting search for AR Tag 0")
                ar_id = 0
                self.command_done = False

                MAX_ITER = 50000
                iterations = 0
                while not ar_id in self.ar_markers and iterations < MAX_ITER:
                    rospy.loginfo("Looking for marker...")
                    rospy.sleep(0.1)
                    rospy.loginfo(str(iterations) + ": AR tag " + str(ar_id) + " is not seen by the camera")
                    iterations+=1

                rospy.loginfo("-----------------------------------------------")
                rospy.loginfo("FOUND AT POS: "+ str(self.ar_markers[ar_id]))
                rospy.loginfo("-----------------------------------------------")
                pos = self.ar_markers[ar_id]
                

                self.pub_arNav.publish("0")
                rospy.loginfo("Starting PowerBot Navigation to AR Tag 0")

        while (self.location_reached == False and self.arTag_reached == False):
            pass
        self.location_reached = False 
        self.arTag_reached = False

        t_user = time.time()

        print "DONE! Handing it to the user...."
        while not(self.rightGripperState.force == 0 and self.leftGripperState.force == 0):
            rospy.sleep(0.5)
            print "Waiting for user to take the object"

        # speak("Going to object (raised) position....")
        # OBJECT POSE (RAISED)4
        rospy.loginfo("DONE NAVIGATING... TAKE THE OBJECT")
        rospy.sleep(2)

        self.goToWaiting(baxterArm)

        # rotate away from the table
        rotation = Int32MultiArray()
        rotation.data = [-1, -1 , 180]

        self.rotation_reached = False
        self.pub_rotNav.publish(rotation)
        while self.rotation_reached != True:
            pass
        self.rotation_reached = False


        # rotate away from the table
        rotation = Int32MultiArray()
        rotation.data = [-1, -1 , 90]

        self.rotation_reached = False
        self.pub_rotNav.publish(rotation)
        while self.rotation_reached != True:
            pass
        self.rotation_reached = False

        # navigate to table
        tablePos = Int32MultiArray()
        tablePos.data = BaxterPositions.default_table

        self.pub_locNav.publish(tablePos)
        while self.location_reached != True:
            pass
        rospy.loginfo("Arrived at default object table coordinates")
        self.location_reached = False 

    #*********************************************************************************#
    
        t_nav_back = time.time()

        times_list = [t_start, t_table, t_ope, t_color, t_grasp, t_user, t_nav_back]
        retrieve_times_to_file(times_list)
        #self.rate.sleep()

    def command_sort(self):
        print "Starting Cleanup Task"
        
        # set arms to waiting position
        self.goToWaiting(0)
        self.goToWaiting(1)

        '''
        # navigate to table position
        tablePos = Int32MultiArray()
        tablePos.data = [300, -1050, 180]

        self.pub_locNav.publish(tablePos)
        while self.location_reached != True:
            pass
        rospy.loginfo("Arrived at default object location coordinates")
        self.location_reached = False
        '''

        #Remove Old PCDs
        removePCDs(OPE_DIR)
        rospy.sleep(1)
        
        #*******************************************************************************************************************#
        # Object Pose Estimation Selection (WORKING)
        # - Estimate Object Poses 
        # - Save OPE Results to txt
        # - Average Hues and save to color file
        # - Grasp Object
        # - Face Navigation to user

        # Run OPE
        OPEAssist.runOPE()
        OPEAssist.loadOPEResults()
        rospy.sleep(1)
        
        # Get OPE Object Colors
        colors_process = Popen(COLORS_CMD, shell=True, preexec_fn=os.setsid)
        rospy.sleep(1)

        #*******************************************************************************************************************#
        # Show OPE Results, Grab the Object
        if OPEAssist.objCount > 0:

            rospy.loginfo("Adding Table Collision Model")
            #ADD Table Collision Model
            self.moveit.addObject("TABLE",
                                  OPEAssist.tablePos,
                                  OPEAssist.tableSize)

            # ADD Object Collision Models
            for x in OPEAssist.objList:
                self.moveit.addObject("OBJECT" + str(x['objNumber']),
                                      x['objPos'],
                                      x['objSize'])

            # Object Selection
            objectNum = -1
            colorFile = open(OPE_DIR + "ObjectColors.txt")
            content = colorFile.readlines()
            print content

            # iterate through all detected objects
            i  = 0
            for colors in content:
                
                objColor = colors.strip("'").rstrip()
                print "checking: " + objColor
                objectNum = i
                
                if OPEAssist.objList[objectNum]['objPos'][1] > 0:
                    #self.goToWaiting(baxterArm = 1)
                    baxterArm = 1
                else:
                    #self.goToWaiting(baxterArm = 0)
                    baxterArm = 0

                self.gotoObject(OPEAssist.objList[objectNum]['objPos'], [0, 0, 0, 0], objectNum)
                force1 = self.measureDeformation(baxterArm)
                force2 = self.measureDeformation(baxterArm)
                
                self.moveGripper(selectedGripper = baxterArm, pos = None, openClose = "close")
                rospy.sleep(1)
                self.goAwayFromObject(OPEAssist.objList[objectNum]['objPos'], [0, 0, 0, 0], objectNum)

                if (force1+force2)/2 > 45:
                    print "nondeformable"
                    
                    if baxterArm == 1:
                        self.gotoSide( BaxterPositions.lowerLeftSidePos_forward, BaxterPositions.normalRot, leftRight = "left")
                    else:
                        self.gotoSide( BaxterPositions.lowerRightSidePos_forward, BaxterPositions.normalRot, leftRight = "right")

                    rospy.sleep(1)
                    self.moveGripper(selectedGripper=baxterArm, pos = None, openClose = "open")
                    rospy.sleep(1)
                    
                else:
                    print "deformable"
                    
                    if baxterArm == 1:
                        self.gotoSide( BaxterPositions.lowerLeftSidePos_backward, BaxterPositions.normalRot, leftRight = "left")
                    else:
                        self.gotoSide( BaxterPositions.lowerRightSidePos_backward, BaxterPositions.normalRot, leftRight = "right")
                    
                    rospy.sleep(1)
                    self.moveGripper(selectedGripper=baxterArm, pos = None, openClose = "open")
                    rospy.sleep(1)
                    
                i = i + 1
                '''
                if objColor == 'red':
                    self.gotoObject(OPEAssist.objList[objectNum]['objPos'], [0, 0, 0, 0], objectNum)
                    self.bringToPos( BaxterPositions.sortedOnePos, BaxterPositions.sortedOneRot)
                elif objColor == 'white':
                    self.gotoObject(OPEAssist.objList[objectNum]['objPos'], [0, 0, 0, 0], objectNum)
                    self.bringToPos( BaxterPositions.sortedTwoPos, BaxterPositions.sortedTwoRot)
                elif objColor == 'pink':
                    self.gotoObject(OPEAssist.objList[objectNum]['objPos'], [0, 0, 0, 0], objectNum)
                    self.bringToPos( BaxterPositions.sortedThreePos, BaxterPositions.sortedThreeRot)
                '''
        #*******************************************************************************************************************#
        else:
            rospy.loginfo("No Objects Detected")
            return 
        
        '''
        pos1, force1, maxforce1 = self.measureDeformation(baxterArm = 1)
        pos2, force2, maxforce2 = self.measureDeformation(baxterArm = 1)
        print pos1, force1, maxforce1
        print pos2, force2, maxforce2

        if force1 >= 0.0 and force1 < 1.0 and force2 >= 0.0 and force2 < 1.0:
            print "deformable"

        else:
            print "non-deformable"

        '''

        '''
        elif force1 >= force2 - 2.0 and force1 <= force2 + 2.0:
            print "non-deformable"
        
        else:
            print "semi-deformable"
        '''
            

    #*******************************************************************************************************************#
    # END CLEANUP TASK
    #*******************************************************************************************************************#

    #*******************************************************************************************************************#
    # Book Retreival Task
    #*******************************************************************************************************************#

    def arMarker_callback(self, data):
        self.ar_markers = {}
        for marker in data.markers:
            #print "adding marker: " + str(marker.id)
            try:
                (trans,rot) = self.tf.lookupTransform('/ar_marker_'+str(marker.id), '/base', rospy.Time(0))
                self.ar_markers[int(marker.id)] = [trans[2], \
                                                   trans[1], \
                                                   trans[0]]

                p = PointStamped()
                p.point.x = 0
                p.point.y = 0
                p.point.z = 0
                p.header.frame_id = '/ar_marker_'+str(marker.id)
                p = self.tf.transformPoint('/base', p)
                self.ar_markers[int(marker.id)] = [p.point.x, p.point.y, p.point.z]
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue


    def bookFound_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + " Found Book at: %d %d\n", data.data[0], data.data[1])

        self.preBookReached = False
        self.bookFound = True
        self.preBookReached = self.gotoPreBook(data.data[0], data.data[1])

    # align gripper pos in y 
    def gotoPreBook(self, x, y):
        rospy.loginfo("Found the book, adjusting")
        pix_tolerance = 2.0
        baxterArm = self.activeArm
        CAMERA_HEIGHT = 640
        CAMERA_WIDTH = 400
        CAMERA_OFFSET = 16          # compensate for offset of camera on baxters hand

        y_error = (CAMERA_HEIGHT/2) - y + CAMERA_OFFSET
        y_error = y_error/10.0

        x_error = (CAMERA_WIDTH/2) - x
        x_error = x_error/10.0
        rospy.loginfo("Manipulator position error: y:" + str(y_error) + " x: " + str(x_error))

        if y_error > pix_tolerance:
            self.adjustEndEffector(xyz = "xyz", val = [-5,-y_error, x_error], baxterArm = baxterArm)
            rospy.sleep(0.3)
            return False

        elif y_error < -1*pix_tolerance:
            self.adjustEndEffector(xyz = "xyz", val = [-5,-y_error, x_error], baxterArm = baxterArm)
            rospy.sleep(0.3)
            return False

        else:
            return True

        return False

    
    # utility functon to move the end effector in xyz
    def adjustEndEffector(self, xyz, val, baxterArm):

        if baxterArm == 0:
            gripperState = self.leftGripperState
        else:
            gripperState = self.rightGripperState

        if xyz == "y" or xyz == "Y":
            new_y = gripperState.y + 0.003*float(val) 
            newRightGripperLoc = [gripperState.x, new_y, gripperState.z]
            print "Current Pos: " + str([gripperState.x, gripperState.y, gripperState.z]) 
            print "Corrected Pos: " + str(newRightGripperLoc) 

            yCorrectedPlan = self.moveit.createPathPlan(newRightGripperLoc, baxterArm, BaxterPositions.normalRot)
            self.moveit.slowPlanVelocity(yCorrectedPlan)
            self.moveit.executePlan(baxterArm, yCorrectedPlan)

        elif xyz == "x" or xyz == "X":
            new_x = gripperState.x + 0.003*float(val)
            newRightGripperLoc = [new_x, gripperState.y, gripperState.z]
            print "Current Pos: " + str([gripperState.x, gripperState.y, gripperState.z]) 
            print "Corrected Pos: " + str(newRightGripperLoc) 

            xCorrectedPlan = self.moveit.createPathPlan(newRightGripperLoc, baxterArm, BaxterPositions.normalRot)
            self.moveit.slowPlanVelocity(xCorrectedPlan)
            self.moveit.executePlan(baxterArm, xCorrectedPlan)

        elif xyz == "z" or xyz == "Z":
            new_z = gripperState.z + 0.003*float(val)
            newRightGripperLoc = [gripperState.x, gripperState.y, new_z]
            print "Current Pos: " + str([gripperState.x, gripperState.y, gripperState.z]) 
            print "Corrected Pos: " + str(newRightGripperLoc) 

            zCorrectedPlan = self.moveit.createPathPlan(newRightGripperLoc, baxterArm, BaxterPositions.normalRot)
            self.moveit.slowPlanVelocity(zCorrectedPlan)
            self.moveit.executePlan(baxterArm, zCorrectedPlan)

        elif xyz == "xyz" or xyz == "XYZ":
            new_x = gripperState.x + 0.003*float(val[0])
            new_y = gripperState.y + 0.003*float(val[1])
            new_z = gripperState.z + 0.003*float(val[2])
            newRightGripperLoc = [new_x, new_y, new_z]
            print "Current Pos: " + str([gripperState.x, gripperState.y, gripperState.z]) 
            print "Corrected Pos: " + str(newRightGripperLoc) 

            zCorrectedPlan = self.moveit.createPathPlan(newRightGripperLoc, baxterArm, BaxterPositions.normalRot)
            self.moveit.slowPlanVelocity(zCorrectedPlan)
            self.moveit.executePlan(baxterArm, zCorrectedPlan)



    # command baxterArm to move to AR tag postion with xyz offset
    def moveToARTag(self, ar_id, baxterArm, off_x=0, off_y=0, off_z=0):
        rospy.loginfo("Looking for AR marker " + str(ar_id) + "...")
        # find the marker
        rep = 10
        while not ar_id in self.ar_markers:
            rospy.loginfo("Looking for marker...")
            rospy.sleep(0.1)
            rep-=1
            if rep <= 0:
                rospy.loginfo("AR tag " + str(ar_id) + " is not seen by the camera")
                return -1

        rospy.loginfo("-----------------------------------------------")
        rospy.loginfo("FOUND AT POS: "+ str(self.ar_markers[ar_id]))
        rospy.loginfo("-----------------------------------------------")
        pos = self.ar_markers[ar_id]

        # create adjusted position
        adj_pos = self.ar_markers[ar_id]
        adj_pos[0] = adj_pos[0] + off_x
        adj_pos[1] = adj_pos[1] + off_y
        adj_pos[2] = adj_pos[2] + off_z

        # move to pre-shelf pose
        rospy.loginfo("Adjust AR POS: " + str(adj_pos))
        shelfPlan = self.moveit.createPathPlan(adj_pos, baxterArm, BaxterPositions.normalRot)
        self.moveit.slowPlanVelocity(shelfPlan)
        success = self.moveit.executePlan(baxterArm, shelfPlan)
        rospy.sleep(1)

        if not success:
            rospy.loginfo("Failed to execute plan to ar tag " + str(ar_id))
            return -1
        else:
            rospy.loginfo("Arrived at AR Tag " + str() )
            return pos

    def command_retrieve_book(self, personStr, title, useMarkers=False):
        
        rospy.loginfo("Starting Book Retrieval Task")
        
        # set arms to waiting position
        self.goToWaiting(0)
        rospy.sleep(1)
        self.goToWaiting(1)
        rospy.sleep(1)

        self.shelf_marker_id = 9
        # set it to use left arm
        self.activeArm = 0
        baxterArm = self.activeArm
        self.pre_shelf_pos = self.moveToARTag(ar_id = self.shelf_marker_id, baxterArm=baxterArm, off_x = -0.15, off_y = 0.0, off_z = 0.04)
        
        if self.pre_shelf_pos == -1:
            rospy.loginfo("Failed to move to preshelf pos")
            return


        # perform OCR
        #bookTitle = "george orwell"
        bookTitle = str(title)
        rospy.loginfo("Looking for Book: " + str(bookTitle))
        self.pub_bookTitle.publish(bookTitle)

        iterations = 0
        max_iter = 10            # number of iterations to scan the shelf
        # scan the shelf
        while self.preBookReached != True and iterations < max_iter:
            #rospy.sleep(3)

            OCR_iterations = 5
            #self.bookFound = False
            for i in range(OCR_iterations):
                self.pub_bookTitle.publish(bookTitle)
                rospy.sleep(1.0)
                if self.bookFound:
                    rospy.loginfo("Book Found after " + str(i) + " Iterations of OCR")
                    break  

            if not self.bookFound:
                rospy.loginfo("STATUS: SEARCHING...")
                self.adjustEndEffector(xyz = "xyz", val = [-5,12,2], baxterArm = baxterArm)
                iterations+=1


            time.sleep(0.6)

            

            # rescan back to far right
            if self.preBookReached != True and self.bookFound != True and iterations == max_iter-1:
                # move the end effector back to avoid hitting the shelf
                rospy.loginfo("STATUS: RESET")
                self.adjustEndEffector(xyz = "xyz", val = [-15,0,0], baxterArm = baxterArm)
                self.pre_shelf_pos = self.moveToARTag(ar_id = self.shelf_marker_id, baxterArm=baxterArm, off_x = -0.15, off_y = 0.0, off_z = 0.04)
                if self.pre_shelf_pos == -1:
                    rospy.loginfo("Failed to move to preshelf pos")
                    return
                iterations=0

            elif self.preBookReached != True and self.bookFound == True and iterations == max_iter-1:
                # move the end effector slightly to un-obscure the book
                rospy.loginfo("STATUS: OBSCURED...adjusting")
                self.adjustEndEffector(xyz = "xyz", val = [-5,-6,2], baxterArm = baxterArm)
        
        if self.preBookReached == True:
            rospy.loginfo("Moving in to grasp the book")
            self.adjustEndEffector(xyz = "xyz", val = [25,0,0], baxterArm = baxterArm)
            rospy.sleep(0.5)

            rospy.loginfo("Closing Gripper and Moving out")
            self.moveGripper(baxterArm, 0, None)
            rospy.sleep(1.0)

            self.adjustEndEffector(xyz = "xyz", val = [-70,0,0], baxterArm = baxterArm)
            rospy.sleep(0.5)

            rospy.loginfo("Going to waiting position")
            self.goToWaiting(0)
            rospy.sleep(0.5)

        #*******************************************************************************************************************#
        # END BOOK RETRIEVE
        #*******************************************************************************************************************#