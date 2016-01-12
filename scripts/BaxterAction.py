#!/usr/bin/env python
import rospy
import BaxterMoveIt
import BaxterPositions
import baxter_interface
from BaxterUtil import clamp

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


class BaxterAction:
    IKSVC_LEFT_URI = "ExternalTools/left/PositionKinematicsNode/IKService"
    IKSVC_RIGHT_URI = "ExternalTools/right/PositionKinematicsNode/IKService"

    def __init__(self):

        # initialize interfaces
        self.rs = baxter_interface.RobotEnable(CHECK_VERSION)
        init_state = self.rs.state().enabled

        self.tf = tf.TransformListener()

        self.moveit = BaxterMoveIt.BaxterMoveIt()

        # Grippers
        self.leftGripper = baxter_interface.Gripper('left', CHECK_VERSION)
        self.rightGripper = baxter_interface.Gripper('right', CHECK_VERSION)

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

        # MIMIC
        self.mimic_timer = None
        self.ik_window_size = 40
        self.detection_window_size = 40
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


                '''
                # rotation
                (r_trans,r_rot) = self.tf.lookupTransform(
                    '/right_hand_1',
                    '/right_elbow_1',
                    rospy.Time(0))

                (l_trans,l_rot) = self.tf.lookupTransform(
                    '/left_hand_1',
                    '/left_elbow_1',
                    rospy.Time(0))
                '''

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
            lh_y = clamp(r_trans[0], -1.2, 1.2)
            rh_y = clamp(l_trans[0], -1.2, 1.2)

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


            #print "ROTATION R:", r_rot
            #print -1.0*math.pi + rh_roll, 0.5*math.pi + rh_pitch, -1.0*math.pi + rh_yaw
            
            #self.set_left_coords(lh_x, lh_y, lh_z, ep=0.5*math.pi + 3.0*(lh_pitch+0.1), ey=-1.0*math.pi - (lh_roll/2.0))
            #self.set_right_coords(rh_x, rh_y, rh_z, er=((-1.0*m.pi) + (r_rot[0]/4.0)), ep=((m.pi*0.5) + (r_rot[1]/4.0)), ey=((-1.0*m.pi) + (r_rot[2]/4.0)))
            
            # gooood
            #self.set_right_coords(rh_x, rh_y, rh_z, ep=0.5*math.pi + 3.0*(rh_pitch+0.1))

            #self.set_left_coords(lh_x, lh_y, lh_z, lh_roll, lh_pitch, lh_yaw)
            self.set_right_coords(rh_x, rh_y, rh_z)
            self.set_left_coords(lh_x, lh_y, lh_z)


        except:
            return


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
                self.rightGripper.command_position(pos, block=False)
            else:
                self.leftGripper.command_position(pos, block=False)

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

    # -------------------------------------------------------------------------------------------------------------------------#
    # arm movement and IK
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
        
    # -------------------------------------------------------------------------------------------------------------------------#
