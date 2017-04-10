#!/usr/bin/env python

import rospy
import socket
import sys, os, sched, time, errno, random

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
# rosrun tf static_transform_publisher 0.17 0.075 0.61 0.00 32.0 -3.00 180.0 /base /camera_link 25
# rosrun tf static_transform_publisher 0.15 0.08 0.56 -0.008 0.08 -0.003 0.5   #better??

# roslaunch openni_launch openni.launch device_id:=B00361708600048B (downward)
# roslaunch openni_launch openni.launch device_id:=A00367801249047A (upward)
# TO GET KINECT ID: lsusb -v -d 045e:02ae | grep -e "Bus\|iSerial"

# for book stuff
# 0: roslaunch baxter_controll DEMO_RETRIEVE.launch
# 1: rosrun tf static_transform_publisher 0.18 0.03 0.54 -0.000 0.088 -0.000 0.5 /base /camera_link 25
# 2: roslaunch ar_track_alvar baxter_indiv.launch

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
        self.action.mimic(self.vision)
        #-----------------------------------------------------------#
        '''
        


'''
bx = BaxterController()
rospy.spin()
'''
COLOR = {'yellow': 'y', 'orange': 'o', 'red': 'r', 'purple': 'p', 'blue': 'b', 'green': 'g', 'pink': 'p', 'white': 'w', 'black': 'k'}

# This function is called when a connection is received from the mobile app
def clientthread(conn1):
    ''' * * * * * * * * * * * * * * * * * * * * * * * * * * *
    *                                                       *
    *   # function to receive commands from mobile app      *
    *                                                       *
    * * * * * * * * * * * * * * * * * * * * * * * * * * * '''
    bx = BaxterController()                        # instatiate the baxter command class
    print 'Waiting for command'

    while not rospy.is_shutdown():
        if use_App:
            client_mesg = conn1.recv(1024)      # message = command,id,color,object : i.e. 1,0,b,bowl
            #client_mesg = "1,0,kyle,white,blah"
        else:
            client_mesg = "2,0,kyle,unix,blah"
            #client_mesg = "1,0,kyle,red,blah"
            
        if client_mesg != "" or client_mesg != None:    # non-empty message was received

            print "Baxter app command message: '"+client_mesg+"'"
            commandID = client_mesg.split(",")                          # parse the comma delimited message

            #**************************************************************************************************************************#
            # case through all the possible command codes
            #**************************************************************************************************************************#
            if (str(commandID[0]) == "1"):                              # command 1 = object retreival
                print "command = 1, Getting " + commandID[2] + " the " + COLOR[commandID[3]] + " " + commandID[4]
                try:
                    bx.action.command_retrieve(personStr = commandID[1], color = str(commandID[3]))
                    #bx.action.command_grasp_object(color = str(commandID[3]))      
                    pass                                                                                   
                except rospy.ROSInterruptException:
                    pass

            elif (str(commandID[0]) == "3"):                              # command 1 = object retreival
                print "command = 2, Getting " + commandID[2] + " the book " + commandID[4] + " by " + commandID[3]
            
                #bx.action.command_retrieve(personStr = commandID[1], color = str(commandID[3]))
                bx.action.command_retrieve_book(personStr = str(commandID[2]), title = commandID[4])      
            
            elif (str(commandID[0]) == "4"):
                print "command = 4, navigating to " + str(commandID[2])
                bx.action.command_navigate(str(commandID[1]))


            return 1

        else:
            return 0;


use_App = True

if use_App:
    host = '' #127.0.0.1
    #port1 = 8935 # command input port
    port1 = random.randint(80, 9999)

    # create a re-bindable socket
    s1 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s1.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    print 'Socket created'
    #Bind socket to local host and port
    try:
        s1.bind((host, port1))
    except socket.error as msg:
        print 'Bind failed. Error Code : ' + str(msg[0]) + ' Message ' + msg[1]
        sys.exit()
    print 'Socket bind complete with port: ' + str(port1)
    s1.listen(1)
    print 'Socket now listening'

    while 1:
        #wait to accept a connection - blocking call
        conn1, addr1 = s1.accept()
        print 'Connected to Baxter Controller ' + addr1[0] + ':' + str(addr1[1]) + ", on port: " + str(port1)
        
        # continually try to receive commands from the app 
        while 1:
     
            # make a call to function for commanding baxter
            result = clientthread(conn1)

            # when function is over, tell the app that command is done
            conn1.send("done")
            continue
            #break

        # remove this to do more than one action    
        #break



    s1.close()

else:
    clientthread(None)




