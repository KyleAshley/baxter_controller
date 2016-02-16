#!/usr/bin/env python

#import rospy
import sys, os, sched, time, errno

import glob
import cv2
from cv2 import *
import cv
import multiprocessing
import numpy as np
import math
import operator
import img_proc_accessories




from cv2 import __version__

def strType(var):
    try:
        if int(var) == float(var):
            return 'int'
    except:
        try:
            float(var)
            return 'float'
        except:
            return 'str'

# extracts encoded RGB color information from .pcd file 
# returns r, g, b averages
def extractRGB(filename):
	f = open(filename, 'r').readlines()
	rgb_list = [[int(float(ff.split()[-1])) << 58 & 0x000ff, int(float(ff.split()[-1])) << 40 & 0x000ff, int(float(ff.split()[-1])) << 32 & 0x000ff] for ff in f[11:]]
	
	print int(float( 4.51182372402e-39)ff
	print int(float( 4.51182372402e-39) * (1<<133))
	r = (sum(a[0] for a in rgb_list))/len(rgb_list)
	g = (sum(a[1] for a in rgb_list))/len(rgb_list)
	b = (sum(a[2] for a in rgb_list))/len(rgb_list)

	print r, g, b

extractRGB(filename = "/home/kyle/catkin_ws/src/baxter_controller/scripts/OPE-Release/object_0.pcd")