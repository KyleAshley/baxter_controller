#!/usr/bin/env python
# Copyright (c) 2014, Andoni Aguirrezabal
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

default_table = [400, -1050, 180]

normalRot = [-0.014345, 0.70062, -0.025133, 0.71295]

leftWaitingPos = [0.41965, 0.60569, 0.28949]
leftWaitingRot = [-0.014345, 0.70062, -0.025133, 0.71295]

rightWaitingPos = [0.41965, -0.60569, 0.28949]
rightWaitingRot = [-0.014345, 0.70062, -0.025133, 0.71295]

goodWaitingPose = {'left_w0': -2.4344275075927735, 'left_w1': 1.1355292769348144, 'left_w2': -3.0587576875488285,
                   'left_e0': -0.3535825712036133, 'left_e1': 2.1402866918518066, 'left_s0': 0.07094661135864258,
                   'left_s1': -0.87398555289917}

trashPos = [0.77654, 0.78714, 0.25557]
trashRot = [-0.014345, 0.70062, -0.025133, 0.71295]

testPos = [0.72195, 0.49096, 0.28836]
testRot = [-0.014345, 0.70062, -0.025133, 0.71295]

lowerLeftSidePos = [0.455, 0.707, -0.012]
lowerLeftSideRot = [0.050, 0.839, -0.012, 0.5]

lowerRightSidePos = [0.455, -0.707, -0.012]
lowerRightSideRot = [0.0020, 0.875, -0.012, 0.5]


joint_names = {
	'left': ['left_e0', 'left_e1', 'left_s0', 'left_s1', 'left_w0', 'left_w1', 'left_w2'],
	'right': ['right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2']
}

# object scanning
ref_pos_r1 = dict(zip(joint_names['right'],[ 0.1537, 1.6183, 0.4337, -0.5184, -1.8795, -1.57, 0.3662]))
ref_pos_l1 = dict(zip(joint_names['left'],[ -1.0128, 1.6659, 0.2136, -0.3275, 2.5402, -1.1417, -0.8528]))

pos1_l_pivot1 = dict(zip(joint_names['left'],[ -0.8018, 1.2498, 0.0533, 0.1871, 2.4052, -1.5696, -0.5821]))
pos1_l_pivot2 = dict(zip(joint_names['left'],[ -1.0128, 1.6659, 0.2136, -0.3275, 2.5402, -1.1417, -0.8528]))
pos1_l_pivot3 = dict(zip(joint_names['left'],[ -1.4484, 1.7625, 0.4360, -0.6135, 2.9195, -1.0039, -1.378]))
pos1_l_pivot4 = dict(zip(joint_names['left'],[ -1.6252, 1.6528, 0.3754, -1.0154, 3.0599, -1.0737, -1.8239]))

ref_pos_r2 = dict(zip(joint_names['right'],[ 0.0, 2.2154, 0.42836, -0.6473, -1.2187, -1.5707, -0.04371]))

ref_pos_l3 = dict(zip(joint_names['left'],[-1.09602, 2.1203, 0.7784, -0.1395, 3.0096, -1.0622, -0.9621]))
ref_pos_r3 = dict(zip(joint_names['right'],[ 0.4467, 1.6375, 0.5073, -0.3919, -2.4808, -1.5707, 0.4778]))

pos2_l_pivot1 = dict(zip(joint_names['left'],[ -0.8728, 1.5957, 0.7037, 0.3635, 3.0549, -1.5347, -0.8275]))
pos2_l_pivot2 = dict(zip(joint_names['left'],[ -1.09602, 2.1203, 0.7784, -0.1395, 3.0096, -1.0622, -0.9621]))
pos2_l_pivot3 = dict(zip(joint_names['left'],[ -1.7088, 2.3316, 0.9759, -0.5714, 3.05914, -0.9154, -1.5351]))
pos2_l_pivot4 = dict(zip(joint_names['left'],[ -2.1863, 2.1191, 1.0745, -0.9065, 3.0587, -1.0649, -1.9190]))


rot1 = [ ]
rot2 = []
rot3 = []
rot4 = []
rot5 = []
rot6 = []
rot7 = []
rot8 = []
rot9 = []
rot10 =[]