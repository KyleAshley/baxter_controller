#!/usr/bin/env python

import sys, os, sched, time, errno


qx = 0.005
qy = 0.135
qz = 0.0
qw = 0.5

x = 0.24
y = 0.018
z = 0.555

# wiki
R = [ 1-2*qy**2, 			2*qx*qy - 2*qz*qw, 		2*qx*qz + 2*qy*qw,
	  2*qx*qy + 2*qz*qw, 	1-2*qx**2 - 2*qz**2, 	2*qy*qz - 2*qx*qw,
	  2*qx*qz - 2*qy*qw, 	2*qy*qz + 2*qx*qw, 		1-2*qx**2 - 2*qy**2 ]

# https://www.fd.cvut.cz/personal/voracsar/GeometriePG/PGR020/matrix2quaternions.pdf
R = [ 1-2*qy**2 - 2*qz**2,	2*qx*qy + 2*qz*qw, 		2*qx*qz - 2*qy*qw,
	  2*qx*qy - 2*qz*qw, 	1-2*qx**2 - 2*qz**2, 	2*qy*qz + 2*qx*qw,
	  2*qx*qz + 2*qy*qw, 	2*qy*qz - 2*qx*qw, 		1-2*qx**2 - 2*qy**2 ]

print R[:3]
print R[3:6]
print R[6:]


m = [   0.172803,  -0.419512,    0.89115,   0.24,
 		-0.983109,  -0.128849,   0.129979,   0.018,
  		0.060296,  -0.898558,  -0.434691,   0.555,
        0,          0,          0,          1 ]

qw = ((1 + m[0] + m[5] + m[10])**0.5) /2
qx = (m[9] - m[6])/( 4 *qw)
qy = (m[2] - m[8])/( 4 *qw)
qz = (m[4] - m[1])/( 4 *qw)

print qz, qy, qz, qw