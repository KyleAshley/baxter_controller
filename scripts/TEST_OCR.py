#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import cv2
import os

bookTitle = "python pocket reference"
#bookTitle = "the dark knight"
#bookTitle = "stiquito for beginners"
#bookTitle = "the x files fight the future"

rospy.init_node('TEST OCR', anonymous=True)
rate = rospy.Rate(60) # 10hz
my_env = os.environ
cv2.namedWindow("Key to exit")

pub_bookTitle = rospy.Publisher('OCR/bookTitle', String, queue_size = 1 )

while(1):
	pub_bookTitle.publish(bookTitle)
	if cv2.waitKey(20) != -1:
		break
