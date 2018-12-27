#!/usr/bin/env python
# http://wiki.ros.org/rospy_tutorials/Tutorials/WritingImagePublisherSubscriber

__team__ = 'Team1' # just for test
# __team__ = 'DreamHigh' # for real team name

__author__ = 'DreamHigh'
__version__ = '0.1'
__license__ = 'MIT'

# Python libs
import sys, time

# numpy and scipy
import numpy as np

# OpenCV
import cv2

# ROS libs
import roslib
import rospy

# ROS Messages
from sensor_msgs.msg import CompressedImage , Image
from std_msgs.msg import Float32

# We do not use cv_bridge , it does not support CompressedImage
# from cv_bridge import CvBridge, CvBridgeError

# syntax : from <package>.<pythonfile> import <class or def>
from lane_detect.controller import Controller

def main(args):
	'''Initializes and cleanup ros node '''

	# Subscribes the images from Unity3D
	# Put it in lane detector
	# Control the car (publish the angle, speed )
	ctr = Controller(__team__)

	rospy.init_node('image_subscriber', anonymous = True )
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down ROS ... "

	cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)
