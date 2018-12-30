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

# MultiThread
# import thread
# import time

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

# def tb_callback(x):
# 	print '[INFO] Value x =', x
#
# def trackbar(thread_name, delay):
# 	cv2.namedWindow('image')
# 	cv2.createTrackbar('R','image',0,255,tb_callback)
# 	cv2.createTrackbar('0 : OFF \n1 : ON', 'image',0,1,tb_callback)
# 	img = np.zeros((100,100,3),np.uint8)
# 	while 1:
# 		cv2.imshow('image',img)
# 		cv2.waitKey(1)
# 		time.sleep(100)

def main(args):
	'''Initializes and cleanup ros node '''

	# Subscribes the images from Unity3D
	# Put it in lane detector
	# Control the car (publish the angle, speed )
	ctr = Controller(__team__)

	print '\n\n\n'
	print ' [INFO] PYTHON version :', sys.version
	print ' [INFO] CV2 version :', cv2.__version__
	print ' [INFO] Team name: ', __team__

	# try:
	# 	thread.start_new_thread(trackbar,("Trackbar",2,))
	# except:
	# 	print("[ERROR] Unable to start thread ! ")

	# cv2.namedWindow('image')
	# cv2.createTrackbar('R','image',0,255,tb_callback)
	# cv2.createTrackbar('0 - 1', 'image',0,1,tb_callback)
	# img = np.zeros((100,100,3),np.uint8)
    #
	# rospy.init_node('image_subscriber', anonymous = True )
    #
	# while 1:
	# 	cv2.imshow('image',img)
	# 	cv2.waitKey(1)
    #
	# 	try:
	# 		rospy.spin()
	# 	except KeyboardInterrupt:
	# 		print "Shutting down ROS ... "
    #
	# 	cv2.destroyAllWindows()

	rospy.init_node('image_subscriber', anonymous = True )
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down ROS ... "

	cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)
