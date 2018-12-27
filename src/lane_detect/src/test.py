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

# We do not use cv_bridge , it does not support CompressedImage
# from cv_bridge import CvBridge, CvBridgeError

# syntax : from <package>.<pythonfile> import <class or def>
import lane_detect.lane_detector
from lane_detect.lane_detector import LaneDetector

import lane_detect.controller
from lane_detect.controller import Controller

def parse_test_arg(args):
	for arg in args:
		if 'test:=' in arg:
			return arg.split(':=')[1]

def main(args):

	''' Test type : image , video '''
	test_type = parse_test_arg(args)

	lane_detector = LaneDetector()

	print '\n\n\n'
	print ' [INFO] PYTHON version ', sys.version
	print ' [INFO] CV2 version ', cv2.__version__
	print ' [INFO] TEST type: ', test_type
	if test_type == 'image':
		lane_detector.debug_img()
	elif test_type == 'video':
		lane_detector.debug_video()
	elif test_type == 'save_video':
		controller = Controller('Team1',manual=True)
		rospy.init_node('image_subscriber', anonymous = True)
		try:
			rospy.spin()
		except KeyboardInterrupt:
			controller.out.release()
			print "Shutting down ROS ... "
	else:
		pass

	print '\n\n\n'
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)
