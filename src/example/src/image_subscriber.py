#!/usr/bin/env python
# http://wiki.ros.org/rospy_tutorials/Tutorials/WritingImagePublisherSubscriber

__author__ = 'DreamHigh Team'
__version__ = '0.1'
__license__ = 'MIT'

# Python libs
import sys, time

# numpy and scipy
import numpy as np
from scipy.ndimage import filters

# OpenCV
import cv2

# ROS libs
import roslib
import rospy

# ROS Messages
from sensor_msgs.msg import CompressedImage , Image, Float32

# We do not use cv_bridge , it does not support CompressedImage
# from cv_bridge import CvBridge, CvBridgeError

VERBOSE = True

class ImageFeature:
	def __init__(self):
		''' Initialize ROS pub , ros sub '''
		# topic where we publish
		# self.image_pub = rospy.Publisher('/output/image_raw/compressed',
		# 	CompressedImage, queue_size = 1 )

		# subscribed Topic
		# self.subscriber = rospy.Subscriber("/camera/image/compressed",
		# 	CompressedImage, self.callback, queue_size=1)

		self.subscriber = rospy.Subscriber("/Team1_image/compressed",
			CompressedImage, self.callback, queue_size=1)

		if VERBOSE:
			print "subscribed to /camera/image/compressed"

	def callback(self,ros_data):
		''' Callback function of subscribed topic.
		Here images get converted and features detected '''
		if VERBOSE:
			print 'received image of type: "%s"' % ros_data.format

		#### direct conversion to CV2 ####
		np_arr = np.fromstring(ros_data.data, np.uint8)
		# image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
		image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0

		cv2.imshow('cv_img', image_np)
		cv2.waitKey(2)

		angle = Float32()
		

		# msg = CompressedImage()
		# msg.header.stamp = rospy.Time.now()
		# msg.format = "jpeg"
		# msg.data = np.array(cv2.imencode('.jpg', image_np)[1]).tostring()
		# # Publish new image
		# self.image_pub.publish(msg)
		# #self.subscriber.unregister()

def main(args):
	'''Initializes and cleanup ros node '''
	ic = ImageFeature()
	rospy.init_node('image_subscriber', anonymous = True )
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down ROS"

	cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)
