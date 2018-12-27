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
from sensor_msgs.msg import CompressedImage , Image
from std_msgs.msg import Float32

# We do not use cv_bridge , it does not support CompressedImage
# from cv_bridge import CvBridge, CvBridgeError

VERBOSE = False

class Controller:

    def __init__(self,team_name):
		''' Initialize ROS pub , ros sub '''

		self.team_name = team_name

		self.publisher_speed = rospy.Publisher('/' + team_name + '_speed', Float32, queue_size = 1 )
		self.publisher_angle = rospy.Publisher('/' + team_name + '_angle', Float32, queue_size = 1 )

		self.subscriber = rospy.Subscriber("/" + team_name + "_image/compressed",
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
        speed = Float32()

        angle.data = 1 ;
        speed.data = 50 ;

        self.publisher_speed.publish(speed)
        self.publisher_angle.publish(angle)
