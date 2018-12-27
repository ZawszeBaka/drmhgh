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

import lane_detector
from lane_detector import LaneDetector

'''
    EVERYTHING IS GOING ON HERE:
        - Subscribes the images from Unity3D
        - Pass the images in LaneDetector for detecting lane (return with the binary image that locates the lane)
        - Pass the images in ObjectDetector for finding the objects
        - Processing the algorithm to move the car

'''

VERBOSE = False

class Controller:

    def __init__(self,team_name, manual=False):
        ''' Initialize ROS pub , ros sub '''

        self.team_name = team_name

        self.lane_detector = LaneDetector()

        if not manual:
            self.publisher_speed = rospy.Publisher('/' + team_name + '_speed', Float32, queue_size = 1 )
            self.publisher_angle = rospy.Publisher('/' + team_name + '_steerAngle', Float32, queue_size = 1 )
            self.subscriber = rospy.Subscriber("/" + team_name + "_image/compressed", CompressedImage, self.callback, queue_size=1)
        else:
            self.subscriber = rospy.Subscriber("/" + self.team_name + "_image/compressed", CompressedImage, self.save_video_callback, queue_size=1)
            self.out = cv2.VideoWriter('/home/yus/Documents/Video/save_video.avi',
                                        cv2.VideoWriter_fourcc('M','J','P','G'),
                                        20, (self.lane_detector.w,self.lane_detector.h))

        # self.skip_frame = 2
        # self.i = 0

        if VERBOSE:
            print "subscribed to /camera/image/compressed"

    def callback(self,ros_data):
        ''' Callback function of subscribed topic.
        Here images get converted and features detected '''
        if VERBOSE:
            print 'received image of type: "%s"' % ros_data.format

        #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        # img = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0

        angle = Float32()
        speed = Float32()

        # if self.i % self.skip_frame == 0 :
        #     angle.data , speed.data = self.lane_detector.detect(img)
        #     self.pre_angle = angle
        #     self.pre_speed = speed
        # else:
        #     angle, speed = self.pre_angle, self.pre_speed
        # self.i += 1

        angle.data, speed.data =self.lane_detector.detect(img)

        print(' [INFO] angle = ', angle.data, ', speed = ', speed.data)

        self.publisher_speed.publish(speed)
        self.publisher_angle.publish(angle)

    def save_video_callback(self,ros_data):
        #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        # img = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0
        self.out.write(img)
        cv2.imshow('img', img)
        cv2.waitKey(1)
