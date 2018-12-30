import os
import numpy as np

# OpenCV
import cv2

# ROS libs
import roslib
import rospy

'''
    Check value in __init__
    detect(..) is called when we actually runs the car test
    debug_img() is called when testing with image
    debug_video() is called when testing with video
'''

class SignRecognizer:
    def __init__(self):
        ''' Initialize values '''
        # print('[debug] CURRENT ', os.path.dirname(os.path.realpath(__file__)))
        self.sign_cascade = cv2.CascadeClassifier(os.path.dirname(os.path.realpath(__file__))+'/cascade.xml')

    def detect(self,img,gray=np.array([])):
        if not gray.any():
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray = gray[:int(self.h/2),:]

        # localizing signs
        ret, sign = self.localize(img,gray,reduced=True)

        # recognizing sign
        if ret:
            print('[INFO] Detected Sign!')
            self.recognize(sign)

    def recognize(self, sign):
        ''' the image that is the nearest sign '''
        pass

    def localize(self,img,gray,reduced=False):
        '''
        Args:
            img : original image (for showing purpose )
            gray : a half top of org image

        scale_factor: specifying how much the image size is reduced at each image scale
        min_neighbors: specifying how many neighbors each candidate rectangle should have to retain it
        min_size: minimum posible object size. Ojbects smaller than that are ignored
        max_size: maximum posible object size. Objects larger than that are ignored
        '''
        signs = self.sign_cascade.detectMultiScale(gray,1.5,2)
        return False, False
        # rs = np.copy(gray)
        # if len(signs)>0:
        #     max_size = 0
        #     sign = dict()
        #     sign['x']=0
        #     sign['y']=0
        #     sign['w']=0
        #     sign['h']=0
        #     for (x,y,w,h) in signs:
        #         if not reduced:
        #             img = cv2.rectangle(img,(x,y),(x+w,y+h), (255,0,0), 3)
        #         if w*h > max_size:
        #             max_size = w*h
        #             sign['x'] = x
        #             sign['y'] = y
        #             sign['w'] = w
        #             sign['h'] = h
        #     if not reduced:
        #         cv2.imshow('Signs detection', img)
        #         cv2.waitKey(1)
        #     return True, rs[sign['y']:sign['y']+sign['h'], sign['x']:sign['x']+sign['w']]
        # else:
        #     return False, False
