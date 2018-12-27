import numpy as np

# OpenCV
import cv2

# ROS libs
import roslib
import rospy

from matplotlib import pyplot as plt

'''
    Check value in __init__
    detect(..) is called when we actually runs the car test
    debug_img() is called when testing with image
    debug_video() is called when testing with video
'''

class LaneDetector:
    def __init__(self):
        ''' Initialize values '''

        self.w = 320 # width
        self.h = 240 # height

        w = np.float32(self.w)
        h = np.float32(self.h)
        self.src = np.float32([[w*2/7-10, h/2-10], # top left
                              [0, h*4/5-3], # bottom left
                              [w*6/7+35, h*4/5-3], # top right
                              [w*2/3+10, h/2-10]]) # bottom right
        self.dst = np.float32([[w*2/7-10, h/2-10], # top left
                              [w*2/7-10, h*4/5-3], # bottom left
                              [w*5/7+10, h*4/5-3], # top right
                              [w*5/7+10, h/2-10]])# bottom

        # Stage 2
        self.sobel_kernel = 3
        self.abs_sobel_thresh_x = (20,100)
        self.abs_sobel_thresh_y = (20,100)
        self._mag_thresh = (30,100)
        self._dir_thresh=(0.7,1.3)
        self.color_thresh_low = (102,63,63)
        self.color_thresh_high = (255,170,170)

        self.s_thresh_min = 0
        self.s_thresh_max = 1

        # Stage 3
        self.margin = 40
        self.minpix = 20
        self.nwindows = 9

    def detect(self, img):
        '''
        Main Process is here !!
        Detect the lane and return the binary image that
        locates the lanes
        '''
        angle, speed = self._debug_process(img,wait_key_val=1,reduced = True)
        return angle, speed

    def debug_img(self):
        ''' debug images with the fixed size only, see self.w, self.h '''

        img_url = '/home/yus/Documents/Pic/first_frame.png'
        img = cv2.imread(img_url)

        self._debug_process(img)

    def debug_video(self):
        video_url = '/home/yus/Documents/Video/video_test.avi'
        cap = cv2.VideoCapture(video_url)
        if not cap.isOpened():
            print('[ERROR] Opening video stream or file. Failed! Make sure the video path exists!')
            return

        while True:
            ret, img = cap.read()
            if ret == True:
                # g_bin = self.apply_gradient_threshold(img,
                #                 abs_sobel_thresh_x = self.abs_sobel_thresh_x,
                #                 abs_sobel_thresh_y = self.abs_sobel_thresh_y,
                #                 _mag_thresh = self._mag_thresh,
                #                 _dir_thresh = self._dir_thresh)
                # warped_region_of_line, Minv = self.warp(g_bin, self.src, self.dst)
                # cv2.imshow('warped bi',warped_region_of_line)
                # hough_lines = self.apply_hough(np.uint8(warped_region_of_line))

                gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

                #
                lowThreshold = 90
                ratio = 3
                canny = cv2.Canny(gray,lowThreshold,ratio*lowThreshold,3)
                cv2.imshow('Canny', canny)
                warped_region_of_line, Minv = self.warp(canny, self.src, self.dst)
                hough_lines = self.apply_hough(np.uint8(warped_region_of_line))

                cv2.imshow('Hough',hough_lines)
                if cv2.waitKey(2) & 0xff == 27:
                    break
            else:
                break



    def _debug_process(self,img,wait_key_val = 0, reduced=False):
        ''' if wait_key_val == 0 , this is image processing
            otherwise, this is video processing '''

        if not reduced:
            img_vs_pts = self.add_points(img, self.src)
            cv2.imshow('Original(src pts)', img_vs_pts)
            self.waitKey(wait_key_val)

        ## Choosing 4 pts
        # warped_img, _ = self.warp(img, self.src, self.dst)
        # warped_img = self.add_points(warped_img,self.dst)
        # cv2.imshow('Warped Image', warped_img)
        # self.waitKey(wait_key_val)

        ### Stage 2:
        # Apply color threshold
        # c_bin = self.apply_color_threshold(img,
        #                 s_thresh_min = self.s_thresh_min,
        #                 s_thresh_max = self.s_thresh_max)

        # gradient threshold
        g_bin = self.apply_gradient_threshold(img,
                        abs_sobel_thresh_x = self.abs_sobel_thresh_x,
                        abs_sobel_thresh_y = self.abs_sobel_thresh_y,
                        _mag_thresh = self._mag_thresh,
                        _dir_thresh = self._dir_thresh)

        # Combine color and gradient with operator OR
        # combined_bin = self.combine_threshold(c_bin, g_bin)
        combined_bin = g_bin;
        if not reduced:
            cv2.imshow('Combined (color+gradient) threshold', combined_bin )
            self.waitKey(wait_key_val)

        # warped line
        warped_region_of_line, Minv = self.warp(combined_bin, self.src, self.dst)
        if not reduced:
            cv2.imshow('Warped Region of line (half bottom)', warped_region_of_line)
            self.waitKey(wait_key_val)

        ### Stage 3:
        # histogram
        histogram = self.get_histogram(warped_region_of_line)
        if wait_key_val == 0:
            plt.plot(histogram)
            plt.title('Histogram ')
            plt.show()

        # slide window
        status, center_windows, lines_info, ret = self.slide_window(warped_region_of_line,
                                histogram,
                                margin = self.margin,
                                minpix = self.minpix,
                                nwindows = self.nwindows,
                                wait_key_val=wait_key_val,
                                reduced = reduced)

        # Draw lane line
        # Error , not fixed yet
        #     lines_info['leftx'] = nonzerox[left_lane_inds]
        if ret:
            rs = self.draw_lane_lines(img,warped_region_of_line,Minv, lines_info)
        else:
            if wait_key_val == 0 :
                print(''' [WARNING] Cannot detect lane \n\t 1. You are out of lane \n\t 2. Change the value \n\t 3. Make sure you choose the correct 4 points ''')
            rs = img

        if not status:
            # If still detected line !!
            n_th_win = 3 # window number 2
            angle = self.calc_angle(center_windows,n_th = n_th_win)
            speed = self.calc_speed()

            rs = self.draw_line(rs, center_windows[n_th_win-1], (self.w/2,self.h))

            cv2.imshow('Lane Detection: Final Result', rs)
            self.waitKey(wait_key_val)

            return angle, speed
        else:
            # Failed to detect line or the car is out of lane
            print('[INFO] Failed to detect line or the car is out of lane !')
            return 0.0, 0.0

    ## ================== COMPUTATION ================================
    def calc_angle(self,center_windows, n_th = 2):
        ''' Calculate the angle '''
        x1,y1 = center_windows[n_th-1]
        x2,y2 = self.w/2, self.h
        return -np.arctan2((x2-x1),(y2-y1))/np.pi * 180.0

    def calc_speed(self):
        ''' Calculate the speed'''
        return 50

    ## ===================== ^^ METHODS ^^ ============================
    # Perspective Transform
    def warp(self,img, src, dst):
        '''
            src: source points
                ex: np.float32([[685, 450],
                                  [1090, 710],
                                  [220, 710],
                                  [595, 450]])
            dst: destination points
                ex: np.float32([[685, 450],
                                  [1090, 710],
                                  [220, 710],
                                  [595, 450]])
        '''

        img_size = (img.shape[1], img.shape[0])

        # the matrix
        M = cv2.getPerspectiveTransform(src, dst)
        Minv = cv2.getPerspectiveTransform(dst, src)

        warped_img = cv2.warpPerspective(img, M, img_size, flags=cv2.INTER_LINEAR)

        return warped_img, Minv

    def apply_color_threshold(self,img, s_thresh_min = 0, s_thresh_max = 1 ):
        # hue, lightness, saturation
        hls = cv2.cvtColor(img, cv2.COLOR_RGB2HLS)

        s_channel = hls[:,:,2]
        s_binary = np.zeros_like(s_channel)
        s_binary[(s_channel >= s_thresh_min) & (s_channel <= s_thresh_max)] = 1

        return s_binary

    def abs_sobel_thresh(self,img, orient='x', sobel_kernel=3, thresh=(0,255)):
        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        isX = True if orient == 'x' else False

        # applying sobel filter corresponding to axis x or y
        sobel = cv2.Sobel(gray, cv2.CV_64F, isX, not isX, ksize=sobel_kernel)

        # after applying kernel, value of each pixel can be exceed 255 or lower than 0
        # so we need to get the absolute value and scale it down to the range 0-255
        abs_sobel = np.absolute(sobel)
        scaled_sobel = np.uint8(255*abs_sobel/np.max(abs_sobel))

        # an array of zeros with the same shape and type as a given array
        grad_binary = np.zeros_like(scaled_sobel)

        # threshold
        grad_binary[(scaled_sobel >= thresh[0]) & (scaled_sobel <= thresh[1])] = 1

        return grad_binary

    def mag_thresh(self,img, sobel_kernel=3, mag_thresh=(0,255)):
        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

        # applying sobel filter corresponding to the axis x , y , respectively
        sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=sobel_kernel)
        sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=sobel_kernel)

        # sqrt( sobelx ^ 2 + sobely ^ 2 )
        abs_sobel = np.sqrt(sobelx**2 + sobely**2)
        scaled_sobel = np.uint8(255*abs_sobel/np.max(abs_sobel))

        # an array of zeros with the same shape and type as a given array
        mag_binary = np.zeros_like(scaled_sobel)

        # threshold
        mag_binary[(scaled_sobel >= mag_thresh[0]) & (scaled_sobel <= mag_thresh[1])] = 1

        return mag_binary

    def dir_threshold(self,img, sobel_kernel=3, thresh=(0, np.pi/2)):
        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

        sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=sobel_kernel)
        sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=sobel_kernel)

        abs_sobelx = np.absolute(sobelx)
        abs_sobely = np.absolute(sobely)

        # angle ( gradient direction ) = arctan ( y / x )
        grad_dir = np.arctan2(abs_sobely, abs_sobelx)

        # an array of zeros with the same shape and type as a given array
        dir_binary = np.zeros_like(grad_dir)

        # threshold
        dir_binary[(grad_dir >= thresh[0]) & (grad_dir <= thresh[1])] = 1

        return dir_binary

    def apply_gradient_threshold(self,img, ksize=3,
                        abs_sobel_thresh_x = (20,100),
                        abs_sobel_thresh_y = (20,100),
                        _mag_thresh = (30,100),
                        _dir_thresh=(0.7,1.3),
                        is_test = False): # kszie = kernel size , ksize = 3 => kernel is 3x3 matrix
        gradx = self.abs_sobel_thresh(img, orient='x', sobel_kernel=ksize, thresh=abs_sobel_thresh_x)
        grady = self.abs_sobel_thresh(img, orient='y', sobel_kernel=ksize, thresh=abs_sobel_thresh_y)

        # magnitude
        mag_binary = self.mag_thresh(img, sobel_kernel=ksize, mag_thresh=_mag_thresh)

        # direction (angle)
        dir_binary = self.dir_threshold(img, sobel_kernel=ksize, thresh=_dir_thresh)

        combined = np.zeros_like(dir_binary)
        combined[((gradx == 1) & (grady == 1)) | ((mag_binary == 1) & (dir_binary == 1))] = 1

        if is_test:
            return abs_sobel_thresh_x, abs_sobel_thresh_y, _mag_thresh, _dir_thresh, combined

        return combined

    def combine_threshold(self,c_bin, g_bin):
        combined_binary = np.zeros_like(g_bin)
        combined_binary[(c_bin == 1) | (g_bin == 1)] = 1

        return combined_binary

    def get_histogram(self,binary_warped):
        # half , from mid-height to bottom
        return np.sum(binary_warped[binary_warped.shape[0]//2:, :], axis = 0)

    def slide_window(self,binary_warped, histogram,
                 margin=50,
                 minpix=50,
                 nwindows = 9,
                 wait_key_val = 0,
                 reduced = False):
        out_img = np.dstack((binary_warped, binary_warped, binary_warped)) * 255

        # midpoint
        # midpoint = np.int(histogram.shape[0]/2)
        status, midpoint = self.find_midpoint(histogram)
        if not status:
            return False, False , False , False
        if not reduced:
            print('[INFO] midpoint = ', midpoint)

        # left boundary to the middle
        leftx_base = np.argmax(histogram[:midpoint])

        # right boundary to the middle
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint

        nwindows = 9
        window_height = np.int(binary_warped.shape[0]/nwindows)

        # indices of the elements that are non-zero  [x_array[...], y_array[...]]
        nonzero = binary_warped.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        leftx_current = leftx_base
        rightx_current = rightx_base

        left_lane_inds = []
        right_lane_inds = []

        center_windows = [] # [ (x1,y1), (x2,y2),...]

        for window in range(nwindows):
            win_y_high = binary_warped.shape[0] - (window) * window_height
            if window != nwindows-1:
                win_y_low = binary_warped.shape[0] - (window+1) * window_height
            else:
                win_y_low = 0

            win_xleft_low = leftx_current - margin
            win_xleft_high = leftx_current + margin

            win_xright_low = rightx_current - margin
            win_xright_high = rightx_current + margin

            cv2.rectangle(out_img, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high), (0,255,0), 2) # img, first_point, second_point, color, thickness
            cv2.rectangle(out_img, (win_xright_low, win_y_low), (win_xright_high, win_y_high), (0,255,0), 2) # img, first_point, second_point, color, thickness

            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                              (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0] # just get the axis x
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                              (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0] # just get the axis x

            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)

            if len(good_left_inds) > minpix:
                # shift the leftx_current to the mean of all good_left_inds in order to follow if the lane is curved
                leftx_current = np.int(np.mean(nonzerox[good_left_inds]))

            if len(good_right_inds) > minpix:
                # shift the rightx_current to the mean of all good_right_inds in order to follow if the lane is curved
                rightx_current = np.int(np.mean(nonzerox[good_right_inds]))

            if len(good_left_inds) > minpix and len(good_right_inds) > minpix:
                center_x = (np.mean(nonzerox[good_left_inds]) + np.mean(nonzerox[good_right_inds]))/2
                center_y = (np.mean(nonzeroy[good_left_inds]) + np.mean(nonzeroy[good_right_inds]))/2
            else:
                center_x = (rightx_current - leftx_current)/2 + leftx_current
                center_y = (win_y_low - win_y_high)/2 + win_y_high

            center_windows.append((center_x,center_y))

            if not reduced:
                out_img[nonzeroy[good_left_inds], nonzerox[good_left_inds]] = [0,0,255]
                out_img[nonzeroy[good_right_inds], nonzerox[good_right_inds]] = [0,0,255]
                out_img = self.add_points(out_img, [(center_x, center_y)], c=(255,255,0))

                if wait_key_val == 0 :
                    cv2.imshow('Sliding Window',out_img)
                    self.waitKey(wait_key_val)
                elif window == nwindows-1:
                    cv2.imshow('Sliding Window',out_img)
                    self.waitKey(wait_key_val)
                else:
                    pass

        lines_info = dict()

        if not reduced:
            lines_info['ploty'] = np.linspace(0,binary_warped.shape[0]-1, binary_warped.shape[0])
            try:
                left_fit = np.polyfit(lefty, leftx, 2)  # x = a y^2 + b y + c
                right_fit = np.polyfit(righty, rightx, 2) # y = a y^2 + b y + c

                # for plotting
                y = np.linspace(0, binary_warped.shape[0]-1, binary_warped.shape[0]) # start , stop, num

                a = left_fit[0]
                b = left_fit[1]
                c = left_fit[2]
                left_fitx = a * y ** 2 + b * y + c
                left_lane_inds = ((nonzerox > (a*(nonzeroy**2) + b*nonzeroy + c - margin)) &
                    (nonzerox < (a*(nonzeroy**2) + b*nonzeroy + c + margin))).nonzero()[0]

                a = right_fit[0]
                b = right_fit[1]
                c = right_fit[2]
                right_fitx = a * y ** 2 + b * y + c
                right_lane_inds = ((nonzerox > (a*(nonzeroy**2) + b*nonzeroy + c - margin)) &
                    (nonzerox < (a*(nonzeroy**2) + b*nonzeroy + c + margin))).nonzero()[0]

                # need fix here !
                lines_info['leftx'] = nonzerox[left_lane_inds]
                lines_info['lefty'] = nonzeroy[left_lane_inds]
                lines_info['rightx'] = nonzerox[right_lane_inds]
                lines_info['righty'] = nonzeroy[right_lane_inds]

                lines_info['left_fitx'] = left_fitx
                lines_info['right_fitx'] = right_fitx
                ret = True

            except:
                ret = False
        else:
            ret = False

        return True, center_windows, lines_info, ret

    def draw_lane_lines(self,original_image, warped_img, Minv, lines_info):
        leftx = lines_info['leftx']
        rightx = lines_info['rightx']
        left_fitx = lines_info['left_fitx']
        right_fitx = lines_info['right_fitx']
        ploty = lines_info['ploty']

        warp_zero = np.zeros_like(warped_img).astype(np.uint8)
        color_warp = np.dstack((warp_zero, warp_zero, warp_zero))

        pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
        pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
        pts = np.hstack((pts_left, pts_right))

        cv2.fillPoly(color_warp, np.int_([pts]), (0,255,0))

        newwarp = cv2.warpPerspective(color_warp, Minv, (original_image.shape[1], original_image.shape[0]))
        result = cv2.addWeighted(original_image, 1, newwarp, 0.3, 0)

        return result

    def find_midpoint(self,histogram, eps=5):

        hist = np.copy(histogram).reshape(-1)

        # find min
        i = 0
        while True:
            if hist[i] != 0:
                break
            elif i == hist.shape[0]-1:
                return False, 0
            else:
                i += 1
        _min = i

        # find max
        i = hist.shape[0]-1
        while True:
            if hist[i] != 0:
                break
            else:
                i -= 1
        _max = i

        def mean(hist, _range):
            count = 0.0
            s = 0.0
            for i in _range:
                s += i*hist[i]
                count += hist[i]
            return s/count

        # find best separated value
        T = _min + float(_max - _min)/2
        while True:
            m1 = mean(hist,range(0,int(T)))
            m2 = mean(hist,range(int(T),hist.shape[0]))
            T_new = (m1+m2)/2
            if abs(T - T_new) < eps:
                break
            else:
                T = T_new

        return True, int(T_new)



    # Helper Functions
    def add_points(self, img, pts, c=(255,0,0)):
        img = np.copy(img)
        for pt in pts:
            try:
                x = int(pt[0])
                y = int(pt[1])
                img = cv2.rectangle(img, (x-2,y-2), (x+2,y+2), c,2)
            except:
                pass
        return img

    def draw_line(self, img, pt1, pt2, c=(0,255,0)):
        try:
            x1 = int(pt1[0])
            y1 = int(pt1[1])
            x2 = int(pt2[0])
            y2 = int(pt2[1])
            return cv2.line(img, (x1,y1), (x2,y2),c,2)
        except:
            return img

    def waitKey(self, value = 0):
        if value == 0:
            while True:
                if cv2.waitKey(value) & 0xFF == ord('q'): # press 'q to quit'
                    break
        else:
            cv2.waitKey(value)

    def apply_hough(self,bi_img,
                    minLineLength = 100,
                    maxLineGap = 10):
        rs = np.copy(bi_img)
        lines = cv2.HoughLinesP(bi_img,1,np.pi/180,100,minLineLength, maxLineGap)
        if lines is not None:
            for x1,y1,x2,y2 in lines[0]:
                rs = cv2.line(rs,(x1,y1),(x2,y2),(0,255,0),5)
        return rs
