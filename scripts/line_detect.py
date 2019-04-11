#!/usr/bin/env python
from __future__ import print_function
import roslib
import sys
import rospy
import cv2
import numpy as np
import math
from std_msgs.msg import String
from sensor_msgs.msg import Image
from mini_rover.msg import imInfo
from cv_bridge import CvBridge, CvBridgeError
import matplotlib.pyplot as plt
#%matplotlib inline


class LineDetect:

    # int for invalid
    invalid_val = 9999

    kernel_sharpen_1 = np.array([[-1, -1, -1], [-1, 9, -1], [-1, -1, -1]])
    kernel_sharpen_2 = np.array([[1, 1, 1], [1, -7, 1], [1, 1, 1]])
    kernel_sharpen_3 = np.array([[-1, -1, -1, -1, -1],
                                 [-1, 2, 2, 2, -1],
                                 [-1, 2, 8, 2, -1],
                                 [-1, 2, 2, 2, -1],
                                 [-1, -1, -1, -1, -1]]) / 8.0

    kernel = np.ones((5,5),np.uint8)
    
    line_height = 75

    # parameters for low-pass filter
    lpf_beta = 0.5
    pre_est_point = np.nan

    clahe = cv2.createCLAHE(clipLimit=1.0, tileGridSize=(8,8))

    def process_image(self, image):
        cv2.imshow("image window11", image)
        cv2.waitKey(3)

        # sharpen the image
        image = cv2.filter2D(image, -1, self.kernel_sharpen_3)

        # increase the brightness of the image
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)  # convert it to hsv
        #hsv[:, :, 2] += 20
        #image_x = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        gray = hsv[:,:,2]
        #equ = cv2.equalizeHist(hsv)
        #gray = self.clahe.apply(gray)
        #plt.imshow(gray)
        #plt.show()
        cv2.imshow("image window01", gray)
        cv2.waitKey(3)
        #return
    # grayscale the image
    	#gray = self.grayscale(image_x)

    # apply gaussian blur
    	#gray = self.morpho(gray)
        gray = cv2.morphologyEx(gray, cv2.MORPH_OPEN, self.kernel)
    # apply sobelLine
    	#gray = self.sobelLine(gray) 

        cv2.imshow("image window1", gray)
        cv2.waitKey(3)
    # return opening

    # using canny edge detector
        low_threshold = 50
        high_threshold = 150
        edges = self.canny(gray, low_threshold, high_threshold)

    # use fillPoly to mask out the region of interest
        iShape = image.shape
        im = imInfo()
        im.x_im_midpt = iShape[1] / 2
        xoffs = 70
        yoffs = 40
        ybottom = iShape[0] / 2 + yoffs
        vertices = np.array([[(iShape[1] / 2 + xoffs, ybottom), (iShape[1] / 2 - xoffs, ybottom),
                              (5, self.line_height+yoffs), (5, 5), (iShape[1]-5, 5), (iShape[1]-5, self.line_height+yoffs)]], dtype=np.int32)
        edges = self.region_of_interest(edges, vertices)

        # cv2.imshow("image window2", edges)
        # cv2.waitKey(3)
    # return edges

    # hough transform
        rho = 2
        theta = np.pi / 180
        threshold = 15
        min_line_len = 50
        max_line_gap = 30
        lines = cv2.HoughLinesP(edges, rho, theta, threshold, np.array(
            []), min_line_len, max_line_gap)

        if lines is None:
            im.x_midpt_frm_lines = self.invalid_val
            im.x_im_intercept_l = self.invalid_val
            im.x_im_intercept_r = self.invalid_val
            return (image, im)

    # find the slopes of left and right lines of the bounding box (region of
    # interest)
        pf = np.polyfit((0, iShape[1] / 2 - xoffs),
                        (self.line_height, ybottom), 1)
        nf = np.polyfit((iShape[1], iShape[1] / 2 + xoffs),
                        (self.line_height, ybottom), 1)
        # print("pf={}".format(pf[0]))
        # print("nf={}".format(nf[0]))

        # allowable margin for slops to be included
        r = 0.5

    # Go through all the detected lines, filter those that are have slopes within r of the left or right lines
    # of the bounding box. This is to exclude, as much as possible, horizontal
    # lines.
        pslopx = []
        pslopy = []
        nslopx = []
        nslopy = []
        # slops = []
        # slops.append(pf[0])
        # slops.append(nf[0])
        m1 = np.zeros_like(image)
        m2 = np.zeros_like(image)
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(m2, (x1, y1), (x2, y2), [255, 0, 0], 2)
                if x2 - x1 != 0:
                    slop = (y2 - y1) / float(x2 - x1)
                    # print("slop={}".format(slop))
                    # slops.append(slop)
                    if (slop > 0 and slop >= pf[0] - r and slop <= pf[0] + r):
                        pslopx.extend((x1, x2))
                        pslopy.extend((y1, y2))
                        cv2.line(m1, (x1, y1), (x2, y2), [0, 255, 0], 2)
                        # print("slop={}, pf={}".format(slop,pf[0]))
                    elif (slop < 0 and slop >= nf[0] - r and slop <= nf[0] + r):
                        nslopx.extend((x1, x2))
                        nslopy.extend((y1, y2))
                        cv2.line(m1, (x1, y1), (x2, y2), [0, 0, 255], 2)
                        # print("slop={}, nf={}".format(slop,nf[0]))

        cv2.line(m2, (0, self.line_height+yoffs),
                 (iShape[1] / 2 - xoffs, ybottom), [255, 255, 0], 2)
        cv2.line(m2, (iShape[1], self.line_height+yoffs), (iShape[
                 1] / 2 + xoffs, ybottom), [255, 0, 255], 2)
        cv2.imshow("image window3", m2)
        cv2.waitKey(3)

        cv2.imshow("image window4", m1)
        #cv2.resizeWindow("image window4", 100,100)
        cv2.waitKey(3)
    # return m1

        mask = np.zeros_like(image)

    # draw the bounding box left and right lines
    # cv2.line(mask, (vSertices[0,1,0], vertices[0,1,1]), (vertices[0,2,0], vertices[0,2,1]), [0,255, 0], 2)
    # cv2.line(mask, (vertices[0,5,0], vertices[0,5,1]), (vertices[0,0,0], vertices[0,0,1]), [0,255, 0], 2)

    # find the line fits for both pos and neg slopes and draw the estimated
    # lines in the pic.
        if pslopx and len(pslopx) >= 2:
            m_ps, c_ps = np.polyfit(pslopx, pslopy, 1)
            # print('m_ps = {}'.format(m_ps))
            # print('c_ps = {}'.format(c_ps))
            p = np.poly1d([m_ps, c_ps])
            y_lt = p(0)
            y_lb = p(iShape[1] / 2 - xoffs)
            cv2.line(mask, (0, int(y_lt)),
                     (iShape[1] / 2 - xoffs, int(y_lb)), [0, 255, 0], 6)
        else:
            y_lt = np.nan

        if nslopx and len(nslopx) >= 2:
            m_ns, c_ns = np.polyfit(nslopx, nslopy, 1)
            # print('m_ns = {}'.format(m_ns))
            # print('c_ns = {}'.format(c_ns))
            n = np.poly1d([m_ns, c_ns])
            y_rt = n(iShape[1])
            y_rb = n(iShape[1] / 2 + xoffs)
            cv2.line(mask, (iShape[1], int(y_rt)), (iShape[
                     1] / 2 + xoffs, int(y_rb)), [0, 255, 0], 6)
        else:
            y_rt = np.nan

        # compute the mid-point between y_lt and y_rt
        if not np.isnan(y_lt) and not np.isnan(y_rt) and y_rt != y_lt:
            im.x_im_intercept_l = (0 - c_ps) / m_ps
            im.x_im_intercept_r = (0 - c_ns) / m_ns
            cur_est_point = (im.x_im_intercept_r -
                             im.x_im_intercept_l) / 2 + im.x_im_intercept_l
            # if cur_est_point < 0:
            # print("p={} {} {}".format(im.x_im_intercept_r,im.x_im_intercept_l,cur_est_point))
            if np.isnan(self.pre_est_point):
                im.x_midpt_frm_lines = cur_est_point
            else:
                im.x_midpt_frm_lines = self.lpf_beta * cur_est_point + \
                    (1 - self.lpf_beta) * self.pre_est_point

            self.pre_est_point = cur_est_point
            cv2.circle(mask, (int(im.x_midpt_frm_lines),
                              im.x_im_midpt), 5, (0, 0, 255), 6)
        else:
            im.x_midpt_frm_lines = self.invalid_val
            if not np.isnan(y_lt):
                im.x_im_intercept_l = (0 - c_ps) / m_ps
                im.x_im_intercept_r = self.invalid_val
            elif not np.isnan(y_rt):
                im.x_im_intercept_l = self.invalid_val
                im.x_im_intercept_r = (0 - c_ns) / m_ns
            else:
                im.x_im_intercept_l = self.invalid_val
                im.x_im_intercept_r = self.invalid_val

        result = self.weighted_img(mask, image)

        return (result, im)

    def sobelLine(self, img, xy_k=3, md_k=15, xy_th=(20, 100), m_th=(30, 100), d_th=(0.7, 1.3)):
    	# x and y directions
    	abs_x = np.absolute(cv2.Sobel(img, cv2.CV_64F, 1, 0))
    	abs_y = np.absolute(cv2.Sobel(img, cv2.CV_64F, 0, 1))
    	scaled_x = np.uint8(255 * abs_x / np.max(abs_x))
    	scaled_y = np.uint8(255 * abs_y / np.max(abs_y))

    	# xy_bin = np.zeros_like(scaled_y)
    	# xy_bin[(scaled_x >= yx_th[0])&(scaled_x <= yx_th[1])&(scaled_y >= yx_th[0])&(scaled_y <= yx_th[1])] = 1

    	#direction and magnetude
    	sobelx = cv2.Sobel(img, cv2.CV_64F, 1, 0, ksize=md_k)
    	sobely = cv2.Sobel(img, cv2.CV_64F, 0, 1, ksize=md_k)

    	# Calculate the gradient magnitude
        gradmag = np.sqrt(sobelx**2 + sobely**2)
    	scale_factor = np.max(gradmag) / 255
    	gradmag = (gradmag / scale_factor).astype(np.uint8)

    	# direction
    	absgraddir = np.arctan2(np.absolute(sobely), np.absolute(sobelx))

    	# md_bin = np.zeros_like(sobelx)
    	# md_bin[(gradmag >= m_th[0])&(gradmag <= m_th[1])&(absgraddir >= d_th[0])&(absgraddir <= d_th[1])] = 1

    	binOut = np.zeros_like(scaled_y)
    	binOut[(scaled_x >= xy_th[0])&(scaled_x <= xy_th[1])&
    		   (scaled_y >= xy_th[0])&(scaled_y <= xy_th[1])&
    		   (gradmag >= m_th[0])&(gradmag <= m_th[1])&
    		   (absgraddir >= d_th[0])&(absgraddir <= d_th[1])] = 1
    	return np.uint8(binOut * 255)

    def morpho(self, img):
    	gray = self.gaussian_blur(img, 3)
        ret, thresh = cv2.threshold(
            gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        kernel = np.ones((9, 9), np.uint8)
        opening = cv2.morphologyEx(
            thresh, cv2.MORPH_CLOSE, kernel, iterations=2)
        return opening

    def grayscale(self, img):
        return cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

    def canny(self, img, low_threshold, high_threshold):
        return cv2.Canny(img, low_threshold, high_threshold)

    def gaussian_blur(self, img, kernel_size):
        return cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)

    def region_of_interest(self, img, vertices):
        # defining a blank mask to start with
        mask = np.zeros_like(img)

        # defining a 3 channel or 1 channel color to fill the mask with
        # depending on the input image
        if len(img.shape) > 2:
            channel_count = img.shape[2]  # i.e. 3 or 4 depending on your image
            ignore_mask_color = (255,) * channel_count
        else:
            ignore_mask_color = 255

        # filling pixels inside the polygon defined by "vertices" with the fill
        # color
        cv2.fillPoly(mask, vertices, ignore_mask_color)

        # returning the image only where mask pixels are nonzero
        masked_image = cv2.bitwise_and(img, mask)
        return masked_image

    def draw_lines(self, img, lines, color=[255, 0, 0], thickness=2):
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(img, (x1, y1), (x2, y2), color, thickness)

    def hough_lines(self, img, rho, theta, threshold, min_line_len, max_line_gap):
        lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array(
            []), minLineLength=min_line_len, maxLineGap=max_line_gap)
        line_img = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
        self.draw_lines(line_img, lines)
        return line_img

# Python 3 has support for cool math symbols.

    def weighted_img(self, img, initial_img, alpha=0.8, beta=1., lamda=0.):
        return cv2.addWeighted(initial_img, alpha, img, beta, lamda)
