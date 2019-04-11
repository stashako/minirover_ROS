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
from cv_bridge import CvBridge, CvBridgeError


class image_converter:

    def __init__(self, out1, out2):
        self.image_pub = rospy.Publisher("image_topic_2", Image, queue_size=10)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            "/webcam/image_raw", Image, self.callback)
        self.out1 = out1
        self.out2 = out2

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        cv2.imshow("image window", self.process_image(cv_image))
        cv2.waitKey(3)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)

    def process_image(self, image):
        self.out1.write(image)
        cv2.imshow("image window11", image)
        cv2.waitKey(3)
    # grayscale the image
        gray = self.grayscale(image)

    # apply gaussian blur
        gray = self.gaussian_blur(gray, 5)
        ret, thresh = cv2.threshold(
            gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        kernel = np.ones((3, 3), np.uint8)
        opening = cv2.morphologyEx(
            thresh, cv2.MORPH_OPEN, kernel, iterations=2)
        gray = opening

        cv2.imshow("image window1", gray)
        cv2.waitKey(3)
    # return opening

    # using canny edge detector
        low_threshold = 50
        high_threshold = 150
        edges = self.canny(gray, low_threshold, high_threshold)

    # use fillPoly to mask out the region of interest
        iShape = image.shape
        xoffs = 70
        yoffs = 0
        ybottom = iShape[0] / 2 + yoffs
        vertices = np.array([[(iShape[1] / 2 + xoffs, ybottom), (iShape[1] / 2 - xoffs,
              ybottom), (0, 200), (0, 0), (iShape[1], 0), (iShape[1], 200)]], dtype=np.int32)
        edges = self.region_of_interest(edges, vertices)

        cv2.imshow("image window2", edges)
        cv2.waitKey(3)
    # return edges

    # hough transform
        rho = 2
        theta = np.pi / 180
        threshold = 15
        min_line_len = 50
        max_line_gap = 30
        lines = cv2.HoughLinesP(edges, rho, theta, threshold, np.array(
            []), min_line_len, max_line_gap)

    # find the slopes of left and right lines of the bounding box (region of
    # interest)
        pf = np.polyfit((0, iShape[1] / 2 - xoffs), (200, ybottom), 1)
        nf = np.polyfit((iShape[1], iShape[1] / 2 + xoffs), (200, ybottom), 1)
        print(pf)
        print(nf)
        r = 0.4

    # Go through all the detected lines, filter those that are have slopes within r of the left or right lines
    # of the bounding box. This is to exclude, as much as possible, horizontal
    # lines.
        pslopx = []
        pslopy = []
        nslopx = []
        nslopy = []
        m1 = np.zeros_like(image)
        m2 = np.zeros_like(image)
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(m2, (x1, y1), (x2, y2), [255, 0, 0], 2)
                slop = (y2 - y1) / (x2 - x1)
                if (slop > 0 and slop >= pf[0] - r and slop <= pf[0] + r):
                    pslopx.extend((x1, x2))
                    pslopy.extend((y1, y2))
                    cv2.line(m1, (x1, y1), (x2, y2), [0, 255, 0], 2)
                elif (slop < 0 and slop >= nf[0] - r and slop <= nf[0] + r):
                    nslopx.extend((x1, x2))
                    nslopy.extend((y1, y2))
                    cv2.line(m1, (x1, y1), (x2, y2), [0, 0, 255], 2)

        cv2.line(m2, (0, 200), (iShape[1] / 2 -
                                xoffs, ybottom), [255, 255, 0], 2)
        cv2.line(m2, (iShape[1], 300), (iShape[1] /
                                        2 + xoffs, ybottom), [255, 0, 255], 5)
        cv2.imshow("image window3", m2)
        cv2.waitKey(3)

        cv2.imshow("image window4", m1)
        cv2.waitKey(3)
    # return m1

        mask = np.zeros_like(image)

    # draw the bounding box left and right lines
    # cv2.line(mask, (vertices[0,1,0], vertices[0,1,1]), (vertices[0,2,0], vertices[0,2,1]), [0,255, 0], 2)
    # cv2.line(mask, (vertices[0,5,0], vertices[0,5,1]), (vertices[0,0,0], vertices[0,0,1]), [0,255, 0], 2)

    # find the line fits for both pos and neg slopes and draw the estimated
    # lines in the pic.
        if pslopx and len(pslopx) >= 2:
            p = np.poly1d(np.polyfit(pslopx, pslopy, 1))
            y_lt = p(0)
            y_lb = p(iShape[1] / 2 - xoffs)
            cv2.line(mask, (0, int(y_lt)),
                     (iShape[1] / 2 - xoffs, int(y_lb)), [0, 255, 0], 6)

        if nslopx and len(nslopx) >= 2:
            n = np.poly1d(np.polyfit(nslopx, nslopy, 1))
            y_rt = n(iShape[1])
            y_rb = n(iShape[1] / 2 + xoffs)
            cv2.line(mask, (iShape[1], int(y_rt)), (iShape[
                     1] / 2 + xoffs, int(y_rb)), [0, 255, 0], 6)

        result = self.weighted_img(mask, image)
        self.out2.write(result)

        return result

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


def main(args):
    out1 = cv2.VideoWriter(
        'output1.avi', cv2.cv.CV_FOURCC(*'XVID'), 20.0, (640, 480))
    out2 = cv2.VideoWriter(
        'output2.avi', cv2.cv.CV_FOURCC(*'XVID'), 20.0, (640, 480))
    cap = cv2.VideoCapture(0)
    num = 1

    while(cap.isOpened()):
        print("here")
        ret, frame = cap.read()

        print("here1")
        if ret is True:
            print("here2")
            frame = cv2.flip(frame, 0)
            out1.write(frame)
            cv2.imshow('frame', frame)
            num = num + 1
            if num == 100:
                break

    image_converter(out1, out2)
    rospy.init_node('processImage', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    out1.release()
    out2.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
