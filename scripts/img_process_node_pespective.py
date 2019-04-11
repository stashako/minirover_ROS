#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from mini_rover.msg import imInfo
from cv_bridge import CvBridge, CvBridgeError
from line_detect import LineDetect
import matplotlib.pyplot as plt

from detect_peaks import detect_peaks

roslib.load_manifest('mini_rover')


class image_processor:

    def img_callback(self, data):
        frame = self.bridge.imgmsg_to_cv2(data)
        self.process_image(frame)

    def __init__(self):

        rospy.init_node('image_processor', anonymous=True)

        #image stream selector
        self.is_image_provided = rospy.get_param("/rover_params/image_provided", False)
        
        #matrix for perspective transformation
        #pts1=np.float32([[10,331],[630,331],[187,262],[453,262]])
        #pts2=np.float32([[100,400],[540,400],[100,0],[540,0]])
        
        #based on checker box calib
        pts1=np.float32([[130,435],[520,426],[454,353],[202,356]])
        pts2=np.float32([[270,430],[370,430],[370,330],[270,330]])
        self.matrix = cv2.getPerspectiveTransform(pts1, pts2)

        # publisher
        self.imInfo_pub = rospy.Publisher(
            "/rover_image/imInfo", imInfo, queue_size=1)

        if self.is_image_provided:
            #subscriber
            img_string = rospy.get_param("rover_params/sub_img_str", "/fcam/raw_image")
            self.image_sub = rospy.Subscriber(img_string, Image, self.img_callback, queue_size=1)
        else:
            self.image_pub = rospy.Publisher(
            "/rover_image/raw_image", Image, queue_size=1)
            # Define the capture object as pointing to the input file
            vinput = rospy.get_param("/rover_params/fcam_input", 1)
            self.capture = cv2.VideoCapture(vinput)

        fps = rospy.get_param("/rover_params/fps", 60)
        flip_horizontal = rospy.get_param(
            "/rover_params/flip_horizontal", False)
        flip_vertical = rospy.get_param("/rover_params/flip_vertical", True)
        self.rotate = rospy.get_param("/rover_params/rotate", False)
        self.rotate_cw = rospy.get_param("/rover_params/rotate_cw", False)

        self.flip_image = True
        self.flip_value = 0
        if flip_horizontal and flip_vertical:
            self.flip_value = 0
        elif flip_horizontal:
            self.flip_value = 1
        elif flip_vertical:
            self.flip_value = -1
        else:
            self.flip_image = False

        rate = rospy.Rate(fps)
        self.bridge = CvBridge()
        self.ld = LineDetect()

        # Enter the main processing loop
        while not rospy.is_shutdown():
            if not self.is_image_provided:
                # Get the next frame from the video
                ret, frame = self.capture.read()

                if ret:
                    if self.rotate:
                        cv2.transpose(frame, frame)
                        if self.rotate_cw:
                            cv2.flip(frame, 1, frame)
                        else:
                            cv2.flip(frame, 0, frame)

                        if self.flip_image:
                            cv2.flip(frame, self.flip_value, frame)
                    # Convert the frame to ROS format, public the raw image
                    try:
                        self.image_pub.publish(
                            self.bridge.cv2_to_imgmsg(frame, encoding="bgr8"))
                    except CvBridgeError as e:
                        print(e)

                    #cv2.imshow("image window1", frame)
                    #cv2.waitKey(3)
                    self.process_image(frame)

            rate.sleep()

    def close_event(self):
        plt.close()

    def process_image(self, frame):                    
            
            cv2.imshow("image window1", frame)
            cv2.waitKey(3)
            #Perspective transformation
            #im_siz=np.array(frame.shape[1::-1]).flatten()
            #print frame.shape[1::-1], frame.shape
            #print self.matrix.shape
            #print np.array([[0, 0, 0]]).shape
            #print cv2.perspectiveTransform(np.array([[[0], [0], [0]]], dtype=np.float32),self.matrix)
            frame = cv2.warpPerspective(frame,self.matrix,frame.shape[1::-1])

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            gray = hsv[:,:,2]
            #print gray.shape 
            not_gray_inx = gray == 0
            gray[not_gray_inx]=255
            #print not_gray.shape
            #start and end number
            cv2.imshow("image window0001", frame)
            cv2.waitKey(3)
            
            zone = 1
            se = gray.shape[0]//zone
            se_c = se//2
            histograms = np.zeros((zone,gray.shape[1]),dtype='int')
            midpt=0
            for i in range(zone):
                histograms[i,:] = np.sum(gray[i*se:(i+1)*se,:], axis=0)
                x1 = detect_peaks(histograms[i,:], valley=True, mpd=gray.shape[1])
                #x2 = detect_peaks(histograms[i,:], mpd=gray.shape[1]/2)

                if x1.size == 1:
                    cv2.circle(frame, (x1[0],(i*se+se_c)),10,(0,0,255))
                    midpt = x1[0]
                else:
                    for j in range(x1.size):
                        cv2.circle(frame, (x1[j],(i*se+se_c)),10,(0,0,255))

                #if x2.size:
                #    for k in range(x2.size):
                #        #print x2[k], (i*se+se_c), frame.shape
                #        cv2.circle(frame, (x2[k],(i*se+se_c)),10,(0,255,0))
                
                #fig = plt.figure()
                #timer = fig.canvas.new_timer(interval = 500)
                #timer.add_callback(self.close_event)
                #plt.plot(histograms[i,:])
                #timer.start()
                #plt.plot(x1, histograms[i,:][x1], '+', mfc=None, mec='r', mew=2, ms=8)
                #plt.show()

 

            #print x1.size, x2.size
            pic_midpts = frame.shape[1]//2 - 10 
            cv2.circle(frame, (pic_midpts,frame.shape[0]//2),10,(255,0,0))
            cv2.imshow("image window001", frame)
            cv2.waitKey(3)
            im = imInfo()
            #im.x_midpt_frm_lines = midpt
            #im.x_im_midpt = frame.shape[1]//2
            im.x_midpt_frm_lines = pic_midpts
            im.x_im_midpt = midpt 

            self.imInfo_pub.publish(im)
            if not np.isnan(im.x_midpt_frm_lines):
                rospy.loginfo("%d %d %d %d", int(im.x_im_midpt), int(
                    im.x_midpt_frm_lines), int(im.x_im_intercept_l), int(im.x_im_intercept_r))
            #return im
            
            #frame, im = self.ld.process_image(frame)
            #self.imInfo_pub.publish(im)
            #if not np.isnan(im.x_midpt_frm_lines):
            #    rospy.loginfo("%d %d %d %d", int(im.x_im_midpt), int(
            #        im.x_midpt_frm_lines), int(im.x_im_intercept_l), int(im.x_im_intercept_r))#

            #cv2.imshow("image window", frame)
            #cv2.waitKey(3)


def main(args):
    try:
        image_processor()
    except KeyboardInterrupt:
        print('Shutting down')


if __name__ == '__main__':
    main(sys.argv)
