#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import matplotlib.pyplot as plt


roslib.load_manifest('mini_rover')


class dual_cam_grab:

    def __init__(self):

        rospy.init_node('dual_cam_grab', anonymous=True)

        # publisher
        self.fcam_pub = rospy.Publisher(
            "/fcam/raw_image", Image, queue_size=1)
        self.bcam_pub = rospy.Publisher(
            "/bcam/raw_image", Image, queue_size=1)

        # Define the capture object as pointing to the input file
        vinput = rospy.get_param("/rover_params/fcam_input", 2)
        self.fcam_capture = cv2.VideoCapture(vinput)
        vinput = rospy.get_param("/rover_params/bcam_input", 1)
        self.bcam_capture = cv2.VideoCapture(vinput)
        
        fps = rospy.get_param("/rover_params/fps", 60)

        self.show_img = rospy.get_param("/rover_params/show_img", False)

        rate = rospy.Rate(fps) #Hz
        self.bridge = CvBridge()

        # Enter the main processing loop
        while not rospy.is_shutdown():
            ######### Front Camera ################ 
            ret, frame = self.fcam_capture.read()

            if ret:

                #Process image
                frame = cv2.transpose(frame)
                frame = cv2.flip(frame,-1)

                # Convert the frame to ROS format, public the raw image
                try:
                    self.fcam_pub.publish(
                        self.bridge.cv2_to_imgmsg(frame, encoding="bgr8"))
                except CvBridgeError as e:
                    print(e)
                
                if self.show_img:
                    cv2.imshow("Front Camera", frame)
                    cv2.waitKey(1)

            ######### Back Camera ################ 
            ret, frame = self.bcam_capture.read()

            if ret:

                #Process image 
                frame = cv2.transpose(frame)
                frame = cv2.flip(frame,-1)

                # Convert the frame to ROS format, public the raw image
                try:
                    self.bcam_pub.publish(
                        self.bridge.cv2_to_imgmsg(frame, encoding="bgr8"))
                except CvBridgeError as e:
                    print(e)
                
                if self.show_img:
                    cv2.imshow("Back Camera", frame)
                    cv2.waitKey(1)

            rate.sleep()

    def close_event(self):
        plt.close()


def main(args):
    try:
        dual_cam_grab()
    except KeyboardInterrupt:
        print('Shutting down')


if __name__ == '__main__':
    main(sys.argv)
