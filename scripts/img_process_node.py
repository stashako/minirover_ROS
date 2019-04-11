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

roslib.load_manifest('mini_rover')


class image_processor:

    def img_callback(self, data):
        frame = self.bridge.imgmsg_to_cv2(data)
        self.process_image(frame)

    def __init__(self):

        rospy.init_node('image_processor', anonymous=True)

        #image stream selector
        is_playback = rospy.get_param("/rover_params/playback", False)
        
        # publisher
        self.image_pub = rospy.Publisher(
            "/rover_image/raw_image", Image, queue_size=1)
        self.imInfo_pub = rospy.Publisher(
            "/rover_image/imInfo", imInfo, queue_size=1)

        if is_playback:
            #subscriber
            img_string = rospy.get_param("rover_params/sub_img_str", "/camera/image_raw")
            self.image_sub = rospy.Subscriber(img_string, Image, self.img_callback, queue_size=1)
        else:
            # Define the capture object as pointing to the input file
            vinput = rospy.get_param("/rover_params/fcam_input", 1)
            self.capture = cv2.VideoCapture(vinput)

        fps = rospy.get_param("/rover_params/fps", 30)
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
            if not is_playback:
                # Get the next frame from the video
                ret, frame = self.capture.read()

                if ret:

                    # Convert the frame to ROS format, public the raw image
                    try:
                        self.image_pub.publish(
                            self.bridge.cv2_to_imgmsg(frame, encoding="bgr8"))
                    except CvBridgeError as e:
                        print(e)

                    self.process_image(frame)

            rate.sleep()

    def process_image(self, frame):                    
            if self.rotate:
                cv2.transpose(frame, frame)
                if self.rotate_cw:
                    cv2.flip(frame, 1, frame)
                else:
                    cv2.flip(frame, 0, frame)

                if self.flip_image:
                    cv2.flip(frame, self.flip_value, frame)

            frame, im = self.ld.process_image(frame)
            self.imInfo_pub.publish(im)
            if not np.isnan(im.x_midpt_frm_lines):
                rospy.loginfo("%d %d %d %d", int(im.x_im_midpt), int(
                    im.x_midpt_frm_lines), int(im.x_im_intercept_l), int(im.x_im_intercept_r))

            cv2.imshow("image window", frame)
            cv2.waitKey(3)


def main(args):
    try:
        image_processor()
    except KeyboardInterrupt:
        print('Shutting down')


if __name__ == '__main__':
    main(sys.argv)
