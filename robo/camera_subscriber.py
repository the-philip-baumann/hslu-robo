#!/usr/bin/python3

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
import numpy as np

from enum import Enum

class AktuelleAusrichtung(Enum):
    FAEHRT_NACH_LINKS = 0
    FAEHRT_NACH_RECHTS = 1
    GERADE_AUS = 2

class CameraSubscriber:

    def __init__(self, robot_name):
        # initialize a node with a name, annonymous=True ensures that the name is unique
        rospy.init_node('camera_listener', anonymous=True)

        self.image = None
        # subscribe to a topic of type CompressedImage  
        topic = '/' + robot_name + '/camera_node/image/compressed'
        # when a message is received, the callback is invoked
        rospy.Subscriber(topic, CompressedImage, self.callback)
        rospy.sleep(2.0)  # needed to make sure the node is indeed initialized

        # create a openCV <-> ROS bridge
        self.cv2_bridge = CvBridge()
        self.rate = rospy.Rate(10)  # the node is running at 10 hz

    def callback(self, data):
        # the callback should be light and fast
        rospy.loginfo("Received camera image of type: '%s'" % data.format)
        self.image = data

    def get_aktuelle_ausrichtung(self):

        SIGNIFIKANZ_FAKTOR = 1.2

        image_hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)

        yellow_lower_hsv = np.array([20, 100, 100])
        yellow_upper_hsv = np.array([30, 255, 255])

        height, width, channels = image_hsv.shape
        half = width // 2
        left_half = width // 2 - width // 3
        right_half = width // 2 + width // 3
        bottom_20_percent = height // 5 * 4

        right = image_hsv[bottom_20_percent:, half:right_half]
        left = image_hsv[bottom_20_percent:, left_half:half]

        mask_right = cv2.inRange(right, yellow_lower_hsv, yellow_upper_hsv)
        yellow_pixel_right = cv2.countNonZero(mask_right)

        mask_left = cv2.inRange(left, yellow_lower_hsv, yellow_upper_hsv)
        yellow_pixel_left = cv2.countNonZero(mask_left)

        if yellow_pixel_right * SIGNIFIKANZ_FAKTOR:
            return AktuelleAusrichtung.FAEHRT_NACH_LINKS
        if yellow_pixel_left * SIGNIFIKANZ_FAKTOR:
            return AktuelleAusrichtung.FAEHRT_NACH_RECHTS
        return AktuelleAusrichtung.GERADE_AUS

    def run(self):
        while not rospy.is_shutdown():
            self.do_image_processing()
            self.rate.sleep()


if __name__ == '__main__':
    robot_name = "delta"
    cs = CameraSubscriber(robot_name)
    cs.run()
