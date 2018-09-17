#!/usr/bin/env python

import sys
import rospy
import cv2
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def display_im(msg):
    try:
        im = bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow('f',im)
        cv2.waitKey(1)
    except CvBridgeError as e:
        print(e)


rospy.init_node('displayer')
sub = rospy.Subscriber("captured_images", Image ,display_im)
bridge = CvBridge()
rospy.spin()
