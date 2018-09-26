#!/usr/bin/env python

import sys
import rospy
import cv2
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from find_white_line import _mask_white

# util to display image messages as filtered binary images

def display_im(msg):
    try:
        im = bridge.imgmsg_to_cv2(msg, "bgr8")
        im = _mask_white(im) if len(sys.argv) > 2 and sys.argv[2] == "bw" else im
        cv2.imshow('f',cv2.resize(im,  (0, 0), fx=3, fy=3))
        cv2.waitKey(1)
    except CvBridgeError as e:
        print(e)



rospy.init_node('displayer')
sub = rospy.Subscriber("captured_images", Image ,display_im)
bridge = CvBridge()
rospy.spin()
