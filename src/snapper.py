#!/usr/bin/env python

import sys
import rospy
import cv2
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def snap_image():
    image = _take_photo()
    try:
        return bridge.cv2_to_imgmsg(image, "bgr8")
    except CvBridgeError as e:
        print(e)

def _take_photo():
    camera_port = 0
    camera = cv2.VideoCapture(camera_port)
    time.sleep(0.1)  # If you don't wait, the image will be dark
    return_value, image = camera.read()
    del(camera)  # so that others can use the camera as soon as possible
    return image

rospy.init_node('fiducials_camera')
bridge = CvBridge()
pub = rospy.Publisher("camera/image_raw", Image, queue_size=1)
rate = rospy.Rate(4)
while not rospy.is_shutdown():
    try:
        pub.publish(snap_image())
    except TypeError as e:
        print (e)
    rate.sleep()
