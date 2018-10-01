#!/usr/bin/env python

import sys
import rospy
import cv2
import time
import json
import os
from sensor_msgs.msg import CompressedImage, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

CALIB_DATA_FILE = "camera_specs.json"

full_path = os.path.realpath(__file__)
path, filename = os.path.split(full_path)

CALIB_DATA_FULLPATH = os.path.join(path,CALIB_DATA_FILE)

def snap_image():
    image = _take_photo()
    try:
        return bridge.cv2_to_compressed_imgmsg(image)
    except CvBridgeError as e:
        print(e)

def _take_photo():
    camera_port = 0
    camera = cv2.VideoCapture(camera_port)
    time.sleep(0.1)  # If you don't wait, the image will be dark
    return_value, image = camera.read()
    del(camera)  # so that others can use the camera as soon as possible
    return image

def _get_im_info_msg():
    with open(CALIB_DATA_FULLPATH, 'r') as f:
        calib_data = json.load(f)
    camera_info_msg = CameraInfo()
    camera_info_msg.width = calib_data["width"]
    camera_info_msg.height = calib_data["height"]
    camera_info_msg.K = calib_data["camera_matrix"]
    camera_info_msg.D = calib_data["distortion"]
    camera_info_msg.R = calib_data["rectification"]
    camera_info_msg.P = calib_data["projection"]
    camera_info_msg.distortion_model = calib_data["distortion_model"]
    return camera_info_msg

rospy.init_node('fiducials_camera')
bridge = CvBridge()
im_pub = rospy.Publisher("/camera/image/compressed", CompressedImage, queue_size=1)
im_info_pub = rospy.Publisher("/camera/camera_info", CameraInfo, queue_size=1)
im_img_info_msg = _get_im_info_msg()

rate = rospy.Rate(4)
while not rospy.is_shutdown():
    try:
        im_info_pub.publish(im_img_info_msg)
        im_pub.publish(snap_image())
    except TypeError as e:
        print (e)
    rate.sleep()
