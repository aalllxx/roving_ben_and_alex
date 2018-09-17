#!/usr/bin/env python

import cv2
import os
import numpy as np

def _load_image():
    image = cv2.imread('/home/alexander-feldman/image.jpeg')
    # print image.shape
    # cv2.imshow('f',image)
    # cv2.waitKey(0)
    return image

def _mask_white(image):
    lower = np.array([100,100,200],dtype='uint8')
    upper = np.array([255,255,255],dtype='uint8')
    mask = cv2.inRange(image,lower,upper)
    # cv2.imshow('f',mask)
    # cv2.waitKey(0)
    return mask

def find_centroid(image):
    if image is None: # for development
        image = _load_image()
    mask = _mask_white(image)
    vector = mask[180:240,:]
    M = cv2.moments(vector)
    if M['m00'] > 0:
        cx = int(M['m10']/M['m00'])
        # cy = int(M['m01']/M['m00'])
        print("centroid is {}, im width is {}".format(cx,image.shape[1]))
        return cx
    return None

find_centroid(None)
