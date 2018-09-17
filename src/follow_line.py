#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from find_white_line import find_centroid

def img_cb(msg):
    try:
        im = bridge.imgmsg_to_cv2(msg, "bgr8")
        global cent
        cent = find_centroid(im)
    except CvBridgeError as e:
        print(e)

# Centroid x pos
cent = 170
turn_thresh = 40
correct_thresh = 5

assert turn_thresh > correct_thresh


# Subscriber to camera
img_sub = rospy.Subscriber("captured_images", Image, img_cb)
# Publish to cmd_vel
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

turning = False
while not rospy.is_shutdown():
    t = Twist()

    if not turning and turn_thresh < abs(cent-170):
        turning = True
    elif turning and correct_thresh < abs(cent-170):
        turning = False

    if turning:
        t.angular.z = 0.1 if cent < 170 else -0.1
    else:
        t.linear.x = 0.2


    cmd_vel_pub.publish(t)
