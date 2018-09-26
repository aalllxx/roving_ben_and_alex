#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from find_white_line import find_centroid
from cv_bridge import CvBridge, CvBridgeError

# main driver for followbot

def img_cb(msg):
    try:
        im = bridge.imgmsg_to_cv2(msg, "bgr8")
        global cx # center of mass horizontal
        global cy # center of mass verticle
        global sh # image shape
        sh = im.shape
        cx, cy = find_centroid(im)
    except CvBridgeError as e:
        print(e)

def shutdown():
    cmd_vel_pub.publish(Twist())

# initialize vars
sh = (1, 1)
cx = 1
cy = 1

# robot turns if center of mass is this far from center of image
turn_thresh = 70
# when making correction, robot turns until center of mass is within this margin of center of image
correct_thresh = 15

assert turn_thresh > correct_thresh

rospy.init_node('follow_line')
# Subscriber to camera
img_sub = rospy.Subscriber("captured_images", Image, img_cb)
# Publish to cmd_vel
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

# update at 30 hz
rate = rospy.Rate(30)

turning = False
bridge = CvBridge()
while not rospy.is_shutdown():
    t = Twist()

    mid = sh[1]/2
    if cx is not None:
        if not turning and turn_thresh < abs(cx-mid):
            turning = True
        elif turning and correct_thresh > abs(cx-mid):
            turning = False

        if turning:
            t.angular.z = 0.1 if cx < mid else -0.1
        elif cy > sh[0]*0.6: # when center of mass y is close to bottom of image
            t.linear.x = 0.05
        else: # when center of mass y is close to top of image (implies line is straight)
            t.linear.x = 0.1

        print('cx={0}, cy={1}, speed={2}'.format(cx, cy, t.linear.x))
    else:
        print('Cannot find line!')

    cmd_vel_pub.publish(t)
    rate.sleep()
    rospy.on_shutdown(shutdown) # stop driving on shutdown
