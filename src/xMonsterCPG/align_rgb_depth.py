#! /usr/bin/python2
import sys
import rospy
import cv2
from std_msgs.msg import String, Header
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32MultiArray
import numpy as np
import time

global bridge, i, h, w, c
i = 0
h = 1080
w = 1920
bridge = CvBridge()
header = Header()

rospy.init_node('align_rgb_depth', anonymous=True)
pub_align = rospy.Publisher("/realsense_d435/camera/depth/image_aligned", Image, queue_size=1)

def callback(data):
    global bridge, h, w, c, i
    i+=1
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    h, w, c = cv_image.shape

def reshape(data):
    global bridge, h, w, header
    header = data.header
    image = bridge.imgmsg_to_cv2(data)
    bigger = cv2.resize(image, (w, h))
    image_message = bridge.cv2_to_imgmsg(bigger)
    image_message.header = header
    pub_align.publish(image_message)


def align_rgb_depth():
    global bridge, h, w, c, i
    if i==0:
        rospy.Subscriber("/realsense_d435/camera/color/image_raw", Image, callback, queue_size=1)
    rospy.Subscriber("/realsense_d435/camera/depth/image_raw", Image, reshape, queue_size=1)
    rospy.spin()

if __name__=="__main__":
    align_rgb_depth()
