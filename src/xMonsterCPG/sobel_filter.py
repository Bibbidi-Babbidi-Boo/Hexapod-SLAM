#!/usr/bin/env python
import sys
import rospy
import cv2
from std_msgs.msg import String, Header
from sensor_msgs.msg import Image, PointCloud2, PointCloud
import sensor_msgs.point_cloud2 as pcl2
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32MultiArray
import numpy as np
import sensor_msgs.point_cloud2
import ros_numpy
from scipy import ndimage
import matplotlib.pyplot as plt
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Point32, Point
import tf
from math import *
import octomap_msgs.srv
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from geometry_msgs.msg import TransformStamped

global x, y, z, roll, pitch, yaw, pos, pointcloud, count, tf_stamp, translation, rotation, bridge
bridge = CvBridge()
translation = []
rotation = []
count = 0
pos = 0
pointcloud = PointCloud()
tf_stamp = TransformStamped()

rospy.init_node('sobel_filter', anonymous=True)
sobel_pub = rospy.Publisher("/sobel_filtered_data", PointCloud2, queue_size=1)

def calculate_slopes(msg):
    global bridge
    # image = bridge.imgmsg_to_cv2(data)

    # global count
    point = np.array(list(sensor_msgs.point_cloud2.read_points(msg, skip_nans=False, field_names = ("x", "y", "z"))))
    print(point.shape)
    # p = []
    # print(point.shape)
    # for i in range(point.shape[0]):
    #     if len(p) == 0:
    #         p.append([point[i][0], point[i][1], point[i][2]])
    #     else:
    #         c=0
    #         for j in range(len(p)):
    #             if p[j][0] == point[i][0] and p[j][1] == point[i][1]:
    #                 p[j][2] = (p[j][2]+point[i][2])/2
    #                 c+=1
    #                 break
    #         if c==0:
    #             p.append([point[i][0], point[i][1], point[i][2]])
    # p = np.array(p)
    # p_x = p[:,0]
    # p_y = p[:,1]
    # p_z = p[:,2]
    # p = np.reshape(p, (720, 1280, 3))
    # sx = ndimage.sobel(im, axis=0, mode='constant')
    # sy = ndimage.sobel(im, axis=1, mode='constant')
    # sob = np.hypot(sx, sy)
    # header = Header()
    # header.seq = count
    # header.stamp = rospy.Time.now()
    # header.frame_id = "map" ## Must be changed to map frame later
    # point_cloud_map = pcl2.create_cloud_xyz32(header, p)
    # sobel_pub.publish(point_cloud_map)
    # count +=1

def main():
    rospy.Subscriber("/octomap_point_cloud_centers", PointCloud2, calculate_slopes, queue_size=1)
    rospy.spin()

if __name__=="__main__":
    main()
