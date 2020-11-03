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
from scipy import ndimage

global x, y, z, roll, pitch, yaw, pos, pointcloud, count, tf_stamp, translation, rotation
translation = []
rotation = []
cnt = 0
pos = 0
pointcloud = PointCloud()
tf_stamp = TransformStamped()

rospy.init_node('depth_map_pointcloud', anonymous=True)
e_map = rospy.Publisher("/elevation_map_pointcloud", PointCloud2, queue_size=1)

def odom_callback(data):
    global x, y, z, roll, pitch, yaw, pos
    if pos == 0:
        for i in range (len(data.name)):
            if data.name[i] == "robot":
                pos = i
                break
    a = data.pose[pos].orientation.x
    b = data.pose[pos].orientation.y
    c = data.pose[pos].orientation.z
    d = data.pose[pos].orientation.w
    l = [a, b, c, d]
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(l)

    x = data.pose[pos].position.x
    y = data.pose[pos].position.y
    z = data.pose[pos].position.z



# /octomap_server/pointcloud_max_z
# /octomap_server/pointcloud_min_z


    # rospy.set_param('~/octomap_server/occupancy_min_z', z)
    # rospy.set_param('~/octomap_server/occupancy_max_z', z+0.1)
    # rospy.set_param('~/octomap_server/pointcloud_min_z', z)
    # rospy.set_param('~/octomap_server/pointcloud_max_z', z+0.1)

    # R2 = np.array([[cos(yaw), -sin(yaw), 0], [sin(yaw), cos(yaw), 0], [0, 0, 1]])
    # far_points = np.array([[x+1.5, y-5, z], [x+5, y+5, z+5], [x-5, y-5, z], [x-1.5, y+5, z+5], [x-10, y+1.5, z], [x+10, y+10, z+5], [x-10, y-10, z], [x+10, y-1.5, z+5]])
    # far_points = np.transpose(np.matmul(R2, np.transpose(far_points)))
    # service = rospy.ServiceProxy('/octomap_server/clear_bbx', octomap_msgs.srv.BoundingBoxQuery)
    # min_map = Point()
    # max_map = Point()
    # #
    # min_map.x = far_points[0][0]
    # min_map.y = far_points[0][1]
    # min_map.z = far_points[0][2]
    # max_map.x = far_points[1][0]
    # max_map.y = far_points[1][1]
    # max_map.z = far_points[1][2]
    # service(min_map, max_map)
    # #
    # min_map.x = far_points[2][0]
    # min_map.y = far_points[2][1]
    # min_map.z = far_points[2][2]
    # max_map.x = far_points[3][0]
    # max_map.y = far_points[3][1]
    # max_map.z = far_points[3][2]
    # service(min_map, max_map)
    #
    # min_map.x = far_points[4][0]
    # min_map.y = far_points[4][1]
    # min_map.z = far_points[4][2]
    # max_map.x = far_points[5][0]
    # max_map.y = far_points[5][1]
    # max_map.z = far_points[5][2]
    # service(min_map, max_map)
    #
    # min_map.x = far_points[6][0]
    # min_map.y = far_points[6][1]
    # min_map.z = far_points[6][2]
    # max_map.x = far_points[7][0]
    # max_map.y = far_points[7][1]
    # max_map.z = far_points[7][2]
    # service(min_map, max_map)
    # print("PUBLISHED")

def calculate_slopes(msg):
    global bridge, roll, pitch, yaw, x, y, z, cnt
    point = np.array(list(sensor_msgs.point_cloud2.read_points(msg, skip_nans=True, field_names = ("x", "y", "z"))))
    p = []
    height = []
    for i in range(point.shape[0]):
        if abs(point[i][0]*cos(yaw)+point[i][1]*sin(yaw)-x) <= 1.5 and abs(-point[i][0]*sin(yaw)+point[i][1]*cos(yaw)-y) <= 1.5:
            if [point[i][0], point[i][1]] in p:
                pos = p.index([point[i][0], point[i][1]])
                height[pos] = (height[pos] + point[i][2])/2
            else:
                p.append([point[i][0], point[i][1]])
                height.append(point[i][2])
    p, height = zip(*sorted(zip(p, height)))
    p = list(p)
    points = list(p)
    height = list(height)
    ordered_points = []
    max_len = 0
    pos = 0
    size_init = []
    while p!= []:
        temp_min = p[0][0]
        temp = []
        while p!= []:
            if p[0][0] == temp_min:
                temp.append(height[0])
                p.pop(0)
                height.pop(0)
            else:
                break
        ordered_points.append(temp)
        if len(temp)>max_len:
            max_len = len(temp)
            pos = len(ordered_points)-1
    boundaries = max(max(x) if isinstance(x, list) else x for x in ordered_points)
    for i in range(len(ordered_points)):
        temp = ordered_points[i]
        size_init.append(len(temp))
        while len(temp)<max_len:
            temp.append(temp[len(temp)-1])
        ordered_points[i] = temp
    ordered_points = np.transpose(np.array(ordered_points))
    sx = ndimage.sobel(ordered_points, axis=0, mode='reflect')
    sy = ndimage.sobel(ordered_points, axis=1, mode='reflect')
    sob = np.transpose(np.hypot(sx, sy))
    cell = np.array([[0, 1, 0], [1, 1, 1], [0, 1, 0]])
    temp = sob*255/np.max(sob)*2
    temp = ndimage.grey_erosion(temp, structure=cell)
    temp = ndimage.grey_dilation(temp, structure=cell)
    temp = ndimage.median_filter(temp,3)
    sob = temp*np.max(sob)/255
    sob = np.array(sob)
    print(np.ndarray.max(sob))
    count=0
    for i in range(sob.shape[0]):
        temp = sob[i]
        index=0
        while index<size_init[i]:
            new = points[count]
            new.append(temp[index])
            points[count] = new
            count +=1
            index +=1
    points = np.array(points)
    print(points.shape)
    points = np.reshape(points, (-1, 3))
    print(points.shape)
    header = Header()
    header.seq = cnt
    header.stamp = rospy.Time.now()
    header.frame_id = "map" ## Must be changed to map frame later
    point_cloud_map = pcl2.create_cloud_xyz32(header, points)
    e_map.publish(point_cloud_map)
    print("PUBLISHED")

    # print(sob.shape, len(size_init))
    # for i in range(len(size_init)):
    #     for j in range(len(sob[0])):


    # print(sob.shape, ordered_points.shape)
    # print(ordered_points.shape)
    print("______________________")
    # for i in range(len(p)):
    #     print("P", p[i][1]/100)
        # if len(p) == 0:
            # p.append([point[i][0], point[i][1], point[i][2]])
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
    cnt+=1


def main():
    rospy.Subscriber("/gazebo/model_states", ModelStates, odom_callback, queue_size=1)
    rospy.Subscriber("/octomap_point_cloud_centers", PointCloud2, calculate_slopes, queue_size=1)
    rospy.spin()

if __name__=="__main__":
    main()
