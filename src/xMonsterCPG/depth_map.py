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

global x, y, z, roll, pitch, yaw, pos, pointcloud, count, tf_stamp, translation, rotation
translation = []
rotation = []
count = 0
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


def point_cloud_to_position(msg):
    global x, y, z, roll, pitch, yaw, pointcloud, count, tf_stamp, translation, rotation
    # try:
    #     # now = rospy.Time.now()
    #     # tf.TransformListener().waitForTransform('/odom', '/realsense_d435_camera_depth_optical_frame', rospy.Time(0), rospy.Duration(1.0))
    #     translation, rotation = tf.TransformListener().lookupTransform('/base_link', '/realsense_d435_camera_depth_optical_frame', rospy.Time(0))
    #     print("GOT")
    # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    #     print("EXPE")
    #     pass
    # # print(translation, rotation)
    # tf_stamp.transform.translation.x = translation[0]
    # tf_stamp.transform.translation.y = translation[1]
    # tf_stamp.transform.translation.z = translation[2]
    # tf_stamp.transform.rotation.x = rotation[0]
    # tf_stamp.transform.rotation.y = rotation[1]
    # tf_stamp.transform.rotation.z = rotation[2]
    # tf_stamp.transform.rotation.w = rotation[3]
    # tf_stamp.header.frame_id = '/base_link'
    # tf_stamp.header.stamp = rospy.Time.now()
    # tf_stamp.child_frame_id = '/realsense_d435_camera_depth_optical_frame'
    # # print(tf_stamp)
    # c = 0
    # pt_x = []
    # pt_y = []
    # pt_z = []
    p = np.transpose(np.array(list(sensor_msgs.point_cloud2.read_points(msg, skip_nans=False, field_names = ("x", "y", "z")))))
    # p = do_transform_cloud(msg, tf_stamp)
    # p[:, [0, 1, 2]] = p[:, [2, 0, 1]]
    # p[:,1] = -p[:,1]
    # p[:,2] = -p[:,2]
    # print(p.shape)

    # for point in sensor_msgs.point_cloud2.read_points(msg, skip_nans=False):
    #     c+=1
    #     if isnan(point[2]):
    #         pt_x.append(3.0)
    #     else:
    #         pt_x.append(point[2])
    #     if isnan(point[0]):
    #         pt_y.append(3.0)
    #     else:
    #         pt_y.append(-point[0])
    #     if isnan(point[1]):
    #         if len(pt_z)!=0:
    #             pt_z.append(pt_z[len(pt_z)-1])
    #         else:
    #             pt_z.append(0)
    #     else:
    #         pt_z.append(-point[1])
    ## Camera to body tf
    # pt_x = np.reshape(np.array(pt_x), (1, len(pt_x)))
    # pt_y = np.reshape(np.array(pt_y), (1, len(pt_y)))
    # pt_z = np.reshape(np.array(pt_z), (1, len(pt_z)))
    # p = np.concatenate((pt_x, pt_y, pt_z), axis = 0)
    # print(p.shape)
    # R_cam = np.array([[cos(0.3), 0, sin(0.3)], [0, 1, 0], [-sin(0.3), 0, cos(0.3)]])
    # p = np.matmul(p, np.transpose(R_cam))
    # p[:,0] += 0.1
    # p[:,2] += 0.3
    rows = 720
    cols = 1080
    # # ## Body frame to world frame
    # R = np.array([[cos(yaw)*cos(pitch), cos(yaw)*sin(pitch)*sin(roll)-sin(yaw)*cos(roll), cos(yaw)*sin(pitch)*cos(roll)+sin(yaw)*sin(roll)],
    #              [sin(yaw)*cos(pitch), sin(yaw)*sin(pitch)*sin(roll)+cos(yaw)*cos(roll), sin(yaw)*sin(pitch)*cos(roll)-cos(yaw)*sin(roll)],
    #              [-sin(pitch), cos(pitch)*sin(roll), cos(pitch)*cos(roll)]])
    # p = np.matmul(p, np.transpose(R))
    # p[:,0] += x+1
    # p[:,1] += y-2
    # p[:,2] += z
    # print(p.shape)
    # p = np.reshape(p, ())
    pt_x = np.array(p[0])
    pt_y = np.array(p[1])
    pt_z = np.array(p[2])
    pt_x = np.reshape(pt_x, (rows, cols))
    pt_y = np.reshape(pt_y, (rows, cols)) # rows, cols for morph. transform
    pt_z = np.reshape(pt_z, (rows, cols))
    #
    pt_x = ndimage.median_filter(pt_x, 3)
    pt_y = ndimage.median_filter(pt_y, 3)
    pt_z = ndimage.median_filter(pt_z, 3)
    #
    # cell = np.array([[0, 0, 1, 0, 0], [0, 1, 1, 1, 0], [1, 1, 1, 1, 1], [0, 1, 1, 1, 0], [0, 0, 1, 0 ,0]])
    # # temp = pt_x*255/np.max(pt_x)
    # # temp = ndimage.grey_erosion(temp, structure=cell)
    # # temp = ndimage.grey_dilation(temp, structure=cell)
    # # pt_x = temp*np.max(pt_x)/255
    # #
    # # temp = pt_y*255/np.max(pt_y)
    # # temp = ndimage.grey_erosion(temp, structure=cell)
    # # temp = ndimage.grey_dilation(temp, structure=cell)
    # # pt_y = temp*np.max(pt_y)/255
    # # temp = pt_z*255/np.max(pt_z)
    # # temp = ndimage.grey_erosion(temp, structure=cell)
    # # temp = ndimage.grey_dilation(temp, structure=cell)
    # # pt_z = temp*np.max(pt_z)/255
    # # pt_x = np.reshape(pt_x, (1, rows*cols))
    # # pt_y = np.reshape(pt_y, (1, rows*cols))
    # # pt_z = np.reshape(pt_z, (1, rows*cols))
    # # p = np.transpose(np.concatenate((pt_x, pt_y, pt_z), axis = 0))
    p = np.transpose(p)
    header = Header()
    header.seq = count
    header.stamp = rospy.Time.now()
    header.frame_id = "realsense_d435_camera_depth_optical_frame" ## Must be changed to map frame later
    point_cloud_map = pcl2.create_cloud_xyz32(header, p)
    e_map.publish(point_cloud_map)

    # R2 = np.array([[cos(yaw), -sin(yaw), 0], [sin(yaw), cos(yaw), 0], [0, 0, 1]])
    # far_points = np.array([[10+x, 10+y, 10], [10+x, -10+y, 10], [-10+x, 10+y, 10], [-10+x, -10+y, 10], [x+2.5*cos(yaw)+2.5*sin(yaw), y+2.5*sin(yaw)-2.5*cos(yaw), 0], [x+2.5*cos(yaw)-2.5*sin(yaw), y+2.5*sin(yaw)+2.5*cos(yaw), 0], [x-2.5*cos(yaw)-2.5*sin(yaw), y-2.5*sin(yaw)+2.5*cos(yaw), 0], [x-2.5*cos(yaw)-2.5*sin(yaw), y-2.5*sin(yaw)+2.5*cos(yaw), 0]])
    # far_points = np.transpose(np.matmul(R2, np.transpose(far_points)))
    # service = rospy.ServiceProxy('/octomap_server/clear_bbx', octomap_msgs.srv.BoundingBoxQuery)
    # min_map = Point()
    # max_map = Point()
    # min_map.x = far_points[5][0]
    # min_map.y = far_points[5][1]
    # min_map.z = far_points[5][2]
    # max_map.x = far_points[1][0]
    # max_map.y = far_points[1][1]
    # max_map.z = far_points[1][2]
    # service(min_map, max_map)
    # #
    # min_map.x = far_points[5][0]
    # min_map.y = far_points[5][1]
    # min_map.z = far_points[5][2]
    # max_map.x = far_points[3][0]
    # max_map.y = far_points[3][1]
    # max_map.z = far_points[3][2]
    # service(min_map, max_map)
    #
    # min_map.x = far_points[4][0]
    # min_map.y = far_points[4][1]
    # min_map.z = far_points[4][2]
    # max_map.x = far_points[0][0]
    # max_map.y = far_points[0][1]
    # max_map.z = far_points[0][2]
    # service(min_map, max_map)
    #
    # min_map.x = far_points[4][0]
    # min_map.y = far_points[4][1]
    # min_map.z = far_points[4][2]
    # max_map.x = far_points[0][0]
    # max_map.y = far_points[0][1]
    # max_map.z = far_points[0][2]
    # service(min_map, max_map)
    #
    # min_map.x = far_points[4][0]
    # min_map.y = far_points[4][1]
    # min_map.z = far_points[4][2]
    # max_map.x = far_points[2][0]
    # max_map.y = far_points[2][1]
    # max_map.z = far_points[2][2]
    # service(min_map, max_map)
    #
    # min_map.x = far_points[6][0]
    # min_map.y = far_points[6][1]
    # min_map.z = far_points[6][2]
    # max_map.x = far_points[2][0]
    # max_map.y = far_points[2][1]
    # max_map.z = far_points[2][2]
    # service(min_map, max_map)
    #
    # min_map.x = far_points[6][0]
    # min_map.y = far_points[6][1]
    # min_map.z = far_points[6][2]
    # max_map.x = far_points[0][0]
    # max_map.y = far_points[0][1]
    # max_map.z = far_points[0][2]
    # service(min_map, max_map)
    #
    # min_map.x = far_points[7][0]
    # min_map.y = far_points[7][1]
    # min_map.z = far_points[7][2]
    # max_map.x = far_points[3][0]
    # max_map.y = far_points[3][1]
    # max_map.z = far_points[3][2]
    # service(min_map, max_map)
    #
    # min_map.x = far_points[7][0]
    # min_map.y = far_points[7][1]
    # min_map.z = far_points[7][2]
    # max_map.x = far_points[1][0]
    # max_map.y = far_points[1][1]
    # max_map.z = far_points[1][2]
    # service(min_map, max_map)
    print("PUBLISHED", count)

    count +=1

def main():
    tf.TransformListener().waitForTransform('/odom', '/realsense_d435_camera_depth_optical_frame', rospy.Time(), rospy.Duration(10.0))
    # rospy.Subscriber("/gazebo/model_states", ModelStates, odom_callback, queue_size=1)
    rospy.Subscriber("/realsense_d435/depth/points", PointCloud2, point_cloud_to_position, queue_size=1)
    rospy.spin()

if __name__=="__main__":
    main()
