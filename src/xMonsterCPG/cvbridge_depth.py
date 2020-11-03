#!/usr/bin/env python
from __future__ import print_function

import roslib

roslib.load_manifest('xMonsterCPG')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32MultiArray
import numpy as np
import time
c_new=None

def depth_callback(data):
    global bridge
    global c_new
    #from cvbridge_test import cmax
    try:
        coord_pub = rospy.Publisher('coordinates_pub', Float32MultiArray, queue_size = 1)
        img = bridge.imgmsg_to_cv2(data, "passthrough")
        # print(img.shape)
        depth_array=np.array(img,dtype=np.float32)
        #print(depth_array)
        #print(str(depth_array.shape))
    
        if c_new is not None:
            #print('okay2')
            #cv2.drawContours(img,c_max.astype(int),-1,(0,255,0),6)
            #c_max1=c_max.reshape(-1,2).astype(int)
            #for row in c_max1:
                #arr.append(depth_array[row[0]][row[1]])
                #print(row)
            #coord_pub.publish(arr)   
            cv2.circle(img, (int(c_new[1]), int(c_new[0])), 300, (255,255,255), 10)
            z=depth_array[int(c_new[0])][int(c_new[1])]
            c_new.append(z)
            # print(c_new)
            coord_pub.publish(Float32MultiArray(data=c_new))

            P = np.array([[695.9951171875, 0.0, 640.0, 0.0], [0.0, 695.9951171875, 360.0, 0.0], [0.0, 0.0, 1.0, 0.0]])
            P_inv = np.linalg.pinv(P)
            point_2d = np.array([[int(c_new[0])*z], [int(c_new[1])*z], [z]])
            # print('c0', int(c_new[0]))
            # print('c0*z',int(c_new[0])*z)
            point_3d = np.dot(P_inv,point_2d)
            print(point_3d)
            c_new = None

        
        cv2.namedWindow("Depth window",cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Depth window', 580, 480)
        cv2.imshow("Depth window", img)
        cv2.waitKey(40)

        
    except CvBridgeError as e:
        print(e)
    

def callback(data):
    #global c_max
    #c_max=np.fromString(data.data,dtype=float32)
    #c_max=np.array(data.data)
    #c_max=c_max.reshape(-1,2)
    #c_max[:,0]*=0.666
    #c_max[:,1]*=0.666
    #c_max=np.rint(c_max)
    #c_max=c_max.reshape(-1,1,2)
    global c_new
    c_coord=data.data.split()
    #print('okay')
    c_new=[float(c_coord[0])*0.66,float(c_coord[1])*0.66]
    	

def sendinfo():
    rospy.init_node('depth_processor', anonymous=True)
    
    while not rospy.is_shutdown():
        
        global bridge
        bridge = CvBridge()
        rospy.Subscriber("/nearest_block",String, callback, queue_size=10)
        rospy.Subscriber("/camera/depth/image_raw", Image, depth_callback, queue_size=10)


        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")
        cv2.destroyAllWindows()


if __name__ == '__main__':
    
    sendinfo()

