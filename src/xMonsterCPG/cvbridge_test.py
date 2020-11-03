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
c_max=None
cnts=None

def callback(data):
    global bridge
    global img_num
    global c_max
    global cnts
    try:
        #rate = rospy.Rate(25.0)

        dir_pub = rospy.Publisher('further_direction', String, queue_size = 1)
        cnts_pub = rospy.Publisher('nearest_block', String, queue_size = 1)
        #cnts_pub = rospy.Publisher('nearest_block', Float32MultiArray, queue_size = 1)
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        # print(cv_image.shape)

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lowerb_hsv = np.array([0, 43, 46])  # range of red color in hsv
        upperb_hsv = np.array([10, 255, 255])
        lowerg_hsv = np.array([35, 43, 46])  # range of green color in hsv
        upperg_hsv = np.array([77, 255, 255])
        mask1 = cv2.inRange(hsv, lowerb_hsv, upperb_hsv)  # build mask
        mask2 = cv2.inRange(hsv, lowerg_hsv, upperg_hsv)
        # use findcontour to get the object outline
        # print(cv2.__version__)
        r, cnts, h = cv2.findContours(mask1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE, )

        #print(cnts)
        #print(str(len(cnts)))

        #image_array=np.array(cv_image,dtype=np.float32)
        #print(str(image_array))
        #print(str(image_array.shape))

        # do the next step if the contour more than 0
        if len(cnts) > 0:
            # find the contour which area is biggest
            c = max(cnts, key=cv2.contourArea)
            # use the minimum circle out the object
            ((x, y), radius) = cv2.minEnclosingCircle(c)

            # compute the moment of the object
            M = cv2.moments(c)
            # compute the center of the gravity
            if int(M["m00"]) > 0:
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                direction = (int(M["m10"] / M["m00"]) - 320, int(M["m01"] / M["m00"]) - 240)

                cv2.circle(cv_image, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                # cv2.circle(cv_image, center, 5, (0, 0, 255), -1)

                z=str(y)+" "+str(x)
                #print(z)
                #c_max1=c.reshape(-1)
                #cnts_pub.publish(Float32MultiArray(data=c_max1))
                #print('Area '+str(cv2.contourArea(c)))
                cnts_pub.publish(z)
                cv2.drawContours(cv_image,c,-1,(0,255,0),6)



                if abs(direction[0]) <= 120:
                    # print('go straight')
                    next_direction = 0
                    object_dir = 0

                elif direction[0] > 120:
                    # print('turn right')
                    next_direction = 1
                    object_dir = 1
                elif direction[0] < -120:
                    # print('turn left')
                    next_direction = -1
                    object_dir = 2
                # print(width)
                else:
                    # print('turn around')
                    next_direction = 2
                    object_dir = 3

        else:
            #print('no object')
            next_direction = 2
            object_dir = 3
        #print('okay3')
        cv2.namedWindow("Image window",cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Image window', 580, 480)
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(40)


    #     cv2.imwrite("Image_dataset_door/"+str(img_num)+".jpg",cv_image)
    #     print(img_num)
    #     # if object_dir==3:
    #     img_num = img_num + 1
    #     image_name = "./data/noobject/"+str(img_num) + '_' + str(object_dir) + ".jpg"
    #     cv2.imwrite(image_name, cv_image)

        # dir_pub.publish(str(next_direction))
        dir_pub.publish(str('0'))
        #rate.sleep()

    except CvBridgeError as e:
        print(e)

    # try:
    # self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    # except CvBridgeError as e:
    # print(e)



def sendinfo():
    global img_num
    img_num = 0
    rospy.init_node('image_processor', anonymous=True)

    while not rospy.is_shutdown():
        import torch
        global bridge
        bridge = CvBridge()

        rospy.Subscriber("/realsense_r200/camera/color/image_raw", Image, callback, queue_size=1)
        # rospy.Subscriber("/camera/color/image_raw", Image, callback, queue_size=10)



        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")
        cv2.destroyAllWindows()


if __name__ == '__main__':
    sendinfo()
