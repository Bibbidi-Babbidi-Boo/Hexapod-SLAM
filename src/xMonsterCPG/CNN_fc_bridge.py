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
import time
import os

import numpy as np

import torch
import torch.nn as nn
import torchvision.models as models
from torchvision import transforms
from torch.autograd import Variable
from PIL import Image as Imagepil
import torchvision
import argparse
import torch.nn.functional as F
# Device configuration

os.environ["CUDA_VISIBLE_DEVICES"] = "0"
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
class CNN:

    def __init__(self, model_path):


        self.model = torch.load(model_path).to(device)
        self.model.eval()
        self.transform = transforms.Compose([
            transforms.ToPILImage(),
            transforms.Resize((224, 224)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[.5, .5, .5], std=[.5, .5, .5])
        ])


    def detect(self, image):

        image = self.transform(image)
        image = image.to(device)
        image = image.unsqueeze(0)
        outputs = self.model(image)
        prob = F.softmax(outputs, dim=1)
        pred = torch.argmax(prob, dim=1).cpu()
        pred = pred.numpy()
        return pred[0]

model = CNN('/home/marmot/catkin_ws/src/Yuna-IMU-CPG-python/src/xMonsterCPG/model/CNN_door_new-20.pth')

def top_callback(data):
    global bridge
    global img_num
    global count

    try:
        # rate = rospy.Rate(25.0)
        dir_pub = rospy.Publisher('further_direction', String, queue_size=1)
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        features = model.detect(cv_image)
        #print(features)
	#print('okay')
 	#cv2.imwrite("Door_new/Door_right/"+str(count)+".jpg",cv_image)
	
        cv2.imshow("Image window", cv_image)

        cv2.waitKey(40)

        dir_pub.publish(str(features))
	count+=1
        
        # rate.sleep()
        # rospy.loginfo(str(next_direction))

    except CvBridgeError as e:
        print(e)


def front_callback(data):
    global bridge
    global img_num
    

    try:
        # rate = rospy.Rate(25.0)
        #dir_pub = rospy.Publisher('front_view', String, queue_size=1)
        cv_image_front = bridge.imgmsg_to_cv2(data, "bgr8")
        
 	#cv2.imwrite("Door_new/Door_right/"+str(count)+".jpg",cv_image)
	
        cv2.imshow("Front_view", cv_image_front)

        cv2.waitKey(40)

        #dir_pub.publish(str(features))
	# rate.sleep()
        # rospy.loginfo(str(next_direction))

    except CvBridgeError as e:
        print(e)


def sendinfo():
    global count
    count=1

    while not rospy.is_shutdown():

        rospy.init_node('image_processor', anonymous=True)
        global bridge

        bridge = CvBridge()

        rospy.Subscriber("/camera/color/image_raw", Image, top_callback, queue_size=1)
	rospy.Subscriber("/left_r200/camera/color/image_raw", Image, front_callback, queue_size=1)	
	
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")
        cv2.destroyAllWindows()


if __name__ == '__main__':


    sendinfo()
