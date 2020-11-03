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


import numpy as np

import torch
import torch.nn as nn
import torchvision.models as models
import torchvision.transforms as transforms
from torch.autograd import Variable
from PIL import Image as Imagepil
from sklearn.decomposition import PCA


def get_vector(image_name):
    # 1. Load the image with Pillow library
    img = Imagepil.fromarray(image_name)    # 2. Create a PyTorch Variable with the transformed image
    t_img = Variable(normalize(to_tensor(scaler(img))).unsqueeze(0))    # 3. Create a vector of zeros that will hold our feature vector
    #    The 'avgpool' layer has an output size of 512
    my_embedding = torch.zeros(512)    # 4. Define a function that will copy the output of a layer
    def copy_data(m, i, o):
        my_embedding.copy_(o.data.squeeze())    # 5. Attach that function to our selected layer
    h = layer.register_forward_hook(copy_data)    # 6. Run the model on our transformed image
    model(t_img)    # 7. Detach our copy function from the layer
    h.remove()
    new_v = my_embedding.numpy()
    new_v = np.reshape(new_v, (32, 16))
    pca = PCA(n_components=1)
    new_v = pca.fit_transform(new_v)
    new_v = np.squeeze(new_v)
    return new_v

def top_callback(data):
    global bridge
    global img_num
    global count

    try:
        # rate = rospy.Rate(25.0)
        dir_pub = rospy.Publisher('further_direction', String, queue_size=1)
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        features = get_vector(cv_image).tolist()
        features = " ".join([str(x) for x in features])



	
        cv2.imshow("Image window", cv_image)

        cv2.waitKey(40)

        dir_pub.publish(str(features))
	count+=1
        print('processing')
        # print(features)
        # rate.sleep()
        # rospy.loginfo(str(next_direction))

    except CvBridgeError as e:
        print(e)


def front_callback(data):
    global bridge
    global img_num
    

    try:
        # rate = rospy.Rate(25.0)
        dir_pub = rospy.Publisher('front_view', String, queue_size=1)
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        
 	#cv2.imwrite("Door_new/Door_right/"+str(count)+".jpg",cv_image)
	
        cv2.imshow("Front_view", cv_image)

        cv2.waitKey(40)

        #dir_pub.publish(str(features))
	# rate.sleep()
        # rospy.loginfo(str(next_direction))

    except CvBridgeError as e:
        print(e)



def sendinfo():
    global count
    count=0

    while not rospy.is_shutdown():

        rospy.init_node('image_processor', anonymous=True)
        global bridge
	

        bridge = CvBridge()
	print(count)
        rospy.Subscriber("/right_r200/camera/color/image_raw", Image, top_callback, queue_size=1)
	rospy.Subscriber("/left_r200/camera/color/image_raw", Image, front_callback, queue_size=1)
	
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")
        cv2.destroyAllWindows()


if __name__ == '__main__':

    model = models.resnet18(pretrained=True)  # Use the model object to select the desired layer
    layer = model._modules.get('avgpool')

    # Set model to evaluation mode
    model.eval()

    scaler = transforms.Resize((224, 224))
    normalize = transforms.Normalize(mean=[0.485, 0.456, 0.406],
                                     std=[0.229, 0.224, 0.225])
    to_tensor = transforms.ToTensor()
    sendinfo()

