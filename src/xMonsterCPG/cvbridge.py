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
import numpy as np

class image_processor:
 
  def __init__(self):
    #self.dir = rospy.Publisher('further_direction',String)
    
    self.bridge = CvBridge()
    
    self.image_sub = rospy.Subscriber("/right_r200/camera/color/image_raw", Image, self.callback)
    

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

      hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
      lowerb_hsv = np.array([0, 43, 46])  # range of red color in hsv
      upperb_hsv = np.array([10, 255, 255])
      lowerg_hsv = np.array([35, 43, 46])  # range of green color in hsv
      upperg_hsv = np.array([77, 255, 255])
      mask1 = cv2.inRange(hsv, lowerb_hsv, upperb_hsv)  # build mask
      mask2 = cv2.inRange(hsv, lowerg_hsv, upperg_hsv)
      # use findcounter to get the object outline
      # print(cv2.__version__)
      r, cnts, h = cv2.findContours(mask1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE, )

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

          rate = rospy.Rate(10.0)

          if abs(direction[0]) <= 120:
            print('go straight')
            rospy.loginfo("hello")
            self.dir.publish("Straight")

          elif direction[0] > 120:
            print('turn right')
            rospy.loginfo("hello")
            self.dir.publish("Right")
          elif direction[0] < -120:
            print('turn left')
            rospy.loginfo("hello")
            self.dir.publish("Left")
          # print(width)
          else:
            print('turn around')
            rospy.loginfo("hello")
            self.dir.publish("Turn")
          rate.sleep()

      else:
        rospy.loginfo("hello")
        self.dir.publish("Turn around")
        print('no object')

      cv2.imshow("Image window", cv_image)
      cv2.waitKey(0)
    except CvBridgeError as e:
      print(e)

    #try:
     # self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    #except CvBridgeError as e:
     # print(e)

def main(args):
  ic = image_processor()
  rospy.init_node('image_processor', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

