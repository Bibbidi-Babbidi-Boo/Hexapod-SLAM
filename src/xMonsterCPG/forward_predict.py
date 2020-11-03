#!/usr/bin/env python  
import rospy

# Because of transformations
import tf_conversions
import tf2_ros
import geometry_msgs.msg
from gazebo_msgs.msg import ModelStates
 
import numpy as np
 
import copy
 

import rospkg

from std_msgs.msg import String
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from visualization_msgs.msg import Marker
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_matrix
from tf.transformations import inverse_matrix
import time





x = np.zeros((5,1))
xImu = np.zeros((5,1))
covariance = np.identity(5)*.1

R = np.identity(5)*.1
Q = np.identity(5)*.1

start = True

p = np.ones((5,5))*.1

dt = 1/100.0
br = tf2_ros.TransformBroadcaster()

marker_publisher = rospy.Publisher('visualization_marker', Marker)


marker = Marker()
marker.header.frame_id = "/world"
marker.type = marker.CYLINDER
marker.action = marker.ADD
marker.scale.x = 0.1
marker.scale.y = 0.1
marker.scale.z = 0.0001  
marker.color.a = 1.0
marker.color.r = 1.0

marker.pose.orientation.w = 1.0
marker.pose.position.x = 0
marker.pose.position.y = 0
marker.pose.position.z = 0
yaw = 0.0
C = np.zeros(3)
control_state = -1
initialX = None
initialY = None


def control_state_callback(data):
    global control_state 
    control_state = data.data

    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.transform.translation.x = x[0]
    t.transform.translation.y = x[1]
    t.transform.translation.z = 0.0
    q = tf_conversions.transformations.quaternion_from_euler(0,0,x[4])
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    t.header.frame_id = "world"
    t.child_frame_id = "robot"

    marker.pose.position.x = x[0]
    marker.pose.position.y = x[1]
    marker.scale.x = covariance[0,0]
    marker.scale.y = covariance[1,1]


    br.sendTransform(t)
    # print(yaw)
    
    

    marker_publisher.publish(marker)


def measurement_update(data):
    global x 
    global dt
    global yaw
    global covariance
    global control_state 

    xOrb = data.pose.position.x
    yOrb = data.pose.position.y

    orientation_list = [data.pose.orientation.x,
                        data.pose.orientation.y,
                        data.pose.orientation.z,
                        data.pose.orientation.w]
                            
    Orbroll, Orbpitch, Orbyaw = euler_from_quaternion(orientation_list)


    z = np.array([[xOrb,yOrb,0,0,Orbyaw]]).T

    C = np.array([[1,0,0,0,0],
         [0,1,0,0,0],
         [0,0,0,0,0],
         [0,0,0,0,0],
         [0,0,0,0,1]])

    S = np.linalg.inv(np.dot(np.dot(C,covariance),C.T) + R)
    K = np.dot(np.dot(covariance,C.T),S)

    x = x + np.dot(K,z-np.dot(C,x))
    covariance = np.dot((np.identity(5)-np.dot(K,C)),covariance)




def gtPose_callback(data):
    global yaw
    global C
    global control_state 
    global initialX
    global initialY

    if control_state < .5:
        return 42


 
    positions = data.pose
    pose = positions[1]
    orientation_list = [pose.orientation.x,
                        pose.orientation.y,
                        pose.orientation.z,
                        pose.orientation.w]
                            
    roll, pitch, yaw = euler_from_quaternion(orientation_list)
    C = inverse_matrix(quaternion_matrix(orientation_list))[0:3,0:3]

    tGT = geometry_msgs.msg.TransformStamped()
    tGT.header.stamp = rospy.Time.now()

    if initialX == None:
        initialX = pose.position.x
        initialY = pose.position.y


    tGT.transform.translation.x = pose.position.x - initialX
    tGT.transform.translation.y = pose.position.y - initialY
    tGT.transform.translation.z = 0.0#pose.position.z

    tGT.transform.rotation.x = orientation_list[0]
    tGT.transform.rotation.y = orientation_list[1]
    tGT.transform.rotation.z = orientation_list[2]
    tGT.transform.rotation.w = orientation_list[3]

    tGT.header.frame_id = "world"
    tGT.child_frame_id = "ground"

    br.sendTransform(tGT)





def IMU_Callback(data):
    global x 
    global dt
    global yaw
    global covariance
    global control_state 
    global xImu

    if control_state < .5:
        return 42


       


    
    f = data.linear_acceleration
    fvec = np.array([f.x,f.y,f.z])
    if np.amax(np.abs(fvec)) > .1:

        fNew = np.dot(C,fvec.T)
        

        w = data.angular_velocity
        wvec = np.array([w.x,w.y,w.z])
        wNew = np.dot(C,wvec.T)

        u = np.array([[fNew[0],fNew[1],fNew[2], wNew[0],wNew[1],wNew[2]]]).T

        A = np.array([[1,0,dt,0,0],
                      [0,1,0,dt,0],
                      [0,0,1,0,0],
                      [0,0,0,1,0],
                      [0,0,0,0,1]])

        B = np.array([[(dt**2)/2.0,0,0,0,0,0],
                      [0,(dt**2)/2.0,0,0,0,0],
                      [dt,0,0,0,0,0],
                      [0,dt,0,0,0,0],
                      [0,0,0,0,0,0]])

        xImu = np.dot(A,xImu) + np.dot(B,u)
        x = np.dot(A,x) + np.dot(B,u)

        covariance = np.dot(np.dot(A,covariance),A.T) + Q

        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.transform.translation.x = xImu[0]
        t.transform.translation.y = xImu[1]
        t.transform.translation.z = 0.0
        q = tf_conversions.transformations.quaternion_from_euler(0,0,xImu[4])
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        t.header.frame_id = "world"
        t.child_frame_id = "robot_imu"




        br.sendTransform(t)
        #print(covariance)
        #x = np.dot(B,u)

        

        # print(np.dot(B,u),u[5],u[5]*.01)
        # print(x)





if __name__ == '__main__':

    rospy.init_node('forward_predictor')
    rospy.Subscriber("/gazebo/model_states", ModelStates, gtPose_callback)
    rospy.Subscriber("camera_imu", Imu, IMU_Callback)
    rospy.Subscriber("/m6/control_state", Float64, control_state_callback)
    rospy.Subscriber("/orb_slam2_stereo/pose", PoseStamped, measurement_update)
 
    rate = rospy.Rate(100.0)

    while not rospy.is_shutdown():
        rate.sleep()
 

