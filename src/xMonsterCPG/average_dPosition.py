#!/usr/bin/env python  
import roslib
import rospy
import math
from std_msgs.msg import Float64MultiArray
import rosbag
import numpy as np
#from setup.setup_xmonster import *
import setup
import SMCF
import rospkg


if __name__ == '__main__':

    rospy.init_node('avg_dpose')
    xmk, imu , hexapod, fbk_imu, fbk_hp = setup.setup_xmonster()

    rate = rospy.Rate(200.0)

    rospack = rospkg.RosPack()
    pkgPath = rospack.get_path('cpg')
    offsets = np.load(pkgPath+'/src/xMonsterCPG/setup/setupFiles/offsets.npy',allow_pickle=True, encoding='latin1')

    CF = SMCF.SMComplementaryFilter(offsets);

    pose = None
    while type(pose) == type(None):
        fbk_imu = imu.get_next_feedback(reuse_fbk=fbk_imu)
        while fbk_imu == None:
            fbk_imu = imu.get_next_feedback(reuse_fbk=fbk_imu)
        CF.update(fbk_imu);
        CF.updateFilter(fbk_imu);
        pose = CF.getBodyPose();
    time = 0;
    lastPose = None;
    currPose = None;
    avg = 0;
    dP = 0;
    while not rospy.is_shutdown() and (fbk_hp != None):
        
        fbk_imu = imu.get_next_feedback(reuse_fbk=fbk_imu)
        while fbk_imu == None:
            fbk_imu = imu.get_next_feedback(reuse_fbk=fbk_imu)

        CF.update(fbk_imu)


        currPose = CF.getBodyPose()

        if time >= 1:
            for x in range(3):
                for y in range(3):
                    dP += abs(currPose[x][y] - lastPose[x][y])
            dP/=9
            avg = ((avg*time)+dP)/(time+1)
            dP = 0

        print('avg dp')
        print(avg)
        lastPose = currPose
        time += 1;
        '''
        print("legTorques")
        sum = 0;
        for torque in fbk_hp.effort:
            sum += abs(torque)
        print(sum)
        '''
        rate.sleep()
        fbk_hp = hexapod.get_next_feedback(reuse_fbk=fbk_hp)
