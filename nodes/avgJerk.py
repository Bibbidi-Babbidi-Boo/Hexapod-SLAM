#!/usr/bin/env python
import rospy
import hebi
import hebiros
from std_msgs.msg import Float64
from hebiros.msg import FeedbackMsg


prevAcc = [0,0,0]
time = 0
avg = 0

def process(feedback):
    global time
    global prevAcc
    global avg
    currAcc = [feedback.accelerometer[0].x, feedback.accelerometer[0].y, feedback.accelerometer[0].z]
    if time >= 1:
        dP = abs(currAcc[0]-prevAcc[0]) + abs(currAcc[1]-prevAcc[1]) + abs(currAcc[2]-prevAcc[2])
        avg = ((avg*time)+dP)/(time+1)
    time += 1
    prevAcc = [currAcc[0], currAcc[1], currAcc[2]]
    rospy.loginfo(avg)


def avgJerk():
    
    rospy.init_node('avgJerk', anonymous = True)
    rospy.Subscriber("hebiros/test1/feedback", FeedbackMsg, process)
    print('here')
    rospy.spin()

if __name__ == '__main__':
    avgJerk()