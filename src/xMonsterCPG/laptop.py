#! /usr/bin/python3

import hebi
import numpy as np
import time
import copy
import setup
import rospy
from Tools.transforms import *
from Tools.PIDController import PIDController
from CPG.updateCPGStance import *
from CPG.laptopCPG import *
# from CPG.noFeedCPG import *
# from CPG.newCPG import *
import rospkg
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_msgs.msg import String
import math
from matplotlib import pyplot as plt
global new_direction, keyboard_input
new_direction='0'
keyboard_input = 's'

jointPositions = np.zeros((1,18))
def joint_callback(data):
    pos = np.array(data.position)
    pos = pos.reshape(1,18)
    pos = pos.reshape(3,6)
    pos = pos.T
    pos = pos.reshape(1,18)
    jointPositions = pos


def callback(data):
    global new_direction
    #rospy.loginfo(rospy.get_caller_id()+"Heard %s",data.data)
    new_direction=data.data

def keyboard_callabck(data):
    global keyboard_input
    keyboard_input = data.data
    # print(type(data.data))


#Setup Modules and Kinematics Object
from setup.xMonsterKinematics import *
xmk = HexapodKinematics();


#initialize the complementary filter object
rospack = rospkg.RosPack()
pkgPath = rospack.get_path('xMonsterCPG')
offsets = np.load(pkgPath+'/src/xMonsterCPG/setup/setupFiles/offsets.npy',allow_pickle=True, encoding='latin1')
rospy.init_node('laptop_cpg')



T_elaspsed = 0
T = 5000 #Time in seconds code operates for
nIter = int(round(T/0.01))

#creating cpg dict
cpg = {
    'initLength': 0,
    's':np.array([0.30, 0.30, 0.30, 0.30, 0.30, 0.30]),
    # 'h': 0.23,
    # 'nomX': np.array([0.40, 0.40, 0.0, 0.0, -0.40, -0.40]),
    # 'nomY': np.array([0.38, -0.38, 0.50,  -0.50, 0.38, -0.38]),

    'h': 0.2249,
    'nomX': np.array([0.51611, 0.51611, 0.0575, 0.0575, -0.45861, -0.45861]),
    'nomY': np.array([0.23158, -0.23158, 0.51276, -0.51276,  0.33118, -0.33118]),
    's1OffsetY': np.array([0.2375*sin(pi/6), -0.2375*sin(pi/6), 0.1875, -0.1875, 0.2375*sin(pi/6), -0.2375*sin(pi/6)]),#robot measurement;  distance on x axis from robot center to
    's1OffsetAngY': np.array([-pi/3, pi/3, 0, 0, pi/3, -pi/3]),
    'n': 2, #limit cycle shape 2:standard, 4:super
    'b': 0.8*np.ones(6), #np.array([.4, .4, .4, .4, .4, .4]), #TUNEABLE: step height in radians %1.0
    'scaling': 10, #TUNEABLE: shifts the units into a reasonable range for cpg processing (to avoid numerical issues)
    'shouldersCorr': np.array([-1, 1, -1, 1, -1, 1]),
    'phase_lags': np.array([pi, pi, 0, pi, 0, pi]),
    'dynOffset': np.zeros([3,6]), #Offset on joints developed through constraining
    'dynOffsetInc': np.zeros([3,6]), #Increment since last iteration
    'x': np.zeros([nIter,6]), #TUNEABLE: Initial CPG x-positions
    'y': np.zeros([nIter,6]), #TUNEABLE: Initial CPG y-positions
    'x0': np.zeros([1,6]), #limit cycle center x
    'y0': np.zeros([6,6]), #limit cycle center y
    'legs': np.zeros([1,18]), #Joint angle values
    'elbowsLast': np.zeros([1,6]), #elbow values
    'torques': np.zeros([1,18]), #Joint torque values
    'torqueOffsets': offsets[2], #Joint torque offset values
    'gravCompTorques': np.zeros([1,18]), #Joint torque values
    'forces': np.zeros([3,6]), #ee force values
    'gravCompForces': np.zeros([3,6]), #Joint torque values
    'forceStance': np.zeros([1,6]), #grounded legs determined by force
    'CPGStance': np.array([False,False,False,False,False,False]), #grounded legs determined by position (lower tripod)
    'CPGStanceDelta': np.zeros([1,6]), #grounded legs determined by position (lower tripod)
    'CPGStanceBiased': np.zeros([1,6]), #grounded legs determined by position (lower tripod)
    'comm_alpha': 1.0, #commanded alpha in the complementary filter (1-this) is the measured joint angles
    'move': True, #true: walks according to cpg.direction, false: stands in place (will continue to stabilize); leave to true for CPG convergence
    'xmk': xmk, #Snake Monster Kinematics object
    'pose': np.eye(3), #%SO(3) describing ground frame w.r.t world frame
    'R': SE3(np.eye(3),[0, 0, 0]), #SE(3) describing body correction in the ground frame
    'G': np.eye(3), #SO(3) describing ground frame w.r.t world frame
    'tp': np.zeros([4,1]),
    'dynY': 0,
    'vY': 0,
    'direction' : 'forward',
    'fullStepLength' : 20000,
    't' : 0
}



def command_callback(data):
    cpg['direction'] = data.data
    cpg['t'] = cpg['initLength'] + 1
    #print(cpg['direction'])


#cpg['pid'] = PIDController([3,6],0.,1,0.005,0.01, pi/2 * cpg['scaling'], pi/2 * cpg['scaling'], pi/2 * cpg['scaling'], pi/2 * cpg['scaling']); # BABY GAINS for climbing
cpg['pid'] = PIDController([3,6],0.000,2,0.005,0.000,pi/2 * cpg['scaling'], pi/2 * cpg['scaling'], pi/2 * cpg['scaling'], pi/2 * cpg['scaling']);
#cpg['torquepid'] = PIDController([1,6],0.000,0.000,0.000,0.000,pi/2 *cpg['scaling'], pi/2 * cpg['scaling'], pi/2 * cpg['scaling'], pi/2*cpg['scaling']);
cpg['T']=SE3(np.eye(3),np.array([0, 0,cpg['h']]))#SE(3) describing desired body orientation in the ground frame


# FIXED GROUND ASSUMPTION
GG = rotx(0/180*pi); # R: This thing is supposed to store the orientation of the ground
[Tx,_,_] = eulerSO3(GG);
actual_bh = 0.25;
Xoffset = 0.08; #based on fixed ground  R: This is essentialy to push the robot forward in y, based on the inclination
desired_transY = -(actual_bh * np.tan(Tx) + Xoffset); # R: Compute the desired "forward push" in y
cpg['eePos'] = np.vstack((cpg['nomX'],cpg['nomY'], -cpg['h'] * np.ones([1,6]) )) # R: Compute the EE positions in body frame
# print(cpg['eePos'])
# [[ 0.4   0.4   0.    0.   -0.4  -0.4 ]
#  [ 0.38 -0.38  0.5  -0.5   0.38 -0.38]
#  [-0.23 -0.23 -0.23 -0.23 -0.23 -0.23]]

ang = cpg['xmk'].getLegIK(cpg['eePos']); #R: This gives the angles corresponding to each of the joints



cpg['nomOffset'] = np.reshape(ang[0:18],[6,3]).T; # R: These are the angles corresponding to the rest position

a = 0.16*np.ones([1,6])
# a = np.array([[0.4092], [0.4092], [0.65], [0.65], [0.4092], [0.4092]])/ math.sqrt(2)

# a = np.array([[0.2618], [0.2618], [0.2618], [0.2618], [0.2618], [0.2618]])
a = np.reshape(a, (1, 6))
aMin = -10;
a[a < aMin] = aMin;
cpg['a'] =   a   * cpg['scaling']

cpg['b'] = cpg['b'] * cpg['scaling']
cpg['nomOffset'] = cpg['nomOffset'] * cpg['scaling']

#CPG Initialization
cpg['wStance'] = 0.80*6; #cpg anglular speed [rad]/[sec]
cpg['wSwing'] = cpg['wStance'] * 6.0;
cpg['K'] = np.array( [[0, -1, -1,  1,  1, -1],
                     [-1,  0,  1, -1, -1,  1],
                     [-1,  1,  0, -1, -1,  1],
                     [ 1, -1, -1,  0,  1, -1],
                     [ 1, -1, -1,  1,  0, -1],
                     [-1,  1,  1, -1, -1,  0]])

u = np.array([-pi/5, 7*pi/5 ,6*pi/5 ,9*pi/5 ,8*pi/5 ,7*pi/5])

cpg['x'][0,:] =  cpg['a'] * np.array([1,-1,-1,1,1,-1]); # R: Initialize the x and y values of the cpg cycle
cpg['y'][0,:] =  np.zeros(6) #np.array([-0.7216 ,   0.7216 ,   0.7216,   -0.7216,   -0.7216,    0.7216]);
cpg['xGr'] = cpg['x'][0,:] # R: x or first joint angle values, corresponding to grounded legs

stance_duration = (cpg['a'] + cpg['b'])/(cpg['wStance'])
cpg['desired_speed'] = cpg['s'] / (stance_duration[0,:])

cpg['cx'] = np.array([0, 0, 0, 0, 0, 0]) * math.pi
cpg['cy'] = np.array([0, 0, 0, 0, 0, 0]) * math.pi
#done initializing cpg

prevFeedbackTime = time.time()
rate = rospy.Rate(100.0)



rospy.Subscriber("m6/joint_states", JointState, joint_callback)
rospy.Subscriber("/hexapod/direction/command", String, command_callback)

rospy.Subscriber("further_direction", String, callback, queue_size=1)




pub_base1 = rospy.Publisher("/m6/base1_position_controller/command", Float64, queue_size=1)
pub_base2 = rospy.Publisher("/m6/base2_position_controller/command", Float64, queue_size=1)
pub_base3 = rospy.Publisher("/m6/base3_position_controller/command", Float64, queue_size=1)
pub_base4 = rospy.Publisher("/m6/base4_position_controller/command", Float64, queue_size=1)
pub_base5 = rospy.Publisher("/m6/base5_position_controller/command", Float64, queue_size=1)
pub_base6 = rospy.Publisher("/m6/base6_position_controller/command", Float64, queue_size=1)

pub_shoulder1 = rospy.Publisher("/m6/shoulder1_position_controller/command", Float64, queue_size=1)
pub_shoulder2 = rospy.Publisher("/m6/shoulder2_position_controller/command", Float64, queue_size=1)
pub_shoulder3 = rospy.Publisher("/m6/shoulder3_position_controller/command", Float64, queue_size=1)
pub_shoulder4 = rospy.Publisher("/m6/shoulder4_position_controller/command", Float64, queue_size=1)
pub_shoulder5 = rospy.Publisher("/m6/shoulder5_position_controller/command", Float64, queue_size=1)
pub_shoulder6 = rospy.Publisher("/m6/shoulder6_position_controller/command", Float64, queue_size=1)

pub_elbow1 = rospy.Publisher("/m6/elbow1_position_controller/command", Float64, queue_size=1)
pub_elbow2 = rospy.Publisher("/m6/elbow2_position_controller/command", Float64, queue_size=1)
pub_elbow3 = rospy.Publisher("/m6/elbow3_position_controller/command", Float64, queue_size=1)
pub_elbow4 = rospy.Publisher("/m6/elbow4_position_controller/command", Float64, queue_size=1)
pub_elbow5 = rospy.Publisher("/m6/elbow5_position_controller/command", Float64, queue_size=1)
pub_elbow6 = rospy.Publisher("/m6/elbow6_position_controller/command", Float64, queue_size=1)


pub_control_state = rospy.Publisher("/m6/control_state", Float64, queue_size=1)

control_state = 0.0
while not rospy.is_shutdown():
    #Time feedback
    timeNow = time.time()
    dt = max(min(timeNow - prevFeedbackTime, 0.01),0.01); #Ensure 25Hz-100Hz for CPG stability
    prevFeedbackTime = timeNow

    rospy.Subscriber("/keys", String, keyboard_callabck, queue_size=1)

    #Updating a value
    a = 0.16*np.ones([1, 6])
    # a = np.array([0.16, 0.16, 0.32, 0.32, 0.16, 0.16])

    # a = np.array([[0.4092], [0.4092], [0.65], [0.65], [0.4092], [0.4092]])/ math.sqrt(2)
    # a = np.array([[0.2618], [0.2618], [0.4618], [0.4618], [0.2618], [0.2618]])/ math.sqrt(3)
    a = np.reshape(a, (1, 6))
    aMin = -10
    a[a < aMin] = aMin



    import numpy as np

    # a_exist = np.array([0.16, 0.16, 0.16, 0.16, 0.16, 0.16])




    #
    # bm = torch.tensor([0, 0, 0, 0, 0, 0, 0], dtype=torch.int32)
    # b = GNN_model(a_exist, a_0, camera)
    # new_a=b.detach().numpy()
    # print(new_a)
    # time.sleep(0.2)
    # print(a)
    # print(new_direction)
    # print(a)
    cpg['a'] =   a   * cpg['scaling']
    # print(cpg['a'])

    #Position feedback
    cpg['legs'] = cpg['comm_alpha'] * cpg['legs'] + (1-cpg['comm_alpha']) * jointPositions # R: Basically weighing the cpg computation and the position feedback

    cpg = updateCPGStance(cpg, cpg['t'])
    cpg,positions = CPG(cpg, cpg['t'], dt)
    t=cpg['t']
    # print(positions)

    # plt.subplot(231)
    # plt.plot(cpg['x'][t+1,:][0],cpg['y'][t+1,:][0],".b")
    # plt.subplot(234)
    # plt.plot(cpg['x'][t+1,:][1],cpg['y'][t+1,:][1],".b")
    # plt.subplot(232)
    # plt.plot(cpg['x'][t+1,:][2],cpg['y'][t+1,:][2],".b")
    # plt.subplot(235)
    # plt.plot(cpg['x'][t+1,:][3],cpg['y'][t+1,:][3],".b")
    # plt.subplot(233)
    # plt.plot(cpg['x'][t+1,:][4],cpg['y'][t+1,:][4],".b")
    # plt.subplot(236)
    # plt.plot(cpg['x'][t+1,:][5],cpg['y'][t+1,:][5],".b")
    # plt.pause(0.01)
    # plt.show()

    if cpg['t'] > cpg['initLength']:
        control_state = 1.0


    if cpg['t'] > (cpg['initLength'] + 50 ):
        cpg['move'] = True
        commanded_position = cpg['legs'][0,:]

        # commanded_position = np.zeros(18)

        pos_leg0 = positions[:, 0]
        pos_leg1 = positions[:, 1]
        pos_leg2 = positions[:, 2]
        pos_leg3 = positions[:, 3]
        pos_leg4 = positions[:, 4]
        pos_leg5 = positions[:, 5]

        with open('pos_leg0.txt', 'a') as f:
            f.write(str(pos_leg0[0]) + ',' + str(pos_leg0[1]) + ',' + str(pos_leg0[2]) + '\n')
        with open('pos_leg1.txt', 'a') as f:
            f.write(str(pos_leg1[0]) + ',' + str(pos_leg1[1]) + ',' + str(pos_leg1[2]) + '\n')
        with open('pos_leg2.txt', 'a') as f:
            f.write(str(pos_leg2[0]) + ',' + str(pos_leg2[1]) + ',' + str(pos_leg2[2]) + '\n')
        with open('pos_leg3.txt', 'a') as f:
            f.write(str(pos_leg3[0]) + ',' + str(pos_leg3[1]) + ',' + str(pos_leg3[2]) + '\n')
        with open('pos_leg4.txt', 'a') as f:
            f.write(str(pos_leg4[0]) + ',' + str(pos_leg4[1]) + ',' + str(pos_leg4[2]) + '\n')
        with open('pos_leg5.txt', 'a') as f:
            f.write(str(pos_leg5[0]) + ',' + str(pos_leg5[1]) + ',' + str(pos_leg5[2]) + '\n')
        #print('saving txt..........')


    else:
        commanded_position = ang[0:18]

        #print('stopped')

    if keyboard_input == 'w':
        control_state = 0.0
        # cpg.update({'move': 'True'})
        cpg.update({'direction': 'forward'})
        # camera = 0
        print('forward')
    elif keyboard_input == 'a':
        control_state = 0.0
        # cpg.update({'move': 'True'})
        cpg.update({'direction': 'left'})
        # camera = -1
        print('left')
    elif keyboard_input == 'd':
        control_state = 0.0
        # cpg.update({'move': 'True'})
        cpg.update({'direction': 'right'})
        # camera == 1
        print('right')
    else:
        control_state = 1.0
        commanded_position = ang[0:18]
        # cpg.update({'move': 'False'})

    # hexapod.send_command(group_command);
    pub_control_state.publish(control_state)


    pub_base1.publish(commanded_position[0])
    pub_base2.publish(commanded_position[3])
    pub_base3.publish(commanded_position[6])
    pub_base4.publish(commanded_position[9])
    pub_base5.publish(commanded_position[12])
    pub_base6.publish(commanded_position[15])
    # print(commanded_position[0],commanded_position[3],commanded_position[6],commanded_position[9],commanded_position[12],commanded_position[15])


    pub_shoulder1.publish(commanded_position[1])
    pub_shoulder2.publish(commanded_position[4])
    pub_shoulder3.publish(commanded_position[7])
    pub_shoulder4.publish(commanded_position[10])
    pub_shoulder5.publish(commanded_position[13])
    pub_shoulder6.publish(commanded_position[16])

    pub_elbow1.publish(commanded_position[2])
    pub_elbow2.publish(commanded_position[5])
    pub_elbow3.publish(commanded_position[8])
    pub_elbow4.publish(commanded_position[11])
    pub_elbow5.publish(commanded_position[14])
    pub_elbow6.publish(commanded_position[17])


    rate.sleep()

    # print("MOPVE", cpg['move'])

    cpg['t'] += 1