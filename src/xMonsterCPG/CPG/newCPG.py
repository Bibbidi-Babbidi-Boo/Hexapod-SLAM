import numpy as np
from CPG.computeOffsets import computeOffsets
from Tools.constrainSE3 import *
from CPG.limitValue import *
from CPG.groundIK import *
from CPG.supelliptRot import *
import copy
import rospy
import math
from matplotlib import pyplot as plt
from std_msgs.msg import Float32MultiArray
pi = np.pi


def CPG(cpg, t, dt):
    pub = rospy.Publisher('leg_ground', Float32MultiArray, queue_size=0)
    np.set_printoptions(precision=5)

    set_lastang = np.array([-pi / 2, pi / 2, -pi / 2, pi / 2, -pi / 2, pi / 2])

    shoulders1 = range(0, 18, 3)  # joint IDs of the shoulders
    shoulders2 = range(1, 18, 3)  # joint IDs of the second shoulder joints
    elbows = range(2, 3, 18)  # joint IDs of the elbow joints

    dynOffsetError = copy.deepcopy(np.zeros([3, 6]));

    if t > cpg['initLength']:
        [dynOffsetError, cpg] = computeOffsets(cpg, t, dt)

    # Scale
    dynOffsetError = dynOffsetError * cpg['scaling']*0

    # if cpg['direction'] == 'left' or cpg['direction'] == 'right':
    #     a = np.array([[0.65], [0.65], [0.65], [0.65], [0.65], [0.65]]) / 2
    #     cpg['a'] = np.reshape(a, (1, 6))

    # CPG Implmentation/Equations
    gamma = 3  # forcing to limit cycle (larger = stronger)
    lambdaa = 1  # coupling strength (larger = weaker)

    if cpg['move']:

        cpg['w'] = 5
        # print('a',cpg['a'])
        # print('b', cpg['b'])
        # print('w', cpg['w'])
        # print('y', cpg['y'][t, :])
        # print('cy', cpg['cy'])
        # print('x', cpg['x'][t, :])
        # print('cx', cpg['cx'])
        dx = -1 * -1 * ((cpg['a'] * cpg['b']) / 2) * cpg['w'] * 2 * ((cpg['y'][t, :] - cpg['cy']) / (cpg['b'] ** 2)) + 1 * (
                    1 - (((cpg['x'][t, :] - cpg['cx']) / cpg['a']) ** 2) - ((cpg['y'][t, :] - cpg['cy']) / cpg['b']) ** 2) * 2 * (
                     (cpg['x'][t, :] - cpg['cx']) / (cpg['a'] ** 2))
        dy = -1 * ((cpg['a'] * cpg['b']) / 2) * cpg['w'] * 2 * ((cpg['x'][t, :] - cpg['cx']) / (cpg['a'] ** 2)) + 1 * (
                    1 - (((cpg['x'][t, :] - cpg['cx']) / cpg['a']) ** 2) - ((cpg['y'][t, :] - cpg['cy']) / cpg['b']) ** 2) * 2 * (
                     (cpg['y'][t, :] - cpg['cy']) / (cpg['b'] ** 2)) + (np.dot(0.3 * cpg['K'], (cpg['y'][t, :] - cpg['cy'])))
        #
        # x = x + 0.02 * f1
        # y = y + 0.02 * f2
        # x_output = x.copy()
        # y_output = y.copy()


        for value in range(6):
            truth = (cpg['CPGStanceDelta'].flatten())[value]
            if truth:
                cpg['xGr'][value] = cpg['x'][t, value]

        x_pos = abs(cpg['nomY'] - cpg['s1OffsetY'])

        dx_fact = 1 / (x_pos * np.reciprocal(np.cos(cpg['s1OffsetAngY'] + cpg['legs'][0, shoulders1])) ** 2)

        dx_const = -1 * cpg['scaling'] * cpg['desired_speed'] * dx_fact
        dx_const = dx[0, :]
        truther = False
        updStab = np.logical_or(cpg['CPGStance'], (dy[0] < 0))
        # print(cpg['CPGStance'])

    else:
        dx = 0
        dy = 0
        truther = True
        dx_const = 0
        updStab = np.logical_or(cpg['CPGStance'], np.array([False, False, False, False, False, False]))

    # test_ang = np.array([0, 0, -1.57,
    #                      0, 0, 1.57,
    #                      0, 0, -1.57,
    #                      0, 0, 1.57,
    #                      0, 0, -1.57,
    #                      0, 0, 1.57])
    # test_ang = np.reshape(test_ang[0:18], [1, 18])
    #
    # test_positions = cpg['xmk'].getLegPositions(test_ang)
    # print(test_positions)
    # [[0.51611  0.51611  0.0575   0.0575 - 0.45861 - 0.45861]
    #  [0.23158 - 0.23158  0.51276 - 0.51276  0.33118 - 0.33118]
    # [-0.2249 - 0.2249 - 0.2249 - 0.2249 - 0.2249 - 0.2249]]
    # ang = cpg['xmk'].getLegIK(test_positions)
    # print(ang)
    # Calculate dynOffsetInc

    cpg['pid'].update(dt, dynOffsetError, updStab)
    cpg['dynOffset'] = cpg['pid'].getCO()
    cpg['dynOffsetInc'] = cpg['pid'].getDeltaCO()
    # print(cpg['dynOffsetInc'])

    # Integrate dx & dy to produce joint commands


    cpg['x'][t + 1, :] = cpg['x'][t, :] + dx * dt  + cpg['dynOffsetInc'][0,:]
    cpg['xGr'] = cpg['xGr'] + dx_const * dt  + cpg['dynOffsetInc'][0,:]
    cpg['y'][t + 1, :] = cpg['y'][t, :] + dy * dt  + cpg['dynOffsetInc'][1,:]
    # print(cpg['dynOffsetInc'])

    # cpg['x'][t+1,:] = -cpg['x'][t,:]
    # cpg['xGr'] = -cpg['xGr']

    # Command CPG-generated values to joints
    yOut = cpg['y'][t + 1, :]
    xOut = np.zeros([1, 6])

    SignBack = -1 if (cpg['direction'] == 'backwards') else 1
    SignLeft = -1 if (cpg['direction'] == 'left') else 1
    SignRight = -1 if (cpg['direction'] == 'right') else 1

    for value in range(6):
        truth = cpg['CPGStance'][value]
        if truth:
            xOut[:, value] = cpg['xGr'][value]
        else:
            if value % 2 == 0:
                xOut[:, value] = SignLeft * SignBack * cpg['x'][t + 1, value]
            else:
                xOut[:, value] = SignRight * SignBack * cpg['x'][t + 1, value]

    cpg['legs'][0,0:18:3] = limitValue((cpg['nomOffset'][0,:] + cpg['shouldersCorr'] * xOut), pi/2 * cpg['scaling'])
    cpg['legs'][0,1:19:3] = cpg['nomOffset'][1,:] + cpg['shouldersCorr'] *  np.maximum(0, yOut)
    #  cpg['shouldersCorr'] helps correct the output with differernt side of motor
    # cpg['nomOffset'])  is the rest ppositon of the robot



    #
    # cpg['legs'][0, 0:18:3] = limitValue((cpg['shouldersCorr'] * xOut),
    #                                     pi / 2 * cpg['scaling'])  # x  make sure x is in the range of [-pi/2, pi/2]
    # cpg['legs'][0, 1:19:3] = cpg['shouldersCorr'] * np.maximum(0, yOut)  # y cpg['shouldersCorr'] helps correct the output with differernt side of motor

    leg_ground = np.int64(yOut>0)
    data_topublish = Float32MultiArray(data=leg_ground)
    pub.publish(data_topublish)
    # print(cpg['legs'][0, 1:19:3])
    # print(cpg['legs'][0, 0:18:3])
    # JOINT 3 - FOR WALKING TRIALS
    cpg['legs'][0, 0:18:3] = cpg['legs'][0, 0:18:3]/2 #/ cpg['scaling']
    # print(cpg['a'])
    cpg['legs'][0, 1:19:3] = cpg['legs'][0, 1:19:3]/2 #/ cpg['scaling']

    I = (np.logical_or(cpg['CPGStance'], dy < 0))  # the leg on the ground

    dist = cpg['nomY']  # coordinate(y) of the end effector of 6 legs

    indicies = []
    for index in range(6):
        if truther:  # moving : truther = false
            truth = I[index]
        else:
            truth = I[0, index]
        if truth:
            indicies.append(index)

    # angs = groundIK(cpg, dist, indicies)
    #
    # for index in indicies:
    #     cpg['legs'][0, 2+index*3] = angs[2+index*3]
    # print(cpg['cx'][3])


    Leglength = 0.325
    z = np.array([-math.pi/2, math.pi/2, -math.pi/2, math.pi/2, -math.pi/2, math.pi / 2])


    # z[0] = -math.pi / 2 + math.asin((Leglength * math.cos(math.pi / 3 + cpg['cx'][0] + cpg['a'][0][0]/2) / (
    #     math.cos(math.pi / 3 + cpg['cx'][0] - cpg['x'][t + 1, :][0] * (-1))) - Leglength) / Leglength)
    # z[1] = math.pi / 2 - math.asin((Leglength * math.cos(math.pi / 3 + cpg['cx'][1] + cpg['a'][0][1]/2) / (
    #     math.cos(math.pi / 3 + cpg['cx'][1] + cpg['x'][t + 1, :][1])) - Leglength) / Leglength)
    #
    # z[2] = -math.pi / 2 + abs(math.asin(((Leglength) / math.cos(cpg['x'][t + 1, :][3]) - Leglength) / Leglength))
    #
    # z[3] = math.pi / 2 - abs(math.asin(((Leglength) / math.cos(cpg['x'][t + 1, :][2] * (-1)) - Leglength) / Leglength))
    #
    # z[4] = -math.pi / 2 + math.asin((Leglength * math.cos(math.pi / 3 + + cpg['cx'][4] + cpg['a'][0][4]/2) / (
    #     math.cos(math.pi / 3 + cpg['cx'][4] + cpg['x'][t + 1, :][4] * (-1))) - Leglength) / Leglength)
    # z[5] = math.pi / 2 - math.asin((Leglength * math.cos(math.pi / 3 + + cpg['cx'][5] + cpg['a'][0][5]/2) / (
    #     math.cos(math.pi / 3 + cpg['cx'][5] - cpg['x'][t + 1, :][5])) - Leglength) / Leglength)
    if cpg['direction'] == 'backwards' or cpg['direction'] == 'forward':
        z[0] = -math.pi / 2 + math.asin((Leglength * math.cos(math.pi / 3 + cpg['cx'][0] + cpg['a'][0][0]/2) / (
            math.cos(math.pi / 3 + cpg['cx'][0] - cpg['legs'][0, 0])) - Leglength) / Leglength)
        z[1] = math.pi / 2 - math.asin((Leglength * math.cos(math.pi / 3 + cpg['cx'][1] + cpg['a'][0][1]/2) / (
            math.cos(math.pi / 3 + cpg['cx'][1] + cpg['legs'][0, 3])) - Leglength) / Leglength)

        z[2] = -math.pi / 2 + abs(math.asin(((Leglength) / math.cos(cpg['legs'][0, 6]*(-1)) - Leglength) / Leglength))

        z[3] = math.pi / 2 - abs(math.asin(((Leglength) / math.cos(cpg['legs'][0, 9] * (-1)) - Leglength) / Leglength))

        z[4] = -math.pi / 2 + math.asin((Leglength * math.cos(math.pi / 3 + + cpg['cx'][4] + cpg['a'][0][4]/2) / (
            math.cos(math.pi / 3 + cpg['cx'][4] + cpg['legs'][0, 12])) - Leglength) / Leglength)
        z[5] = math.pi / 2 - math.asin((Leglength * math.cos(math.pi / 3 + + cpg['cx'][5] + cpg['a'][0][5]/2) / (
            math.cos(math.pi / 3 + cpg['cx'][5] - cpg['legs'][0, 15])) - Leglength) / Leglength)
    else:
        z[0] = -math.pi / 2
        z[1] = math.pi / 2
        z[2] = -math.pi / 2
        z[3] = math.pi / 2
        z[4] = -math.pi / 2
        z[5] = math.pi / 2


    # print(cpg['a'][0][0])
    # print(cpg['x'][t + 1, :][0])
    # print(cpg['x'][t + 1, :])

    cpg['legs'][0, 2] = z[0]
    cpg['legs'][0, 5] = z[1]
    cpg['legs'][0, 8] = z[2]
    cpg['legs'][0, 11] = z[3]
    cpg['legs'][0, 14] = z[4]
    cpg['legs'][0, 17] = z[5]

    # for i in range(6):
    #     if cpg['legs'][0, 1 + 3 * i] != 0:
    #         z[i] = (-1)**(i+1) * math.pi / 2

    cpg['legs'] = np.reshape(cpg['legs'][0:18], [1, 18])

    # legs = np.copy(cpg['legs'])
    # print(legs)

    positions = cpg['xmk'].getLegPositions(cpg['legs'])
    # print(cpg['legs'])

    return cpg,positions
