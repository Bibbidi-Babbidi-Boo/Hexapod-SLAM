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

pi = np.pi


def CPG(cpg, t, dt):
    np.set_printoptions(precision=5)

    set_lastang = np.array([-pi / 2, pi / 2, -pi / 2, pi / 2, -pi / 2, pi / 2])

    shoulders1 = range(0, 18, 3)  # joint IDs of the shoulders
    shoulders2 = range(1, 18, 3)  # joint IDs of the second shoulder joints
    elbows = range(2, 3, 18)  # joint IDs of the elbow joints

    # dynOffsetError = copy.deepcopy(np.zeros([3,6]));
    #
    # if t > cpg['initLength']:
    #     [dynOffsetError,cpg] = computeOffsets(cpg, t, dt);
    #
    # #Scale
    # dynOffsetError = dynOffsetError * cpg['scaling']

    # CPG Implmentation/Equations
    gamma = 3  # forcing to limit cycle (larger = stronger)
    lambdaa = 1  # coupling strength (larger = weaker)

    # x0 = cpg['dynOffset'][0,:]
    # y0 =  cpg['dynOffset'][1,:]

    # cpg['x0'] = -x0
    # cpg['x'] = -cpg['x']
    #
    # cpg['y0'] = y0

    if cpg['move']:
        tauH = 100  # sharpness of transition
        cpg['w'] = cpg['wSwing']  # cpg anglular speed [rad]/[sec]

        H = (abs((cpg['x'][t, :]) / (cpg['a'])) ** cpg['n'] + abs((cpg['y'][t, :]) / (cpg['b'])) ** cpg['n'])
        dHdx = (cpg['n'] * np.sign(cpg['x'][t, :]) * (abs((cpg['x'][t, :]) / cpg['a']) ** (cpg['n'] - 1))) / (cpg['a'])
        dHdy = (cpg['n'] * np.sign(cpg['y'][t, :]) * (abs((cpg['y'][t, :]) / cpg['b']) ** (cpg['n'] - 1))) / (cpg['b'])

        dHdx = dHdx / np.sqrt(dHdx * dHdx + dHdy * dHdy)
        dHdy = dHdy / np.sqrt(dHdx * dHdx + dHdy * dHdy)

        dx = dHdx * gamma * (1 - H) + cpg['w'] * dHdy
        dy = dHdy * gamma * (1 - H) - cpg['w'] * dHdx

        dz_coup = supelliptRot(cpg['a'].T, cpg['b'].T, cpg['n'], np.array([cpg['x'][t, :], cpg['y'][t, :]]).T,
                               cpg['phase_lags'])  # K

        dx = dx + (dz_coup[:, 0].T - (cpg['x'][t, :])) / lambdaa

        dy = dy + (dz_coup[:, 1].T - (cpg['y'][t, :])) / lambdaa

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

    else:
        dx = 0
        dy = 0
        truther = True
        dx_const = 0
        updStab = np.logical_or(cpg['CPGStance'], np.array([False, False, False, False, False, False]))

    test_ang = np.array([0, 0, -1.57,
                         0, 0, 1.57,
                         0, 0, -1.57,
                         0, 0, 1.57,
                         0, 0, -1.57,
                         0, 0, 1.57])
    test_ang = np.reshape(test_ang[0:18], [1, 18])

    test_positions = cpg['xmk'].getLegPositions(test_ang)
    # print(test_positions)
    # [[0.51611  0.51611  0.0575   0.0575 - 0.45861 - 0.45861]
    #  [0.23158 - 0.23158  0.51276 - 0.51276  0.33118 - 0.33118]
    # [-0.2249 - 0.2249 - 0.2249 - 0.2249 - 0.2249 - 0.2249]]
    ang = cpg['xmk'].getLegIK(test_positions)
    # print(ang)

    # cpg['pid'].update(dt, dynOffsetError, updStab);
    # cpg['dynOffset'] = cpg['pid'].getCO();
    # cpg['dynOffsetInc'] = cpg['pid'].getDeltaCO();

    # Apply dynOffsetInc
    # Integrate dx & dy to produce joint commands
    cpg['x'][t + 1, :] = cpg['x'][t, :] + dx * dt  # + cpg['dynOffsetInc'][0,:]
    cpg['xGr'] = cpg['xGr'] + dx_const * dt  # + cpg['dynOffsetInc'][0,:]
    cpg['y'][t + 1, :] = cpg['y'][t, :] + dy * dt  # + cpg['dynOffsetInc'][1,:]
    # print(cpg['dynOffsetInc'])

    # Integrate dx & dy to produce joint commands
    # cpg['x'][t+1,:] = cpg['x'][t,:] + dx * dt
    # cpg['xGr'] = cpg['xGr'] + dx_const * dt
    # cpg['y'][t+1,:] = cpg['y'][t,:] + dy * dt
    # # print(cpg['x'][t+1,:])
    # print(cpg['y'][t+1,:])

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

    # cpg['legs'][0,0:18:3] = limitValue((cpg['nomOffset'][0,:] + cpg['shouldersCorr'] * xOut), pi/2 * cpg['scaling'])
    # cpg['legs'][0,1:19:3] = cpg['nomOffset'][1,:] + cpg['shouldersCorr'] *  np.maximum(0, yOut)
    # print('x', cpg['nomOffset'])
    cpg['legs'][0, 0:18:3] = limitValue((cpg['nomOffset'][0, :] + cpg['shouldersCorr'] * xOut),
                                        pi / 2 * cpg['scaling'])  # x  make sure x is in the range of [-pi/2, pi/2]
    cpg['legs'][0, 1:19:3] = cpg['nomOffset'][1, :] + cpg['shouldersCorr'] * np.maximum(0, yOut)  # y
    # print(cpg['legs'][0, 0:18:3])
    # JOINT 3 - FOR WALKING TRIALS
    cpg['legs'][0, 0:18:3] = cpg['legs'][0, 0:18:3] / cpg['scaling']
    # print(cpg['a'])
    cpg['legs'][0, 1:19:3] = cpg['legs'][0, 1:19:3] / cpg['scaling']

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

    angs = groundIK(cpg, dist, indicies)
    # print(angs)
    joint_rest = np.array([0, 0, -1.57, 0, 0, 1.57, 0, 0, -1.57, 0, 0, 1.57, 0, 0, -1.57, 0, 0, 1.57])
    if cpg['direction'] == 'backwards' or cpg['direction'] == 'forward' :
        for index in indicies:
            cpg['legs'][0, 2 + index * 3] = angs[2 + index * 3]  # +(cpg['dynOffset'][2,index]/cpg['scaling'])
    else:
        for index in indicies:
            cpg['legs'][0, 2 + index * 3] = joint_rest[2 + index * 3]

    cpg['legs'] = np.reshape(cpg['legs'][0:18], [1, 18])

    legs = np.copy(cpg['legs'])
    # print(legs)

    positions = cpg['xmk'].getLegPositions(cpg['legs'])
    # print(cpg['legs'])

    return cpg, positions
