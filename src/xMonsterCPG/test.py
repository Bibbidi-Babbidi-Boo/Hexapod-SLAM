#! /usr/bin/python3

import hebi 
import numpy as np
import time 
import copy
import setup
import SMCF
from Tools.transforms import *
from Tools.PIDController import PIDController
from CPG.forceFB import *
from CPG.updateForceStance import *
from CPG.updateCPGStance import *
from CPG.CPGgs import *
import rospkg



#Setup Modules and Kinematics Object
xmk, imu , hexapod, fbk_imu, fbk_hp = setup.setup_xmonster()

group_command = hebi.GroupCommand(hexapod.size)
hexapod.command_lifetime = 0;

#initialize the complementary filter object
rospack = rospkg.RosPack()
pkgPath = rospack.get_path('xMonsterCPG')
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


T_elaspsed = 0
T = 5000 #Time in seconds code operates for
nIter = int(round(T/0.01)) 

#creating cpg dict
cpg = {
    'initLength': 150,
    's':np.array([0.30, 0.30, 0.30, 0.30, 0.35, 0.35]),
    'h': 0.23,
    'nomX': np.array([0.40, 0.40, 0.0, 0.1, -0.40, -0.40]),
    'nomY': np.array([0.38, -0.38, 0.50,  -0.50, 0.35, -0.35]),
    's1OffsetY': np.array([0.2375*sin(pi/6), -0.2375*sin(pi/6), 0.1875, -0.1875, 0.2375*sin(pi/6), -0.2375*sin(pi/6)]),#robot measurement;  distance on x axis from robot center to
    's1OffsetAngY': np.array([-pi/3, pi/3, 0, 0, pi/3, -pi/3]),
    'n': 2, #limit cycle shape 2:standard, 4:super  
    'b': 0.5*np.ones(6)*2, #np.array([.4, .4, .4, .4, .4, .4]), #TUNEABLE: step height in radians %1.0
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
    'vY': 0
}


#cpg['pid'] = PIDController([3,6],0.,1,0.005,0.01, pi/2 * cpg['scaling'], pi/2 * cpg['scaling'], pi/2 * cpg['scaling'], pi/2 * cpg['scaling']); # BABY GAINS for climbing
cpg['pid'] = PIDController([3,6],0.000,2,0.005,0.000,pi/2 * cpg['scaling'], pi/2 * cpg['scaling'], pi/2 * cpg['scaling'], pi/2 * cpg['scaling']);
#cpg['torquepid'] = PIDController([1,6],0.000,0.000,0.000,0.000,pi/2 *cpg['scaling'], pi/2 * cpg['scaling'], pi/2 * cpg['scaling'], pi/2*cpg['scaling']);
cpg['T']=SE3(np.eye(3),np.array([0, 0,cpg['h']]))#SE(3) describing desired body orientation in the ground frame


# FIXED GROUND ASSUMPTION
GG = rotx(0/180*pi); # R: This thing is supposed to store the orientation of the ground
[Tx,_,_] = eulerSO3(GG); 
actual_bh = 0.25; 
Xoffset = 0.08; #based on fixed ground  R: This is essentialy to push the robot forward in y, based on the inclination
desired_transY = actual_bh * np.tan(Tx) + Xoffset; # R: Compute the desired "forward push" in y
cpg['eePos'] = np.vstack((cpg['nomX'],cpg['nomY'], -cpg['h'] * np.ones([1,6]) )) # R: Compute the EE positions in body frame

ang = cpg['xmk'].getLegIK(cpg['eePos']); #R: This gives the angles corresponding to each of the joints
cpg['nomOffset'] = np.reshape(ang[0:18],[6,3]).T; # R: These are the angles corresponding to the rest position

a = 0.16*2.5*np.ones([1,6]); 
aMin = 0.01; 
a[a < aMin] = aMin;
cpg['a'] =   a   * cpg['scaling']

cpg['b'] = cpg['b'] * cpg['scaling']
cpg['nomOffset'] = cpg['nomOffset'] * cpg['scaling']

#CPG Initialization
cpg['wStance'] = 0.80*4; #cpg anglular speed [rad]/[sec] 
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
cpg['xGr'] = cpg['x'][0,:]; # R: x or first joint angle values, corresponding to grounded legs

stance_duration = (cpg['a'] + cpg['b'])/(cpg['wStance']);
cpg['desired_speed'] = cpg['s'] / (stance_duration[0,:]);

#done initializing cpg

codeStartTime = time.time();

fbk_hp = hexapod.get_next_feedback(reuse_fbk=fbk_hp)
while fbk_hp == None:
    fbk_hp = hexapod.get_next_feedback(reuse_fbk=fbk_hp)

prevFeedbackTime = fbk_hp.receive_time[0]

storeF = np.zeros([3,6,nIter])
storeT = np.zeros([18,nIter])
obj = np.zeros([3,nIter])

for t in range(nIter):
    time_ct = range(t) 

    storeF[:,:,t] = forceFB(cpg);

    storeT[:,t] = cpg['torques'] 

    nForce = np.squeeze(storeF[2,:,time_ct]);
    
    legPos = cpg['xmk'].getLegPositions(cpg['legs']);
    plane_legPos = legPos[0:3,:]
    obj[:,t] = np.dot(plane_legPos, np.reshape((storeF[2,:,t] - np.mean(storeF[2,:,t])).T,[6,1]))[:,0]

    if t == cpg['initLength']:
        cpg['move'] = False;

    #Get body feedback
    fbk_hp = hexapod.get_next_feedback(reuse_fbk=fbk_hp)
    while fbk_hp == None:
        fbk_hp = hexapod.get_next_feedback(reuse_fbk=fbk_hp)
    
    #Time feedback
    dt = max(min(fbk_hp.receive_time[0] - prevFeedbackTime, 0.01),0.01); #Ensure 25Hz-100Hz for CPG stability

    prevFeedbackTime = fbk_hp.receive_time[0];
    
    #Torque feedback
    cpg['torques'] = fbk_hp.effort - cpg['torqueOffsets']
    torqueLimit = 5 #changed from 10 to 5

    #Position feedback
    cpg['legs'] = cpg['comm_alpha'] * cpg['legs'] + (1-cpg['comm_alpha']) * fbk_hp.position; # R: Basically weighing the cpg computation and the position feedback

    #IMU feedback
    fbk_imu = imu.get_next_feedback(reuse_fbk=fbk_imu)
    while fbk_imu == None:
        fbk_imu = imu.get_next_feedback(reuse_fbk=fbk_imu)

    CF.update(fbk_imu) # R: Use feedback to update orientation/body pose
    cpg['pose'] = CF.getBodyPose()
    gravVec = np.linalg.lstsq(cpg['pose'][0:3,0:3], [[0],[0],[-1]]) # R: Estimate gravity vector in the body frame
    gravVec = gravVec[0]

    #Evaluate stance 
     # R: used to update the stance based on force feedback values
    cpg = updateCPGStance(cpg,t); # R: basically used to update which legs are on the ground based on whether the value of the 
                                  # angle in the vertical plane is lower
                                  # than the mean position

    cpg = CPG(cpg, t, dt);

    if t > cpg['initLength']:
        
 
        cpg['move'] = True;
        group_command.position = cpg['legs'][0,:]
    else:
        group_command.position = ang[0:18]

    
    
    
    hexapod.send_command(group_command);


