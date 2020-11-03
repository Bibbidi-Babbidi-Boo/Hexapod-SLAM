import numpy as np 
import math
from Tools.transforms import *

#(cpg['a'].T,      cpg['b'].T,      cpg['n'],      np.array([cpg['x'][t,:], cpg['y'][t,:]]).T,       cpg['phase_lags'])
def supelliptRot(a,b,d,z,angs):

    curr_r = (abs(z[:, 0]/a[:, 0].T)**d + abs(z[:, 1]/b.T)**d)**(1/d)
    
    curr_a = a[:, 0]*curr_r
    curr_b = b*curr_r
    
    curr_ang = np.arctan2(np.sign(z[:,1])* (abs(z[:,1]/curr_b))**(d/2), np.sign(z[:,0])* (abs(z[:,0]/curr_a))**(d/2))
    
    matShape = np.shape(z)[0]   # =6
    # print(matShape)
    new_angs = np.zeros((matShape, matShape, 2))
    ph_lags = np.zeros((6, 6))
    angs = [0, np.pi, np.pi, 0, 0, np.pi]
    for i in range(6):
        for j in range(6):
            ph_lags[i,j] = angs[i] - angs[j]         # K matrix

    # print(z)
    # print(ph_lags)
    for osc1 in range(matShape):
        for osc2 in range(matShape):
            rot_matrix = rotz(ph_lags[osc1, osc2])[0:3, 0:3]
            new_angs[osc1, osc2, :] = np.dot(rot_matrix[0:2, 0:2], np.array([np.sign(z[osc2,0]) * (abs(z[osc2,0]/curr_a[osc2]))**(d/2), np.sign(z[osc2,1]) * (abs(z[osc2,1]/curr_b[osc2]))**(d/2)]).T).T


    x = np.zeros(np.shape(z))      # shape(z)=(6,2)



    for osc1 in range(matShape):
        temp = np.multiply(np.array([curr_a[osc1],curr_b[osc1]]),np.sign(np.squeeze(new_angs[osc1,:,:])))
        temp2 = np.multiply(temp, np.power((np.abs(np.squeeze(new_angs[osc1,:,:]))),(2/d))) - z[osc1,:]
        x[osc1,:]= np.sum(temp2,axis=0)

    
    return x

