import numpy as np

def forceFB(cpg):
    J = cpg['xmk'].getLegJacobians(cpg['legs']);
    F = np.zeros([3,6])
    T = cpg['torques']
    M = np.sum(cpg['xmk'].getLegMasses(),0)

    
    for leg in range(6):
        curr_J = J[0:3,0:3,leg]
        #print(T[:,(3*(leg+1)-3):(3*(leg+1))].T)
        #print(3*(leg+1)-3,3*(leg+1))


        curr_F = np.dot(np.linalg.pinv(curr_J.T) ,T[:,(3*(leg+1)-3):(3*(leg+1))].T)


        F[:,leg] = -curr_F[:,0];
    return F
