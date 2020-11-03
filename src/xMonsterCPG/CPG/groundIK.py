import numpy as np
from math import sqrt

def groundIK(cpg,dist,indicies):


    #base = 1:3:18;
    #shoulders = 2:3:18;
    #elbows = 3:3:18;

    #q1 = cpg['legs'][0,3*index]
    #q2 = cpg['legs'][0,3*index+1]
    #q3 = cpg['legs'][0,3*index+2]

    #elbowOut = q3;

    positions = cpg['xmk'].getLegPositions(cpg['legs'])
    # print('positions',positions)#xyz coordinates of the end effectors in world coordinate
    # positions_com = cpg['xmk'].getLegPositions_com(cpg['legs'])
    # print('positions_com',positions_com)

    # print('.................start loop...............')
    for index in indicies:
        reqd_position = positions[:, index]    # select the leg that on the ground

        inter_FK = cpg['xmk'].getLegFrames(cpg['legs'])    #xyz coordinates of the end effectors in world coordinate
        # print(inter_FK)
        # print('....................')
        ee_Frame = np.squeeze(inter_FK[index, 5, :, :])
        homeFrame = np.squeeze(inter_FK[index, 2, :, :])
        # print(index,ee_Frame)
        varFrame = np.squeeze(inter_FK[index, 3, :, :])

        reqd_dist = dist[index]    # selected leg end effector's Y coordinate

        ee_home = np.dot(np.linalg.pinv(homeFrame), ee_Frame)

        z0 = ee_home[2, 3]    #coordinate of ee in homeframe z axis (constant)
        # print(index,z0)


        
        ee_varFrame = np.dot(np.linalg.pinv(varFrame),ee_Frame[:,3])        # vector : ee in varframe


        fixed_dist = np.linalg.norm(ee_varFrame[0:3])    #the length of joint3
        # print(fixed_dist)

        



        last_fixpt = np.dot(np.linalg.pinv(homeFrame),varFrame[:,3])  # vector : var in homeframe
        # print(index, last_fixpt)

        x1 = last_fixpt[0]    # 0.325 first link length
        y1 = last_fixpt[1]    #0
        z1 = last_fixpt[2]    #0

        d = fixed_dist**2 - (z1 - z0)**2  # constant 0.10562
        # print(index,d)
       
        

        
        tf_yvec = np.dot(np.linalg.pinv(homeFrame), np.array([[0,1,0,0]]).T)      #  the vector on word frame y axis expressed in homeframe
        # print(index, tf_yvec)



        new_dist = reqd_dist - homeFrame[1,3]     # distance in y : ee to home
        # print(index, homeFrame[:,3])
        # print(new_dist)
        '''
        print("new_dist")
        print(new_dist)
        print("tf_yvec")
        print(tf_yvec[2])
        print("z0")
        print(z0)
        print("tf_yvec")
        print(tf_yvec[0])
        '''

        k = (new_dist - tf_yvec[2]*z0)/tf_yvec[0]
        b = tf_yvec[1]/tf_yvec[0]

        '''
        print("K")
        print(k)
        print("B")
        print(b)
        print("x1")
        print(x1)
        print("y1")
        print(y1)
        print("d")
        print(d)
        '''

        x01 = (k + b*(b*x1 - y1 + sqrt((1 + b**2)*d - (-k + x1 + b*y1)**2)))/(1+b**2) #error here
        y01 = (k - x01)/b
        
        x02 = (k + b*(b*x1 - y1 - sqrt((1 + b**2)*d - (-k + x1 + b*y1)**2)))/(1+b**2)
        y02 = (k - x02)/b
        
        final_pt1 = np.dot(homeFrame, np.array([[x01, y01, z0, 1]]).T)
        final_pt2 = np.dot(homeFrame, np.array([[x02, y02, z0, 1]]).T)


        if final_pt1[2] > final_pt2[2]:
            final_pt = final_pt2[0:3]
        else:
            final_pt = final_pt1[0:3]



        #print('this is DD',homeFrame)

        positions[:, index] = final_pt[:, 0]



    angs = cpg['xmk'].getLegIK(positions)


    return angs
