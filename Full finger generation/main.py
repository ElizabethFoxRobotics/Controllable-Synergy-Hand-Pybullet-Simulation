
import pybullet as p
import time
import pybullet_data
import math
import numpy as np
import matplotlib.pyplot as plt
import scipy.optimize as optimize
import csv
import os.path
from os import path
from generate_gripper import *
from get_consts_and_matrices import *
maxF = [100, 100, 100] #N, random #, for AX-12A


def getTc(p,contacts,fing,realJoints):
    #ASSUMPTION: all forces are normal
    njs = len(realJoints) #p.getNumJoints(fing)
    Tc = np.zeros(njs)
    f_t = np.zeros(6)
    madeContact = 0
    for i in range(len(contacts)):
        cpt = contacts[i][6] #vector position on finger, in world coords
        n_hat=contacts[i][7] #vector normal to contact, towards obj !
        #t_hat = np.array([1,1,-(n_hat[0]+n_hat[1])/n_hat[2]])
        #o_hat = np.cross(n) #other two vectors in contact frame
        #Ri = np.hstack(n_hat,t_hat,o_hat)
        #Ri_bar = np.vstack(np.hstack(Ri,np.zeros((3,3))),np.hstack(np.zeros((3,3)),Ri))

        JT = np.zeros((njs,6))
        linknum = contacts[i][4] #finger link index  #do link indexes correspond to joint indexes?
        if linknum != -1:
            if linknum > 2 and linknum < 11:
                jointNum = 3
            elif linknum >12 and linknum <21:
                jointNum = 6
            elif linknum > 22:
                jointNum = 9
            else:
                print('uncounted link')
                import pdb; pdb.set_trace()
            for j in range(jointNum):
                jinfo = p.getJointInfo(fing,realJoints[j])
                if jinfo[16] == -1:
                    jparinfo = p.getBasePositionAndOrientation(fing)
                    jparorn = np.asarray(p.getMatrixFromQuaternion(jparinfo[1])) #vec4 world orientation of link
                    jparorn=jparorn.reshape((3,3))
                    Zcj = np.asarray(jparinfo[0]) + jparorn@np.asarray(jinfo[14]) #origin of coordinate frame associated with joint of linknum
                else:
                    jparinfo = p.getLinkState(fing,jinfo[16])
                    jparorn = np.asarray(p.getMatrixFromQuaternion(jparinfo[5])) #vec4 world orientation of link
                    jparorn=jparorn.reshape((3,3))
                    Zcj = np.asarray(jparinfo[4]) + jparorn@np.asarray(jinfo[14]) #origin of coordinate frame associated with joint of linknum
                zj = [0,0,-1] #joint z axis
                sr = cpt - Zcj
                JT[j][0:3] = sr#np.cross(zj,sr)#Ssr @ zj
                JT[j][3:]=zj


            #print('JT', JT)
            #J_cal = p.calculateJacobian(fing,linknum,np.asarray(lorn) @ np.asarray(cpt-ls[4],jpos,jvs, jas))
            #print('J-cal', J_cal)
            fi = np.hstack((contacts[i][9]*np.asarray(n_hat),np.zeros(3))) #column vector of contact force/torque
            if madeContact==0:
                f_t = fi.copy()
                madeContact = 1
            else:
                f_t += fi


            Tc += JT @ -fi
            sumf = sum(JT @ -fi)
            #print('Tc of this contact: ', JT @ -fi, ' sum ', sumf )
    return Tc, f_t


def close_CUH(p,fList,obj,ps,fs,realJoints,s_des,Tc_last, count,stiffnessDegree):
    f1 = fList[0]
    f2 = fList[1]
    #!!! will have to change this to any collision, not just obj, including other fingers
    for i in range(len(fList)):
        jointStates_thisf = p.getJointStates(fList[i],realJoints)
        jointReaction_thisf = [(y) for z, x, y, a in jointStates_thisf]
        jointAngles_thisf = [(z) for z, x, y, a in jointStates_thisf]

        this_pset = ps[len(realJoints)//3*i:len(realJoints)//3*(i+1)]
        q = jointAngles_thisf.copy() #joint angles zeroed and scaled for transmission matrix
        for ii in range(len(realJoints)//3):
            q[3*ii] = -jointAngles_thisf[3*ii] + math.pi/2
            q[3*ii+1] = math.pi - jointAngles_thisf[3*ii+1]
            q[3*ii+2] = math.pi/2 -jointAngles_thisf[3*ii+2]
        this_R,this_s0,this_sall = getTransmissionMatrix(q,this_pset)
        this_R.shape = (1,this_R.size)

        cp_thisf=p.getContactPoints(bodyA = obj,bodyB = fList[i])
        fing_Tc, fing_cf_t=getTc(p,cp_thisf,fList[i],realJoints)
        for j in range(len(fList)):
            #if j!=i: !!!!!!!!!! getting self-collision in finger; might need to switch to urdf
                cp_thisf=p.getContactPoints(bodyA = fList[j],bodyB = fList[i])
                this_Tc, this_cf_t=getTc(p,cp_thisf,fList[i],realJoints)
                fing_Tc += this_Tc
                fing_cf_t += this_cf_t
        this_E = getStiffnessMatrix(q,this_pset,fs,this_sall,stiffnessDegree)
        if i==0:
            R = this_R.copy()
            s0=this_s0
            sall=this_sall.copy()
            Tc = fing_Tc.copy()
            cf_t = fing_cf_t.copy()
            q_all = q.copy()
            E = this_E.copy()
        else:
            R = np.concatenate((R,this_R),axis=1)
            s0=s0+this_s0
            sall = sall+this_sall
            Tc = np.concatenate((Tc,fing_Tc))
            q_all = np.concatenate((q_all,q))
            E = np.concatenate((np.concatenate((E,np.zeros((E.shape[0],this_E.shape[1]))),axis=1),np.concatenate((np.zeros((this_E.shape[0],E.shape[1])),E),axis=1)),axis=0)

    Tc_all = Tc.copy()
    #q0 = getq0(q,ps)

    Einv = np.linalg.inv(E)
    RT = np.transpose(R)

    qarr = q_all.copy()
    qarr.shape = (qarr.size,1)

    Tc.shape = (Tc.size,1)
    RERTinv = 1/(R @ Einv @ RT) #RERT is 1x1
    RERTinv = RERTinv[0][0] #change from array to float

    dq_calc = Einv @ RT*RERTinv*(s_des-s0)#change in q
    df_calc = RERTinv*(s_des-s0) #change in f

    f_des = fs + df_calc
    q_des =  np.ndarray.tolist(np.squeeze(qarr + dq_calc))

    s1 = getTransmissionMatrix(q_des,ps)[1]

    J_des = q_des.copy()

    for i in range(len(ps)):

        J_des[3*i] = -q_des[3*i] + math.pi/2
        J_des[3*i+1] = math.pi - q_des[3*i+1]
        J_des[3*i+2] = math.pi/2 -q_des[3*i+2]
    return J_des, s0, f_des, Tc_all, cf_t, q_all,sall, s1,np.squeeze(dq_calc)




def setAndRunGripper(p, fList, realJoints, obj, ps, totalCount, writer, stiffnessDegree):
    running = 0
    for j in range(len(fList)):
        jointPos = getZeroPos(ps[len(realJoints)//3*j:len(realJoints)//3*(j+1)])
        for i in range(len(realJoints)):
            p.resetJointState(fList[j],realJoints[i],jointPos[i],0) #move to initial position for given pressures

    s_end = 0.01
    ds = 0.00001

    fs = 0;
    s_now = 0;
    for i in range(len(fList)):
        s_now+=getZeroAct(p,fList[i],realJoints)
    s_last = s_now
    count = 0

    longercount = 0
    sall_last =0
    maxIter = 6000;
    while(abs(s_now-s_end)>ds) and fs > -25 and count < maxIter:
        count +=1
        totalCount +=1

        for i in range(len(fList)):
            jointStates_thisf = p.getJointStates(fList[i],realJoints)
            jointReaction_thisf = [(y) for z, x, y, a in jointStates_thisf]
            jointAngles_thisf = [(z) for z, x, y, a in jointStates_thisf]

            q = jointAngles_thisf.copy() #joint angles zeroed and scaled for transmission matrix
            for ii in range(len(realJoints)//3):
                q[3*ii] = -jointAngles_thisf[3*ii] + math.pi/2
                q[3*ii+1] = math.pi - jointAngles_thisf[3*ii+1]
                q[3*ii+2] = math.pi/2 -jointAngles_thisf[3*ii+2]
            this_s_now = getTransmissionMatrix(q,ps)[1]

            if i==0:
                s_now=this_s_now
            else:
                s_now=s_now+this_s_now

        if totalCount == 1:
            s_anticipated = s_now #s if it hit s_des as it is supposed to
        if s_now < s_end:
            s_des = min(s_now+ds,s_end)
            print('????????????????????????????????????????????????????????????????????????????')
        else:
            s_des = max(s_anticipated-ds,s_end)
        Tc_last = np.zeros(len(realJoints)*len(fList));

        jointStates,s_now_new, fs,Tc_last,contactForce_total,q_now,sall,s_expected,dq_calc = close_CUH(p,fList,obj,ps.copy(),fs,realJoints.copy(),s_des,Tc_last,count,stiffnessDegree)


        contactForce_mag = contactForce_total[0]**2+contactForce_total[1]**2+contactForce_total[2]**2

        if s_now-s_last > .001:
            longercount+=1
            print('longer? ', longercount, (s_now-s_last), count)
        s_last = s_now
        q_des = jointStates.copy()
        for i in range(len(ps)):
            q_des[3*i] = -jointStates[3*i] + math.pi/2
            q_des[3*i+1] = math.pi - jointStates[3*i+1]
            q_des[3*i+2] = math.pi/2 -jointStates[3*i+2]
        writer.writerow([q_des[0],q_des[1],q_des[2],q_des[3],q_des[4],q_des[5],q_des[6],q_des[7],q_des[8],q_now[0],q_now[1],q_now[2],q_now[3],q_now[4],q_now[5],q_now[6],q_now[7],q_now[8],dq_calc[0],dq_calc[1],dq_calc[2],dq_calc[3],dq_calc[4],dq_calc[5],dq_calc[6],dq_calc[7],dq_calc[8], s_now,s_expected,s_des,sall[0],sall[1],sall[2],fs,contactForce_mag]) #q_des this step (gen next q_now); q and s_now current; fs next step
        #!!! only writing 1st finger info

        sall_last = sall
        #!!! not overdamped??? adding gains adds too much velocity
        for i in range(len(fList)):
            this_jointStates = jointStates[len(realJoints)*i:len(realJoints)*(i+1)]

            p.setJointMotorControlArray(bodyIndex=fList[i],
                jointIndices=realJoints,
                controlMode=p.POSITION_CONTROL,
                #forces = maxF*3,
                targetPositions = this_jointStates,
                #targetVelocities = [0]*9,
                positionGains = [1]*9,
                velocityGains = [1]*9
                )

        for nss in range(10):
            p.stepSimulation()

        s_anticipated = s_des

    if(abs(s_now-s_end)<.00001):
        print('s limit')
    if (fs < -100):
        print('f limit')

    return running, totalCount

def main():

    physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
    cam_pos = [-.11,-0,.1]

    p.resetDebugVisualizerCamera(.2,0,-91,cam_pos)
    p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW,enable=0)
    p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW,enable=0)
    p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW,enable=0)

    p.setPhysicsEngineParameter(numSolverIterations=20)

    inputs = [1,1,1,0,0,0,0,0,.75, 5, 90]#[1, .5, 1.5, -30, -20, -10, 10, -30, .75, 4, 30] #proxLen, midLen,distLen; 5x angles; R; finger disp, finger ang; fang > 0

    fList, realJoints = generate_Gripper(inputs) #!!! make all work for 1-5 fingers
    fL = fList[0]
    fR = fList[1]
    x = 0.05
    robj = 0.01
    baseobj=p.createCollisionShape(p.GEOM_CYLINDER,radius = robj,height=.015875)
    #baseobj=p.createCollisionShape(p.GEOM_BOX,halfExtents = [robj,robj, 0.015875])

    #obj=p.createMultiBody(baseMass = .15875*math.pi*robj**2*1250,baseCollisionShapeIndex=baseobj,basePosition=[-.1,0,0],)#[-x,0,0])
    obj=p.createMultiBody(baseCollisionShapeIndex=baseobj,basePosition=[-.55,-.025,0],)#[-x,0,0])

    numJoints = len(realJoints)*len(fList)
    #p.changeDynamics(obj,-1,lateralFriction = 0.62, rollingFriction=0.25)
    #for i in range(len(fList)):
    #p.changeDynamics(fList[i],-1,lateralFriction = 0.62, rollingFriction=0.25)
    input('set position')
    p1 =0
    p2 = 0
    p3 = 0
    p4 = 0
    p5 = 0
    p6 = 0
    ps = [p1,p2,p3,p4,p5,p6]

    totalCount = 0
    running = 1

    fields = ['f1_q1d', 'f1_q2d', 'f1_q3d','f2_q1d', 'f2_q2d', 'f2_q3d','f3_q1d', 'f3_q2d', 'f3_q3d', 'f1_q1', 'f1_q2', 'f1_q3','f2_q1', 'f2_q2', 'f2_q3','f3_q1', 'f3_q2', 'f3_q3', 'd1_q1', 'd1_q2', 'd1_q3','d2_q1', 'd2_q2', 'd2_q3','d3_q1', 'd3_q2', 'd3_q3', 's','s_e','s_d', 's1','s2','s3', 'force']
    filename = input('filename')
    while path.exists(filename+'.csv'):
        filename = filename+'_v2'
    #with open(filename +'.csv', 'w') as csvfile:

    stiffnessDegree = input('Stiffness degree: ')
    with open(filename+'.csv',  'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(fields)
        while running:
            running, totalCount = setAndRunGripper(p, fList, realJoints, obj, ps, totalCount, writer, stiffnessDegree)

    input('press to disconnect')
    p.disconnect()


if __name__ == '__main__':
    main()
