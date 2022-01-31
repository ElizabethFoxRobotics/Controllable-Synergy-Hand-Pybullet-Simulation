
import pybullet as p
import pybullet_data
import math


def generate_Gripper(inputs):

    #!!! change inputs to finger separation (and number??)
    fang = inputs[10]*math.pi/180
    fdist = inputs[9]*0.0254
    sThic=.015875
    palm_shape = p.createCollisionShape(p.GEOM_BOX,halfExtents=[.01/2,fdist/2,sThic/2],collisionFramePosition=[.01/2,fdist/2,0])
    palm_vis = p.createVisualShape(p.GEOM_BOX,halfExtents=[.01/2,fdist/2,sThic/2],visualFramePosition=[.01/2,fdist/2,0],rgbaColor=[1,1,1, 1])
    #palm = p.createMultiBody(baseMass=999,baseCollisionShapeIndex=palm_shape,baseVisualShapeIndex=palm_vis,basePosition=[0,-fdist/2,0],baseOrientation=p.getQuaternionFromEuler([0,0,0]))
    fL = p.loadURDF('C:\\Users\\user\\Documents\\Python\\CS Hand\\optimization\\using URDF\\urdfs\\fullFingerv4\\urdf\\fullFingerv4.urdf',basePosition=[0,-fdist/2,0],baseOrientation=p.getQuaternionFromEuler([0,math.pi,fang]), useFixedBase=1)
    fR = p.loadURDF('C:\\Users\\user\\Documents\\Python\\CS Hand\\optimization\\using URDF\\urdfs\\fullFingerv4\\urdf\\fullFingerv4.urdf',basePosition=[0,fdist/2,0],baseOrientation=p.getQuaternionFromEuler([0,0,fang]), useFixedBase=1)

    #fL_attach =p.createConstraint(palm,-1,fL,-1,p.JOINT_FIXED,[0,0,0],[0.01,-fdist/2,0],[0,0,0])
    #p.setCollisionFilterPair(fL, palm,-1,-1,0)
    #p.setCollisionFilterPair(fR, palm,-1,-1,0)
    #fR_attach =p.createConstraint(palm,-1,fR,-1,p.JOINT_FIXED,[0,0,0],[0.01,fdist/2,0],[0,0,0])
    #p.changeConstraint(fL_attach,maxForce=1000)
    #p.changeConstraint(fR_attach,maxForce=1000)


    nj=p.getNumJoints(fL)
    realJoints = []
    for i in  range(nj):
        ji=p.getJointInfo(fL,i)
        if ji[2] != 4:
            realJoints.append(i)
    jlims = [math.pi/2-170.*math.pi/180, math.pi/2-90.*math.pi/180, math.pi-74.*math.pi/180, math.pi-59.*math.pi/180,math.pi/2-170.*math.pi/180, math.pi/2-90.*math.pi/180]
    for i in range(int(len(realJoints)/3)):
        p.changeDynamics(fL,realJoints[3*i],jointLowerLimit=(jlims[0]),jointUpperLimit=(jlims[1]))
        p.changeDynamics(fL,realJoints[3*i+1],jointLowerLimit=(jlims[2]),jointUpperLimit=(jlims[3]))
        p.changeDynamics(fL,realJoints[3*i+2],jointLowerLimit=(jlims[4]),jointUpperLimit=(jlims[5]))

        p.changeDynamics(fR,realJoints[3*i],jointLowerLimit=(math.pi/2-170.*math.pi/180),jointUpperLimit=(math.pi/2-120.*math.pi/180))
        p.changeDynamics(fR,realJoints[3*i+1],jointLowerLimit=(math.pi-74.*math.pi/180),jointUpperLimit=(math.pi-59.*math.pi/180))
        p.changeDynamics(fR,realJoints[3*i+2],jointLowerLimit=(math.pi/2-170.*math.pi/180),jointUpperLimit=(math.pi/2-120.*math.pi/180))

        p.enableJointForceTorqueSensor(fL,realJoints[3*i],1)
        p.enableJointForceTorqueSensor(fL,realJoints[3*i+1],1)
        p.enableJointForceTorqueSensor(fL,realJoints[3*i+2],1)

    fList = [fL, fR]
    return fList, realJoints

def getZeroAct(p,f1,realJoints):
    #returns cable length
    jointStates_f1 = p.getJointStates(f1,realJoints)
    jointAngles_f1 = [(z) for z, x, y, a in jointStates_f1]

    s0t = 0
    lf = 0.01 #m; length of virtual arms of flexure
    Lp = (0.47601069+.125)*0.0254 #length of link before chamfer
    La = ((.8+1/16)*0.0254 -Lp)/math.cos(math.radians(20)) #length until tendon along chamfer
    LL = math.sqrt(Lp**2+La**2-2*Lp*La*math.cos(math.radians(160))) #distance base of link to center of tendon on chamfer
    aa = math.asin(La*math.sin(math.radians(160))/LL) #angle between LL and link face
    q = jointAngles_f1.copy() #joint angles zeroed and scaled for transmission matrix

    for i in range(int(len(q)/3)):
        ar1 = jointAngles_f1[3*i] + math.pi/2
        ar2 = math.pi - jointAngles_f1[3*i+1]
        ar3 = math.pi/2 -jointAngles_f1[3*i+2]

        b = math.pi/2-ar2/2 #angle from flexure line to virtual links of ar2
        lenf = lf*math.sqrt(2-2*math.cos(ar2)) #flexure length
        r = math.sqrt(lenf**2+LL**2-2*lenf*LL*math.cos(ar3+aa-b)) #length ar1 to top tendon
        a1 = math.asin(LL/r*math.sin(ar3+aa-b)) #angle from flexure line to r
        a2 = math.pi-ar1-b-a1+aa #angle from r to top of link (Lp)
        s0 = math.sqrt(LL**2+r**2-2*LL*r*math.cos(a2)) #length of tendon

        s0t = s0t + s0
    return s0t
