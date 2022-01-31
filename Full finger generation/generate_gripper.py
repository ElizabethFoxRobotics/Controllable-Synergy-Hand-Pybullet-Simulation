
import pybullet as p
import pybullet_data
import math


def generate_Gripper(inputs):
    proxLen = 0.0254*inputs[0]
    midLen = 0.0254*inputs[1]
    distLen = 0.0254*inputs[2]

    aproxb = inputs[3]*math.pi/180
    aproxt = inputs[4]*math.pi/180
    amidb = inputs[5]*math.pi/180
    amidt = inputs[6]*math.pi/180
    adistb = inputs[7]*math.pi/180
    distR = inputs[8]*0.0254;

    #inputs[9] = 1
    fdist = inputs[9]*0.0254
    fang = inputs[10]*math.pi/180
    sThic=.015875

    lLen=0.0254*.25

    intThic = .1*0.0254

    interfaceWidth = (.54142525)*0.0254
    diagWidth = (.33299575)*0.0254



    diagRot = 150.5*math.pi/180
    LL = math.sqrt(interfaceWidth**2+diagWidth**2-2*interfaceWidth*diagWidth*math.cos(diagRot)); #distance base of link to center of tendon on chamfer
    aa = math.asin(diagWidth*math.sin(diagRot)/LL);
    cableRoute = .09375*.0254
    diagPhi = math.pi - diagRot - aa


    baseID=p.createCollisionShape(p.GEOM_BOX,halfExtents=[interfaceWidth/2,lLen/2,sThic/2],collisionFramePosition=[-interfaceWidth/2,lLen/2,0])

    intBID = p.createCollisionShape(p.GEOM_BOX,halfExtents=[interfaceWidth/2,intThic/2,sThic/2],collisionFramePosition=[-interfaceWidth/2,-intThic/2,0])
    intTID = p.createCollisionShape(p.GEOM_BOX,halfExtents=[interfaceWidth/2,intThic/2,sThic/2],collisionFramePosition=[-interfaceWidth/2,intThic/2,0])

    intBD = p.createCollisionShape(p.GEOM_BOX,halfExtents=[diagWidth/2,intThic/2,sThic/2],collisionFramePosition=[diagWidth/2,intThic/2,0])
    intTD = p.createCollisionShape(p.GEOM_BOX,halfExtents=[diagWidth/2,intThic/2,sThic/2],collisionFramePosition=[diagWidth/2,-intThic/2,0])


    proxIID=p.createCollisionShape(p.GEOM_BOX,halfExtents=[intThic/2,proxLen/2,sThic/2],collisionFramePosition=[-intThic/2,-proxLen/2,0])
    midIID=p.createCollisionShape(p.GEOM_BOX,halfExtents=[intThic/2,midLen/2,sThic/2],collisionFramePosition=[-intThic/2,-midLen/2,0])
    distIID=p.createCollisionShape(p.GEOM_BOX,halfExtents=[intThic/2,distLen/2,sThic/2],collisionFramePosition=[-intThic/2,-distLen/2,0])



    #prox
    x = math.sqrt(LL**2+proxLen**2-2*LL*proxLen*math.cos(math.pi/2+aproxt-aa))
    B1 = math.asin(LL*math.sin(math.pi/2+aproxt-aa)/x)
    B2 = math.pi/2 + aproxb -aa -B1
    intS = math.sqrt(LL**2+x**2-2*LL*x*math.cos(B2))
    proxRot = diagPhi + math.acos((x**2-intS**2-LL**2)/(-2*LL*intS))
    proxEBdiagWidth = cableRoute/math.sin(math.pi-proxRot)
    trapX = math.sqrt(intS**2+proxEBdiagWidth**2-2*intS*proxEBdiagWidth*math.cos(math.pi-proxRot))
    y1 = math.asin(intS*math.sin(math.pi-proxRot)/trapX)
    if proxRot > math.pi/2:  #!!! should work in most cases but not sure about all
        y1 = math.pi - y1
    y2 = proxRot - y1
    proxRotTop = 3*math.pi/2 + diagPhi -aproxt+aa-B1 -B2-proxRot+diagPhi
    proxETdiagWidth = math.sin(y2)*trapX/math.sin(proxRotTop)
    proxELen = trapX * math.sin(math.pi -y2 -proxRotTop)/math.sin(proxRotTop)

    proxintBDE = p.createCollisionShape(p.GEOM_BOX,halfExtents=[proxEBdiagWidth/2,intThic/2,sThic/2],collisionFramePosition=[proxEBdiagWidth/2,intThic/2,0])
    proxintTDE = p.createCollisionShape(p.GEOM_BOX,halfExtents=[proxETdiagWidth/2,intThic/2,sThic/2],collisionFramePosition=[proxETdiagWidth/2,-intThic/2,0])
    proxEID=p.createCollisionShape(p.GEOM_BOX,halfExtents=[proxELen/2,intThic/2,sThic/2],collisionFramePosition=[-proxELen/2,-intThic/2,0])

    #mid
    x = math.sqrt(LL**2+midLen**2-2*LL*midLen*math.cos(math.pi/2+amidt-aa))
    B1 = math.asin(LL*math.sin(math.pi/2+amidt-aa)/x)
    B2 = math.pi/2 + amidb -aa -B1
    intS = math.sqrt(LL**2+x**2-2*LL*x*math.cos(B2))
    midRot = diagPhi + math.acos((x**2-intS**2-LL**2)/(-2*LL*intS))
    midEBdiagWidth = cableRoute/math.sin(math.pi-midRot)
    trapX = math.sqrt(intS**2+midEBdiagWidth**2-2*intS*midEBdiagWidth*math.cos(math.pi-midRot))
    y1 = math.asin(intS*math.sin(math.pi-midRot)/trapX)
    if midRot > math.pi/2:  #!!! should work in most cases but not sure about all
        y1 = math.pi - y1
    y2 = midRot - y1
    midRotTop = 3*math.pi/2 + diagPhi -amidt+aa-B1 -B2-midRot+diagPhi
    midETdiagWidth = math.sin(y2)*trapX/math.sin(midRotTop)
    midELen = trapX * math.sin(math.pi -y2 -midRotTop)/math.sin(midRotTop)

    midintBDE = p.createCollisionShape(p.GEOM_BOX,halfExtents=[midEBdiagWidth/2,intThic/2,sThic/2],collisionFramePosition=[midEBdiagWidth/2,intThic/2,0])
    midintTDE = p.createCollisionShape(p.GEOM_BOX,halfExtents=[midETdiagWidth/2,intThic/2,sThic/2],collisionFramePosition=[midETdiagWidth/2,-intThic/2,0])
    midEID=p.createCollisionShape(p.GEOM_BOX,halfExtents=[midELen/2,intThic/2,sThic/2],collisionFramePosition=[-midELen/2,-intThic/2,0])


    #fix to prev but with R instead of top interface
    if distLen <= distR: #will not work for all negative angle options
        print('invalid length option')
        distR = distLen*.95
    distEBdiagWidth = .10771458*.0254
    distintBDE = p.createCollisionShape(p.GEOM_BOX,halfExtents=[distEBdiagWidth/2,intThic/2,sThic/2],collisionFramePosition=[midEBdiagWidth/2,intThic/2,0])
    LLt_dist = math.sqrt(interfaceWidth**2+(diagWidth+distEBdiagWidth)**2-2*interfaceWidth*(diagWidth+distEBdiagWidth)*math.cos(diagRot)) #LL total (including full chamfer)
    B1 = math.acos((LLt_dist**2+interfaceWidth**2-(diagWidth+distEBdiagWidth)**2)/(2*interfaceWidth*LLt_dist)) #angle between interface and LLt
    x = math.sqrt(LLt_dist**2+(distLen-distR)**2-2*LLt_dist*(distLen-distR)*math.cos(math.pi/2+adistb-B1))
    distELen = math.sqrt(x**2-distR**2)
    a1 = math.pi - diagRot - B1
    a2 = math.acos((x**2+LLt_dist**2-(distLen-distR)**2)/(2*x*LLt_dist))
    a3 = math.atan2(distR,distELen)
    distRot = a1 + a2 + a3
    tana = adistb-B1+a2+a3
    distDiagRot = math.pi/2-tana/2
    distdiag = math.cos(distDiagRot)*distR*2
    distTrapHeight = distR - math.sqrt(distR**2-(distdiag/2)**2)
    distTopLen = distTrapHeight/math.cos(distDiagRot)
    distTanLen = distTopLen*2


    distTop = p.createCollisionShape(p.GEOM_BOX,halfExtents=[distTopLen/2,intThic/2,sThic/2],collisionFramePosition=[distTopLen/2,-intThic/2,0])
    distTan = p.createCollisionShape(p.GEOM_BOX,halfExtents=[distTanLen/2,intThic/2,sThic/2],collisionFramePosition=[distTanLen/2,-intThic/2,0])
    distEID=p.createCollisionShape(p.GEOM_BOX,halfExtents=[distELen/2,intThic/2,sThic/2],collisionFramePosition=[-distELen/2,-intThic/2,0])
    #import pdb; pdb.set_trace()

    flexLen = .01*2 #L_f*2
    flexWidth = .125/4*0.0254
    hflexID=p.createCollisionShape(p.GEOM_BOX,halfExtents=[flexWidth/2,flexLen/4,sThic/2],collisionFramePosition=[-flexWidth/2,-flexLen/4,0])

    ajn = 30 #number of links
    palm = p.createCollisionShape(p.GEOM_BOX,halfExtents=[.01/2,fdist/2,sThic/2],collisionFramePosition=[.01/2,fdist/2,0])
    masses =[.1]*(ajn+1)#[.01, .01, .05, .05, .05, .05]*3 + [.05, .2]

    fL=p.createMultiBody(baseCollisionShapeIndex=baseID,basePosition=[0,-fdist/2,0],baseOrientation=p.getQuaternionFromEuler([0,0,-fang]),linkMasses=masses,linkVisualShapeIndices=[-1]*(ajn+1),linkInertialFramePositions=[[0,0,0]]*(ajn+1),linkInertialFrameOrientations=[[0,0,0,1]]*(ajn+1),
        linkCollisionShapeIndices=[hflexID,hflexID,intBID,intBD,proxintBDE, proxIID,intTID,intTD,proxintTDE,proxEID,
                                   hflexID,hflexID,intBID,intBD,midintBDE, midIID,intTID,intTD,midintTDE,midEID,
                                   hflexID,hflexID,intBID,intBD,distintBDE, distIID,distTop,distTan,distTop,distEID,
                                   palm],
        linkOrientations=[[0,0,0,1],[0,0,0,1],[0,0,0,1],p.getQuaternionFromEuler([0,0,-diagRot]),[0,0,0,1],p.getQuaternionFromEuler([0,0,aproxb]),p.getQuaternionFromEuler([0,0,aproxt]),p.getQuaternionFromEuler([0,0,diagRot]),p.getQuaternionFromEuler([0,0,-0]),p.getQuaternionFromEuler([0,0,-proxRot]),
                          [0,0,0,1],[0,0,0,1],[0,0,0,1],p.getQuaternionFromEuler([0,0,-diagRot]),[0,0,0,1],p.getQuaternionFromEuler([0,0,amidb]),p.getQuaternionFromEuler([0,0,amidt]),p.getQuaternionFromEuler([0,0,diagRot]),p.getQuaternionFromEuler([0,0,-0]),p.getQuaternionFromEuler([0,0,-midRot]),
                          [0,0,0,1],[0,0,0,1],[0,0,0,1],p.getQuaternionFromEuler([0,0,-diagRot]),[0,0,0,1],p.getQuaternionFromEuler([0,0,adistb]),p.getQuaternionFromEuler([0,0,math.pi]),p.getQuaternionFromEuler([0,0,-math.pi/2+distDiagRot]),p.getQuaternionFromEuler([0,0,distDiagRot-math.pi/2]),p.getQuaternionFromEuler([0,0,-distRot]),
                          p.getQuaternionFromEuler([0,0,fang])],
        linkPositions=[[0,0,0],[0,-flexLen/2,0],[0,-flexLen/2,0],[-interfaceWidth,0,0],[diagWidth,0,0],[0,0,0],[0,-proxLen,0],[-interfaceWidth,0,0],[diagWidth,0,0],[proxEBdiagWidth,0,0],
                        [0,0,0],[0,-flexLen/2,0],[0,-flexLen/2,0],[-interfaceWidth,0,0],[diagWidth,0,0],[0,0,0],[0,-midLen,0],[-interfaceWidth,0,0],[diagWidth,0,0],[midEBdiagWidth,0,0],
                        [0,0,0],[0,-flexLen/2,0],[0,-flexLen/2,0],[-interfaceWidth,0,0],[diagWidth,0,0],[0,0,0],[0,-distLen,0],[distTopLen,0,0],[distTanLen,0,0],[distEBdiagWidth,0,0],
                        [0,0,0]],
        linkParentIndices=[0,1,2,3,4,3,6,7,8,5,
                           7,11,12,13,14,13,16,17,18,15,
                           17,21,22,23,24,23,26,27,28,25,
                            0],
        linkJointTypes=[p.JOINT_REVOLUTE,p.JOINT_REVOLUTE,p.JOINT_REVOLUTE,p.JOINT_FIXED,p.JOINT_FIXED,p.JOINT_FIXED,p.JOINT_FIXED,p.JOINT_FIXED,p.JOINT_FIXED,p.JOINT_FIXED,
                        p.JOINT_REVOLUTE,p.JOINT_REVOLUTE,p.JOINT_REVOLUTE,p.JOINT_FIXED,p.JOINT_FIXED,p.JOINT_FIXED,p.JOINT_FIXED,p.JOINT_FIXED,p.JOINT_FIXED,p.JOINT_FIXED,
                        p.JOINT_REVOLUTE,p.JOINT_REVOLUTE,p.JOINT_REVOLUTE,p.JOINT_FIXED,p.JOINT_FIXED,p.JOINT_FIXED,p.JOINT_FIXED,p.JOINT_FIXED,p.JOINT_FIXED,p.JOINT_FIXED,
                        p.JOINT_FIXED],
        linkJointAxis=[[0,0,-1],[0,0,-1],[0,0,-1],[0,0,-1],[0,0,-1],[0,0,-1],[0,0,-1],[0,0,-1],[0,0,-1],[0,0,-1],
                        [0,0,-1],[0,0,-1],[0,0,-1],[0,0,-1],[0,0,-1],[0,0,-1],[0,0,-1],[0,0,-1],[0,0,-1],[0,0,-1],
                        [0,0,-1],[0,0,-1],[0,0,-1],[0,0,-1],[0,0,-1],[0,0,-1],[0,0,-1],[0,0,-1],[0,0,-1],[0,0,-1],
                        [0,0,-1]])

    fR=p.createMultiBody(baseCollisionShapeIndex=baseID,basePosition=[0,fdist/2,0],baseOrientation=p.getQuaternionFromEuler([math.pi,0,fang]),linkMasses=masses,linkVisualShapeIndices=[-1]*(ajn+1),linkInertialFramePositions=[[0,0,0]]*(ajn+1),linkInertialFrameOrientations=[[0,0,0,1]]*(ajn+1),
        linkCollisionShapeIndices=[hflexID,hflexID,intBID,intBD,proxintBDE, proxIID,intTID,intTD,proxintTDE,proxEID,
                                   hflexID,hflexID,intBID,intBD,midintBDE, midIID,intTID,intTD,midintTDE,midEID,
                                   hflexID,hflexID,intBID,intBD,distintBDE, distIID,distTop,distTan,distTop,distEID,
                                   palm],
        linkOrientations=[[0,0,0,1],[0,0,0,1],[0,0,0,1],p.getQuaternionFromEuler([0,0,-diagRot]),[0,0,0,1],p.getQuaternionFromEuler([0,0,aproxb]),p.getQuaternionFromEuler([0,0,aproxt]),p.getQuaternionFromEuler([0,0,diagRot]),p.getQuaternionFromEuler([0,0,-0]),p.getQuaternionFromEuler([0,0,-proxRot]),
                          [0,0,0,1],[0,0,0,1],[0,0,0,1],p.getQuaternionFromEuler([0,0,-diagRot]),[0,0,0,1],p.getQuaternionFromEuler([0,0,amidb]),p.getQuaternionFromEuler([0,0,amidt]),p.getQuaternionFromEuler([0,0,diagRot]),p.getQuaternionFromEuler([0,0,-0]),p.getQuaternionFromEuler([0,0,-midRot]),
                          [0,0,0,1],[0,0,0,1],[0,0,0,1],p.getQuaternionFromEuler([0,0,-diagRot]),[0,0,0,1],p.getQuaternionFromEuler([0,0,adistb]),p.getQuaternionFromEuler([0,0,math.pi]),p.getQuaternionFromEuler([0,0,-math.pi/2+distDiagRot]),p.getQuaternionFromEuler([0,0,distDiagRot-math.pi/2]),p.getQuaternionFromEuler([0,0,-distRot]),
                          p.getQuaternionFromEuler([0,0,fang])],
        linkPositions=[[0,0,0],[0,-flexLen/2,0],[0,-flexLen/2,0],[-interfaceWidth,0,0],[diagWidth,0,0],[0,0,0],[0,-proxLen,0],[-interfaceWidth,0,0],[diagWidth,0,0],[proxEBdiagWidth,0,0],
                        [0,0,0],[0,-flexLen/2,0],[0,-flexLen/2,0],[-interfaceWidth,0,0],[diagWidth,0,0],[0,0,0],[0,-midLen,0],[-interfaceWidth,0,0],[diagWidth,0,0],[midEBdiagWidth,0,0],
                        [0,0,0],[0,-flexLen/2,0],[0,-flexLen/2,0],[-interfaceWidth,0,0],[diagWidth,0,0],[0,0,0],[0,-distLen,0],[distTopLen,0,0],[distTanLen,0,0],[distEBdiagWidth,0,0],
                        [0,0,0]],
        linkParentIndices=[0,1,2,3,4,3,6,7,8,5,
                           7,11,12,13,14,13,16,17,18,15,
                           17,21,22,23,24,23,26,27,28,25,
                            0],
        linkJointTypes=[p.JOINT_REVOLUTE,p.JOINT_REVOLUTE,p.JOINT_REVOLUTE,p.JOINT_FIXED,p.JOINT_FIXED,p.JOINT_FIXED,p.JOINT_FIXED,p.JOINT_FIXED,p.JOINT_FIXED,p.JOINT_FIXED,
                        p.JOINT_REVOLUTE,p.JOINT_REVOLUTE,p.JOINT_REVOLUTE,p.JOINT_FIXED,p.JOINT_FIXED,p.JOINT_FIXED,p.JOINT_FIXED,p.JOINT_FIXED,p.JOINT_FIXED,p.JOINT_FIXED,
                        p.JOINT_REVOLUTE,p.JOINT_REVOLUTE,p.JOINT_REVOLUTE,p.JOINT_FIXED,p.JOINT_FIXED,p.JOINT_FIXED,p.JOINT_FIXED,p.JOINT_FIXED,p.JOINT_FIXED,p.JOINT_FIXED,
                        p.JOINT_FIXED],
        linkJointAxis=[[0,0,-1],[0,0,-1],[0,0,-1],[0,0,-1],[0,0,-1],[0,0,-1],[0,0,-1],[0,0,-1],[0,0,-1],[0,0,-1],
                        [0,0,-1],[0,0,-1],[0,0,-1],[0,0,-1],[0,0,-1],[0,0,-1],[0,0,-1],[0,0,-1],[0,0,-1],[0,0,-1],
                        [0,0,-1],[0,0,-1],[0,0,-1],[0,0,-1],[0,0,-1],[0,0,-1],[0,0,-1],[0,0,-1],[0,0,-1],[0,0,-1],
                        [0,0,-1]])
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
