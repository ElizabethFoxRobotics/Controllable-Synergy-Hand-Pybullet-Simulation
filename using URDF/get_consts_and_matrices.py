import math
import numpy as np

def getTransmissionMatrix(js,ps):
    #js is joint angles, in radians, assumed 1 finger
    sall = []
    for i in range(int(len(js)/3)):
        ar1 = js[3*i]
        ar2 = js[3*i+1]
        ar3 = js[3*i+2]
        R,s0 = getFlexureTransmissionMatrix(ar1,ar2,ar3,ps)
        sall.append(s0)
        if i == 0:
            Rt = R
            s0t = s0
        else:
            Rt = np.concatenate([Rt,R])
            s0t = s0t + s0
    return Rt, s0t, sall

def getFlexureTransmissionMatrix(ar1,ar2,ar3,pres):
    #ar1-3 are virtual revolute joints approximating flexure motion, in radians
    #ar1 is at base of flexure, ar3 is at top of flexure, and ar2 is between connected by equal virtual arms; all are internal
    lf = 0.01 #m; length of virtual arms of flexure
    Lp = (0.41642525+.125)*0.0254 #length of link before chamfer
    La = .33299575*.0254 #length until tendon along chamfer
    LL = math.sqrt(Lp**2+La**2-2*Lp*La*math.cos(math.radians(150.5))) #distance base of link to center of tendon on chamfer
    aa = math.asin(La*math.sin(math.radians(150.5))/LL) #angle between LL and link face

    b = math.pi/2-ar2/2 #angle from flexure line to virtual links of ar2
    lenf = lf*math.sqrt(2-2*math.cos(ar2)) #flexure length
    r = math.sqrt(lenf**2+LL**2-2*lenf*LL*math.cos(ar3+aa-b)) #length ar1 to top tendon
    a1 = math.asin(LL/r*math.sin(ar3+aa-b)) #angle from flexure line to r
    a2 = ar1-b-a1+aa #angle from r to top of link (Lp)
    e = math.sqrt(LL**2+r**2-2*LL*r*math.cos(a2)) #length of tendon

    de_dar1 = LL*r*math.sin(a2)/e

    dlenf_dar2 = lf**2*math.sin(ar2)/lenf
    dr_dar2 = (.5*LL*lenf*math.sin(ar3+aa-b)-LL*dlenf_dar2*math.cos(ar3+aa-b)+lenf*dlenf_dar2)/r
    if math.sqrt(r**2-LL**2*math.sin(ar3+aa-b)**2) == 0:
        import pdb; pdb.set_trace()
    da1_dar2 = (.5*LL*math.cos(ar3+aa-b) -LL*dr_dar2*math.sin(ar3+aa-b)/r)/math.sqrt(r**2-LL**2*math.sin(ar3+aa-b)**2)
    da2_dar2 = .5 - da1_dar2
    de_dar2 = (LL*r*da2_dar2*math.sin(a2)-LL*math.cos(a2)*dr_dar2+r*dr_dar2)/e

    dr_dar3 = LL*lenf*math.sin(ar3+aa-b)/r
    da2_dar3 = -(LL*math.cos(ar3+aa-b) -LL*dr_dar3*math.sin(ar3+aa-b)/r)/math.sqrt(r**2-LL**2*math.sin(ar3+aa-b)**2)
    de_dar3 = (LL*r*da2_dar3*math.sin(a2)-LL*math.cos(a2)*dr_dar3+r*dr_dar3)/e

    R = np.array([de_dar1, de_dar2, de_dar3]) #transmission matrix
    return R, e

def getStiffnessMatrix(js,ps, fs, ss,stiffnessDegree):
    #assuming js is length 3*ps and is in radians

    ks = np.zeros((len(ps)*3,len(ps)*3))
    zs = np.zeros(len(ps)*3)
    for i in range(len(ps)):
        ar1 = js[3*i]*180/math.pi
        ar2 = js[3*i+1]*180/math.pi
        ar3 = js[3*i+2]*180/math.pi

        ks[3*i][3*i+1] = 0
        ks[3*i][3*i+2] = 0

        ks[3*i+1][3*i] = 0
        ks[3*i+1][3*i+2] = 0

        ks[3*i+2][3*i] = 0
        ks[3*i+2][3*i+1] = 0

        if stiffnessDegree == '0':
            ks[3*i][3*i] = 1.3958#3.6253 - 1.8729*10**-3*ps[i] -441.12*ss[i] +.56036*ps[i]*ss[i]  +2.2398*10**4*ss[i]**2 -9.6871*ps[i]*ss[i]**2 -4.2291*10**5*ss[i]**3
            ks[3*i+1][3*i+1] = -9.1331#-25.787 + .050594*ps[i] +2.6460*10**3*ss[i] -7.5268*ps[i]*ss[i] -1.0163*10**5*ss[i]**2 +198.29*ps[i]*ss[i]**2 +1.1199*10**6*ss[i]**3;
            ks[3*i+2][3*i+2] = 1.3045#3.0021 - 2.9592*10**-3*ps[i] -326.*ss[i] +.70316*ps[i]*ss[i] +1.4488*10**4*ss[i]**2 -13.312*ps[i]*ss[i]**2 -2.5028*10**5*ss[i]**3
        elif stiffnessDegree == '1':
            ks[3*i][3*i] = 1.1428+0.0020376*ps[i]
            ks[3*i+1][3*i+1] = -8.9193-0.0014419*ps[i]
            ks[3*i+2][3*i+2] = 1.0243+0.0023864*ps[i]
        elif stiffnessDegree == '2':
            ks[3*i][3*i] = 1.4656+0.0038458*ps[i]-41.7673*ss[i]
            ks[3*i+1][3*i+1] = -10.7629-0.012006*ps[i]-241.0364*ss[i]
            ks[3*i+2][3*i+2] =  1.3206+0.0040215*ps[i]-38.1848*ss[i]
        elif stiffnessDegree == '3':
            ks[3*i][3*i] = 2.2159-0.0006959*ps[i]-123.9711*ss[i]+0.3357*ps[i]*ss[i]+1152.2*ss[i]**2
            ks[3*i+1][3*i+1] = -16.9136+0.017012*ps[i]+924.3115*ss[i]-2.1125*ps[i]*ss[i]-11710*ss[i]**2
            ks[3*i+2][3*i+2] = 2.0103-0.0008581*ps[i]-110.7704*ss[i]+0.3555*ps[i]*ss[i]+834.2210*ss[i]**2
        else:

            ks[3*i][3*i] = 3.6253 - 1.8729*10**-3*ps[i] -441.12*ss[i] +.56036*ps[i]*ss[i]  +2.2398*10**4*ss[i]**2 -9.6871*ps[i]*ss[i]**2 -4.2291*10**5*ss[i]**3
            ks[3*i+1][3*i+1] =-25.787 + .050594*ps[i] +2.6460*10**3*ss[i] -7.5268*ps[i]*ss[i] -1.0163*10**5*ss[i]**2 +198.29*ps[i]*ss[i]**2 +1.1199*10**6*ss[i]**3
            ks[3*i+2][3*i+2] =3.0021 - 2.9592*10**-3*ps[i] -326.*ss[i] +.70316*ps[i]*ss[i] +1.4488*10**4*ss[i]**2 -13.312*ps[i]*ss[i]**2 -2.5028*10**5*ss[i]**3

    return ks


def getq0(q,ps):
    q0 = q.copy()
    for i in range(len(ps)):
        q0[3*i] = -2.025*10**-8*ps[i]**3 +7.862*10**-6*ps[i]**2 +.0001574*ps[i] +2.576
        q0[3*i+1] = 2.694*10**-9*ps[i]**3 -1.169*10**-6*ps[i]**2 +5.571*10**-5*ps[i] +1.114
        q0[3*i+2] = -1.672*10**-8*ps[i]**3 +6.104*10**-6*ps[i]**2 +.0002662*ps[i] +2.636
    return q0

def getZeroPos(ps):
    jA = []
    #[-math.pi/2 + 0.5857, math.pi - 1.1189, math.pi/2 - 2.6197, -math.pi/2 + 0.5857, math.pi - 1.1189, math.pi/2 - 2.6197,-math.pi/2 + 0.5857, math.pi - 1.1189, math.pi/2 - 2.6197]

    for i in range(len(ps)):
        jA.append(-(180-(1.16*10**-6*ps[i]**3 -0.0004504*ps[i]**2 -0.009018*ps[i]+32.43))*math.pi/180.+math.pi/2)
        jA.append(-(2.466*10**-6*ps[i]**2 -0.004527*ps[i] +63.94)*math.pi/180.+math.pi)
        jA.append(-(-8.14*10**-5*ps[i]**2 +0.06316*ps[i]+150.3)*math.pi/180.+math.pi/2)

    #print('jA', jA)
    return jA
