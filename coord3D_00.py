'''Changes:
        -first implementation in 3D'''
import math
import numpy as np
import coord_class_3D as cc
import time

poseList=list()#global list of all found poses for searched point
poseIndex=0#index of current pose in poseList
currentPose=cc.Pose(cc.Point(0,0,0),cc.Point(0,0,0),cc.Point(0,0,0))
path=list()#list of poses that represent a path (used in findPointsInList())

checklistA=[0,0,0,False,False] #checklist for next_angle(): iteration,starting angle,direction,zero,ninety
                               #if direction==0, go left-right, if direction==-1 go right, if direction==1 go left
checklistB=[0,0,0,False,False]

checklistC=[0,0,0,False,False]

def quadrant(c):
    """Determines the quadrant of given Point in it's base."""
    if c.x>=0:
        if c.z>=0:
            return 1
        elif c.z<0:
            return 4
    elif c.x<0:
        if c.z>=0:
            return 2
        elif c.z<0:
            return 3

def dot_gen(alpha):
    '''Generates points based on given angle, using a formula that can be found in the diary.'''
    if alpha==0:
            x=cc.l*math.cos(((180+alpha)/2)*math.pi/180)
            z=cc.l*math.sin(((180+alpha)/2)*math.pi/180)
    else:
        r=math.sqrt(2*(180*6/math.pi/alpha)**2 *(1-math.cos(alpha*math.pi/180)))
        x=r*math.cos(((180+alpha)/2)*math.pi/180)
        z=r*math.sin(((180+alpha)/2)*math.pi/180)

    
    return x,z

def next_angle(checklist,step=cc.step):
    if checklist[0]==0:
        if checklist[1]>=90:
            checklist[0]=checklist[0]+1
            checklist[1]=90
            checklist[2]=1
            checklist[3]=False
            checklist[4]=True

            return False, 90
        elif checklist[1]==0:
            checklist[0]=checklist[0]+1
            checklist[2]=-1
            checklist[3]=True
            checklist[4]=False

            return False, 0
        else:
            checklist[0]=checklist[0]+1
            checklist[2]=0
            checklist[3]=False
            checklist[4]=False

            return False, checklist[1]

    else:
        if checklist[2]==-1:#go right
            add = step * checklist[0]
            result= checklist[1] + add
            if result>=90:
                result=90
                checklist[4]=True
            checklist[0]=checklist[0]+1

            if checklist[3]==True and checklist[4]==True:#if both borders have been hit
                return True, result
            else:
                return False, result
        
        elif checklist[2]==1:#go left
            add = step * checklist[0]
            result= checklist[1] - add
            if result<=0:
                result=0
                checklist[3]=True
            checklist[0]=checklist[0]+1

            if checklist[3]==True and checklist[4]==True:#if both borders have been hit
                return True, result
            else:
                return False, result

        elif checklist[2]==0:#go left-right
            add = step * (checklist[0]//2 + 1) * (int)(math.pow(-1,checklist[0]))
            result= checklist[1] + add
            if result<=0:
                result=0
                checklist[3]=True
            elif result>=90:
                result=90
                checklist[4]=True
            
            checklist[0]=checklist[0]+1

            if checklist[3]!=True and checklist[4]!=True:
                return False,result
            elif checklist[3]==True:
                checklist[1]=checklist[1] + (int)(math.fabs(add))
                checklist[0]=1
                checklist[2]=-1

                return False,result
            elif checklist[4]==True:
                checklist[1]=checklist[1] - (int)(math.fabs(add))
                checklist[0]=1
                checklist[2]=1

                return False, result


def get_vecs(prevPoint,currPoint,c):
    """Returns a new Base of vectors (IN CURRENT BASE, DOESN'T HAVE TO BE i,j,k )"""

    if math.fabs(currPoint.alpha)==90:#special case when full turn(by 90 deg)
        if currPoint.alpha<0:
            qv=cc.i#q vector
            try:
                kc=c.z/c.x#slope of line OC
                if kc>0:
                    pv=cc.k
                elif kc<=0:
                    pv=-cc.k
            except ZeroDivisionError:
                pv=cc.k
        elif currPoint.alpha>0:
            qv=-cc.i#q vector
            try:
                kc=c.z/c.x#slope of line OC
                if kc>0:
                    pv=-cc.k
                elif kc<=0:
                    pv=cc.k
            except ZeroDivisionError:
                pv=cc.k

    elif currPoint.alpha==0:#special case when no turn(0 deg)
        pv=cc.i
        qv=cc.k
    
    else:#all other cases of alpha
        kp=math.tan(currPoint.alpha * math.pi / 180)
        p=cc.Linear(currPoint,kp)
        kq=-1/math.tan(currPoint.alpha * math.pi / 180)
        q=cc.Linear(currPoint,kq)

        #getting point Q (line AQ makes vector qv) 
        #two x coordinates,solving linear equation(pre-solved,check folder for photo)
        qx1=(currPoint.x * (kq**2 + 1) + math.sqrt(kq**2 + 1)) / (kq**2 + 1)
        qx2=(currPoint.x * (kq**2 + 1) - math.sqrt(kq**2 + 1)) / (kq**2 + 1)

        Q1=cc.Point(qx1,0,q.get_Z(qx1))
        Q2=cc.Point(qx2,0,q.get_Z(qx2))

        #The point furthest away from previous point is point Q

        if prevPoint.dist(Q1)>=prevPoint.dist(Q2):
            Q=Q1
        else:
            Q=Q2

        #getting vector qv
        qv=np.array([(Q.x-currPoint.x),0,(Q.z-currPoint.z)])

        #getting point P (line AP makes vector pv) 
        #two x coordinates,solving linear equation
        px1=(currPoint.x * (kp**2 + 1) + math.sqrt(kp**2 + 1)) / (kp**2 + 1)
        px2=(currPoint.x * (kp**2 + 1) - math.sqrt(kp**2 + 1)) / (kp**2 + 1)

        P1=cc.Point(px1,0,p.get_Z(px1))
        P2=cc.Point(px2,0,p.get_Z(px2))

        #The point closest to C is point P

        if c.dist(P1)<=c.dist(P2):
            P=P1
        else:
            P=P2

        #getting vector pv
        pv=np.array([(P.x-currPoint.x),0,(P.z-currPoint.z)])

    #getting rv - the third vector in the base, which is perpendicular to pv and qv
    rv=np.cross(qv,pv)

    newBase=cc.Base(pv,rv,qv)

    return newBase

def get_sign(X,Y):
    """Returns the sign of alpha of point Y.Previous point needed(ex. (A,B),(B,C)).

    -1 for right, 1 for left"""
    if not((X.alpha + 90)%90==0 and (X.alpha + 90)%180!=0):
        kX=math.tan(math.radians(X.alpha + 90))
        try:
            if math.fabs(Y.x-X.x)<=3.6739403974420594e-16:
                raise ZeroDivisionError
            kY=(Y.z-X.z)/(Y.x-X.x)
            if kY>kX:
                return 1
            elif kY<=kX:
                return -1
        except ZeroDivisionError:
            if kX<0:
                return -1
            elif kX>0:
                return 1
            else:
                if Y.z>=0:
                    return 1
                else:
                    return -1
    else:
        if X.z>=0:
            if Y.x>=X.x:
                return -1
            elif Y.x<X.x:
                return 1
        elif X.z<0:
            if Y.x>X.x:
                return 1
            elif Y.x<=X.x:
                return -1

def from_B(c,tempB,A,once=False):

    #convert point C to point in base z,w,v
    #using vector BC
    bc=np.array([c.x-tempB.x,c.y-tempB.y,c.z-tempB.z])        
    #making transition matrix
    T=np.array(([tempB.ownBase.x[0],tempB.ownBase.y[0],tempB.ownBase.z[0]],[tempB.ownBase.x[1],tempB.ownBase.y[1],tempB.ownBase.z[1]],[tempB.ownBase.x[2],tempB.ownBase.y[2],tempB.ownBase.z[2]]))
    #vector BC in base z,w,v is gotten from equation Tx'=x where x' is x in new base
    new_bc=np.linalg.solve(T,bc)
    #getting point C in new base from its new vector
    newC=cc.Point(new_bc[0],new_bc[1],new_bc[2])

    #determening angle phi of point C
    try:
        if math.fabs(newC.x)<=3.6739403974420594e-16:
            raise ZeroDivisionError
        c.phi=math.degrees(math.atan2(newC.y,newC.x))
    except ZeroDivisionError:
        if newC.y>=0:
            c.phi=90
        else:
            c.phi=-90
    rotC=newC.rotateZ(-c.phi)     #Y COORDINATE SHOULD NOW BE 0 in tempB's base, rotating around regular z axis

    #getting alpha3 from tan(beta3)=y/x
    try:
        if math.fabs(rotC.x)<=3.6739403974420594e-16:
            raise ZeroDivisionError
        beta3=np.arctan(rotC.z/rotC.x)
        beta3=180*beta3/np.pi
        alpha3=180-2*beta3
    except ZeroDivisionError:
        alpha3=0

    #checking if alpha3 isn't suitable:
    if alpha3<0 or alpha3>90:
        return False,cc.Point(0,0,0)#if false is returned, increment call_numB by 1

    else:
        test_x,test_z=dot_gen(-alpha3)#negative alpha3 for right(>) side
        if math.fabs(test_x-rotC.x)<0.1 and math.fabs(test_z-rotC.z)<0.1:# error margin of 1 mm
            #alpha3 is good
            B=cc.Point(tempB.x,tempB.y,tempB.z,tempB.alpha,tempB.phi,tempB.ownBase)#done finding point B
            C=cc.Point(c.x,c.y,c.z,alpha3,c.phi)#final point C with positive alpha

            '''DETERMINING THE SIGN OF ALPHA3'''
            signC=cc.Point(rotC.x,rotC.y,rotC.z,C.alpha,C.phi)#a copy of rotC used to determine the sign of alpha3
            signB=cc.Point(B.x,B.y,B.z,B.alpha,B.phi,B.ownBase)#a copy of B used to determine the sign of alpha3

            #transition matrix of i,j,k to point A base
            D=np.array(([A.ownBase.x[0],A.ownBase.y[0],A.ownBase.z[0]],[A.ownBase.x[1],A.ownBase.y[1],A.ownBase.z[1]],[A.ownBase.x[2],A.ownBase.y[2],A.ownBase.z[2]]))

            translatedSignB=np.array([signB.x-A.x,signB.y-A.y,signB.z-A.z])

            #turning translatedSignB to p,q,r base
            translatedSignBInPQR=np.linalg.solve(D,translatedSignB)

            cInIJK=np.matmul(T,np.array([signC.x,signC.y,signC.z]))
            cInPQR=np.linalg.solve(D,cInIJK)

            #translating so point A is the new origin for point cInPQR
            cInPQR[0]+=translatedSignBInPQR[0]
            cInPQR[1]+=translatedSignBInPQR[1]
            cInPQR[2]+=translatedSignBInPQR[2]

            try:
                signPhi=math.degrees(math.atan2(translatedSignBInPQR[1],translatedSignBInPQR[0]))
            except ZeroDivisionError:
                signPhi=90

            rotSignB=cc.Point(translatedSignBInPQR[0],translatedSignBInPQR[1],translatedSignBInPQR[2]).rotateAxis(-signPhi,cc.norm.z)
            rotSignC=cc.Point(cInPQR[0],cInPQR[1],cInPQR[2]).rotateAxis(-signPhi,cc.norm.z)
        
            C.alpha=alpha3*get_sign(cc.Point(rotSignB[0],rotSignB[1],rotSignB[2],tempB.alpha),cc.Point(rotSignC[0],rotSignC[1],rotSignC[2]))
            '''Finished determining sign of alpha3'''

            #Reducing values to 0 if coordinate<=1e-6:
            A.reduce()
            B.reduce()
            C.reduce()

            #adding to poseList
            poseList.append(cc.Pose(A,B,C))

            if once==True:
                return True,C
            else:
                return False,cc.Point(0,0,0)#if false is returned, increment call_numB by 1
        else:
            return False,cc.Point(0,0,0)#if false is returned, increment call_numB by 1

def from_A(c,A,step=cc.step,once=False):
    '''Starts search for new c from given Point A, it's base vectors pqBase and the step for the angle search.
    
    Returns Points B and C'''

    global checklistB

    #convert point C to point in base p,q
    #using vector AC
    ac=np.array([c.x-A.x,c.y-A.y,c.z-A.z])
    #making transition matrix
    T=np.array(([A.ownBase.x[0],A.ownBase.y[0],A.ownBase.z[0]],[A.ownBase.x[1],A.ownBase.y[1],A.ownBase.z[1]],[A.ownBase.x[2],A.ownBase.y[2],A.ownBase.z[2]]))
    #vector BC in base p,q is gotten from equation Tx'=x where x' is x in new base
    new_ac=np.linalg.solve(T,ac)
    #getting point C in new base from its new vector
    newC=cc.Point(new_ac[0],new_ac[1],new_ac[2])

    #determening angle phi of point C
    try:
        if math.fabs(newC.x)<=3.6739403974420594e-16:
            raise ZeroDivisionError
        bPhi=math.degrees(math.atan2(newC.y,newC.x))
    except ZeroDivisionError:
        if newC.y>=0:
            bPhi=90
        else:
            bPhi=-90

    rotC=newC.rotateZ(-bPhi)     #Y COORDINATE SHOULD NOW BE 0 in A's base,and we're roatating round the A's z axis, but because newC is already in A's base, we dont use rotateAxis around A.ownBase.z

    try:
        if math.fabs(rotC.x)<=3.6739403974420594e-16:
            raise ZeroDivisionError
        start_angleB=180 - 2*(math.degrees(math.atan(rotC.z/rotC.x)))
    except ZeroDivisionError:
        start_angleB=0
    
    checklistB=[0,start_angleB,0,False,False]##initializing checlistB
    endB, angle=next_angle(checklistB,step)

    while(endB!=True):
        alpha2=angle##setting alpha2 as angle from next_angle(),then calling next_angle again
        endB, angle=next_angle(checklistB,step)

        xb,zb=dot_gen(-alpha2)#negative alpha2 for right(>) side
        tempB=cc.Point(xb,0,zb,alpha2,bPhi)#Point B in p,q base 

        #new B-base vectors in base p,q,r
        if tempB.alpha>0:#sign change because get_vecs requires negative alpha
            tempB.alpha=-tempB.alpha
        
        zwBase=get_vecs(cc.Point(0,0,0),tempB,cc.Point(rotC.x,rotC.y,rotC.z))#A is 0,0,0 in its own base

        '''DETERMINING THE SIGN OF ALPHA2'''
        signB=cc.Point(tempB.x,tempB.y,tempB.z,tempB.alpha,tempB.phi)#a copy of tempB(prerotation) used to determine the sign of alpha2
        
        #turning signB to i,j,k base
        signB.x=tempB.x*A.ownBase.x[0] + tempB.y*A.ownBase.y[0] + tempB.z*A.ownBase.z[0]
        signB.y=tempB.x*A.ownBase.x[1] + tempB.y*A.ownBase.y[1] + tempB.z*A.ownBase.z[1]
        signB.z=tempB.x*A.ownBase.x[2] + tempB.y*A.ownBase.y[2] + tempB.z*A.ownBase.z[2]

        #translating so point A is the new origin for point signB
        signB.x+=A.x
        signB.y+=A.y
        signB.z+=A.z

        try:
            if math.fabs(signB.x)<=3.6739403974420594e-16:
                raise ZeroDivisionError
            signPhi=math.degrees(math.atan2(signB.y,signB.x))
        except ZeroDivisionError:
            signPhi=90

        rotSignB=signB.rotateAxis(-signPhi,cc.norm.z)
        rotSignA=A.rotateAxis(-signPhi,cc.norm.z)
        
        tempB.alpha=alpha2*get_sign(cc.Point(rotSignA[0],rotSignA[1],rotSignA[2],A.alpha),cc.Point(rotSignB[0],rotSignB[1],rotSignB[2])) #change to right sign from previous change to negative^^^
        '''Finished determining sign of alpha2'''

        #rotating point B back where it should be
        rotB=tempB.rotateZ(bPhi)
        tempB.x=rotB.x
        tempB.y=rotB.y
        tempB.z=rotB.z

        #rotating zwBase back where it should be
        zwRot=zwBase.rotateAxis(bPhi,cc.norm.z)
        tempB.ownBase=zwRot
        
        #turning B to i,j,k base
        tempB.x=rotB.x*A.ownBase.x[0] + rotB.y*A.ownBase.y[0] + rotB.z*A.ownBase.z[0]
        tempB.y=rotB.x*A.ownBase.x[1] + rotB.y*A.ownBase.y[1] + rotB.z*A.ownBase.z[1]
        tempB.z=rotB.x*A.ownBase.x[2] + rotB.y*A.ownBase.y[2] + rotB.z*A.ownBase.z[2]

        #translating so point A is the new origin for point B
        tempB.x+=A.x
        tempB.y+=A.y
        tempB.z+=A.z

        #checking if chosen point is OK
        #1) is the distance between B and C in the range minimal r and l([min(r),l])
        Bdist=c.dist(tempB)

        if not (Bdist>=cc.mindist and Bdist<=cc.l):
            continue

        #translating zwBase from base p,q,r into base i,j,k
        newZX=(zwBase.x[0] * A.ownBase.x[0] + zwBase.x[1] * A.ownBase.y[0] + zwBase.x[2] * A.ownBase.z[0])
        newZY=(zwBase.x[0] * A.ownBase.x[1] + zwBase.x[1] * A.ownBase.y[1] + zwBase.x[2] * A.ownBase.z[1])
        newZZ=(zwBase.x[0] * A.ownBase.x[2] + zwBase.x[1] * A.ownBase.y[2] + zwBase.x[2] * A.ownBase.z[2])
        
        newWX=(zwBase.y[0] * A.ownBase.x[0] + zwBase.y[1] * A.ownBase.y[0] + zwBase.y[2] * A.ownBase.z[0])
        newWY=(zwBase.y[0] * A.ownBase.x[1] + zwBase.y[1] * A.ownBase.y[1] + zwBase.y[2] * A.ownBase.z[1])
        newWZ=(zwBase.y[0] * A.ownBase.x[2] + zwBase.y[1] * A.ownBase.y[2] + zwBase.y[2] * A.ownBase.z[2])

        newVX=(zwBase.z[0] * A.ownBase.x[0] + zwBase.z[1] * A.ownBase.y[0] + zwBase.z[2] * A.ownBase.z[0])
        newVY=(zwBase.z[0] * A.ownBase.x[1] + zwBase.z[1] * A.ownBase.y[1] + zwBase.z[2] * A.ownBase.z[1])
        newVZ=(zwBase.z[0] * A.ownBase.x[2] + zwBase.z[1] * A.ownBase.y[2] + zwBase.z[2] * A.ownBase.z[2])

        new_zv=np.array([newZX,newZY,newZZ])
        new_wv=np.array([newWX,newWY,newWZ])
        new_vv=np.array([newVX,newVY,newVZ])

        tempB.ownBase=cc.Base(new_zv,new_wv,new_vv)

        good,foundPoint=from_B(c,tempB,A,once)

        if good==False or once==False:
            continue
        elif good==True:
            return True,tempB,foundPoint

    return False,cc.Point(0,0,0),cc.Point(0,0,0)

def first_search(c,once=False,step=cc.step):
    """The main search function, returns a list of possible Poses
    
    If once==True, returns only one Pose encapsulated in a list.
    Default angle step is cc.step=0.5deg"""
    global poseList
    poseList=list()

    #1 Distance check
    if cc.root.dist(c)>3*cc.l:
        return poseList
    

    #determening angle phi of point C
    try:
        if math.fabs(c.x)<=3.67e-16:
            raise ZeroDivisionError
        aPhi=math.degrees(math.atan2(c.y,c.x))
    except ZeroDivisionError:
        if c.y>=0:
            aPhi=90
        else:
            aPhi=-90

    rotC=c.rotateAxis(-aPhi,cc.norm.z)     #Y COORDINATE SHOULD NOW BE 0

    #2 Quadrant check
    quad=0
    quad=quadrant(cc.Point(rotC[0],rotC[1],rotC[2]))
    
    #3 finding point A
    global checklistA

    try:
        if math.fabs(rotC[0])<=3.6739403974420594e-16:
            raise ZeroDivisionError
        start_angleA=180 - 2*(math.degrees(math.atan(rotC[2]/rotC[0])))
    except ZeroDivisionError:
        start_angleA=0
    
    checklistA=[0,start_angleA,0,False,False]##initializing checlistA
    endA, angle=next_angle(checklistA,step)

    while(endA!=True):
        alpha1=angle#setting alpha1 as angle from next_angle(),then calling next_angle again
        endA, angle=next_angle(checklistA,step)

        xa,za=dot_gen(-alpha1)#negative alpha1 for right(>) side

        #deciding which side is A on(rigth or left,by quadrants)
        if quad==2 or quad==3:#if C is to the left, make x negative
            xa=-xa
        if quad==1 or quad==4:
            alpha1=-alpha1#turning to the right

        A=cc.Point(xa,0,za,alpha1,aPhi)

        #Finding the base vectors of A (PRE-ROTATION)
        pqBase=get_vecs(cc.root,A,c)
        
        #rotating A to where it should be
        rotA=A.rotateAxis(aPhi,cc.norm.z)
        A.x=rotA[0]
        A.y=rotA[1]
        A.z=rotA[2]

        #rotating A's base to where it should be
        pqRot=pqBase.rotateAxis(aPhi,cc.norm.z)
        A.ownBase=pqRot

        good,foundB,foundC=from_A(c,A,step,once)

        if good==False or once==False:
            continue
        else:
            break

    return poseList    

def nextC(c,once=False,step=cc.step):
    '''Finds the requested next point, with minimal movement.
    
    Search first starts from current point B, if no possible Cs exist, than form A, further from the start'''
    global poseList
    global poseIndex

    #getting all of the Points from the current Pose
    currentA=poseList[poseIndex].A
    currentB=poseList[poseIndex].B
    currentC=poseList[poseIndex].C

    #clearing poseList and settinf poseIndex to 0, because we don't need old poses anymore
    poseList.clear()
    poseIndex=0

    #trying from currentB first
    goodC,newC=from_B(c,currentB,currentA,once)

    if goodC==True:
        return True
    else:
        #trying from currentA second
        goodB,newB,newC=from_A(c,currentA,step,once)

        if goodB==True:
            return True
        else:
            #search from the start
            first_search(c,once,step)
    
    if poseList!=[]:
        return True
    else:
        return False

def findPointsInList(pointList,once=False,step=cc.step):
    global currentPose
    global poseList
    global path
    for e in pointList:
        if poseList==[]:
            first_search(pointList[0],once,step)
        else:
            nextC(e,once,step)
        #currentPose=poseList[0]
        #path.append(poseList[0])

def angleDifference(pose1,pose2):
    return -(pose1.A.alpha-pose2.A.alpha), -(pose1.B.alpha-pose2.B.alpha), -(pose1.C.alpha-pose2.C.alpha)

if __name__=="__main__":
    a=input("Input x coordinate:")
    b=input("Input y coordinate:")
    c=input("Input z coordinate:")
    C=cc.Point(float(a),float(b),float(c))
    step=cc.step
    start=time.time()
    poses=first_search(C,True)
    end=time.time()
    if poses!=[]:
        for i in range(len(poses)):
            poses[i].printp()
        poses[0].plot()
        #poses[-1].plot()
        print("Total poses",len(poses))
        print("time(s):",end-start)
    else:
        print("No possible alphas for this point")
    '''
    pointList=list()
    for i in range(40):
        for j in range(40):
            for k in range(40):
                pointList.append(cc.Point(i*0.25 + 5,j*0.25 + 5,k*0.25 + 5))
    
    start=time.time()
    print("start")
    findPointsInList(pointList,True)
    end=time.time()
    print("end\ntotal time(s):",end-start)

    print(len(poseList))'''