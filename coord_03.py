'''Changes:
        -new function to replace angle_list() and sorter()'''
import math
import numpy as np
import coord_class as cc
import time

poseList=list()#global list of all found poses for searched point
poseIndex=0#index of current pose in poseList
currentPose=cc.Pose(cc.Point(0,0),cc.Point(0,0),cc.Point(0,0))
path=list()#list of poses that represent a path (used in findPointsInList())

checklistA=[0,0,0,False,False] #checklist for next_angle(): iteration,starting angle,direction,zero,ninety
                               #if direction==0, go left-right, if direction==-1 go right, if direction==1 go left
checklistB=[0,0,0,False,False]

checklistC=[0,0,0,False,False]

def quadrant(c):
    """Determines the quadrant of given Point in it's base."""
    if c.x>=0:
        if c.y>=0:
            return 1
        elif c.y<0:
            return 4
    elif c.x<0:
        if c.y>=0:
            return 2
        elif c.y<0:
            return 3

def dot_gen(alpha):
    '''Generates points based on given angle, using a formula that can be found in the diary.'''
    if alpha==0:
            x=cc.l*math.cos(((180+alpha)/2)*math.pi/180)
            y=cc.l*math.sin(((180+alpha)/2)*math.pi/180)
    else:
        r=math.sqrt(2*(180*6/math.pi/alpha)**2 *(1-math.cos(alpha*math.pi/180)))
        x=r*math.cos(((180+alpha)/2)*math.pi/180)
        y=r*math.sin(((180+alpha)/2)*math.pi/180)

    
    return x,y

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
    """Returns a new Base of vectors (IN CURRENT BASE, DOESN'T HAVE TO BE i,j )"""

    if math.fabs(currPoint.alpha)==90:#special case when full turn(by 90 deg)
        if currPoint.alpha<0:
            qv=cc.i#q vector
            try:
                kc=c.y/c.x#slope of line OC
                if kc>0:
                    pv=cc.j
                elif kc<=0:
                    pv=-cc.j
            except ZeroDivisionError:
                pv=cc.j
        elif currPoint.alpha>0:
            qv=-cc.i#q vector
            try:
                kc=c.y/c.x#slope of line OC
                if kc>0:
                    pv=-cc.j
                elif kc<=0:
                    pv=cc.j
            except ZeroDivisionError:
                pv=cc.j

    elif currPoint.alpha==0:#special case when no turn(0 deg)
        pv=cc.i
        qv=cc.j
    
    else:#all other cases of alpha
        kp=math.tan(currPoint.alpha * math.pi / 180)
        p=cc.Linear(currPoint,kp)
        kq=-1/math.tan(currPoint.alpha * math.pi / 180)
        q=cc.Linear(currPoint,kq)

        #getting point Q (line AQ makes vector qv) 
        #two x coordinates,solving linear equation(pre-solved,check folder for photo)
        qx1=(currPoint.x * (kq**2 + 1) + math.sqrt(kq**2 + 1)) / (kq**2 + 1)
        qx2=(currPoint.x * (kq**2 + 1) - math.sqrt(kq**2 + 1)) / (kq**2 + 1)

        Q1=cc.Point(qx1,q.get_Y(qx1))
        Q2=cc.Point(qx2,q.get_Y(qx2))

        #The point furthest away from previous point is point Q

        if prevPoint.dist(Q1)>=prevPoint.dist(Q2):
            Q=Q1
        else:
            Q=Q2

        #getting vector qv
        qv=np.array([(Q.x-currPoint.x),(Q.y-currPoint.y)])

        #getting point P (line AP makes vector pv) 
        #two x coordinates,solving linear equation
        px1=(currPoint.x * (kp**2 + 1) + math.sqrt(kp**2 + 1)) / (kp**2 + 1)
        px2=(currPoint.x * (kp**2 + 1) - math.sqrt(kp**2 + 1)) / (kp**2 + 1)

        P1=cc.Point(px1,p.get_Y(px1))
        P2=cc.Point(px2,p.get_Y(px2))

        #The point closest to C is point P

        if c.dist(P1)<=c.dist(P2):
            P=P1
        else:
            P=P2

        #getting vector pv
        pv=np.array([(P.x-currPoint.x),(P.y-currPoint.y)])

    newBase=cc.Base(pv,qv)

    return newBase

def get_sign(X,Y,Z):
    """Returns the sign of alpha of point Z.Previous two points needed(ex. (O,A,B),(A,B,C)).

    -1 for right, 1 for left"""
    try:
        kxy=(Y.y-X.y)/(Y.x-X.x)
        try:
            kyz=(Z.y-Y.y)/(Z.x-Y.x)
            if kyz>kxy:
                return 1
            elif kyz<=kxy:
                return -1
        except ZeroDivisionError:
            if kxy<0:
                return -1
            elif kxy>0:
                return 1
            else:
                if Z.y>=0:
                    return 1
                else:
                    return -1
    except ZeroDivisionError:
        if Y.y>=0:
            if Z.x>=Y.x:
                return -1
            elif Z.x<Y.x:
                return 1
        elif Y.y<0:
            if Z.x>Y.x:
                return 1
            elif Z.x<=Y.x:
                return -1

def from_B(c,tempB,A,once=False):
    #convert point C to point in base z,w
    #using vector BC
    bc=np.array([c.x-tempB.x,c.y-tempB.y])
    #making transition matrix
    T=np.array(([tempB.ownBase.x[0],tempB.ownBase.y[0]],[tempB.ownBase.x[1],tempB.ownBase.y[1]]))
    #vector BC in base z,w is gotten from equation Tx'=x where x' is x in new base
    new_bc=np.linalg.solve(T,bc)
    #getting point C in new base from its new vector
    newC=cc.Point(new_bc[0],new_bc[1])
    #getting alpha3 from tan(beta3)=y/x
    try:
        beta3=np.arctan(newC.y/newC.x)
        beta3=180*beta3/np.pi
        alpha3=180-2*beta3
    except ZeroDivisionError:
        alpha3=0

    #checking if alpha3 isn't suitable:
    if alpha3<0 or alpha3>90:
        return False,cc.Point(0,0,0)#if false is returned, increment call_numB by 1

    else:
        test_x,test_y=dot_gen(-alpha3)#negative alpha3 for right(>) side
        if math.fabs(test_x-newC.x)<0.1 and math.fabs(test_y-newC.y)<0.1:# error margin of 1 mm
            #alpha3 is good
            B=cc.Point(tempB.x,tempB.y,tempB.alpha,tempB.ownBase)#done finding point B
            C=cc.Point(c.x,c.y,alpha3)#final point C with positive alpha
            C.alpha=alpha3*get_sign(A,B,C)

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

    try:
        #convert point C to point in base m,n
        #using vector AC
        ac=np.array([c.x-A.x,c.y-A.y])
        #making transition matrix
        T=np.array(([A.ownBase.x[0],A.ownBase.y[0]],[A.ownBase.x[1],A.ownBase.y[1]]))
        #vector BC in base m,n is gotten from equation Tx'=x where x' is x in new base
        new_ac=np.linalg.solve(T,ac)
        #getting point C in new base from its new vector
        newC=cc.Point(new_ac[0],new_ac[1])

        start_angleB=180 - 2*(math.degrees(math.atan(newC.y/newC.x)))
    except ZeroDivisionError:
        start_angleB=0
    
    checklistB=[0,start_angleB,0,False,False]##initializing checlistB
    endB, angle=next_angle(checklistB,step)

    while(endB!=True):
        alpha2=angle##setting alpha1 as angle from next_angle(),then calling next_angle again
        endB, angle=next_angle(checklistB,step)

        xb,yb=dot_gen(-alpha2)#negative alpha2 for right(>) side
        tempB=cc.Point(xb,yb,alpha2)#Point B in p,q base 

        #turning B to i,j base
        tempB.x=xb*A.ownBase.x[0]+yb*A.ownBase.y[0]
        tempB.y=xb*A.ownBase.x[1]+yb*A.ownBase.y[1]

        #translating so point A is the new origin for point B
        tempB.x+=A.x
        tempB.y+=A.y

        #checking if chosen point is OK
        #1) is the distance between B and C in the range minimal r and l([min(r),l])
        Bdist=c.dist(tempB)

        if not (Bdist>=cc.mindist and Bdist<=cc.l):
            continue

        #2) is it possible to get point C with this B
        if tempB.alpha>0:#sign change because get_vecs requires negative alpha
            tempB.alpha=-tempB.alpha
            
        zwBase=get_vecs(A,tempB,c)#new B-base vectors in base p,q
            
        tempB.alpha=alpha2*get_sign(cc.root,A,tempB) #change to right sign

        #translating into base i,j
        newZX=(zwBase.x[0] * A.ownBase.x[0] + zwBase.x[1] * A.ownBase.y[0])
        newZY=(zwBase.x[0] * A.ownBase.x[1] + zwBase.x[1] * A.ownBase.y[1])
        newWX=(zwBase.y[0] * A.ownBase.x[0] + zwBase.y[1] * A.ownBase.y[0])
        newWY=(zwBase.y[0] * A.ownBase.x[1] + zwBase.y[1] * A.ownBase.y[1])

        new_zv=np.array([newZX,newZY])
        new_wv=np.array([newWX,newWY])

        tempB.ownBase=cc.Base(new_zv,new_wv)

        good,foundPoint=from_B(c,tempB,A,once)

        if good==False or once==False:
            continue
        elif good==True:
            return True,tempB,foundPoint

    return False,cc.Point(0,0),cc.Point(0,0)

def first_search(c,once=False,step=cc.step):
    """The main search function, returns a list of possible Poses
    
    If once==True, returns only one Pose encapsulated in a list.
    Default angle step is cc.step=0.5deg"""
    global poseList
    poseList=list()

    #1 Distance check
    if cc.root.dist(c)>3*cc.l:
        return poseList
    
    #2 Quadrant check
    quad=0
    quad=quadrant(c)
    
    #3 finding point A
    global checklistA

    try:
        start_angleA=180 - 2*(math.degrees(math.atan(c.y/c.x)))
    except ZeroDivisionError:
        start_angleA=0
    
    checklistA=[0,start_angleA,0,False,False]##initializing checlistA
    endA, angle=next_angle(checklistA,step)

    while(endA!=True):
        alpha1=angle#setting alpha1 as angle from next_angle(),then calling next_angle again
        endA, angle=next_angle(checklistA,step)

        xa,ya=dot_gen(-alpha1)#negative alpha1 for right(>) side

        #deciding which side is A on(rigth or left,by quadrants)
        if quad==2 or quad==3:#if C is to the left, make x negative
            xa=-xa
        if quad==1 or quad==4:
            alpha1=-alpha1#turning to the right

        A=cc.Point(xa,ya,alpha1)#done finding point A
        #4 lines p and q and vectors p and q for secondary coordinate system
        pqBase=get_vecs(cc.root,A,c)
        A.ownBase=pqBase

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
            first_search(pointList[0])
        else:
            nextC(e)
        currentPose=poseList[0]
        path.append(poseList[0])

def angleDifference(pose1,pose2):
    return -(pose1.A.alpha-pose2.A.alpha), -(pose1.B.alpha-pose2.B.alpha), -(pose1.C.alpha-pose2.C.alpha)

if __name__=="__main__":
    a=input("Input x coordinate:")
    b=input("Input y coordinate:")
    C=cc.Point(float(a),float(b))
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