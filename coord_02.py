'''Changes:
        -Splitting first_search() into smaller functions, which can be reused in later searches. 
        -Changed sign in dot_gen()'''
import math
import numpy as np
import coord_class as cc
import time

poseList=list()#global list of all found poses for searched point
poseIndex=0#index of current pose in poseList
currentPose=cc.Pose(cc.Point(0,0),cc.Point(0,0),cc.Point(0,0))
path=list()#list of poses that represent a path (used in findPointsInList())

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

def sorter(dicti,listi):
    """Uses two holders to sort the list by the distance from point C(which is saved in dicti)
    
        The sorter sorts the angles so that the shortest distances come first.
    """
    holder1=0
    holder2=0
    for key in dicti.keys():
        if key==0.0:
            listi.append(key)
        else:
            holder1=key
            for index in range(len(listi)):
                if dicti[listi[index]]>=dicti[holder1]:
                    holder2=listi[index]
                    listi[index]=holder1
                    holder1=holder2
            listi.append(holder1)

def angle_list(c,prevPoint,step):
    """Calculates and sorts angles, based on distance from c,prevPoint being the latest Point"""
    alpha=0
    sort=list()#list of sorted angles(sorted by distance from point C)
    temp_dict=dict()#saves angles and distances of points that belong to those angles from point C
    alpha_range=(int)(90/step + 1)

    for k in range(0,alpha_range):
        alpha=k*step
        x,y=dot_gen(-alpha)#negative alpha for right(>) side

        a=x*prevPoint.ownBase.x[0]+y*prevPoint.ownBase.y[0]#translating to vector in root base(i,j)
        b=x*prevPoint.ownBase.x[1]+y*prevPoint.ownBase.y[1]

        m=prevPoint.x+a#translating to the real point in root coord. system
        n=prevPoint.y+b

        imaginary=cc.Point(m,n,alpha)#final point
        temp_dict[alpha]=c.dist(imaginary)

    sorter(temp_dict,sort)#now we have a list of sorted alpha angles
    return sort

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

    call_numB=0##call number for point B
    sortB=angle_list(c,A,step)
    while(call_numB<len(sortB)):
        alpha2=sortB[call_numB]#calculating x and y from alpha2
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
            call_numB+=1
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
            call_numB+=1
            continue
        elif good==True:
            return True,tempB,foundPoint

    return False,cc.Point(0,0),cc.Point(0,0)
    
def old_first_search(c,once=False,step=cc.step):
    """The main search function, returns a list of possible Poses
    
    If once==True, returns only one Pose encapsulated in a list.
    Default angle step is cc.step=0.5deg"""
    poseList=list()

    #1 Distance check
    if cc.root.dist(c)>3*cc.l:
        return poseList
    
    #2 Quadrant check
    quad=0
    quad=quadrant(c)
    
    #3 finding point A
    call_numA=0##call number for point A
    sortA=angle_list(c,cc.root,step)#list for alpha1

    while(call_numA<len(sortA)):
        alpha1=sortA[call_numA]#calculating x and y from alpha1
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

        '''---------------------------------'''
        #5 finding point B    
        call_numB=0##call number for point B
        sortB=angle_list(c,A,step)
        while(call_numB<len(sortB)):
            alpha2=sortB[call_numB]#calculating x and y from alpha2
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
                call_numB+=1
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

            '''-------------------------------------------'''
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

            #checking if alpha3 is suitable:
            if alpha3<0 or alpha3>90:
                call_numB+=1
                continue

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
                        return [cc.Pose(A,B,C)]
                    else:
                        call_numB+=1
                        continue
                else:
                    call_numB+=1
                    continue
    
        if call_numB>=len(sortB):
            call_numA+=1
            continue
        else:
            break

    return poseList

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
    call_numA=0##call number for point A
    sortA=angle_list(c,cc.root,step)#list for alpha1

    while(call_numA<len(sortA)):
        alpha1=sortA[call_numA]#calculating x and y from alpha1
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
            call_numA+=1
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
    searh_for_these=[cc.Point(15,6),cc.Point(15,5),cc.Point(15,4),cc.Point(4,15)]
    findPointsInList(searh_for_these)
    for e in path:
        e.printp()
        e.plot()

    print("angle difference between first and last pose:",angleDifference(path[0],path[-1]))