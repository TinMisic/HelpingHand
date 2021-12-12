import math
import numpy as np

class Point:
    def __init__(self,x,y,alpha=0.0):
        self.x=x
        self.y=y
        self.alpha=alpha
        
class Linear:
    def __init__(self,X,slope):
        self.X=Point(X.x,X.y,X.alpha)
        self.k=slope
        self.intercept=-(self.k * X.x) +X.y

    def get_Y(self,xCoord):#the actual Linear function, returns Point on line with given x coordinate
        yCoord=self.k*xCoord + self.intercept

        return yCoord

#GLOBALS
sortA=list()##global list of A angles
sortB=list()##global list of B angles
sortC=list()##global list of C angles
l=6#cm  !!!TEMPORARY!!! length of one arm element(15 joints in one element, 15x6deg=90deg per element)
root=Point(0,0,0)# root point
i=np.array([1,0])#root base vector 
j=np.array([0,1])#root base vector
step=0.5 #step by witch alpha increases
mindist=math.sqrt(2*(180*6/math.pi/90)**2 *(1-math.cos(90*math.pi/180)))#minimal distance between points(curvature of 90)

def dist(a,b):
    return math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2)

def quadrant(c):
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

def sorter(dicti,listi):
    """Uses two holders to sort the list by the distance from point C(which is saved in dicti)"""
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

def dot_gen(alpha):
    #alpha=angle of dot
    if alpha==0:
            x=l*math.cos(((180-alpha)/2)*math.pi/180)
            y=l*math.sin(((180-alpha)/2)*math.pi/180)
            r=l
    else:
        r=math.sqrt(2*(180*6/math.pi/alpha)**2 *(1-math.cos(alpha*math.pi/180)))
        x=r*math.cos(((180-alpha)/2)*math.pi/180)
        y=r*math.sin(((180-alpha)/2)*math.pi/180)

    
    return x,y,r

def angle_list(c,vec1,vec2,prevPoint):
    #vec1,vec2=vectors of base
    alpha=0
    sort=list()#list of sorted angles(sorted by distance from point C)
    temp_dict=dict()#saves angles and distances of points that belong to those angles from point C

    for k in range(0,181):
        alpha=k*step
        x,y,r=dot_gen(alpha)

        a=x*vec1[0]+y*vec2[0]#translating to vector in root base(i,j)
        b=x*vec1[1]+y*vec2[1]

        m=prevPoint.x+a#translating to the real point in root coord. system
        n=prevPoint.y+b

        imaginary=Point(m,n,alpha)#final point
        temp_dict[alpha]=dist(imaginary,c)

    sorter(temp_dict,sort)#now we have a list of sorted alpha angles
    return sort

def get_vecs(prevPoint,currPoint,c):
    if math.fabs(currPoint.alpha)==90:#special case when full turn(by 90 deg)
        if currPoint.alpha<0:
            qv=i#q vector
            try:
                kc=c.y/c.x#slope of line OC
                if kc>0:
                    pv=j
                    sgn_alpha=1.0
                elif kc<=0:
                    sgn_alpha=-1.0
                    pv=-j
            except ZeroDivisionError:
                pv=j
                sgn_alpha=1.0
        elif currPoint.alpha>0:
            qv=-i#q vector
            try:
                kc=c.y/c.x#slope of line OC
                if kc>0:
                    pv=-j
                    sgn_alpha=1.0
                elif kc<=0:
                    pv=j
                    sgn_alpha=-1.0
            except ZeroDivisionError:
                pv=j
                sgn_alpha=-1.0

    elif currPoint.alpha==0:#special case when no turn(0 deg)
        pv=i
        qv=j
        sgn_alpha=-1.0
    
    else:#all other cases of alpha
        kp=math.tan(currPoint.alpha * math.pi / 180)
        p=Linear(currPoint,kp)
        kq=-1/math.tan(currPoint.alpha * math.pi / 180)
        q=Linear(currPoint,kq)

        #getting point Q (line AQ makes vector qv) 
        #two x coordinates,solving linear equation(pre-solved,check folder for photo)
        qx1=(currPoint.x * (kq**2 + 1) + math.sqrt(kq**2 + 1)) / (kq**2 + 1)
        qx2=(currPoint.x * (kq**2 + 1) - math.sqrt(kq**2 + 1)) / (kq**2 + 1)

        Q1=Point(qx1,q.get_Y(qx1))
        Q2=Point(qx2,q.get_Y(qx2))

        #The point furthest away from previous point is point Q

        if dist(Q1,prevPoint)>=dist(Q2,prevPoint):
            Q=Q1
        else:
            Q=Q2

        #getting vector qv
        qv=np.array([(Q.x-currPoint.x),(Q.y-currPoint.y)])

        #getting point P (line AP makes vector pv) 
        #two x coordinates,solving linear equation
        px1=(currPoint.x * (kp**2 + 1) + math.sqrt(kp**2 + 1)) / (kp**2 + 1)
        px2=(currPoint.x * (kp**2 + 1) - math.sqrt(kp**2 + 1)) / (kp**2 + 1)

        P1=Point(px1,p.get_Y(px1))
        P2=Point(px2,p.get_Y(px2))

        #The point closest to C is point P

        if dist(P1,c)<=dist(P2,c):
            P=P1
        else:
            P=P2

        #getting vector pv
        pv=np.array([(P.x-currPoint.x),(P.y-currPoint.y)])

        #getting the sign of the next alpha
        try:
            kac=(c.y-currPoint.y)/(c.x-currPoint.x)#slope od line AC
        except ZeroDivisionError:
            kac=math.copysign(1,kq)*math.inf
        
        if kac< kq:
            sgn_alpha=-1.0
        elif kac>=kq:
            sgn_alpha=1.0

    return pv,qv,sgn_alpha


'''
return tuple: succes(true/false),A,B,C
'''
def search(xc,yc):
#1 Constant definition
    #already declared

#2 c input
    c=Point(xc,yc)

#3 length check
    if dist(root,c)>3*l:
        return False,Point(0,0,0),Point(0,0,0),Point(0,0,0)
    
#4 quadrant check
    quad=0
    quad=quadrant(c)
    
#5 finding point A
    call_numA=0##call number for point A
    sortA=angle_list(c,i,j,root)
    #print(sortA)

    while(call_numA<len(sortA)):
        alpha1=sortA[call_numA]#calculating x and y from alpha1
        xa,ya,ra=dot_gen(alpha1)

        #deciding which side is A on(rigth or left,by quadrants)
        if quad==2 or quad==3:#if C is to the left, make x negative
            xa=-xa
        if quad==1 or quad==4:
            alpha1=-alpha1

        A=Point(xa,ya,alpha1)#done finding point A
        #6 lines p and q and vectors p and q for secondary coordinate system
        pv,qv,b_sgn=get_vecs(root,A,c)

        #7 finding point B    
        call_numB=0##call number for point B
        sortB=angle_list(c,pv,qv,A)
        while(call_numB<len(sortB)):
            alpha2=sortB[call_numB]#calculating x and y from alpha2
            xb,yb,rb=dot_gen(alpha2)
            tempB=Point(xb,yb,math.copysign(alpha2,b_sgn))#Point B in p,q base with alpha with correct sign

            #turning B to i,j base
            tempB.x=xb*pv[0]+yb*qv[0]
            tempB.y=xb*pv[1]+yb*qv[1]

            #translating so point A is the new origin for point B
            tempB.x+=A.x
            tempB.y+=A.y

            #checking if chosen point is OK
            #1) is the distance between B and C in the range minimal r and l([min(r),l])
            Bdist=dist(tempB,c)

            if not (Bdist>=mindist and Bdist<=l):
                call_numB+=1
                continue

            #2) is it possible to get point C with this B
            '''This sign change is right, but needs to be impemented differently'''
            if tempB.alpha>0:
                tempB.alpha=-tempB.alpha

            
            zv,wv,c_sgn=get_vecs(A,tempB,c)#new B-base vectors in base p,q
            '''change of sign back to original'''
            tempB.alpha=math.copysign(alpha2,b_sgn)
            #translating into base i,j
            newZX=(zv[0] * pv[0] + zv[1] * qv[0])
            newZY=(zv[0] * pv[1] + zv[1] * qv[1])
            newWX=(wv[0] * pv[0] + wv[1] * qv[0])
            newWY=(wv[0] * pv[1] + wv[1] * qv[1])

            new_zv=np.array([newZX,newZY])
            new_wv=np.array([newWX,newWY])


            #convert point C to point in base z,w
            #using vector BC
            bc=np.array([c.x-tempB.x,c.y-tempB.y])
            #making transition matrix
            T=np.array(([new_zv[0],new_wv[0]],[new_zv[1],new_wv[1]]))
            #vector BC in base z,w is gotten from equation Tx'=x where x' is x in new base
            new_bc=np.linalg.solve(T,bc)
            #getting point C in new base from its new vector
            newC=Point(new_bc[0],new_bc[1])
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
                test_x,test_y,test_r=dot_gen(alpha3)
                if math.fabs(test_x-newC.x)<0.1 and math.fabs(test_y-newC.y)<0.1:# error margin of 1 mm
                    #alpha3 is good
                    B=Point(tempB.x,tempB.y,tempB.alpha)#done finding point B
                    C=Point(c.x,c.y,math.copysign(alpha3,c_sgn))#final point C with alpha3
                    break
                else:
                    call_numB+=1
                    continue
    
        if call_numB>=len(sortB):
            call_numA+=1
            continue
        else:
            break

    if call_numA>=len(sortA):
        return False,Point(0,0,0),Point(0,0,0),Point(0,0,0)        
    
    #done
    '''plt.scatter([pv[0],qv[0],zv[0],wv[0]],[pv[1],qv[1],zv[1],wv[1]],color="magenta")
    plt.scatter([A.x,B.x,C.x],[A.y,B.y,C.y],color="green")'''
    print("ra:"+(str)(ra))
    print("pv:"+(str)(pv[0]+A.x)+","+(str)(pv[1]+A.y))
    print("qv:"+(str)(qv[0]+A.x)+","+(str)(qv[1]+A.y))
    print("zv:"+(str)(new_zv[0]+B.x)+","+(str)(new_zv[1]+B.y))
    print("wv:"+(str)(new_wv[0]+B.x)+","+(str)(new_wv[1]+B.y))
    return True,A,B,C


if __name__=="__main__":
    a=input("Input x coordinate:")
    b=input("Input y coordinate:")
    boole,A1,B1,C1=search(float(a),float(b))
    if boole==True:
        print("    Point A:"+(str)(A1.x)+","+(str)(A1.y)+",alpha1:"+(str)(A1.alpha))
        print("    Point B:"+(str)(B1.x)+","+(str)(B1.y)+",alpha2:"+(str)(B1.alpha))
        print("    Point C:"+(str)(C1.x)+","+(str)(C1.y)+",alpha3:"+(str)(C1.alpha))
    else:
        print("No possible alphas for this point")
    input("Press any key")
    '''
    correct_count=0
    for first in range(40):
        apple=first/2
        for second in range(40):
            banana=second/2
            boole,A1,B1,C1=search(apple,banana)
            if boole==True:
                correct_count+=1
                print(first,end=" ,")
                print(second)
                print("    Point A:"+(str)(A1.x)+","+(str)(A1.y)+",alpha1:"+(str)(A1.alpha))
                print("    Point B:"+(str)(B1.x)+","+(str)(B1.y)+",alpha2:"+(str)(B1.alpha))
                print("    Point C:"+(str)(C1.x)+","+(str)(C1.y)+",alpha3:"+(str)(C1.alpha))
            else:
                print("No possible alphas for",end=" ")
                print(first,end=" ,")
                print(second)
    print("Found "+(str)(correct_count)+"out of 400")    '''     
