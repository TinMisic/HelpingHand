import coord_class as cc
import math

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

def get_part_A(alpha,prev_base,prev_point):
    '''From alpha generates:Point translated to base i,j ,that point's base vectors'''
    prim_x,prim_y=dot_gen(alpha)
    x = prim_x * prev_base.x[0] + prim_y * prev_base.y[0]
    y = prim_x * prev_base.x[1] + prim_y * prev_base.y[1]
    A=cc.Point(x,y,alpha)


def pose_gen(alpha1,alpha2,alpha3):
    '''Generates a Pose based on inputted angles. Left turn is positive angle, right turn is negative.'''

    #Point A
    xa,ya=dot_gen(alpha1)
    A=cc.Point(xa,ya,alpha1)

    #Point B
    xb,yb=dot_gen(alpha2)
    B=cc.Point(xb,yb,alpha2)
    B.rotate(alpha1).translate(A)

    #Point C
    xc,yc=dot_gen(alpha3)
    C=cc.Point(xc,yc,alpha3)
    C.rotate(alpha1+alpha2).translate(B)

    return cc.Pose(A,B,C)

if __name__=="__main__":
    pose=pose_gen(-90,25.34762188502134,-74.35050602755159)
    pose.printp()
    pose.plot()