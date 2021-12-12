import math
import numpy as np
import plotly.graph_objects as go

#Classes

class Base:
    """A class that contains two vectors of a base.
    
        Methods:
        __init__(self,x,y)

    """
    def __init__(self,x,y):
        self.x=x
        self.y=y

class Point:
    """A representation of a point with coordinates x and y 
        and angle alpha which was used to get said coordinates.

        Methods:

        -init

        -dist: Returns thistance from this point to another
    """
    def __init__(self,x,y,alpha=0.0,ownBase=Base(np.array([1,0]),np.array([0,1]))):
        self.x=x
        self.y=y
        self.alpha=alpha
        self.ownBase=ownBase
    
    def dist(self,B):
        return math.sqrt((self.x-B.x)**2 + (self.y-B.y)**2)

    def translate(self,toPoint):
        self.x+=toPoint.x
        self.y+=toPoint.y
        
        return self

    def rotate(self,angle):
        phi=math.radians(angle)
        operator=np.array([[math.cos(phi),-math.sin(phi)],[math.sin(phi),math.cos(phi)]])
        point_arr=np.array([self.x,self.y])

        rotated=np.matmul(operator,point_arr)
        self.x=rotated[0]
        self.y=rotated[1]

        return self
        
class Linear:
    """A representation of a linear function determined from one Point and the sope of the line.
    
        Methods:

        -init

        -get_Y: solves the linear function for given x
    """
    def __init__(self,X,slope):
        self.X=Point(X.x,X.y,X.alpha)
        self.k=slope
        self.intercept=-(self.k * X.x) +X.y

    def get_Y(self,xCoord):#the actual Linear function, returns Point on line with given x coordinate
        yCoord=self.k*xCoord + self.intercept

        return yCoord


class Pose:
    """A structure that contains three Points that make up one movement or Pose"""
    def __init__(self,A,B,C):
        self.A=A
        self.B=B
        self.C=C

    def printp(self):
        print("A:"+(str)(self.A.x)+", "+(str)(self.A.y)+"   alpha1:"+(str)(self.A.alpha))
        print("B:"+(str)(self.B.x)+", "+(str)(self.B.y)+"   alpha2:"+(str)(self.B.alpha))
        print("C:"+(str)(self.C.x)+", "+(str)(self.C.y)+"   alpha3:"+(str)(self.C.alpha))

    def plot(self):
        x=(root.x,self.A.x,self.B.x,self.C.x)
        y=(root.y,self.A.y,self.B.y,self.C.y)
        layout = go.Layout(yaxis=dict(scaleanchor="x", scaleratio=1))
        fig = go.Figure(data=go.Scatter(x=x,y=y),layout=layout)
        fig.show()

#Constants:
l=6#cm  !!!TEMPORARY!!! length of one arm element(15 joints in one element, 15x6deg=90deg per element)
i=np.array([1,0])#root base vector 
j=np.array([0,1])#root base vector
norm=Base(i,j)
root=Point(0,0,0,norm)# root point
step=0.5 #default step by witch alpha increases
mindist=math.sqrt(2*(180*6/math.pi/90)**2 *(1-math.cos(90*math.pi/180)))#minimal distance between points(curvature of 90)

if __name__=="__main__":
    A=Point(1,0)
    B=Point(1,1)
    print(A.x,",",A.y)
    A.translate(B)
    print(A.x,",",A.y)

    