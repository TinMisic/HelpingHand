import turtle as tr
import math

tin=tr.Turtle()
tin.dot(3)
tin.pu()

print("right")
for alpha in range(1,91):
    r=math.sqrt(2*(180*6/math.pi/alpha)**2 *(1-math.cos(alpha*math.pi/180)))
    x=r*math.cos(((180+alpha)/2)*math.pi/180)
    y=r*math.sin(((180+alpha)/2)*math.pi/180)
    tin.goto((x*30),(y*30))
    tin.pd()
    tin.dot(3,"blue")
    tin.pu()
    print((str)(x)+","+(str)(y)+",a="+(str)(alpha))

print("left")
for nalpha in range(1,91):
    alpha=-nalpha
    r=math.sqrt(2*(180*6/math.pi/alpha)**2 *(1-math.cos(alpha*math.pi/180)))
    x=r*math.cos(((180+alpha)/2)*math.pi/180)
    y=r*math.sin(((180+alpha)/2)*math.pi/180)
    tin.goto((x*30),(y*30))
    tin.pd()
    tin.dot(3,"red")
    tin.pu()
    print((str)(-(x))+","+(str)(y)+",a="+(str)(alpha))



input("Press any key...")