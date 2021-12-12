import turtle as tr
'''import matplotlib.pyplot as plt
x=[1,2,3,4,5,6,7]
y=[1,2,3,4,5,6,7]
plt.scatter(x,y,s=2)
plt.xlabel("x")
plt.ylabel("y")
plt.xticks([w for w in range(len(x))],[w for w in range(len(y))])
plt.autoscale(tight=False)
plt.grid()
plt.show()'''
import math

def elipse_y(x_list):
    a=4.347616672111258
    b=4.1545462121557606
    q=1.8454537878442394
    y_list=list()
    for x in x_list:
        y=(b * math.sqrt(a**2 - x**2) / a) + q
        y_list.append(y)
    return y_list
        
def elipse(x):
    a=4.347616672111258
    b=4.1545462121557606
    q=1.8454537878442394
    y=(b * math.sqrt(a**2 - x**2) / a) + q
    return y

def simpleLinear(x,kd):
    y= kd*x
    return y

dot_dict=dict()
dot_dict[0] = 6
for alpha in range(1,361):
    beta = (180 - alpha) / 2
    r = (180*6)/(alpha*math.pi)
    kdn = math.tan(beta*math.pi/180)
    dnsqrd=2 * (r**2) * (1 - math.cos(alpha*math.pi/180))
    x = math.sqrt( dnsqrd / (1 + kdn**2))
    y = simpleLinear(x,kdn)
    dot_dict[x] = y

tin=tr.Turtle()
tin.dot(3)
tin.pu()

y_list=elipse_y(dot_dict.keys())
counter=0

for key in dot_dict.keys():
    print(key,",",dot_dict[key])
    tin.goto((key*30),(dot_dict[key]*30))
    tin.pd()
    tin.dot(3)
    tin.pu()

for key in dot_dict.keys():
    print(-key,",",dot_dict[key])
    tin.goto(-(key*30),(dot_dict[key]*30))
    tin.pd()
    tin.dot(3)
    tin.pu()

print("elipse formula\n\n")

for key in dot_dict.keys():
    print(key,y_list[counter])
    tin.goto((key*30),(y_list[counter]*30))
    tin.pd()
    tin.dot(3,"magenta")
    tin.pu()
    counter+=1

for alpha in range(1,361):
    r=math.sqrt(2*(180*6/math.pi/alpha)**2 *(1-math.cos(alpha*math.pi/180)))
    x=r*math.cos(((180-alpha)/2)*math.pi/180)
    y=r*math.sin(((180-alpha)/2)*math.pi/180)
    tin.goto((x*30),(y*30))
    tin.pd()
    tin.dot(3,"green")
    tin.pu()

a=input()
print(elipse(1.535))