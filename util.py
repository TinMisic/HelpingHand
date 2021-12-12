import math

#constants
l=497.5875#mm
gear_r=23#mm
width=30#mm

def get_gear_angle(section_angle):
    r=180*l/math.pi/section_angle
    if section_angle>0:#left
        rL=r - width/2#left radius shortens
        rD=r + width/2#right radius lengthens
    elif section_angle<=0:#right
        rL=r + width/2#left radius lengthens
        rD=r - width/2#right radius shortens
    
    L=rL * math.pi * section_angle / 180# length of side strings.L+D=l
    D=rD * math.pi * section_angle / 180

    return -(180 * (L - D) / gear_r / math.pi)

if __name__=="__main__":
    print(get_gear_angle(90))