import coord_01 as coord
import coord_class as cc
import math
import numpy as np
import time

total=0
counter=0

for i in range(20):
    for j in range(20):
        C=cc.Point(float(i),float(j))
        step=cc.step
        start=time.time()
        poses=coord.first_search(C,True,step)
        end=time.time()
        if poses!=[]:
            passed=end-start
            total+=passed
            counter+=1

print("%d succesfull, average time: %f s",counter,total/counter)