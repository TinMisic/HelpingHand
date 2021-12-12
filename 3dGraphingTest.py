'''# Standard import
import matplotlib.pyplot as plt
# Import 3D Axes 
from mpl_toolkits.mplot3d import axes3d
# Set up Figure and 3D Axes 
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
# Get some 3D data
X = [0, 1, 2]
Y = [0, 1, 8]
Z = [0, 1, 9]

ax.set_xlim([-15,15])
ax.set_ylim([-15,15])
ax.set_zlim([-15,15])
# Plot using Axes notation and standard function calls
ax.plot(X, Y, Z)
plt.show()'''
import plotly.express as px
import plotly.express as px
import pandas as pd
import numpy as np
dataDict={"x":[1,2,3],"y":[4,5,6],"z":[7,8,9]}
df = pd.DataFrame(data=dataDict)
fig = px.line_3d(x=[0,3.8197186342054885,9.625907714424466,15.0],y=[0,0,0,0],z=[0,3.8197186342054885,5.12541143368091,4])
fig.show()