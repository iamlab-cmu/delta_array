#!/usr/bin/env python
from Prismatic_Delta import Prismatic_Delta

import time
import serial
import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import ipdb


s_p = 1.5 #side length of the platform
s_b = 4.3 #side length of the base
l = 4.5 #length of leg attached to platform


Delta = Prismatic_Delta(s_p, s_b, l)

# SHOW WORKSPACE
# leg_height = 5
# pts = Delta.bound_workspace(leg_height) # samples workspace for actuator position between 0-5cm

# RANDOM TRAJ

heights = np.array([ np.arange(0.1,5.1,.1).reshape((50,1)), np.arange(0.05,2.55,.05).reshape((50,1)), np.arange(0.1,5.1,0.1).reshape((50,1))])
# ipdb.set_trace()
pts = Delta.FK_Traj(heights.reshape((50,3)))
ipdb.set_trace()
# graph pts with colormap
fig = plt.figure()
ax = plt.axes(projection='3d')
ax.scatter3D(pts[:,0],pts[:,1],pts[:,2],color='b')
# ax = fig.add_subplot(projection='3d')
# ax.scatter(pts[:,0], pts[:,1], pts[:,2], marker='o')
# plt.plot(pts[:,1],pts[:,2],'*')
# plt.jet()
plt.show()