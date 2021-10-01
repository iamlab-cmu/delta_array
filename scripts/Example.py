#!/usr/bin/env python
from Prismatic_Delta import Prismatic_Delta

import time
import serial
import math
import numpy as np
import matplotlib.pyplot as plt


s_p = 1.5 #side length of the platform
s_b = 4.3 #side length of the base
l = 4.5 #length of leg attached to platform


Delta = Prismatic_Delta(s_p, s_b, l)

leg_height = 5
pts = Delta.bound_workspace(leg_height) # samples worksapce for actuator position between 0-5cm
print(pts)
# graph pts with colormap
fig = plt.figure()
ax = fig.add_subplot(projection='3d')
ax.scatter(pts[:,0], pts[:,1], pts[:,2], marker='o')
plt.jet()
plt.show()