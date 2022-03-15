from Prismatic_Delta import Prismatic_Delta
import numpy as np
import matplotlib.pyplot as plt
from time import sleep

s_p = 1.5 #side length of the platform
s_b = 4.3 #side length of the base
l = 4.5 #length of leg attached to platform

Delta = Prismatic_Delta(s_p, s_b, l)
thetas = np.linspace(0, 2*np.pi, 36)
from delta_control.get_coords import RoboCoords

import numpy as np
import matplotlib.pyplot as plt
RC = RoboCoords()
xvec = np.array((0,0.5))
a = RC.rotate(xvec, np.pi/6)
plt.quiver(0,0,a[0],a[1])