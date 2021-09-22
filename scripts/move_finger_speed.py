from DeltaArray import DeltaArray
import numpy as np
from time import sleep

# da = DeltaArray('/dev/ttyACM0') -- CHANGE PORT
da = DeltaArray('COM17')

# PRESET POSITIONS
p = np.ones((8, 12)) * 0.0012
p[0, 3:6] = np.array([0.01, 0.01, 0.01])
p[1, 3:6] = np.array([0.02, 0.02, 0.02])
p[2, 3:6] = np.array([0.03, 0.03, 0.03])
p[3, 3:6] = np.array([0.04, 0.04, 0.04])

speeds = np.ones((8, 12)) * 0
speeds[0, 3:6] = np.array([50, 50, 50])
speeds[1, 3:6] = np.array([100,100,100])
speeds[2, 3:6] = np.array([150, 150, 150])
speeds[3, 3:6] = np.array([200,200,200])

def print_posn():
    da.wait_until_done_moving()
    posn = da.get_joint_positions()
    print(posn[3:6])  # Motors 4-6


def retract():
    da.reset()
    print_posn()


retract()
# print_posn()
for i in range(0, 4): # LOOP THROUGH ALL PRESET POSITIONS
    print(i)
    da.move_joint_speed_position(p[i, :].reshape(1,12), speeds[i,:].reshape(1,12))
    da.wait_until_done_moving()
    sleep(2)

# RESET TO FULLY RETRACTED ACTUATORS
retract()

da.close()