from DeltaArray import DeltaArray
import numpy as np
from time import sleep

# da = DeltaArray('/dev/ttyACM0') -- CHANGE PORT
da = DeltaArray('COM7')

# PRESET POSITIONS
p = np.ones((8, 12)) * 0.0012
p[0, 3:6] = np.array([0.01, 0.01, 0.01])
p[1, 3:6] = np.array([0.02, 0.01, 0.01])
p[2, 3:6] = np.array([0.01, 0.02, 0.01])
p[3, 3:6] = np.array([0.01, 0.01, 0.02])

p[4, 3:6] = np.array([0.02, 0.02, 0.02])
p[5, 3:6] = np.array([0.03, 0.02, 0.02])
p[6, 3:6] = np.array([0.02, 0.03, 0.02])
p[7, 3:6] = np.array([0.02, 0.02, 0.03])


def print_posn():
    da.wait_until_done_moving()
    posn = da.get_joint_positions()
    print(posn[3:6])  # Motors 4-6


def retract():
    da.reset()
    print_posn()


retract()
# print_posn()
for i in range(0, 8): # LOOP THROUGH ALL PRESET POSITIONS
    duration = [1.0]
    da.move_joint_position(p[i, :].reshape(1,12), duration)
    da.wait_until_done_moving()
    sleep(5)
    # print_posn()
    print(i)

# RESET TO FULLY RETRACTED ACTUATORS
retract()

da.close()