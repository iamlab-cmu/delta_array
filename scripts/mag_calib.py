from DeltaArray import DeltaArray
import numpy as np
from time import sleep
from numpy import random

da = DeltaArray('/dev/ttyACM0') #-- CHANGE PORT
# da = DeltaArray('COM9')

# PRESET POSITIONS
posns = np.ones((1, 12)) * 0.0012

num_points = 10
rands = random.rand(num_points,3)


def print_posn():
    da.wait_until_done_moving()
    posn = da.get_joint_positions()
    print(posn[3:6])  # Motors 4-6


def retract():
    da.reset()
    print_posn()


retract()
# print_posn()
for i in range(0, num_points): # LOOP THROUGH ALL PRESET POSITIONS
    duration = [1.0]
    posns[0][3:6] = rands[i,:] * (0.0988/2.5) # keep random value under 0.0988 (max height)
    # print(posns[0][3:6])
    da.move_joint_position(posns, duration)
    da.wait_until_done_moving()
    sleep(3)
    print(str(i) + '/' + str(num_points))
    # print_posn()
    

# RESET TO FULLY RETRACTED ACTUATORS
retract()

da.close()