from DeltaArray import DeltaArray
#from OptiTrackStreaming.DataStreamer import OptiTrackDataStreamer
import numpy as np
from time import sleep
import time

# da = DeltaArray('/dev/ttyACM0') -- CHANGE PORT
da = DeltaArray('COM3')

# for this to work enter the "Data Streaming Pane" in Motive:
#   "Broadcast Frame Data" is turned on
#   Command Port = 1510
#   Data Port = 1511
#   Multicast Interface = 239.255.42.99

# https://v22.wiki.optitrack.com/index.php?title=Data_Streaming
#op = OptiTrackDataStreamer()

# PRESET POSITIONS
p = np.ones((8, 12)) * 0.005
for i in range(1,9):
    p[i-1,3:6] = .01 * np.array([i,i,i])
    
# p[0, 3:6] = np.array([.01,.01,.01])
# p[1, 3:6] = np.array([.02,.02,.02])
# p[2, 3:6] = np.array([0.03, 0.03, 0.03])
# p[3, 3:6] = np.array([0.04, 0.04, 0.04])
# p[4, 3:6] = np.array([0.05, 0.05, 0.05])
# p[7, 3:6] = np.array([0.08, 0.08, 0.08])

#p[:,3] -= .0015
#p[:,4] -= .0015
#p[:,5] += .001


def print_posn():
    posn = da.get_joint_positions()
    print(posn[3:6])  # Motors 4-6


def retract():
    da.reset()
    da.wait_until_done_moving()

def adjust_act_command(command):
    #Nx12 trajectory to adjust
    traj = np.zeros_like(command)

    traj[:,3] = command[:,3]*.95976 + .00211
    traj[:,4] = command[:,4]*.97326
    traj[:,5] = command[:,5]*.9819 + .0014

    return traj

retract()
p = adjust_act_command(p)
print(p)

#pos_0,rot_0,t = op.get_closest_datapoint(time.time())

# print_posn()
for i in range(0, 8): # LOOP THROUGH ALL PRESET POSITIONS
    duration = [1.0]
    da.move_joint_position(p[i, :].reshape(1,12), duration)
    s = time.time()
    da.wait_until_done_moving()

    print("Desired Position:", p[i,3:6])

    print("Potentiometer Reading:",end=" ")
    print_posn()
    
    print("\n-------------------------------\n")
    #print("wait_until_done_moving took:",time.time()-s)
    #s = time.time()
    #retract()
    #print("retraction took:",time.time()-s)

    #pos,rot,t = op.get_closest_datapoint(time.time())
    #print("relative_position = ",pos-pos_0)
    #print_posn()
    input()


# RESET TO FULLY RETRACTED ACTUATORS
#retract()

da.close()
#op.close()