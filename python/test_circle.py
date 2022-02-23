from Prismatic_Delta import Prismatic_Delta
from DeltaArray import DeltaArray

import time
from serial import Serial
import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from time import sleep
import linear_actuator_pb2

NUM_MOTORS = 12

# arduino = Serial('/dev/ttyACM0', 57600)  
arduino = Serial('COM7', 57600)  
delta_message = linear_actuator_pb2.lin_actuator()
delta_message.id = 1

# da = DeltaArray('/dev/ttyACM0')

s_p = 1.5 #side length of the platform
s_b = 4.3 #side length of the base
l = 4.5 #length of leg attached to platform


Delta = Prismatic_Delta(s_p, s_b, l)


# xvals = np.arange(-2.0, 2.0, 0.5)
# yvals = np.arange(-2.0, 2.0, 0.5)
# zvals = np.arange(5.,10, 0.5)

thetas = np.linspace(0, 2*np.pi, 10)

# the radius of the circle
r = 1

# # compute x1 and x2
# xvals = r*np.cos(theta)
# yvals = r*np.sin(theta)

# for x in xvals:
#     for y in yvals:
sers = []


# for theta in np.flip(thetas):
#     # for z in zvals:
#     ee_pts = [r*np.cos(theta), r*np.sin(theta), 10.0]
#     pts = Delta.IK(ee_pts)
#     pts = np.array(pts) * 0.01
#     # print(theta)
#     for i in range(0, 12, 3):
#         for j in range(3):
#             delta_message.joint_pos.append(pts[j])
#     # print(delta_message)
#     # _ = [delta_message.joint_pos.append(desired_joint_positions[i]) for i in range(12)]
#     serialized = delta_message.SerializeToString()
#     print(serialized)
#     arduino.write(bytes(b'\xa6') + serialized + bytes(b'\xa7'))
#     reachedPos = str(arduino.readline())
#     while reachedPos[0]!="~": 
#         print(reachedPos)
#         reachedPos = str(arduino.readline().decode())
#     sers.append(serialized)
#     delta_message.Clear()
#     delta_message.id = 1
#     time.sleep(0.3)

# for theta in thetas:
#     # for z in zvals:
#     ee_pts = [r*np.cos(theta), r*np.sin(theta), 10.0]
#     pts = Delta.IK(ee_pts)
#     pts = np.array(pts) * 0.01
#     # print(theta)
#     for i in range(0, 12, 3):
#         for j in range(3):
#             delta_message.joint_pos.append(pts[j])
#     # print(delta_message)
#     # _ = [delta_message.joint_pos.append(desired_joint_positions[i]) for i in range(12)]
#     serialized = delta_message.SerializeToString()
#     print(serialized)
#     arduino.write(bytes(b'\xa6') + serialized + bytes(b'\xa7'))
#     reachedPos = str(arduino.readline())
#     while reachedPos[0]!="~": 
#         print(reachedPos)
#         reachedPos = str(arduino.readline().decode())
#     sers.append(serialized)
#     delta_message.Clear()
#     delta_message.id = 1
#     time.sleep(0.3)



    
# print(sers)
while True:
    x = input("Position in cm? ")
    ee_pts = list(np.array(x.split(","), dtype=float))
    print(ee_pts)
    pts = Delta.IK(ee_pts)
    pts = np.array(pts) * 0.01
    print(pts)



# def create_joint_positions(val):
#     for i in range(12):
#         if i<NUM_MOTORS:
#             delta_message.joint_pos.append(val)
#         else:
#             delta_message.joint_pos.append(0.0)

# while True:
#     x = input("Position in cm? ")
#     create_joint_positions(float(x)/100)

#     print(delta_message)

#     serialized = delta_message.SerializeToString()
#     arduino.write(bytes(b'\xa6')  + serialized + bytes(b'\xa7'))
#     reachedPos = str(arduino.readline())
#     while reachedPos[0]!="~": 
#         print(reachedPos)
#         reachedPos = str(arduino.readline().decode())
#     print(reachedPos)
#     delta_message.Clear()