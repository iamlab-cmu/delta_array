from sympy import Q
import delta_array_pb2
import numpy as np
from serial import Serial
from math import *
import time
from Prismatic_Delta import Prismatic_Delta
from FINAL_delta_array_12motor import DeltaArrayAgent
import telnetlib

# arduino = Serial('COM7', 57600)

host = "192.168.0.182"
port = 80
timeout = 100

esp01 = telnetlib.Telnet(host, port, timeout)

# esp01.write(bytes(b'\xa6') + serialized + bytes(b'\xa7'))


DDA = DeltaArrayAgent(esp01, 1)
s_p = 1.5 #side length of the platform
s_b = 4.3 #side length of the base
l = 4.5 #length of leg attached to platform

Delta = Prismatic_Delta(s_p, s_b, l)

def create_joint_positions(val):
    a = []
    for i in range(4):
        for j in range(3):
            a.append(val[j])
    return a

traj = [[np.cos(theta), np.sin(theta), 10] for theta in np.arange(0, np.pi*2, 0.6)]

for t in traj:
    # x = input("Position in cm? ")
    # ee_pts = list(np.array(x.split(","), dtype=float))
    # print(ee_pts)
    pts = Delta.IK(t)
    pts = np.array(pts) * 0.01
    jts = create_joint_positions(pts)
    DDA.move_joint_position(jts)

    while DDA.get_joint_positions():
        pass
    print("DONE MOVING FOR REALS")
    time.sleep(0.5)

# while True:
#     x = input("Position in cm? ")
#     ee_pts = list(np.array(x.split(","), dtype=float))
#     print(ee_pts)
#     pts = Delta.IK(ee_pts)
#     pts = np.array(pts) * 0.01
#     print(pts)