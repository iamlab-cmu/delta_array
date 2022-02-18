#!/usr/bin/env python
from Prismatic_Delta import Prismatic_Delta
from DeltaArray import DeltaArray

import time
import serial
import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from time import sleep
# import ipdb

da = DeltaArray('/dev/ttyACM0')

s_p = 1.5 #side length of the platform
s_b = 4.3 #side length of the base
l = 4.5 #length of leg attached to platform


Delta = Prismatic_Delta(s_p, s_b, l)

# SHOW WORKSPACE
# leg_height = 5
# pts = Delta.bound_workspace(leg_height) # samples workspace for actuator position between 0-5cm
#
# # TEST FORWARD KINEMATICS
# h1,h2,h3 = np.arange(0.1,5.1,.1).reshape((50,1)), np.arange(0.05,2.55,.05).reshape((50,1)), np.arange(0.1,5.1,0.1).reshape((50,1))
# heights = np.column_stack((h1,h2,h3))
# pts = Delta.FK_Traj(heights)
#
# # VERIFY IK WITH SAME PTS
# inv_pts = Delta.IK_Traj(pts)
# print("Kinematics Error = ")
# print(np.linalg.norm(heights-inv_pts))
# print(pts.shape)

# MAKE SHAPE
# p1,p2,p3 = np.arange(-1.,1.,.1).reshape((20,1)), np.arange(-1.,1.,.1).reshape((20,1)), np.ones((20,1)) * 6 # DIAG
# p1,p2,p3 = np.arange(-1.,1.,.1).reshape((20,1)), np.ones((20,1)) * 1, np.ones((20,1)) * 6 # XY plane
# xvals = np.arange(-3.0, 3.0, 0.5)
# yvals = np.arange(-2.8, 3.4, 0.5)
# zvals = np.arange(5.,10., 0.5)
xvals = np.arange(-2.0, 2.0, 0.5)
yvals = np.arange(-2.0, 2.0, 0.5)
zvals = np.arange(5.,10., 0.5)
# ipdb.set_trace()
# p1,p2,p3 = np.ones((20,1)) * 0., np.arange(-2.,0.,.1).reshape((20,1)) , np.ones((20,1)) * 4.7
# shape_pts = np.column_stack((p1,p2,p3))
# inv_pts = Delta.IK_Traj(shape_pts)
# # print(inv_pts*0.1)
# pts = inv_pts * 0.01 # CONVERT CM TO METERS
# # print(shape_pts)
# # print(shape_pts[7,:])
# # print(pts)
# # min X dist @ Z=5,10 = -3.0 
# # max X dist @ Z=5,10 = 3.0
# # min Y dist @ Z=5,10 = -2.8
# # max Y dist @ Z=5,10 = 3.4
# # min Z dist = 4.7
# # max Z dist = 14.1

# min actuator height: 0.005
# max actuator height: 0.0988

# arr = np.arange(-5., 5.1, 0.1)
# for i in range(len(arr)):
#     ee_pts = [arr[i], 0, 10]
#     act_hgt = Delta.IK(ee_pts)
#     act_hgt = np.array(act_hgt)
#     act_hgt = act_hgt * 0.01
#     if act_hgt[0] < 0.005 or act_hgt[0] > 0.0988 or act_hgt[1] < 0.005 or act_hgt[1] > 0.0988 or act_hgt[2] < 0.005 or act_hgt[2] > 0.0988:
#         continue
#     else:
#         print(act_hgt, arr[i])

# # graph pts with colormap
# fig = plt.figure()
# ax = plt.axes(projection='3d')
# ax.scatter3D(shape_pts[:,0],shape_pts[:,1],shape_pts[:,2],color='b')
# plt.show()


def print_posn():
    da.wait_until_done_moving()
    posn = da.get_joint_positions()
    print(posn[3:6])  # Motors 4-6


def retract():
    da.reset()
    print_posn()

def invalidpt(pts):
    return pts[0] < 0.005 or pts[1] < 0.005 or pts[2] < 0.005 or pts[0] > 0.0988 or pts[1] > 0.0988 or pts[2] > 0.0988


posns = np.ones((1, 12)) * 0.0012
count = 0
retract()
# print_posn()
num_points = xvals.shape[0] * yvals.shape[0] * zvals.shape[0]
shape_pts = np.zeros((num_points, 3))
for x in xvals:
    for y in yvals:
        for z in zvals:
            duration = [1.0]
            ee_pts = [x, y, z]
            pts = Delta.IK(ee_pts)
            pts = np.array(pts) * 0.01
            # ipdb.set_trace()
            # print([pts], [ee_pts])
            if invalidpt(pts):
                print('INVALID POINTS: ', str(pts))
                count += 1
                continue
            shape_pts[count,:] = ee_pts
            posns[0][3:6] = pts
            print(ee_pts)
            # da.move_joint_position(posns, duration)
            # da.wait_until_done_moving()
            sleep(2)
            print(str(count) + '/' + str(num_points))
            count += 1
            # print_posn()

# fig = plt.figure()
# ax = plt.axes(projection='3d')
# ax.scatter3D(shape_pts[:,0],shape_pts[:,1],shape_pts[:,2],color='b')
# plt.show()

# posns = np.ones((1, 12)) * 0.0012
# retract()
# # print_posn()
# num_points = np.shape(pts)[0]
# for i in range(0, num_points):  # LOOP THROUGH ALL PRESET POSITIONS
#     duration = [1.0]
#     # posns[0][3:6] = rands[i, :] * (0.0988 / 2.5)  # keep random value under 0.0988 (max height)
#     # print(posns[0][3:6])
#     # ipdb.set_trace()
#     posns[0][3:6] = pts[i]
#     print(pts[i])
#     da.move_joint_position(posns, duration)
#     da.wait_until_done_moving()
#     sleep(3)
#     print(str(i) + '/' + str(num_points))
#     # print_posn()

# RESET TO FULLY RETRACTED ACTUATORS
retract()

da.close()