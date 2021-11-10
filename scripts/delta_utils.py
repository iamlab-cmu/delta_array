import numpy as np
import time
from math_util import quaternion_rotation_matrix
from DeltaArray import DeltaArray
from OptiTrackStreaming.DataStreamer import OptiTrackDataStreamer
import config

def initialize_array_recording():
    da = DeltaArray('COM3')

    # for this to work enter the "Data Streaming Pane" in Motive:
    #   "Broadcast Frame Data" is turned on
    #   Command Port = 1510
    #   Data Port = 1511
    #   Multicast Interface = 239.255.42.99

    # https://v22.wiki.optitrack.com/index.php?title=Data_Streaming
    op = OptiTrackDataStreamer()

    return da,op

def retract(da):
    da.reset()
    da.wait_until_done_moving()


def adjust_act_command(command):
    #Nx12 trajectory to adjust
    #Actuators 3:6 are calibrated with linear regression
    traj = np.zeros_like(command)

    traj[:,3] = command[:,3]*.95976 + .00211
    traj[:,4] = command[:,4]*.97326
    traj[:,5] = command[:,5]*.9819 + .0014

    return traj

def goto_position(da,pos,timeout=6):
    # pos specified in cm -> converted to m

    act_cmd = np.zeros((1,12))
    act_cmd[0,3:6] = pos/100 
    act_cmd = adjust_act_command(act_cmd)
    da.move_joint_position(act_cmd, [1.])
    return da.wait_until_done_moving(timeout=timeout)

def record_trajectory(da,op,heights):
    pos_0,rot_0 = center_delta(da,op)

    _,rot_quat,_ = op.get_closest_datapoint(time.time())
    start_rot = quaternion_rotation_matrix(rot_quat)

    act_pos = []
    ee_pos = []
    ee_rot = []

    for i,height in enumerate(heights): 
        print(100*i/len(heights),"\%")
        goto_position(da,height)
        pos,rot_quat,t = op.get_closest_datapoint(time.time())
        rot = quaternion_rotation_matrix(rot_quat)

        act_pos.append(height)
        ee_pos.append(rot_0 @ (pos-pos_0))
        ee_rot.append(start_rot.T @ rot)

    retract(da)
    return np.array(act_pos),np.array(ee_pos),np.array(ee_rot)

def close_all(da,op):
    da.close()
    op.close()


def center_delta(da,op):
    # return the transformation that moves the Delta at actuator position (0,0,0)
    # onto the origin
    # with actuator 3 lying on the +y axis
    retract(da)

    m = config.MIN_ACTUATOR_HEIGHT
    traj = np.zeros((6,12))
    traj[:,3] = m
    traj[:,4] = np.linspace(m,m+.03,6)
    traj[:,5] = np.linspace(m,m+.03,6)
    
    cmd_traj = adjust_act_command(traj)
    poses = []
    for cmd in cmd_traj:
        da.move_joint_position(cmd.reshape(1,12),[1.0])
        da.wait_until_done_moving()
        pos,rot,t = op.get_closest_datapoint(time.time())
        poses.append(pos)

    poses = np.array(poses)
    xy_poses = poses[:,:2] - poses[0,:2]
    axs_dir = np.mean(xy_poses,axis=0)
    axs_dir = axs_dir/np.linalg.norm(axs_dir)
    rot_mat = np.eye(3)
    rot_mat[1,:2] = axs_dir
    rot_mat[0,:2] = [axs_dir[1],-axs_dir[0]]

    retract(da)

    return poses[0]-[0,0,m],rot_mat
