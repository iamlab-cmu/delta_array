from DeltaArray import DeltaArray
from OptiTrackStreaming.DataStreamer import OptiTrackDataStreamer
import numpy as np
from time import sleep
import time
import config
from math_util import quaternion_rotation_matrix


# da = DeltaArray('/dev/ttyACM0') -- CHANGE PORT
da = DeltaArray('COM3')

# for this to work enter the "Data Streaming Pane" in Motive:
#   "Broadcast Frame Data" is turned on
#   Command Port = 1510
#   Data Port = 1511
#   Multicast Interface = 239.255.42.99

# https://v22.wiki.optitrack.com/index.php?title=Data_Streaming
op = OptiTrackDataStreamer()


def sample_domain(max_height, max_actuator_dif,spacing=.005):
    pts = []
    m = config.MIN_ACTUATOR_HEIGHT
    for x in np.arange(m,m+max_height,spacing):
        for y in np.arange(m,m+max_height,spacing):
            for z in np.arange(m,m+max_height,spacing):
                if np.amax([x,y,z])-np.amin([x,y,z]) > max_actuator_dif:
                    continue
                else:
                    pts.append([x,y,z])
    return np.array(pts)

def limit_sample(sample):
    '''
    Get rid of redundant points in sample that have an offset of [z,z,z] 
    from each other, the kinematics at these points can be determined in post-processing
    '''

    new_sample = []
    for i,pt in enumerate(sample):
        count_similar_points = np.var(1000*(sample-pt),axis=1) < 1e-5 # pure z translation
        if np.sum(count_similar_points[:i]) < 2:
            new_sample.append(pt)
    return np.array(new_sample)



def save_training_data(act_pos,ee_pos,ee_rot, filename):
    np.savez(filename,act_pos=act_pos,ee_pos=ee_pos,ee_rot=ee_rot)

def adjust_act_command(command):
    #Nx12 trajectory to adjust
    #Actuators 3:6 are calibrated with linear regression
    traj = np.zeros_like(command)

    traj[:,3] = command[:,3]*.95976 + .00211
    traj[:,4] = command[:,4]*.97326
    traj[:,5] = command[:,5]*.9819 + .0014

    return traj

sample = sample_domain(.05,.043,.005)

sample = limit_sample(sample)

# PRESET POSITIONS
p = np.ones((sample.shape[0], 12)) * 0.0012
p[:,3:6] = sample
p = adjust_act_command(p)


def print_posn():
    #da.wait_until_done_moving()
    posn = da.get_joint_positions()
    print(posn[3:6])  # Motors 4-6


def retract():
    da.reset()
    da.wait_until_done_moving()

def center_delta(da,op):
    # return the transformation that moves the Delta at actuator position (0,0,0)
    # onto the origin
    # with actuator 3 lying on the +y axis

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

    return poses[0]-[0,0,m],rot_mat


retract()

pos_0,rot_0 = center_delta(da,op)

retract()

print(pos_0)

act_pos = []
ee_pos = []
ee_rot = []

# print_posn()
for i in range(0, p.shape[0]): # LOOP THROUGH ALL PRESET POSITIONS
    duration = [1.0]
    da.move_joint_position(p[i, :].reshape(1,12), duration)
    print(100*i/p.shape[0],"%","i","=",i)
    da.wait_until_done_moving()
    pos,rot_quat,t = op.get_closest_datapoint(time.time())
    rot = quaternion_rotation_matrix(rot_quat)

    act_pos.append(sample[i,:])
    ee_pos.append(rot_0@(pos-pos_0))
    ee_rot.append(rot)
    save_training_data(act_pos,ee_pos,ee_rot,"training_data_rot_sparse^2")

# RESET TO FULLY RETRACTED ACTUATORS
retract()

da.close()
op.close()