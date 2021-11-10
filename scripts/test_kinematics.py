from Prismatic_Delta import Prismatic_Delta
from Model import NN
import numpy as np
from delta_utils import record_trajectory, initialize_array_recording, close_all


fk_save_file = "./models/training_data_rot_fk_full_aug_op_up.h5"
ik_save_file = "./models/training_data_rot_ik_full_aug_op_up.h5"

aug_model = NN(ik_save_file,fk_save_file,
			ik_save_file,fk_save_file,load=True)

fk_save_file = "./models/training_data_rot_fk_full_base_op_up.h5"
ik_save_file = "./models/training_data_rot_ik_full_base_op_up.h5"

base_model = NN(ik_save_file,fk_save_file,
			ik_save_file,fk_save_file,load=True)

s_p = 1.538 #side length of the platform
s_b = 2.65 #side length of the base
l = 4.75 #length of leg attached to platform

rigid_Delta = Prismatic_Delta(s_p, s_b, l)

traj_height = 1.5
spiral_traj = [[0,0,traj_height]]
num_rad_steps = 30
step_size = .5  #cm in radial direction
max_spiral_width = 4.6 #cm determined experimentally

#rigid IK has origin at the base of the Delta
# move origin to Delta position for input [0,0,0]
rigid_ik_offset = rigid_Delta.FK([0,0,0])[0,2]

rigid_ik = [rigid_Delta.IK([0,0,traj_height+rigid_ik_offset])]

for step in np.arange(1,4.6,step_size):
    for rad in np.linspace(0,2*np.pi,num=num_rad_steps,endpoint=False):
        pt = np.array([step*np.cos(rad),step*np.sin(rad),traj_height])
        offset_pt = pt + [0,0,rigid_ik_offset] 
        if rigid_Delta.is_valid(offset_pt,4.3):
            spiral_traj.append(pt)
            rigid_ik.append(rigid_Delta.IK(offset_pt))

spiral_traj = np.array(spiral_traj)

rigid_ik = np.array(rigid_ik)
base_ik,base_valid = base_model.predict_ik(spiral_traj,return_valid_mask=True)
aug_ik,aug_valid = aug_model.predict_ik(spiral_traj,return_valid_mask=True)

valid_mask = base_valid & aug_valid

rigid_ik = rigid_ik[valid_mask]
base_ik = base_ik[valid_mask]
aug_ik = aug_ik[valid_mask]

da,op = initialize_array_recording()

act_pos_rigid, ee_pos_rigid, ee_rot_rigid = record_trajectory(da,op,rigid_ik)
act_pos_base, ee_pos_base, ee_rot_base = record_trajectory(da,op,base_ik)
act_pos_aug, ee_pos_aug, ee_rot_aug = record_trajectory(da,op,aug_ik)


save_file = "Delta_2_Kinematic_Test_op_up"
np.savez(save_file,
    act_pos_rigid = act_pos_rigid,ee_pos_rigid=ee_pos_rigid,ee_rot_rigid=ee_rot_rigid,
    act_pos_base = act_pos_base,ee_pos_base=ee_pos_base,ee_rot_base=ee_rot_base,
    act_pos_aug = act_pos_aug,ee_pos_aug=ee_pos_aug,ee_rot_aug=ee_rot_aug,
    traj = spiral_traj)


close_all(da,op)