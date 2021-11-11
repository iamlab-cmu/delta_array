from numpy.core.defchararray import center
from Prismatic_Delta import Prismatic_Delta
from Model import NN
import numpy as np
from delta_utils import record_trajectory, initialize_array_recording, close_all, center_delta


fk_save_file = "./models/training_data_aug_fk.h5"
ik_save_file = "./models/training_data_aug_ik.h5"

model_aug = model1 = NN(ik_save_file,fk_save_file,
			ik_save_file,fk_save_file,load=True)

fk_save_file = "./models/training_data_base_fk.h5"
ik_save_file = "./models/training_data_base_ik.h5"

model_base = model1 = NN(ik_save_file,fk_save_file,
			ik_save_file,fk_save_file,load=True)

fk_save_file = "./models/training_data_flat_fk.h5"
ik_save_file = "./models/training_data_flat_ik.h5"

model1 = NN(ik_save_file,fk_save_file,
			ik_save_file,fk_save_file,load=True)


fk_save_file = "./models/training_data_flat_fk_2.h5"
ik_save_file = "./models/training_data_flat_ik_2.h5"

model2 = NN(ik_save_file,fk_save_file,
			ik_save_file,fk_save_file,load=True)


fk_save_file = "./models/training_data_flat_fk_3.h5"
ik_save_file = "./models/training_data_flat_ik_3.h5"

model3 = NN(ik_save_file,fk_save_file,
			ik_save_file,fk_save_file,load=True)

s_p = 1.538 #side length of the platform
s_b = 2.65 #side length of the base
l = 4.75 #length of leg attached to platform

rigid_Delta = Prismatic_Delta(s_p, s_b, l)

traj_height = 2
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
ik_1 = model1.ik(spiral_traj)
ik_2 = model2.ik(spiral_traj)
ik_3 = model3.ik(spiral_traj)

ik_base = model_base.ik(spiral_traj)
ik_aug = model_aug.ik(spiral_traj)

da,op = initialize_array_recording()

pos_0,rot_0 = center_delta(da,op)

act_pos_rigid, ee_pos_rigid, ee_rot_rigid = record_trajectory(da,op,rigid_ik,pos_0,rot_0)
act_pos_1, ee_pos_1, ee_rot_1 = record_trajectory(da,op,ik_1,pos_0,rot_0)
act_pos_2, ee_pos_2, ee_rot_2 = record_trajectory(da,op,ik_2,pos_0,rot_0)
act_pos_3, ee_pos_3, ee_rot_3 = record_trajectory(da,op,ik_3,pos_0,rot_0)

act_pos_aug,ee_pos_aug,ee_rot_aug = record_trajectory(da,op,ik_aug,pos_0,rot_0)
act_pos_base,ee_pos_base,ee_rot_base = record_trajectory(da,op,ik_base,pos_0,rot_0)


save_file = "./Kinematic_Tests/Delta_1_Kinematic_Test_Flat"

# np.savez(save_file,
#     act_pos_rigid = D["act_pos_rigid"],ee_pos_rigid=D["ee_pos_rigid"],ee_rot_rigid=D["ee_rot_rigid"],
#     act_pos_1 = D["act_pos_1"],ee_pos_1=D["ee_pos_1"],ee_rot_1=D["ee_rot_1"],
#     act_pos_2=D["act_pos_2"], ee_pos_2=D["ee_pos_2"], ee_rot_2=D["ee_rot_2"],
#     act_pos_3=D["act_pos_3"], ee_pos_3=D["ee_pos_3"], ee_rot_3=D["ee_rot_3"],
#     act_pos_aug=act_pos_aug,ee_pos_aug=ee_pos_aug,ee_rot_aug=ee_rot_aug,
#     act_pos_base=act_pos_base,ee_pos_base=ee_pos_base,ee_rot_base=ee_rot_base,
#     traj = spiral_traj)

np.savez(save_file,
    act_pos_rigid = act_pos_rigid,ee_pos_rigid=ee_pos_rigid,ee_rot_rigid=ee_rot_rigid,
    act_pos_1 = act_pos_1,ee_pos_1=ee_pos_1,ee_rot_1=ee_rot_1,
    act_pos_2=act_pos_2, ee_pos_2=ee_pos_2, ee_rot_2=ee_rot_2,
    act_pos_3=act_pos_3, ee_pos_3=ee_pos_3, ee_rot_3=ee_rot_3,
    act_pos_aug=act_pos_aug,ee_pos_aug=ee_pos_aug,ee_rot_aug=ee_rot_aug,
    act_pos_base=act_pos_base,ee_pos_base=ee_pos_base,ee_rot_base=ee_rot_base,
    traj = spiral_traj)


close_all(da,op)