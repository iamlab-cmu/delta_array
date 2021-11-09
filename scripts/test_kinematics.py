from Prismatic_Delta import Prismatic_Delta
from Model import NN
from DeltaArray import DeltaArray
from OptiTrackStreaming.DataStreamer import OptiTrackDataStreamer
import numpy as np

def goto_position(self,da,pos,timeout=6):
    act_cmd = np.zeros((1,12))
    act_cmd[0,3:6] = pos
    act_cmd = adjust_act_command(act_cmd)
    da.move_joint_position(act_cmd, [1.])
    return da.wait_until_done_moving(timeout=timeout)

fk_save_file = "./models/training_data_rot_fk_full_aug"
ik_save_file = "./models/training_data_rot_ik_full_aug"

aug_model = NN(ik_save_file,fk_save_file,
			ik_save_file,fk_save_file,load=True)

fk_save_file = "./models/training_data_rot_fk_full"
ik_save_file = "./models/training_data_rot_ik_full"

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

for step in np.arange(step_size,4.6,step_size):
    for rad in np.linspace(0,2*pi,num=num_rad_steps,endpoint=False):
        pt = [step*np.cos(rad),step*np.sin(rad),traj_height]
        if rigid_Delta.is_valid(pt,4.3):
            spiral_traj.append(pt)

sprial_traj = np.array(spiral_traj)

rigid_ik = Delta.IK_Traj(sprial_traj)
base_ik,base_valid = base_model.predict_ik_traj(spiral_traj)
aug_ik,aug_valid = aug_model.predict_ik_traj(spiral_traj)

breakpoint()



    