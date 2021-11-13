from math import e
import numpy as np
from Model import NN
from examine_sample import load_training_data, get_n_to_1
from Prismatic_Delta import Prismatic_Delta

fk_load_file = "./models/training_data_flat_fk.h5"
ik_load_file = "./models/training_data_flat_ik.h5"

fk_save_file = "./models/training_data_flat_fk_3.h5"
ik_save_file = "./models/training_data_flat_ik_3.h5"

fk_save_file = "./models/training_data_aug_fk.h5"
ik_save_file = "./models/training_data_aug_ik.h5"

files = ["./training_data/training_data_op_up.npz","./training_data/training_data_rot.npz"]

model = NN(ik_save_file,fk_save_file,
			ik_load_file,fk_load_file,load=False)

act_pos,ee_pos,ee_rot = load_training_data(files[1])
act_pos *= 100
model.train_networks(act_pos,ee_pos,ee_rot,augment_data=True)

model.save_all()
# s_p = 1.538 #side length of the platform
# s_b = 2.65 #side length of the base
# l = 4.75 #length of leg attached to platform

# rigid_Delta = Prismatic_Delta(s_p, s_b, l)
# max_rad = 4.5 
# z_des = config.LEARN_Z

# model.learn_online(max_rad,z_des,rigid_Delta,learn_rigid=False)
# model.save_all()