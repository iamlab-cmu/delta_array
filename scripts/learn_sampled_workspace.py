from math import e
import numpy as np
from Model import NN
from examine_sample import load_training_data, get_n_to_1
from Prismatic_Delta import Prismatic_Delta
import config

# Files to load model from if load=True in model initialization
fk_load_file = "./models/flat_fk.h5"
ik_load_file = "./models/flat_ik.h5"

# Files to save model to after training
fk_save_file = "./models/flat_fk_3.h5"
ik_save_file = "./models/flat_ik_3.h5"

model = NN(ik_save_file,fk_save_file,
			ik_load_file,fk_load_file,load=False)

s_p = 1.538 #side length of the platform
s_b = 2.65 #side length of the base
l = 4.75 #length of leg attached to platform

rigid_Delta = Prismatic_Delta(s_p, s_b, l)
max_rad = 4.5 
z_des = config.LEARN_Z

model.learn_online(max_rad,z_des,rigid_Delta,learn_rigid=False)
model.save_all()