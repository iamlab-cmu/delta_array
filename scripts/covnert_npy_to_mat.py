from scipy.io import savemat
import numpy as np

file_name = "Delta_1_Kinematic_Test_op_up"

D = np.load(file_name+".npz")

savemat(file_name + ".mat",mdict={key:D[key] for key in D})