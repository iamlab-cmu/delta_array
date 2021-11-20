from scipy.io import savemat
import numpy as np

file_name = "force_test_2_data_tpu_z"

D = np.load(file_name+".npz")

savemat(file_name + ".mat",mdict={key:D[key] for key in D})