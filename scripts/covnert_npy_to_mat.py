from scipy.io import savemat
import numpy as np
import glob
import os
npzFiles = glob.glob("*.npy")
for f in npzFiles:
    fm = os.path.splitext(f)[0]+'.mat'
    data = np.load(f)
    d = {"training_data":data}
    savemat(fm, d)
    print('generated ', fm, 'from', f)