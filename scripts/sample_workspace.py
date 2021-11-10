from DeltaArray import DeltaArray
from OptiTrackStreaming.DataStreamer import OptiTrackDataStreamer
import numpy as np
import time
import config
from math_util import quaternion_rotation_matrix
from delta_utils import record_trajectory


# da = DeltaArray('/dev/ttyACM0') -- CHANGE PORT
da = DeltaArray('COM3')

# for this to work enter the "Data Streaming Pane" in Motive:
#   "Broadcast Frame Data" is turned on
#   Command Port = 1510
#   Data Port = 1511
#   Multicast Interface = 239.255.42.99

# https://v22.wiki.optitrack.com/index.php?title=Data_Streaming
op = OptiTrackDataStreamer()


def sample_domain(max_height, max_actuator_dif,spacing=.5):
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
        count_similar_points = np.var((sample-pt),axis=1) < 1e-5 # pure z translation
        if np.sum(count_similar_points[:i]) < 2:
            new_sample.append(pt)
    return np.array(new_sample)



def save_training_data(act_pos,ee_pos,ee_rot, filename):
    np.savez(filename,act_pos=act_pos,ee_pos=ee_pos,ee_rot=ee_rot)



sample = sample_domain(5,4.5,.3)
#sample = limit_sample(sample)

act_pos,ee_pos,ee_rot = record_trajectory(da,op,sample)

save_training_data(act_pos,ee_pos,ee_rot,"training_data_op_up")

da.close()
op.close()