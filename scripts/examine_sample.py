import numpy as np
import random

def load_training_data(filename, discard_flipped_ax = True):
	data =  np.load(filename)
	#data[:,1,:] -= [ 0.12610888, -0.15389195,  0.26082584]
	act_pos = 100*data["act_pos"]
	ee_rot = data["ee_rot"]
	ee_pos = 100*data["ee_pos"]

	if discard_flipped_ax:
		act_pos,ee_pos,ee_rot = discard_flipped_axes(act_pos,ee_pos,ee_rot)
	
	act_pos,ee_pos,ee_rot = discard_high_knees(act_pos,ee_pos,ee_rot)

	return act_pos,ee_pos,ee_rot

def discard_high_knees(act_pos,ee_pos,ee_rot):
	rigid_offset = 4.76 + .5

	valid_mask = np.all(act_pos < ((ee_pos[:,2]+rigid_offset)*np.ones(ee_pos.shape).T).T,1)
	return act_pos[valid_mask],ee_pos[valid_mask],ee_rot[valid_mask]


def get_n_to_1(act_pos,ee_pos,ee_rot,num_samples = None):
	'''
	find different actuator inputs that lead to the same position/rotation

	Args:
		num_samples = number of different sets of actuator positions to return
			i.e. is num_samples = 2, two sets M,N will be returned where
			every actuator position M maps to the same ee position, and every 
			actuator position in N maps to the same ee position
	'''
	# 
	# but ee_pos,ee_rot are the same
	if num_samples == None:
		num_samples = len(act_pos)
	
	aliasing_act_poses = []
	aliasing_masks = []

	indeces = list(range(len(act_pos)))
	random.shuffle(indeces)

	skip_inds = []
	for i in indeces:
		if i in skip_inds:
			continue

		# less than 1 mm difference in position
		close_pos_mask = np.linalg.norm(ee_pos - ee_pos[i],axis=1) < .1

		#less than 5 degrees difference in the rotation matrices
		#close_rot_mask = np.arccos(np.clip((np.trace(ee_rot @ ee_rot[i].T,axis1=1,axis2=2)-1)/2,-1,1))*360/2/np.pi < 5
		close_rot_mask = np.ones(ee_pos.shape[0],dtype=np.bool)

		far_act_pos_mask = (np.linalg.norm(act_pos-act_pos[i],axis=1) > .5) | np.all((act_pos==act_pos[i]),1)

		close_mask = close_pos_mask & close_rot_mask & far_act_pos_mask

		if np.sum(close_mask) > 1:
			aliasing_act_poses.append(act_pos[close_pos_mask])
			aliasing_masks.append(close_pos_mask)
			skip_inds.extend(np.argwhere(close_pos_mask).squeeze())
			if len(aliasing_masks) == num_samples:
				break

	return aliasing_act_poses, aliasing_masks

def check_data_match(act_pos_0,ee_pos_0,ee_rot_0,act_pos_1,ee_pos_1,ee_rot_1):
	pos_err = []
	rot_err = []
	for i,pos in enumerate(act_pos_0):
		j = np.argwhere(np.linalg.norm(act_pos_1-pos,axis=1)<1e-4)
		if j.shape[0] == 0:
			continue
		j = j.squeeze()
		pos_err.append(np.linalg.norm(ee_pos_0[i]-ee_pos_1[j]))
		rot_err.append(np.arccos(np.clip((np.trace(ee_rot_0[i] @ ee_rot_1[j].T)-1)/2,-1,1))*360/2/np.pi)

	return np.array(pos_err),np.array(rot_err)
	
def discard_flipped_axes(act_pos,ee_pos,ee_rot):
	keep_mask = np.ones(len(act_pos),dtype=np.bool)
	for i in range(1,len(act_pos)):
		if abs(np.arccos(np.clip((np.trace(ee_rot[i] @ ee_rot[0].T)-1)/2,-1,1))*360/2/np.pi) > 90:
			keep_mask[i] = False
	print(len(keep_mask)-np.sum(keep_mask),len(keep_mask))
	return act_pos[keep_mask],ee_pos[keep_mask],ee_rot[keep_mask]


if __name__ == "__main__":
	files = ["./training_data/training_data_rot.npz"]

	a,b,c = load_training_data(files[0])

	poses,masks = get_n_to_1(a,b,c)

	count = 0
	dists = []
	for p,m in zip(poses,masks):
		max_dist = 0
		for act in p:
			dist = np.min((np.max(abs(p-act),1)[np.any(abs(p-act) > 0,1)]))
			if dist > max_dist:
				max_dist = dist
		dists.append(max_dist)
				
	breakpoint()


	#pos_err,rot_err = check_data_match(a,b,c,a1,b1,c1)

	#pos_err,rot_err = check_data_match(a2,b2,c2,a1,b1,c1)