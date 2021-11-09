import tensorflow.keras as keras
import tensorflow as tf
from tensorflow.keras.models import Model
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense,Activation,BatchNormalization
from tensorflow.keras.layers import Input
from tensorflow.keras.models import load_model
import numpy as np 
import config
import random

from scipy.spatial.transform import Rotation as R

class NN():
	def __init__(self,ik_save_file,fk_save_file,
			ik_load_file,fk_load_file,load=True):

		self.fk = self.create_fk_model()
		self.ik = self.create_ik_model()

		self.fk_save_file = fk_save_file
		self.ik_save_file = ik_save_file

		self.fk_load_file = fk_load_file
		self.ik_load_file = ik_load_file

		self.ik_Adam = tf.keras.optimizers.Adam(.0005)

		self.max_height = config.MAX_ACTUATOR_HEIGHT

		if(load):
			self.load_all()

	def mat2quat(self,rots):
		return R.from_matrix(rots).as_quat()
	
	def quat2mat(self,rots):
		return R.from_quat(rots).as_matrix()

	def evaluate_fk(self,act_pos,ee_pos,ee_rot):
		ee_pos_guess,ee_rot_guess = self.predict_fk(act_pos)

		pos_err = np.mean(np.linalg.norm(ee_pos-ee_pos_guess,axis=1))
		rot_err = np.mean(abs(np.arccos(np.clip((np.trace(np.moveaxis(ee_rot_guess,2,1) @ ee_rot)-1)/2,-1,1))*360/2/np.pi))

		print("FK position error (cm):",pos_err)
		print("FK rotation error (deg):",rot_err)

	def train_networks(self,act_pos,ee_pos,ee_rot):
		self.train_fk(act_pos,ee_pos,ee_rot)
		self.train_ik(act_pos)

	def train_fk(self,act_pos,ee_pos,ee_rot):
		# augment data by rotating actuators/end effector by +- 120 degrees
		# then flip actuators 2,3 to get corresponding data with the end effector
		# flipped over the x axis

		n = act_pos.shape[0]
		aug_act_pos = np.zeros((n*6,3))
		aug_act_pos[:n,:] = act_pos
		
		aug_ee_pos = np.zeros((n*6,3))
		aug_ee_rot = np.zeros((n*6,3,3))
		aug_ee_pos[:n,:] = ee_pos
		aug_ee_rot[:n,:] = ee_rot

		aug_act_pos[n:2*n,:] = np.column_stack((act_pos[:,1:],act_pos[:,0]))
		rot = R.from_rotvec([0,0,2*np.pi/3]).as_matrix()
		aug_ee_pos[n:2*n,:] = (rot @ ee_pos.T).T
		aug_ee_rot[n:2*n,:,:] = rot @ ee_rot

		aug_act_pos[2*n:3*n,:] = np.column_stack((act_pos[:,2],act_pos[:,:2]))
		rot = R.from_rotvec([0,0,-2*np.pi/3]).as_matrix()
		aug_ee_pos[2*n:3*n,:] = (rot @ ee_pos.T).T
		aug_ee_rot[2*n:3*n,:,:] = rot @ ee_rot

		aug_act_pos[3*n:,:] = np.column_stack((aug_act_pos[:3*n,0],aug_act_pos[:3*n,2],aug_act_pos[:3*n,1]))
		rot = np.eye(3)
		rot[0,0]=-1 #flip over x axis
		aug_ee_pos[3*n:,:] = (rot @ aug_ee_pos[:3*n,:].T).T
		aug_ee_rot[3*n:,:,:] = rot @ aug_ee_rot[:3*n,:,:]

		aug_ee_pos_final = np.zeros_like(aug_ee_pos)
		for i in range(6*n):
			close_mask = np.isclose(np.linalg.norm(aug_act_pos-aug_act_pos[i],axis=1),0)
			aug_ee_pos_final[close_mask,:] = np.mean(aug_ee_pos[close_mask,:],axis=0)

		unique_mask = np.unique(aug_ee_pos_final,axis=0,return_index=True)[1]
		aug_act_pos = aug_act_pos[unique_mask]
		aug_ee_pos_final = aug_ee_pos_final[unique_mask]
		aug_ee_rot = aug_ee_rot[unique_mask]

		aug_ee = np.column_stack((aug_ee_pos_final,self.mat2quat(aug_ee_rot)))

		self.fk.fit(aug_act_pos,aug_ee,verbose=1,epochs=15)

	def train_ik(self,act_pos):
		'''
		IK needs an initial guess to find the actuator position because the IK is not 1 to 1
		This function maps ee_pos -> act_pos:
			for the initial guess (act_pos)
			and for the initial guesses (act_pos) + normal_dist(mean=[0,0,0],guess_std) 
		So the network will return the ik solution that is closest to the initial guess
		'''
		guesses_per_sample = config.IK_GUESSES_PER_SAMPLE
		std = config.IK_GUESS_STD
		num_samples = len(act_pos)

		inp = np.zeros((guesses_per_sample*num_samples,6))
		out = np.zeros((guesses_per_sample*num_samples,3))
		start_ind = 0

		# use fk to predict rather than using data directly
		# because data has been 'improved' by data augmentation
		ee_pos = self.predict_fk(act_pos)[0] 
		for act,ee in zip(act_pos,ee_pos):
			end_ind = start_ind + guesses_per_sample

			noise = np.random.normal(0,std,(guesses_per_sample,3))
			
			inp[start_ind:end_ind,:3] = ee
			inp[start_ind:end_ind,3:] = noise + act
			out[start_ind:end_ind,:] = act

			start_ind += guesses_per_sample

		self.ik.fit(inp,out,verbose=0,epochs=20)
		# with tf.GradientTape(watch_accessed_variables=False) as g:
		# 	g.watch(self.ik.trainable_weights)
		# 	ik_out = self.ik(inp)
		# 	fk_out = self.fk(ik_out)
			
		# 	ik_loss = tf.reduce_mean(tf.keras.losses.MSE(fk_out[:,:3],))
		# 	ik_grads = g.gradient(ik_loss,self.ik.trainable_weights)
		# self.ik_Adam.apply_gradients(zip(ik_grads,self.ik.trainable_weights))

	def predict_fk(self,act_pos):
		pred = self.fk.predict(act_pos)
		ee_pos = pred[:,:3]
		ee_rot = self.quat2mat(pred[:,3:])

		return ee_pos,ee_rot

	def predict_ik(self,ee_pos,ik_guess=None):
		if ik_guess is None:
			return self.predict_ik_without_guess(ee_pos)
		inp = np.column_stack((ee_pos,ik_guess))
		
		return self.ik.predict(inp)

	def predict_ik_without_guess(self,ee_pos):
		'''
		Args:
			ee_pos: N x 3 array of end effector positions 

		Correct guess is known along the line [0,0,z] (it is [z,z,z])

		Start from closest point on [0,0,z] and step by ~.2 cm at a time until ee_pos
			is reached using each solution for point i as the guess for point i+1
		'''

		easy_pos = np.zeros(ee_pos.shape)
		easy_pos[:,2] = ee_pos[:,2]
		easy_guess = (ee_pos[:,2]*np.ones(ee_pos.shape).T).T

		max_dist = np.max(abs(ee_pos[:,:2]))
		num_pts = int((max_dist/.2)) + 1
		traj = np.linspace(easy_pos,ee_pos,num_pts)

		curr_guess = easy_guess
		for t in traj:
			pred = self.predict_ik(t,curr_guess)
			curr_guess = pred
		
		return curr_guess

	def predict_ik_traj(self,traj,mask_tolerance=.3):
		'''
		Args:
			traj: Nx3 trajectory of desired end effector positions for Delta
			mask_tolerance : see returned Valid_Mask

			Both args are in cm

		Returns 
			IK: Nx3 inverse kinematic result for each point in trajectory
			Valid_Mask: N x 1 mask, False where FK(IK) != traj (outside mask_tolerance)

		The IK for each point in the trajectory is used as the guess for the next point
			unless traj[i] is more than .3 cm from traj[i+1] or valid_mask[i] == False
		'''

		IK = []
		Valid_Mask = []

		curr_guess = None
		last_pt = None
		for pt in traj:
			if last_pt is not None and np.linalg.norm(pt-last_pt) > .3:
				curr_guess = None
			curr_guess = self.predict_ik(np.expand_dims(pt,0),ik_guess=curr_guess)
			IK.append(curr_guess)
			f,_ = self.predict_fk(curr_guess)
			if np.linalg.norm(f-pt) > mask_tolerance:
				Valid_Mask.append(False)
				curr_guess = None
			else:
				Valid_Mask.append(True)
			last_pt = pt
		

		return np.array(IK).squeeze(),np.array(Valid_Mask)


	def create_fk_model(self,input_dim = 3, output_dim = 7):
		'''
		Maps actuator position to [ee_pos,ee_rot]
		where ee_rot is a quaternion
		'''

		model = Sequential()
		model.add(Dense(256, input_dim=input_dim, activation='relu'))
		model.add(Dense(256, activation='relu'))
		model.add(Dense(256, activation='relu'))
		model.add(Dense(output_dim, activation='linear'))

		model.compile(optimizer=keras.optimizers.Adam(),
               loss='MSE')
		return model


	def create_ik_model(self,input_dim = 6, output_dim = 3):
		"""
		Maps [ee_pos,ik_guess] to actuator position

		The IK function is not 1 to 1, so an initial guess for the solution
		is necessary to distinguish between ik solutions

		IK does not learn an inverse for rotation because rotation is coupled with position, so the 
		caller would have no way of specifying a correct position/rotation pair.
		"""

		model = Sequential()
		model.add(Dense(256, input_dim=input_dim, activation='relu'))
		model.add(Dense(256, activation='relu'))
		model.add(Dense(256, activation='relu'))
		model.add(Dense(output_dim, activation='linear'))

		model.compile(optimizer=keras.optimizers.Adam(),
               loss='MSE')

		return model

	def save_all(self):
		self.save(self.fk,self.fk_save_file)
		self.save(self.ik,self.ik_save_file)

	def load_all(self):
		self.load_fk()
		self.load_ik()

	def load(self,model_file):
		return keras.models.load_model(model_file)

	def load_fk(self):
		self.fk = self.load(self.fk_load_file)

	def load_ik(self):
		self.ik = self.load(self.ik_load_file)

	def save(self,model,model_file):
		model.save(model_file)

	def sample_workspace(self,max_height,sample_dens):
		num_pts = int(max_height/sample_dens)
		heights = np.empty((num_pts**3,3))
		idx = 0
		for i in range(num_pts):
			for j in range(num_pts):
				for k in range(num_pts):
					heights[idx,:] = sample_dens*np.array([i,j,k])
					idx += 1
		pred = self.predict(self.fk,heights)
		return pred
