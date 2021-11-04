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

	def train_networks(self,act_pos,ee_pos,ee_rot):
		self.train_fk(act_pos,ee_pos,ee_rot)
		self.train_ik(act_pos,ee_pos)

	def train_fk(self,act_pos,ee_pos,ee_rot):
		quat_rot = self.mat2quat(ee_rot)
		ee = np.column_stack((ee_pos,quat_rot))

		self.fk.fit(act_pos,ee,verbose=0,epochs=40)

	def train_ik(self,act_pos,ee_pos):
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
		for act,ee in zip(act_pos,ee_pos):
			end_ind = start_ind + guesses_per_sample

			noise = np.random.normal(0,std,(guesses_per_sample,3))
			
			inp[start_ind:end_ind,:3] = ee
			inp[start_ind:end_ind,3:] = noise + act
			out[start_ind:end_ind,:] = act

			start_ind += guesses_per_sample

		self.ik.fit(inp,out,verbose=0,epochs=40)
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

	def predict_ik_from_guess(self,ee_pos,ik_guess):
		inp = np.column_stack((ee_pos,ik_guess))
		
		return self.ik.predict(inp)


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
