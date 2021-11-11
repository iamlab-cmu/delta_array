import tensorflow.keras as keras
import tensorflow as tf
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense
from tensorflow.keras.layers import Input
from tensorflow.keras.models import load_model
import numpy as np 
import config
import random
import time

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

	def train_networks(self,act_pos,ee_pos,ee_rot,augment_data=False):
		self.train_fk(act_pos,ee_pos,ee_rot,augment_data=augment_data)
		self.train_ik(act_pos)

	def train_fk(self,act_pos,ee_pos,ee_rot,augment_data=False):
		# augment data by rotating actuators/end effector by +- 120 degrees
		# then flip actuators 2,3 to get corresponding data with the end effector
		# flipped over the x axis

		if augment_data:
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

			for i in range(len(aug_act_pos)):
				act_pos = aug_act_pos[i]
				if np.allclose(act_pos,act_pos[0]):
					aug_ee_pos_final[i] = [0,0,act_pos[0]]

			aug_ee = np.column_stack((aug_ee_pos_final,self.mat2quat(aug_ee_rot)))
		else:
			aug_ee = np.column_stack((ee_pos,self.mat2quat(ee_rot)))
			aug_act_pos = act_pos

		self.fk.fit(aug_act_pos,aug_ee,verbose=1,epochs=15)

	def train_ik(self,act_pos):
		'''
		IK needs an initial guess to find the actuator position because the IK is not 1 to 1
		This function maps ee_pos -> act_pos:
			for the initial guess (act_pos)
			and for the initial guesses (act_pos) + normal_dist(mean=[0,0,0],guess_std) 
		So the network will return the ik solution that is closest to the initial guess
		'''
		out = self.fk(act_pos)[:,:3]

		self.ik.fit(out,act_pos,verbose=1,epochs=20)

	def predict_fk(self,act_pos):
		pred = self.fk.predict(act_pos)
		ee_pos = pred[:,:3]
		ee_rot = self.quat2mat(pred[:,3:])

		return ee_pos,ee_rot

	def predict_ik(self,ee_pos,valid_tol = .3, return_valid_mask=False):
		ik_pred = self.ik(ee_pos)

		if not return_valid_mask:
			return ik_pred

		fk_pred = self.predict_fk(ik_pred)[0]
		valid_mask = np.linalg.norm(fk_pred-ee_pos,axis=1) < valid_tol
		
		return ik_pred,valid_mask


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


	def create_ik_model(self,input_dim = 3, output_dim = 3):
		"""
		Maps [ee_pos] to actuator position

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

	
	def get_points_on_circle(self,num_pts,max_rad,z_des):
		rads = np.random.uniform(0,max_rad,num_pts)
		angles = np.random.uniform(0,2*np.pi,num_pts)
		c = np.cos(angles)
		s = np.sin(angles)

		pts = np.column_stack((rads*c,rads*s,np.ones(num_pts)*z_des))
		return pts
	
	def get_spiral(self,num_rotations,pts_per_rot,max_rad,z_des):
		num_pts = pts_per_rot*num_rotations
		start_angle = np.random.uniform(0,2*np.pi)
		radii = np.linspace(0,max_rad,num=num_pts)
		rads = np.linspace(start_angle,start_angle+2*np.pi*num_rotations,num=num_pts)

		pts = np.vstack((np.cos(rads)*radii,np.sin(rads)*radii,z_des*np.ones(len(rads)))).T
		return pts

	
	def sample_mem_buf(self,mem_buf):
		num_samples = min(len(mem_buf),config.BATCH_SIZE)
		inds = np.random.choice(np.arange(len(mem_buf)),num_samples,replace=False)
		d = np.array(mem_buf)[inds]
		act_pos,ee_pos,ee_rot,goal_pos = list(zip(*d))
		return np.array(act_pos),np.array(ee_pos),np.array(ee_rot),np.array(goal_pos)

	def train_on_batch(self,mem_buf):
		act_pos,ee_pos,ee_rot,goal_pos = self.sample_mem_buf(mem_buf)
		ee = np.column_stack((ee_pos,ee_rot))

		self.fk.fit(act_pos,ee,epochs=1,verbose=0)
		with tf.GradientTape(watch_accessed_variables=False) as g:
			g.watch(self.ik.trainable_weights)
			ik_out = self.ik(goal_pos)
			fk_out = self.fk(ik_out)[:,:3]
			
			ik_loss = tf.reduce_mean(tf.keras.losses.MSE(fk_out,goal_pos))
			ik_grads = g.gradient(ik_loss,self.ik.trainable_weights)

		self.ik_Adam.apply_gradients(zip(ik_grads,self.ik.trainable_weights))


	def learn_online(self,max_rad,z_des,rigid_delta,learn_rigid=False):
		from delta_utils import record_trajectory, center_delta, initialize_array_recording, close_all
		da,op = initialize_array_recording()
		
		pos_0, rot_0 = center_delta(da,op)
		mem_buf = []

		if learn_rigid:
			rigid_ik_offset = rigid_delta.FK([0,0,0])[0,2]
			pre_train_pts = self.get_points_on_circle(200,max_rad,z_des+rigid_ik_offset)
			
			rigid_ik = rigid_delta.IK_Traj(pre_train_pts)
			pre_train_pts[:,-1] -= rigid_ik_offset

			rots = np.column_stack((np.zeros((200,3)),np.ones((200,1))))
			print("Pretraining with Rigid Kinematics")
			self.fk.fit(rigid_ik,np.column_stack((pre_train_pts,rots)),epochs=10)
			self.ik.fit(pre_train_pts,rigid_ik,epochs=10)
		
		start_time = time.time()

		for epoch in range(config.TRAINING_EPOCHS):
			print("Epoch:",epoch)

			if epoch % 25 == 0:
				#evaluate to check if done early
				traj = self.get_spiral(3,30,max_rad,z_des)
				ik_pts = self.ik(traj)

				act_poses,ee_poses,ee_rots = record_trajectory(da,op,ik_pts,pos_0=pos_0,rot_0=rot_0)

				eval_error = np.mean(np.linalg.norm(ee_poses-traj,axis=1))
				print("Evaluation Error:",eval_error)

				mem_buf.extend(list(zip(act_poses,ee_poses,ee_rots,traj)))

				if eval_error < .2:
					return

			pts = self.get_points_on_circle(config.NUM_POINTS_PER_EP,max_rad,z_des)
			traj = []
			for i in range(len(pts)-1):
				dist = np.linalg.norm(pts[i]-pts[i+1])
				traj.extend(np.linspace(pts[i],pts[i+1],num=int(dist/.75)+1,endpoint=False))
			traj.append(pts[-1])
			traj = np.array(traj)

			ik_pts = self.ik(traj)

			act_poses,ee_poses,ee_rots = record_trajectory(da,op,ik_pts,pos_0=pos_0,rot_0=rot_0)

			print("Trajectory Error:",np.mean(np.linalg.norm(ee_poses-traj,axis=1)))

			mem_buf.extend(list(zip(act_poses,ee_poses,ee_rots,traj)))

			for i in range(min(epoch+5,config.TRAIN_PER_EP)):
				self.train_on_batch(mem_buf)
			self.save_all()
		
		end_time = time.time()
		print("Training took:",end_time-start_time)
		close_all(da,op)

			








		
