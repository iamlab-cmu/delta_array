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

class NN():
	def __init__(self,ik_save_file="./models/rigid_ik_a2c_model",fk_save_file="./models/rigid_fk_a2c_model",
			ik_load_file="./models/rigid_ik_a2c_model",fk_load_file="./models/rigid_fk_a2c_model",load=True):

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

		self.mem_buf = []
		self.mem_size = config.MEM_SIZE 
		self.batch_size = config.BATCH_SIZE 
		self.train_per_ep = config.TRAIN_PER_EP

	def evaluate(self,data):
		heights = data[:,0,:]
		ee_poses = data[:,1,:]

		fk_pred = self.predict(self.fk,heights)
		ik_pred = self.predict(self.ik,ee_poses)

		# print("fk_pred:",fk_pred[:3])
		# print("ik_pred: ",ik_pred[:3])
		# print("fk_ik:",self.inv_norm(self.fk(self.norm_cdf(ik_pred[:3]))))
		# print("ee_poses:",ee_poses[:3])
		# print("heights:",heights[:3])
		fk_error = np.mean(np.sqrt(np.sum(np.square(ee_poses-fk_pred),axis=1)))
		ik_error = np.mean(np.sqrt(np.sum(np.square(heights-ik_pred),axis=1)))

		print(" fk error ",fk_error)
		print(" ik error ",ik_error)

		print("\n-----------------------\n")

		return fk_error,ik_error


	def train_on_batch(self,heights,ee_poses):
		self.fk.fit(heights,ee_poses,verbose=0)
		#self.ik.fit(ee_poses,heights,verbose=0)

		with tf.GradientTape(watch_accessed_variables=False) as g:
			g.watch(self.ik.trainable_weights)
			ik_out = self.ik(ee_poses)
			fk_out = self.fk(ik_out)
			
			ik_loss = tf.reduce_mean(tf.keras.losses.MSE(fk_out,ee_poses))
			ik_grads = g.gradient(ik_loss,self.ik.trainable_weights)
		self.ik_Adam.apply_gradients(zip(ik_grads,self.ik.trainable_weights))

		# print("heights:",heights[:3])
		# print("ee_poses:",ee_poses[:3])

		# print("ik out:",ik_out[:3])
		# print("fk out:",fk_out[:3])
		# print("loss:" ,ik_loss)

		# print("\n---------------------\n")

	def train_from_mem(self):
		if len(self.mem_buf) == 0:
			return
		for i in range(self.train_per_ep):
			heights,ee_poses = self.sample_memory()
			self.train_on_batch(heights,ee_poses)


	def predict(self,model,x):
		x_out = model.predict(x)
		return x_out

	def remember(self,data):
		self.mem_buf.extend(data)
		if len(self.mem_buf) > self.mem_size:
			self.mem_buf = self.mem_buf[len(self.mem_buf)-self.mem_size:]

	def sample_memory(self,batch_size=None):
		if batch_size is None:
			batch_size = self.batch_size
		batch = random.choices(self.mem_buf,k=self.batch_size)
		batch = np.array(batch,dtype=float)
		heights = batch[:,0,:]
		ee_locs = batch[:,1,:]
		return heights,ee_locs


	def create_fk_model(self,input_dim = 3, output_dim = 3):
		model = Sequential()
		model.add(Dense(256, input_dim=input_dim, activation='relu'))
		model.add(Dense(256, activation='relu'))
		model.add(Dense(256, activation='relu'))
		model.add(Dense(output_dim, activation='linear'))

		model.compile(optimizer=keras.optimizers.Adam(),
               loss='MSE')
		return model


	def create_ik_model(self,input_dim = 3, output_dim = 3):
		model = Sequential()
		model.add(Dense(256, input_dim=input_dim, activation='relu'))
		model.add(Dense(256, activation='relu'))
		model.add(Dense(256, activation='relu'))
		model.add(Dense(output_dim, activation='linear'))

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
