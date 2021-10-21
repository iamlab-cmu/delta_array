import numpy as np
from Model import NN

def load_training_data(filename):
	data =  np.load(filename).astype(float)
	#data[:,1,:] -= [ 0.12610888, -0.15389195,  0.26082584]
	data[:,1,:] -= data[0,1,:]
	data *= 100 #convert to cm
	return data

model = NN(fk_save_file="./models/training_data1_fk",
			ik_save_file="./models/training_data1_ik",load=False)

def train_from_file(filename):
	model = NN(fk_save_file="./models/training_data1_fk",
			ik_save_file="./models/training_data1_ik",load=False)
	
	data = load_training_data(filename)
	heights = data[:,0,:]
	ee_poses = data[:,1,:]

	model.fk.fit(heights,ee_poses,epochs=40,verbose=0)
	#model.ik.fit()
	return model

m1 = train_from_file("training_data.npy")
m2 = train_from_file("training_data2.npy")
m3 = train_from_file("training_data3.npy")
m4 = train_from_file("training_data4.npy")

test_data = np.random.uniform(low=0,high=4,size=(100,3))
e1 = m1.fk(test_data)
e2 = m2.fk(test_data)
e3 = m3.fk(test_data)
e4 = m4.fk(test_data)

print("1,2:",np.mean(np.linalg.norm(e1-e2,axis=1)))
print("3,2:",np.mean(np.linalg.norm(e3-e2,axis=1)))
print("1,3:",np.mean(np.linalg.norm(e1-e3,axis=1)))
print("1,4:",np.mean(np.linalg.norm(e1-e4,axis=1)))
print("3,4:",np.mean(np.linalg.norm(e3-e4,axis=1)))
print("2,4:",np.mean(np.linalg.norm(e4-e2,axis=1)))

