import numpy as np

############### Actuator Config ####################
MAX_ACTUATOR_HEIGHT = 4.3 #cm
MIN_ACTUATOR_HEIGHT = .5 #cm

ACTUATOR_ARDUINO_PORT = "/dev/cu.usbmodem14201" 


##############  Neural Net Params ##################

BATCH_SIZE = 64
MEM_SIZE = 500000
TRAIN_PER_EP = 32
TRAINING_EPOCHS = 300
NUM_POINTS_PER_EP = 5


############## Load Cell Params ###########
LOAD_CELL_PARAMS = {"m0":20.3,"b0":.0096,
	"m1":20.3,"b1":.0096,
	"m2":20.3,"b2":.0096,
	"m3":20.3,"b3":.0096}

# Map Load cell number to the direction of force the Delta is applying
# towards that load cell (from the Delta's frame of reference)
LOAD_CELL_DIRS = {
    0 : np.array([0,0,1]),
    1 : np.array([1,0,0]),
    2 : np.array([0,1,0]),
    3 : np.array([-1,0,0]),
    4 : np.array([0,-1,0])
}


NANO_PORT = "/dev/cu.usbserial-1410" 
INCREMENT = .1 #cm
LOAD_CELL_LAG = .25 #settling time for load cell in seconds
LOAD_CELL_HEIGHT = 1.9 #height above Delta