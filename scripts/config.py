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
LEARN_Z = 2 #value of z at which to learn Kinematics

############## Load Cell Params ###########
LOAD_CELL_PARAMS = {"m0":20.3,"b0":.0096,
	"m1":20.3,"b1":.0096,
	"m2":20.3,"b2":.0096,
	"m3":20.3,"b3":.0096}

# Map Load cell number to the direction of force the Delta is applying
# towards that load cell (from the Delta's frame of reference)
LOAD_CELL_DIRS = {
    0 : np.array([0,-1,0]),
    1 : np.array([1,0,0]),
    2 : np.array([0,1,0]),
    3 : np.array([-1,0,0])
}

#For z direction Test
LOAD_CELL_DIRS = {
    0:np.array([0,0,1])
}

################# Force Test Params ###############################
RETRACT_POS = [.75,.75,.75] #actuator position for fully retracted Delta
TEST_HEIGHT = 3.5 #Height of Delta at which to test forces
REST_HEIGHT = 1.25
DIST2CELL = .25 # cm from finger to load cell

#For z direction test:
TEST_HEIGHT = 3.5 #Height of Delta at which to test forces
REST_HEIGHT = 1.25
DIST2CELL = 0 # cm from finger to load cell

TEST_INC = .3 #increment by TEST_INC cm until twisting is observed



NANO_PORT = "/dev/cu.usbserial-1410" 
INCREMENT = .1 #cm
LOAD_CELL_LAG = 1 #settling time for load cell in seconds
LOAD_CELL_HEIGHT = 1.9 #height above Delta